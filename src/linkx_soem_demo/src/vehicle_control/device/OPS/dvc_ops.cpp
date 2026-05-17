/**
 * @file  dvc_ops.cpp
 * @brief 全方位平面定位系统 OPS-9 驱动实现（CAN 透传模式）
 *
 *  接收路径：转换器把 OPS 28 字节串口帧拆为 4 个 CAN 帧（DLC=8/8/8/4），
 *  全部用同一 CAN ID。本类用滑窗重组并按 [0D 0A ... 0A 0D] 自同步。
 *
 *  发送路径：把 4 字节 ASCII 命令（"ACTR"/"ACT0"）或
 *  4 字节 ASCII + 4 字节 float（"ACTJ/X/Y" + value）打包为单 CAN 帧
 *  下发，转换器透传到 RS232 → OPS。
 */

#include "dvc_ops.h"
#include <cstring>
#include <iostream>

namespace
{
/// 浮点反序列化：从 buf 读 4 字节小端序 IEEE 754 float
inline float ops_bytes_to_float_le(const uint8_t *buf)
{
    float v;
    std::memcpy(&v, buf, sizeof(v));
    return v;
}

/// 浮点序列化：把 v 以 4 字节小端序 IEEE 754 写入 buf
inline void ops_float_to_bytes_le(float v, uint8_t *buf)
{
    std::memcpy(buf, &v, sizeof(v));
}
} // namespace

/* ============================================================
 *  生命周期
 * ========================================================== */

/**
 * @brief 绑定底层 CAN 通道与目标 ID；不下发任何命令
 */
void Class_OPS::Init(linkx_t *linkx_ptr, uint8_t can_channel, uint32_t can_id)
{
    linkx_handler_ = linkx_ptr;
    can_channel_   = can_channel;
    can_id_        = can_id;

    rx_buffer_len_       = 0;
    data_                = {};
    status_              = OPS_Status_DISABLE;
    alive_flag_          = 0;
    pre_alive_flag_      = 0;
    rx_frame_count_      = 0;
    resync_count_        = 0;
    tail_mismatch_count_ = 0;

    std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

/* ============================================================
 *  CAN 接收 / 重组 / 解析
 * ========================================================== */

/**
 * @brief 处理一帧 CAN 数据片段
 *
 *  1) 取 dlen 字节追加到 rx_buffer_
 *  2) 缓冲若超过容量，丢弃最旧字节直到能容纳新数据
 *  3) 调用 Try_Parse_Frame_ 推进解析（一次回调可能解出多帧）
 */
void Class_OPS::CAN_RxCpltCallback(const uint8_t *rx_data, uint8_t dlen)
{
    if (rx_data == nullptr || dlen == 0) return;

    // 安全裁剪：CAN 经典帧最长 8 字节
    uint8_t copy_len = (dlen > 8) ? 8 : dlen;

    // 累加存活计数（TIM_Alive 用以检测离线）
    alive_flag_ += 1;

    // 若新数据装不下，先把缓冲前部丢够空间
    if (rx_buffer_len_ + copy_len > OPS_RX_BUFFER_CAPACITY)
    {
        const size_t need_drop = (rx_buffer_len_ + copy_len) - OPS_RX_BUFFER_CAPACITY;
        Drop_Front_(need_drop);
    }

    std::memcpy(rx_buffer_ + rx_buffer_len_, rx_data, copy_len);
    rx_buffer_len_ += copy_len;

    Try_Parse_Frame_();
}

/**
 * @brief 滑窗对齐 + 整帧解析
 *
 *  循环：
 *    - 找首个 [0D 0A] 头；找不到时只保留缓冲末尾的 1 个字节
 *      （防止头跨包丢失），其余丢弃
 *    - 找到但不在 index 0：丢弃头之前的字节并 resync_count_+1
 *    - 缓冲不足 28 字节：等待下一片段
 *    - 检查偏移 26/27 处尾标 [0A 0D]：
 *        ✗ 尾错位：丢前 2 字节（跳过假头）继续找
 *        ✓ 尾正确：解 6 个 float、丢前 28 字节、rx_frame_count_+1
 */
void Class_OPS::Try_Parse_Frame_()
{
    while (true)
    {
        // ---- 1. 找头 [0D 0A] ----
        size_t header_idx = SIZE_MAX;
        if (rx_buffer_len_ >= 2)
        {
            for (size_t i = 0; i + 1 < rx_buffer_len_; ++i)
            {
                if (rx_buffer_[i]   == OPS_FRAME_HEADER_BYTE0 &&
                    rx_buffer_[i+1] == OPS_FRAME_HEADER_BYTE1)
                {
                    header_idx = i;
                    break;
                }
            }
        }

        if (header_idx == SIZE_MAX)
        {
            // 没找到完整头：保留末尾最多 1 字节（可能是头的第一个字节）
            if (rx_buffer_len_ >= 1)
            {
                rx_buffer_[0] = rx_buffer_[rx_buffer_len_ - 1];
                rx_buffer_len_ = 1;
            }
            return;
        }

        // ---- 2. 把头对齐到 index 0 ----
        if (header_idx > 0)
        {
            resync_count_ += 1;
            Drop_Front_(header_idx);
        }

        // ---- 3. 数据不足整帧：等下一片段 ----
        if (rx_buffer_len_ < OPS_FRAME_TOTAL_LEN)
            return;

        // ---- 4. 校验帧尾 [0A 0D] ----
        if (rx_buffer_[OPS_FRAME_TOTAL_LEN - 2] != OPS_FRAME_TAIL_BYTE0 ||
            rx_buffer_[OPS_FRAME_TOTAL_LEN - 1] != OPS_FRAME_TAIL_BYTE1)
        {
            // 头匹配但尾不匹配：当前对齐失败，跳过这个假头继续找
            tail_mismatch_count_ += 1;
            Drop_Front_(2);
            continue;
        }

        // ---- 5. 解析 24 字节载荷 = 6 × float32 LE ----
        const uint8_t *p = rx_buffer_ + OPS_FRAME_PAYLOAD_OFFSET;
        data_.yaw_deg      = ops_bytes_to_float_le(p +  0);
        data_.pitch_deg    = ops_bytes_to_float_le(p +  4);
        data_.roll_deg     = ops_bytes_to_float_le(p +  8);
        data_.pos_x_mm     = ops_bytes_to_float_le(p + 12);
        data_.pos_y_mm     = ops_bytes_to_float_le(p + 16);
        data_.yaw_rate_dps = ops_bytes_to_float_le(p + 20);

        rx_frame_count_ += 1;

        // ---- 6. 丢弃已消费的整帧，继续看缓冲里有没有下一帧 ----
        Drop_Front_(OPS_FRAME_TOTAL_LEN);
    }
}

/**
 * @brief 把缓冲前 n 字节丢弃，剩余前移
 */
void Class_OPS::Drop_Front_(size_t n)
{
    if (n == 0) return;
    if (n >= rx_buffer_len_)
    {
        rx_buffer_len_ = 0;
        return;
    }
    const size_t remain = rx_buffer_len_ - n;
    std::memmove(rx_buffer_, rx_buffer_ + n, remain);
    rx_buffer_len_ = remain;
}

/* ============================================================
 *  存活检测
 * ========================================================== */

/**
 * @brief 周期回调：alive_flag 自上次未变 → DISABLE
 */
void Class_OPS::TIM_Alive_PeriodElapsedCallback()
{
    status_ = (alive_flag_ == pre_alive_flag_) ? OPS_Status_DISABLE : OPS_Status_ENABLE;
    pre_alive_flag_ = alive_flag_;
}

/* ============================================================
 *  发送命令（→ CAN → 转换器 → RS232 → OPS）
 * ========================================================== */

/** @brief 把 4 字节 ASCII 命令打包为 CAN 帧（DLC=8，后 4 字节填 0） */
void Class_OPS::Send_Cmd_4_(const char prefix[4])
{
    if (linkx_handler_ == nullptr) return;
    uint8_t buf[8] = {0};
    std::memcpy(buf, prefix, 4);
    linkx_quick_can_send(linkx_handler_, can_channel_, can_id_, buf);
}

/** @brief 把 4 字节 ASCII + 4 字节 float LE 打包为 CAN 帧（DLC=8） */
void Class_OPS::Send_Cmd_4_Float_(const char prefix[4], float v)
{
    if (linkx_handler_ == nullptr) return;
    uint8_t buf[8] = {0};
    std::memcpy(buf, prefix, 4);
    ops_float_to_bytes_le(v, buf + 4);
    linkx_quick_can_send(linkx_handler_, can_channel_, can_id_, buf);
}

/** @brief 发送 "ACTR"：开始零漂校准（约 15 分钟，全程保持模块绝对静止） */
void Class_OPS::Send_Calibrate()
{
    Send_Cmd_4_(OPS_CMD_CALIBRATE_PREFIX);
    std::cout << "[OPS] Send ACTR (calibration begin, keep static for ~15 min)" << std::endl;
}

/** @brief 发送 "ACT0"：当前角度/坐标全部置零 */
void Class_OPS::Send_Zero()
{
    Send_Cmd_4_(OPS_CMD_ZERO_PREFIX);
    std::cout << "[OPS] Send ACT0 (zero angles & coords)" << std::endl;
}

/** @brief 发送 "ACTJ" + float：标定当前航向角到指定值（deg） */
void Class_OPS::Send_Update_Yaw(float yaw_deg)
{
    Send_Cmd_4_Float_(OPS_CMD_UPDATE_YAW_PREFIX, yaw_deg);
    std::cout << "[OPS] Send ACTJ yaw=" << yaw_deg << " deg" << std::endl;
}

/** @brief 发送 "ACTX" + float：标定当前 X 坐标到指定值（mm） */
void Class_OPS::Send_Update_X(float x_mm)
{
    Send_Cmd_4_Float_(OPS_CMD_UPDATE_X_PREFIX, x_mm);
    std::cout << "[OPS] Send ACTX x=" << x_mm << " mm" << std::endl;
}

/** @brief 发送 "ACTY" + float：标定当前 Y 坐标到指定值（mm） */
void Class_OPS::Send_Update_Y(float y_mm)
{
    Send_Cmd_4_Float_(OPS_CMD_UPDATE_Y_PREFIX, y_mm);
    std::cout << "[OPS] Send ACTY y=" << y_mm << " mm" << std::endl;
}
