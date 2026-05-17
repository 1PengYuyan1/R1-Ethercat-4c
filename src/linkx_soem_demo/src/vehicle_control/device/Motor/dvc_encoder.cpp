/**
 * @file dvc_encoder_brt.cpp
 * @author Your_Name (your@email.com)
 * @brief 布瑞特编码器CAN通信实现
 * @version 0.1
 * @date 2024-01-01
 *
 * @copyright Your_Company (c) 2024
 */
#include "dvc_encoder.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <cstdint>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
namespace
{
constexpr bool kEncoderDebugPrintEnable = false;
constexpr uint64_t kEncoderDebugPrintPeriodMs = 100;

inline int32_t positive_mod_i32(int32_t value, int32_t mod)
{
    int32_t r = value % mod;
    return (r < 0) ? (r + mod) : r;
}

inline uint64_t encoder_now_ms()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
}

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 编码器初始化
 *
 * @param linkx_ptr 网关句柄
 * @param slave_index EtherCAT 从站索引
 * @param encoder_id 编码器 CAN ID(1-255)
 * @param single_turn_res 单圈分辨率
 * @param baudrate 通信波特率枚举（仅写入 config，不主动下发 0x03）
 */
void Class_Encoder_BRT::Init(linkx_t *linkx_ptr, uint16_t slave_index, uint8_t encoder_id,
                             uint16_t single_turn_res, Enum_BRT_Encoder_Baudrate baudrate)
{
    this->LinkX_Ptr = linkx_ptr; // 保存网关指针
    this->Slave_Index = slave_index;
    this->config = {};
    // 初始化配置参数
    config.can_id = encoder_id;
    config.zero_offset = 0;
    config.single_turn_resolution = single_turn_res;
    config.max_turns = 50;              // 默认最大圈数
    config.work_mode = BRT_MODE_QUERY;  // 默认查询模式，周期发读值指令
    config.auto_send_time = 1000;       // 默认 1000μs (1ms)，对应 0x05 指令单位
    config.velocity_sample_time = 1000; // 默认 1000ms (1s)，对应 0x0B 指令单位
    // baudrate 是手册 0x03 的枚举（500K/1M/250K/125K/100K），
    // 这里只记录在 config 中，由上层显式调用 CAN_Send_SetBaudrate 才实际下发。
    switch (baudrate)
    {
    case BRT_BAUDRATE_500K: config.baudrate = 500000; break;
    case BRT_BAUDRATE_1M:   config.baudrate = 1000000; break;
    case BRT_BAUDRATE_250K: config.baudrate = 250000; break;
    case BRT_BAUDRATE_125K: config.baudrate = 125000; break;
    case BRT_BAUDRATE_100K: config.baudrate = 100000; break;
    default:                config.baudrate = BRT_ENCODER_DEFAULT_BAUDRATE; break;
    }

    // 初始化接收数据
    data = {};
    data.status = BRT_STATUS_DISABLE;

    // 多圈解包 / 掉电恢复 状态复位
    total_unwrapped_pulses = 0;
    last_raw_abs = 0;
    restore_valid = false;
    position_restore_ok = false;
    degraded_mode = false;
    pending_restore_valid = false;
    pending_saved_pulses = 0;
    pending_saved_raw = 0;
    last_persist_ms = 0;

    // 逻辑零位锚点：默认未校准（角度回退到 L mod 14336，不是机械零）
    logical_zero_anchor = 0;
    logical_zero_anchor_valid = false;
}

/**
 * @brief 上电恢复函数：用磁盘里的 saved_pulses、saved_raw 与首次读到的 current_raw_abs 做 diff 缝合
 *
 * 恢复算法（与 UPPER_COMPUTER_STEER_ENCODER_MOD_PLAN.md §5 对齐）：
 *   delta = current_raw - saved_raw
 *   边界缝合后 new_total = saved_total + delta
 * 若 |delta| > 10 圈（40960 脉冲）判定异常，置 degraded_mode。
 */
void Class_Encoder_BRT::Init_Restore_Position(int64_t saved_pulses,
                                              int32_t saved_raw,
                                              int32_t current_raw_abs)
{
    int32_t delta = current_raw_abs - saved_raw;
    if (delta > ENC_HALF_PULSES_I32)
        delta -= ENC_MAX_PULSES_I32;
    else if (delta < -ENC_HALF_PULSES_I32)
        delta += ENC_MAX_PULSES_I32;

    total_unwrapped_pulses = saved_pulses + static_cast<int64_t>(delta);
    last_raw_abs = current_raw_abs;
    restore_valid = true;

    const int32_t abs_delta = (delta < 0) ? -delta : delta;
    if (abs_delta > ENC_ABNORMAL_THRESHOLD)
    {
        degraded_mode = true;
        position_restore_ok = false;
        std::cerr << "\n"
                  << "================================================================\n"
                  << "[ENC][!! POWER-OFF ABNORMAL !!] id=0x" << std::hex
                  << static_cast<int>(config.can_id) << std::dec << "\n"
                  << "  |delta|=" << abs_delta << " > " << ENC_ABNORMAL_THRESHOLD
                  << " pulses (>10 turns)\n"
                  << "  Steer encoder power-off displacement abnormal,\n"
                  << "  please execute Homing before running.\n"
                  << "================================================================\n"
                  << std::endl;
    }
    else
    {
        degraded_mode = false;
        position_restore_ok = true;
    }
}

/**
 * @brief 运行时多圈解包：每次新 raw_abs 增量缝合到 total_unwrapped_pulses
 *
 * 首帧策略：
 *   - 已通过 Set_Pending_Restore 注入磁盘 saved_pulses → 走 Init_Restore_Position
 *   - 否则视作冷启动：以 current_raw_abs 作为初始累计值（无降级告警）
 */
void Class_Encoder_BRT::Update_Unwrapped_Total(int32_t current_raw_abs)
{
    if (!restore_valid)
    {
        if (pending_restore_valid)
        {
            Init_Restore_Position(pending_saved_pulses, pending_saved_raw, current_raw_abs);
            pending_restore_valid = false;
        }
        else
        {
            // 冷启动：没有先验 saved_pulses，直接把当前 raw 作为初始绝对位置
            total_unwrapped_pulses = static_cast<int64_t>(current_raw_abs);
            last_raw_abs = current_raw_abs;
            restore_valid = true;
            position_restore_ok = false; // 未恢复，视为降级
            degraded_mode = true;
        }
        return;
    }

    int32_t delta = current_raw_abs - last_raw_abs;
    if (delta < -ENC_HALF_PULSES_I32)
        delta += ENC_MAX_PULSES_I32;
    else if (delta > ENC_HALF_PULSES_I32)
        delta -= ENC_MAX_PULSES_I32;

    total_unwrapped_pulses += static_cast<int64_t>(delta);
    last_raw_abs = current_raw_abs;
}

/**
 * @brief 把逻辑累计轴归零（或设为指定值），并清空写盘节流时戳
 *
 * 用于找零（Homing）流程：保存 raw 域 zero_offset 后调用一次，
 * 让 total_unwrapped_pulses 直接对齐到机械零位（步骤文档 §5 step 3）。
 * 同时把 last_persist_ms 清零，使上层下一次 Save_Steer_Unwrapped_Pulses
 * 不会被 200ms 节流挡住，零点能立即落盘。
 */
void Class_Encoder_BRT::Reset_Total_Unwrapped_Pulses(int64_t new_total)
{
    total_unwrapped_pulses = new_total;
    // 已经有累计基线，认为后续可继续做增量
    restore_valid = true;
    position_restore_ok = true;
    degraded_mode = false;
    pending_restore_valid = false;
    last_persist_ms = 0; // 解除写盘节流，触发立即落盘
}

/**
 * @brief CAN接收完成回调函数
 *
 * @param Rx_Data 接收到的数据
 */
void Class_Encoder_BRT::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    // 更新存活标志
    alive_flag += 1;
    data.rx_count++;

    // 数据处理
    Data_Process(Rx_Data);
}

/**
 * @brief 定时器存活检测回调函数
 *
 */
void Class_Encoder_BRT::TIM_Alive_PeriodElapsedCallback()
{
    // 检测编码器是否在线
    if (alive_flag == pre_alive_flag)
    {
        data.status = BRT_STATUS_DISABLE;
    }
    else
    {
        data.status = BRT_STATUS_ENABLE;
    }
    pre_alive_flag = alive_flag;
}

/**
 * @brief 定时查询回调函数
 *
 */
void Class_Encoder_BRT::TIM_Query_PeriodElapsedCallback()
{
    if (config.work_mode == BRT_MODE_QUERY)
    {
        // 查询模式下定期读取编码器值
        CAN_Send_ReadEncoderValue();
    }
}

/**
 * @brief 发送读取编码器值指令
 *
 */
void Class_Encoder_BRT::CAN_Send_ReadEncoderValue()
{
    uint8_t payload[1] = {0x00};
    CAN_Send_Command(BRT_CMD_READ_ENCODER_VALUE, payload, 1);
}

/**
 * @brief 发送设置编码器ID指令
 *
 * @param new_id 新的编码器ID(1-255)
 */
void Class_Encoder_BRT::CAN_Send_SetEncoderID(uint8_t new_id)
{
    uint8_t payload[1] = {new_id};
    CAN_Send_Command(BRT_CMD_SET_ENCODER_ID, payload, 1);
}

/**
 * @brief 发送设置波特率指令
 *
 * @param baudrate 波特率枚举值
 */
void Class_Encoder_BRT::CAN_Send_SetBaudrate(Enum_BRT_Encoder_Baudrate baudrate)
{
    uint8_t payload[1] = {(uint8_t)baudrate};
    CAN_Send_Command(BRT_CMD_SET_BAUDRATE, payload, 1);
}

/**
 * @brief 发送设置工作模式指令
 *
 * @param mode 工作模式
 */
void Class_Encoder_BRT::CAN_Send_SetMode(Enum_BRT_Encoder_Mode mode)
{
    uint8_t payload[1] = {(uint8_t)mode};
    CAN_Send_Command(BRT_CMD_SET_MODE, payload, 1);
    config.work_mode = mode;
}

/**
 * @brief 发送设置自动发送时间指令
 *
 * @param time_us 自动发送时间(微秒)
 */
void Class_Encoder_BRT::CAN_Send_SetAutoSendTime(uint16_t time_us)
{
    // 字节序依据：BRT CAN 通信协议手册 V2.5 §6.4.3 / §6.5.2 明确：
    //   "数据域的内容为多字节时，低字节在前"
    //   "16 位数据低字节在前；32 位数据低字节在前"
    // 手册第 13 页 0x05 示例：1000us(0x03E8) 下发为 [0x05][0x01][0x05][0xE8][0x03]
    uint8_t payload[2];
    payload[0] = time_us & 0xFF;        // 低字节在前
    payload[1] = (time_us >> 8) & 0xFF; // 高字节在后
    CAN_Send_Command(BRT_CMD_SET_AUTO_SEND_TIME, payload, 2);
    config.auto_send_time = time_us;
}

/**
 * @brief 发送设置零点指令
 *
 */
void Class_Encoder_BRT::CAN_Send_SetZero()
{
    uint8_t payload[1] = {0x00};
    CAN_Send_Command(BRT_CMD_SET_ZERO, payload, 1);
}

/**
 * @brief 发送设置方向指令
 *
 * @param direction 递增方向
 */
void Class_Encoder_BRT::CAN_Send_SetDirection(Enum_BRT_Encoder_Direction direction)
{
    uint8_t payload[1] = {(uint8_t)direction};
    CAN_Send_Command(BRT_CMD_SET_DIRECTION, payload, 1);
}

/**
 * @brief 发送读取角速度指令
 *
 */
void Class_Encoder_BRT::CAN_Send_ReadAngularVelocity()
{
    uint8_t payload[1] = {0x00};
    CAN_Send_Command(BRT_CMD_READ_ANGULAR_VELOCITY, payload, 1);
}

/**
 * @brief 发送设置速度采样时间指令
 *
 * @param time_ms 采样时间(毫秒)
 */
void Class_Encoder_BRT::CAN_Send_SetVelocitySampleTime(uint16_t time_ms)
{
    // 字节序依据见 CAN_Send_SetAutoSendTime（手册 V2.5 §6.4.3 低字节在前）。
    // 手册第 14 页 0x0B 示例：1000ms(0x03E8) 下发为 [0x05][0x01][0x0B][0xE8][0x03]
    uint8_t payload[2];
    payload[0] = time_ms & 0xFF;        // 低字节在前
    payload[1] = (time_ms >> 8) & 0xFF; // 高字节在后
    CAN_Send_Command(BRT_CMD_SET_VELOCITY_SAMP_TIME, payload, 2);
    config.velocity_sample_time = time_ms;
}

/**
 * @brief 发送设置中点指令
 *
 */
void Class_Encoder_BRT::CAN_Send_SetMidpoint()
{
    uint8_t payload[1] = {0x01};
    CAN_Send_Command(BRT_CMD_SET_MIDPOINT, payload, 1);
}

/**
 * @brief 发送设置当前值指令
 *
 * @param value 要设置的编码器值
 */
void Class_Encoder_BRT::CAN_Send_SetCurrentValue(uint32_t value)
{
    // 字节序依据见 CAN_Send_SetAutoSendTime（手册 V2.5 §6.4.3 低字节在前，
    // 32 位数据 4 字节也是低字节在前）。
    // 手册第 14 页 0x0D 示例 0x00012345(74565) 下发为 [0x07][0x01][0x0D][0x45][0x23][0x01][0x00]
    uint8_t payload[4];
    payload[0] = value & 0xFF; // 低字节在前
    payload[1] = (value >> 8) & 0xFF;
    payload[2] = (value >> 16) & 0xFF;
    payload[3] = (value >> 24) & 0xFF;
    CAN_Send_Command(BRT_CMD_SET_CURRENT_VALUE, payload, 4);
}

/**
 * @brief 发送设置5圈值指令
 *
 */
void Class_Encoder_BRT::CAN_Send_Set5TurnValue()
{
    uint8_t payload[1] = {0x01};
    CAN_Send_Command(BRT_CMD_SET_5_TURN_VALUE, payload, 1);
}

/**
 * @brief 数据处理函数
 *
 * @param rx_data_buffer 接收数据缓冲区
 */
void Class_Encoder_BRT::Data_Process(uint8_t *inputs_ptr)
{
    // 解析数据长度
    uint8_t data_len = inputs_ptr[0];
    uint8_t encoder_id = inputs_ptr[1];
    uint8_t command = inputs_ptr[2];

    // 检查ID是否匹配
    if (encoder_id != config.can_id)
        return;

    switch (command)
    {
    case BRT_CMD_READ_ENCODER_VALUE:
    {
        if (data_len == 7)
        {
            // ================= 编码器原始值解析 =================
            data.encoder_value = (uint32_t)inputs_ptr[3] |
                                 ((uint32_t)inputs_ptr[4] << 8) |
                                 ((uint32_t)inputs_ptr[5] << 16) |
                                 ((uint32_t)inputs_ptr[6] << 24);

            // 50 圈模式防御性检查：若硬件不在 50-turn 量程，解卷阈值将失真。
            if (data.encoder_value >= ENC_TRUE_MAX_PULSES)
            {
                static uint64_t s_last_warn_ms[256] = {0};
                const uint8_t encoder_slot = config.can_id;
                const uint64_t now_ms = encoder_now_ms();
                if ((now_ms - s_last_warn_ms[encoder_slot]) >= 1000)
                {
                    std::cerr << "[ENC][HW-CFG-WARN] id=0x" << std::hex
                              << static_cast<int>(config.can_id) << std::dec
                              << " raw=" << data.encoder_value
                              << " >= ENC_TRUE_MAX_PULSES=" << ENC_TRUE_MAX_PULSES
                              << " ; encoder is not in 50-turn mode, unwrap thresholds are invalid."
                              << std::endl;
                    s_last_warn_ms[encoder_slot] = now_ms;
                }
            }

            // ============ 物理对账域（50 圈 raw，标定/诊断/零点持久化） ============
            // 1) raw_true：把 raw_abs 折叠到 [0, ENC_TRUE_MAX_PULSES)
            const int32_t raw_true = positive_mod_i32(
                static_cast<int32_t>(data.encoder_value % ENC_TRUE_MAX_PULSES),
                static_cast<int32_t>(ENC_TRUE_MAX_PULSES));

            // 1.5) 多圈解包：基于 raw_true 维护 total_unwrapped_pulses（逻辑长轴 L）
            //      首帧若已 Set_Pending_Restore 会内部走 Init_Restore_Position
            Update_Unwrapped_Total(raw_true);

            // 2) raw_true_cal 仅保留为「现场对账/诊断字段」，控制不再使用
            const int32_t raw_true_cal = positive_mod_i32(
                raw_true - config.zero_offset,
                static_cast<int32_t>(ENC_TRUE_MAX_PULSES));
            data.raw_true_cal = raw_true_cal;

            // 3) 机械角度统一从【逻辑长轴】计算（虚拟无限位移方案）：
            //      phase = positive_mod(total_unwrapped_pulses - logical_zero_anchor, 14336)
            //    设计要点：
            //      • L 始终维持不变量 L mod 204800 == raw_true（不在校准时重置）
            //      • 校准瞬间把当时的 L 记到 logical_zero_anchor（持久化到 zero 文件）
            //      • 这样 Init_Restore_Position 用 raw 缝合 L 不会破坏角度参考点
            //    若锚点未加载（首次跑、或 zero 文件只有 2 列旧格式）则按 L mod 14336 输出，
            //    此时角度只保证连续，不保证机械零 → 用户应重新跑一次 CAPTURE_STEER_ZERO=1
            const int64_t steer_mod_i64 = static_cast<int64_t>(STEER_PULSES_PER_TURN);
            int64_t logical_offset = total_unwrapped_pulses - logical_zero_anchor;
            int64_t steer_phase_i64 = logical_offset % steer_mod_i64;
            if (steer_phase_i64 < 0) steer_phase_i64 += steer_mod_i64;
            const int32_t steer_raw_logical = static_cast<int32_t>(steer_phase_i64);
            data.wheel_posture_radian_true =
                static_cast<float>(steer_raw_logical) / STEER_PULSES_PER_TURN_F * 2.0f * PI;
            data.wheel_posture_angle_true =
                static_cast<float>(steer_raw_logical) / STEER_PULSES_PER_TURN_F * 360.0f;

            // 5. 保留编码器物理单圈弧度 (如果其他底层闭环逻辑还需要用到纯电机端的角度)
            data.current_radian = static_cast<float>(data.encoder_value % ENC_RAW_PER_TURN) / BRT_RAW_RES * 2.0f * PI;
            data.wheel_posture_valid = 1;
            data.current_rounds = static_cast<float>(data.encoder_value / ENC_RAW_PER_TURN);

            
            static uint64_t s_last_print_ms[256] = {0};
            const uint8_t encoder_slot = config.can_id;
            const uint64_t now_ms = encoder_now_ms();
            if (kEncoderDebugPrintEnable &&
                (now_ms - s_last_print_ms[encoder_slot]) >= kEncoderDebugPrintPeriodMs)
            {
                std::cout << "[ENC] id=0x" << std::hex << static_cast<int>(config.can_id) << std::dec
                          << std::fixed << std::setprecision(3)
                          << " raw_abs=" << data.encoder_value
                          << " raw_true_cal=" << data.raw_true_cal
                          << " wheel_deg_true=" << data.wheel_posture_angle_true
                          << " zero_offset_true=" << config.zero_offset
                          << " omega_rpm=" << data.current_omega
                          << " status=" << static_cast<int>(data.status)
                          << std::endl;
                s_last_print_ms[encoder_slot] = now_ms;
            }
        }
    }
    break;

    case BRT_CMD_READ_ANGULAR_VELOCITY:
    {
        if (data_len == 7) // 4字节数据 + 3字节头
        {
            // 解析32位角速度值(有符号整数)
            int32_t angular_velocity_raw = (int32_t)inputs_ptr[3] |
                                           ((int32_t)inputs_ptr[4] << 8) |
                                           ((int32_t)inputs_ptr[5] << 16) |
                                           ((int32_t)inputs_ptr[6] << 24);

            // 计算角速度(转/分钟)
            data.current_omega = (float)angular_velocity_raw /
                                 config.single_turn_resolution /
                                 (config.velocity_sample_time / 1000.0f / 60.0f);
        }
    }
    break;

    default:
        // ============== set 指令 ACK 解析 ==============
        // 手册：0x02~0x0F 的所有设置类指令应答帧固定为 [0x04][id][cmd][status]，
        // status==0x00 视为成功，其他值为错误码。
        if (data_len == 4 &&
            (command == BRT_CMD_SET_ENCODER_ID ||
             command == BRT_CMD_SET_BAUDRATE ||
             command == BRT_CMD_SET_MODE ||
             command == BRT_CMD_SET_AUTO_SEND_TIME ||
             command == BRT_CMD_SET_ZERO ||
             command == BRT_CMD_SET_DIRECTION ||
             command == BRT_CMD_SET_VELOCITY_SAMP_TIME ||
             command == BRT_CMD_SET_MIDPOINT ||
             command == BRT_CMD_SET_CURRENT_VALUE ||
             command == BRT_CMD_SET_5_TURN_VALUE))
        {
            const uint8_t status = inputs_ptr[3];
            data.last_ack_cmd = command;
            data.last_ack_status = status;
            data.last_ack_ms = encoder_now_ms();

            // 三个语义最重的指令单独留通道，便于上层判断是否标定成功
            if (command == BRT_CMD_SET_MODE)
                data.mode_ack_status = status;
            else if (command == BRT_CMD_SET_ZERO)
                data.zero_ack_status = status;
            else if (command == BRT_CMD_SET_MIDPOINT)
                data.midpoint_ack_status = status;

            if (status != 0x00)
            {
                // 失败时打印一次错误码（节流：每个 cmd 1 秒一次，避免刷屏）
                static uint64_t s_last_err_ms[256] = {0};
                const uint64_t now_ms = data.last_ack_ms;
                if ((now_ms - s_last_err_ms[command]) >= 1000)
                {
                    std::cerr << "[ENC][ACK-ERR] id=0x" << std::hex
                              << static_cast<int>(config.can_id)
                              << " cmd=0x" << static_cast<int>(command)
                              << " status=0x" << static_cast<int>(status)
                              << std::dec << std::endl;
                    s_last_err_ms[command] = now_ms;
                }
            }
        }
        break;
    }
}

/**
 * @brief 发送CAN指令
 *
 * @param command 指令码
 * @param data 数据缓冲区
 * @param data_len 数据长度
 *
 * ⚠️  LinkX TX 队列「同 CAN ID 后发覆盖前发」陷阱：
 *     linkx.c 的 linkx_tx_queue_push_or_update 把同 (can_id) 的后发帧
 *     直接覆盖前发帧。给同一编码器连发多个 set 指令时，如果两次 push
 *     之间没让 EtherCAT 把第一帧 pop 出去，第二帧会把第一帧覆盖掉。
 *
 *     现象：表面看 set 指令发了，但实际只有最后一帧上 wire，前面的全丢。
 *     生产代码不发 set，所以无影响。但如果未来要在线配置编码器：
 *       1) 用 quiet TX 路径（不调 TIM_Query 期间发 set）
 *       2) 每次 CAN_Send_Command 后跑 ec_step 让帧 pop 出去再发下一帧
 *       3) 详细方案参见 src/test_mains/encoder_ack_test_main.cpp PHASE C
 */
void Class_Encoder_BRT::CAN_Send_Command(uint8_t command, uint8_t *payload, uint8_t data_len)
{
    uint8_t payload_len = data_len;
    if (payload_len > 5)
        payload_len = 5;

    // 构造发送数据包
    tx_data[0] = payload_len + 3; // 总长度: 数据长度 + 3字节头
    tx_data[1] = config.can_id;
    tx_data[2] = command;

    // 拷贝数据
    for (uint8_t i = 0; i < payload_len; i++)
    {
        tx_data[3 + i] = payload[i];
    }

    // 填充剩余字节为0
    for (uint8_t i = payload_len + 3; i < 8; i++)
    {
        tx_data[i] = 0x00;
    }

    // 发送CAN数据：与原 STM32 版本保持一致，使用编码器节点 ID（如 0x05~0x08）
    linkx_quick_can_send(LinkX_Ptr, static_cast<uint8_t>(Slave_Index), config.can_id, tx_data);

}

/************************ End of file ************************/
