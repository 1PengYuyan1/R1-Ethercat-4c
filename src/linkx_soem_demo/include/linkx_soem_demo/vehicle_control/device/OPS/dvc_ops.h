/**
 * @file  dvc_ops.h
 * @brief 全方位平面定位系统 OPS-9 驱动（沈阳艾克申，UART V3.4 协议）
 *
 *  数据通路：
 *      OPS  ──RS232 115200 8N1──►  泥人 RS232↔CAN 转换器 (MODE0 透传)
 *           ──CAN 经典帧──►        EtherCAT-4C 通道 3
 *           ──linkx_quick_recv──►  Class_OPS::CAN_RxCpltCallback
 *
 *  OPS UART 帧固定长度 28 字节（200fps，即 5ms 一帧）：
 *      [0]  0x0D
 *      [1]  0x0A                ─── 帧头 (header)
 *      [2..5]    float yaw_deg      (航向角, ±180°)
 *      [6..9]    float pitch_deg    (俯仰角)
 *      [10..13]  float roll_deg     (横滚角)
 *      [14..17]  float pos_x_mm     (X 坐标, mm; 分辨率 0.03896 mm)
 *      [18..21]  float pos_y_mm     (Y 坐标, mm)
 *      [22..25]  float yaw_rate_dps (航向角速度, ±500 dps)
 *      [26] 0x0A
 *      [27] 0x0D                ─── 帧尾 (tail)
 *      浮点字节序：低字节在前 (IEEE 754 little-endian)
 *
 *  CAN 透传机制（手册 §6.1.1）：
 *      串口 28 字节 → 转换器拆为 4 帧 CAN（DLC = 8 / 8 / 8 / 4）
 *      所有片段使用同一个用户配置的 CAN ID（出厂默认可改）
 *      故 RX 必须基于头/尾标志重组并自同步
 *
 *  OPS 命令（手册 §3.5），由本类反向打包成 CAN 帧 → 转换器 → RS232 → OPS：
 *      "ACTR"           标定（约 15 分钟，全程保持静止）
 *      "ACT0"           清零（角度/坐标全部归 0，单次使用中可多次发）
 *      "ACTJ" + float   更新航向角 (deg)
 *      "ACTX" + float   更新 X 坐标 (mm)
 *      "ACTY" + float   更新 Y 坐标 (mm)
 *      连续调用 ACTJ/ACTX/ACTY 之间需间隔 ≥10ms（手册 §3.5.5）
 */

#ifndef DVC_OPS_H
#define DVC_OPS_H

#include <cstdint>
#include <cstddef>
#include "linkx.h"
#include "linkx4c_handler.h"

/* ============================================================
 *  协议常量（OPS V3.4 / 转换器 MODE0）
 * ========================================================== */

/// OPS UART 帧长度（含头/尾共 28 字节）
#define OPS_FRAME_TOTAL_LEN     28
/// OPS 数据载荷长度（24 字节 = 6 × float32）
#define OPS_FRAME_PAYLOAD_LEN   24
/// 数据载荷在帧内的起始偏移
#define OPS_FRAME_PAYLOAD_OFFSET 2

/// 帧头第 1/2 字节：0x0D 0x0A
#define OPS_FRAME_HEADER_BYTE0  0x0D
#define OPS_FRAME_HEADER_BYTE1  0x0A
/// 帧尾第 1/2 字节：0x0A 0x0D
#define OPS_FRAME_TAIL_BYTE0    0x0A
#define OPS_FRAME_TAIL_BYTE1    0x0D

/// 接收存活检测窗口阈值；TIM_Alive 周期回调调用时本次未累加 → DISABLE
/// 单帧 5ms 节拍，给 50ms 容差防抖
#define OPS_ALIVE_WINDOW_MS     50

/// 重组缓冲容量：>2 个完整 OPS 帧足以容忍最坏的拆帧时序
#define OPS_RX_BUFFER_CAPACITY  64

/* ============================================================
 *  OPS 命令 ASCII 前缀（4 字节）
 *      命令帧总长：4 字节（无参数）或 8 字节（含 1 个 float）
 *      转换器 MODE0：≤8 字节将打包为单 CAN 帧透传
 * ========================================================== */
#define OPS_CMD_CALIBRATE_PREFIX "ACTR"
#define OPS_CMD_ZERO_PREFIX      "ACT0"
#define OPS_CMD_UPDATE_YAW_PREFIX "ACTJ"
#define OPS_CMD_UPDATE_X_PREFIX   "ACTX"
#define OPS_CMD_UPDATE_Y_PREFIX   "ACTY"

/**
 * @brief OPS 存活状态
 */
enum Enum_OPS_Status
{
    OPS_Status_DISABLE = 0,
    OPS_Status_ENABLE,
};

/**
 * @brief 解析后的 OPS 数据
 *        所有字段单位与手册一致：角度 (deg)、坐标 (mm)、角速度 (dps)
 */
struct Struct_OPS_Data
{
    float yaw_deg;        ///< 航向角 (heading), ±180°
    float pitch_deg;      ///< 俯仰角 (pitch)
    float roll_deg;       ///< 横滚角 (roll)
    float pos_x_mm;       ///< X 坐标 (mm)
    float pos_y_mm;       ///< Y 坐标 (mm)
    float yaw_rate_dps;   ///< 航向角速度 (dps), ±500
};

/**
 * @brief OPS-9 驱动类（CAN 透传模式）
 *
 *  生命周期：
 *    Init()                      仅绑定底层资源，不下发命令
 *    CAN_RxCpltCallback()        每个 CAN 帧片段调用一次（由 task 层分发）
 *    TIM_Alive_PeriodElapsedCallback()  存活检测（建议 50~100ms 调用一次）
 *    Send_Calibrate / Zero / Update_*   主动下发命令到 OPS
 *
 *  线程安全：本类未做加锁，假设 RX/TX/状态访问都在同一控制线程内调用。
 */
class Class_OPS
{
public:
    /**
     * @brief 绑定底层 CAN 通道与目标 ID
     * @param linkx_ptr   LinkX 句柄
     * @param can_channel EtherCAT-4C 上承载 OPS 的 CAN 通道（本工程为 3）
     * @param can_id      转换器配置的 CAN ID（双向同 ID 时直接用此值发命令）
     */
    void Init(linkx_t *linkx_ptr, uint8_t can_channel, uint32_t can_id);

    /**
     * @brief 接收一帧 CAN 数据片段
     * @param rx_data 数据区指针（最多 8 字节）
     * @param dlen    本帧实际数据长度（CAN DLC，1~8；最后一个片段为 4）
     *
     *  内部维护一个滑动重组缓冲，按 [0D 0A ... 0A 0D] 自同步；
     *  解析成功一帧 28 字节后写入 data_，并把 rx_frame_count_ 自增。
     */
    void CAN_RxCpltCallback(const uint8_t *rx_data, uint8_t dlen);

    /**
     * @brief 存活检测周期回调（建议 50~100ms）
     *        若两次回调间未累计接收任何片段 → status_ = DISABLE
     */
    void TIM_Alive_PeriodElapsedCallback();

    /* ------- 数据读取 ------- */
    inline Enum_OPS_Status Get_Status() const     { return status_; }
    inline float Get_Yaw_Deg() const              { return data_.yaw_deg; }
    inline float Get_Pitch_Deg() const            { return data_.pitch_deg; }
    inline float Get_Roll_Deg() const             { return data_.roll_deg; }
    inline float Get_Pos_X_Mm() const             { return data_.pos_x_mm; }
    inline float Get_Pos_Y_Mm() const             { return data_.pos_y_mm; }
    inline float Get_Yaw_Rate_Dps() const         { return data_.yaw_rate_dps; }
    inline Struct_OPS_Data Get_Data() const       { return data_; }

    /* ------- 诊断计数 ------- */
    inline uint32_t Get_Rx_Frame_Count() const    { return rx_frame_count_; }
    inline uint32_t Get_Resync_Count() const      { return resync_count_; }
    inline uint32_t Get_Tail_Mismatch_Count() const { return tail_mismatch_count_; }
    inline uint32_t Get_Outlier_Reject_Count() const { return outlier_reject_count_; }

    /* ------- 主动命令 ------- */
    /** @brief 发送 "ACTR"：开始 ~15 分钟的零漂校准；过程中保持模块静止 */
    void Send_Calibrate();
    /** @brief 发送 "ACT0"：把当前角度/坐标置零（不影响 yaw_rate） */
    void Send_Zero();
    /** @brief 发送 "ACTJ" + float yaw_deg：标定当前航向 */
    void Send_Update_Yaw(float yaw_deg);
    /** @brief 发送 "ACTX" + float x_mm：标定当前 X 坐标 */
    void Send_Update_X(float x_mm);
    /** @brief 发送 "ACTY" + float y_mm：标定当前 Y 坐标 */
    void Send_Update_Y(float y_mm);

protected:
    /* ------- 底层资源 ------- */
    linkx_t *linkx_handler_ = nullptr;
    uint8_t  can_channel_   = 0;
    uint32_t can_id_        = 0;

    /* ------- 滑窗重组缓冲 ------- */
    uint8_t  rx_buffer_[OPS_RX_BUFFER_CAPACITY];
    size_t   rx_buffer_len_ = 0;

    /* ------- 解析后的状态 ------- */
    Struct_OPS_Data data_   = {};
    Enum_OPS_Status status_ = OPS_Status_DISABLE;

    /* ------- 存活检测 ------- */
    uint32_t alive_flag_     = 0;
    uint32_t pre_alive_flag_ = 0;

    /* ------- 诊断计数 ------- */
    uint32_t rx_frame_count_      = 0;  ///< 成功解析出的完整 OPS 帧数
    uint32_t resync_count_        = 0;  ///< 因头部不匹配而丢弃 1 字节的次数
    uint32_t tail_mismatch_count_ = 0;  ///< 头匹配但尾不匹配的次数
    uint32_t outlier_reject_count_ = 0; ///< 帧间跳变超阈值被丢弃的次数

    /* ------- Outlier 过滤 -------
     *  OPS 在 ACT0 后偶发整段 yaw/xy 大幅突变 (实测 yaw 1s 跳 13°+),
     *  非物理可达; 与上一帧比超阈值则丢弃 candidate, 保留 data_ 不变。
     *  ACT0 时设 zero_jump_exempt_next_ 给清零本身的合法大跳变让路。
     *  连续拒绝超 max_consecutive_rejects 后强制接受, 防止真实大动作被永久卡死。
     */
    static constexpr float    OUTLIER_MAX_DXY_MM       = 50.0f;  // 5ms 帧间最大 xy 跳变 ≈ 10 m/s 等效
    static constexpr float    OUTLIER_MAX_DYAW_DEG     = 10.0f;  // 5ms 帧间最大 yaw 跳变 ≈ 2000 °/s
    static constexpr uint32_t OUTLIER_MAX_CONSECUTIVE  = 10;     // 连续拒 10 帧 (50ms) 后强制接受
    bool     zero_jump_exempt_next_ = false;
    uint32_t consecutive_rejects_   = 0;

    /* ------- 内部辅助 ------- */
    /** @brief 在 rx_buffer_ 上做头部对齐 + 整帧解析（消耗式，可一次解多帧） */
    void Try_Parse_Frame_();
    /** @brief 把 rx_buffer_ 前 n 字节丢弃，剩余前移 */
    void Drop_Front_(size_t n);
    /** @brief 发送只含 4 字节 ASCII 命令的 CAN 帧（DLC=8，后 4 字节填 0） */
    void Send_Cmd_4_(const char prefix[4]);
    /** @brief 发送 4 字节 ASCII + 4 字节 float 命令的 CAN 帧（DLC=8） */
    void Send_Cmd_4_Float_(const char prefix[4], float v);
};

#endif // DVC_OPS_H
