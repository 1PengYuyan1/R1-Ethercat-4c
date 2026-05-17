/**
 * @file dvc_encoder_brt.h
 * @author Your_Name (your@email.com)
 * @brief 布瑞特编码器CAN通信驱动
 * @version 0.1
 * @date 2024-01-01
 *
 * @copyright Your_Company (c) 2024
 */

#ifndef DVC_ENCODER_H
#define DVC_ENCODER_H

#include <cstdint>
#include <cmath>
#include "linkx4c_handler.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// 布瑞特编码器默认参数
#define BRT_ENCODER_DEFAULT_BAUDRATE 1000000 // 1000kbps
#define BRT_ENCODER_SINGLE_TURN_RES 1024     // 默认单圈分辨率
#define BRT_RAW_RES 4096.0f

// ============== 编码器量程语义常量 ==============
// 物理域：编码器单周期物理量程（50圈），仅用于解卷阈值与诊断/对账
#define ENC_TRUE_TURNS 50u
#define ENC_RAW_PER_TURN 4096u
#define ENC_TRUE_MAX_PULSES (ENC_TRUE_TURNS * ENC_RAW_PER_TURN) // 204800

// 舵向减速比：电机端 3.5 圈 = 输出轴 1 圈
// STEER_PULSES_PER_TURN = 14336：舵向输出轴一圈对应的编码器脉冲数（3.5 * 4096）
// 这是"逻辑相位域"的周期，所有舵角语义统一用 phase = (L - anchor) mod 14336
#define STEER_GEAR_RATIO_F 3.5f
#define STEER_PULSES_PER_TURN 14336u
#define STEER_PULSES_PER_TURN_F static_cast<float>(STEER_PULSES_PER_TURN)

// 多圈解包 + 掉电恢复相关常量
// 原始单周期量程（与 ENC_TRUE_MAX_PULSES 同义，但使用 int32 语义）
#define ENC_MAX_PULSES_I32 static_cast<int32_t>(ENC_TRUE_MAX_PULSES)
#define ENC_HALF_PULSES_I32 (ENC_MAX_PULSES_I32 / 2) // 102400
// 工程异常阈值：上电 diff 若超过 10 圈（40960 脉冲）则判定异常，强制 Homing
#define ENC_ABNORMAL_THRESHOLD (10 * static_cast<int32_t>(ENC_RAW_PER_TURN))
#define ENC_DEGRADED_DIFF_PULSES ENC_ABNORMAL_THRESHOLD
// 持久化魔数 'ENCP'
#define ENC_PERSIST_MAGIC 0x454E4350u

// CAN指令码定义
#define BRT_CMD_READ_ENCODER_VALUE 0x01
#define BRT_CMD_SET_ENCODER_ID 0x02
#define BRT_CMD_SET_BAUDRATE 0x03
#define BRT_CMD_SET_MODE 0x04
#define BRT_CMD_SET_AUTO_SEND_TIME 0x05
#define BRT_CMD_SET_ZERO 0x06
#define BRT_CMD_SET_DIRECTION 0x07
#define BRT_CMD_READ_ANGULAR_VELOCITY 0x0A
#define BRT_CMD_SET_VELOCITY_SAMP_TIME 0x0B
#define BRT_CMD_SET_MIDPOINT 0x0C
#define BRT_CMD_SET_CURRENT_VALUE 0x0D
#define BRT_CMD_SET_5_TURN_VALUE 0x0F

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 布瑞特编码器工作模式
 */
typedef enum
{
    BRT_MODE_QUERY = 0x00,               // 查询模式
    BRT_MODE_AUTO_RETURN_ANGLE = 0xAA,   // 自动返回角度值
    BRT_MODE_AUTO_RETURN_VELOCITY = 0x02 // 自动返回角速度值
} Enum_BRT_Encoder_Mode;

/**
 * @brief 布瑞特编码器波特率
 */
typedef enum
{
    BRT_BAUDRATE_500K = 0x00, // 默认500kbps
    BRT_BAUDRATE_1M = 0x01,   // 1Mbps
    BRT_BAUDRATE_250K = 0x02, // 250kbps
    BRT_BAUDRATE_125K = 0x03, // 125kbps
    BRT_BAUDRATE_100K = 0x04  // 100kbps
} Enum_BRT_Encoder_Baudrate;

/**
 * @brief 布瑞特编码器递增方向
 */
typedef enum
{
    BRT_DIRECTION_CW = 0x00, // 顺时针递增
    BRT_DIRECTION_CCW = 0x01 // 逆时针递增
} Enum_BRT_Encoder_Direction;

/**
 * @brief 布瑞特编码器状态
 */
typedef enum
{
    BRT_STATUS_DISABLE = 0,
    BRT_STATUS_ENABLE,
    BRT_STATUS_ERROR
} Enum_BRT_Encoder_Status;

/**
 * @brief 布瑞特编码器接收数据结构
 */
typedef struct
{
    uint32_t encoder_value; // 编码器原始值（raw_abs，未做任何归一化）

    float current_radian; // 当前编码器弧度（弧度）

    // -------- 物理对账域（50 圈 raw，标定诊断/外部对账） --------
    int32_t raw_true_cal;            // 零点补偿后 raw [0, ENC_TRUE_MAX_PULSES)
    float wheel_posture_radian_true; // 舵向角度（基于逻辑相位计算）[0, 2pi)
    float wheel_posture_angle_true;  // 舵向角度（基于逻辑相位计算）[0, 360)

    float Last_radian;               // 上一次单圈编码器的的弧度值
    float wheel_posture_radian_last; // 上一次轮组弧度（rad）

    float current_omega;              // 编码器角速度(转/分钟)
    float wheel_current_omega_radian; // 映射到轮组的角速度（弧度/分钟）

    float target_omega;  // 编码器目标角速度(转/分钟)
    float target_radian; // 编码器目标角度

    float current_rounds; // 编码器当前圈数

    float position_delta; // 编码器位置差

    float pre_encoder;        // 编码器前一次编码器值
    float pre_circle_encoder; // 编码器上一次单圈编码器的值

    uint8_t speed_init_flag;        //
    Enum_BRT_Encoder_Status status; // 编码器状态

    uint32_t rx_count;           // 累计接收帧数（单调递增）
    uint8_t wheel_posture_valid; // 是否已收到至少一次合法角度数据

    // ============== set 指令 ACK 状态 ==============
    // 对照手册：0x02~0x0F 的设置类指令均回 [0x04][id][cmd][status]，
    // status==0x00 视为成功，其他值为错误码。
    uint8_t last_ack_cmd;     // 最近一次收到 ACK 的指令码
    uint8_t last_ack_status;  // 0x00=成功，其他=错误码
    uint64_t last_ack_ms;     // 收到 ACK 的时间戳（ms，steady_clock）
    uint8_t mode_ack_status;     // 0x04 Set Mode 最近一次 ACK
    uint8_t zero_ack_status;     // 0x06 Set Zero 最近一次 ACK
    uint8_t midpoint_ack_status; // 0x0C Set Midpoint 最近一次 ACK
} Struct_BRT_Encoder_Data;

/**
 * @brief 布瑞特编码器配置参数
 */
typedef struct
{
    uint8_t can_id;                  // CAN节点ID
    int32_t zero_offset;             // 零点偏移（脉冲）
    uint32_t baudrate;               // 通信波特率
    uint16_t single_turn_resolution; // 单圈分辨率
    uint32_t max_turns;              // 最大圈数
    Enum_BRT_Encoder_Mode work_mode; // 工作模式
    uint16_t auto_send_time;         // 自动发送时间(微秒)
    uint16_t velocity_sample_time;   // 速度采样时间(毫秒)
} Struct_BRT_Encoder_Config;

/**
 * @brief 布瑞特编码器类
 */
class Class_Encoder_BRT
{
public:
    // baudrate 参数对应手册 0x03 指令的取值；默认 1Mbps，仅写入 config，不主动下发。
    void Init(linkx_t *linkx_ptr, uint16_t slave_index, uint8_t encoder_id, uint16_t single_turn_res,
              Enum_BRT_Encoder_Baudrate baudrate = BRT_BAUDRATE_1M); // 基本功能函数
    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    // 指令发送函数
    void CAN_Send_ReadEncoderValue();
    void CAN_Send_SetEncoderID(uint8_t new_id);
    void CAN_Send_SetBaudrate(Enum_BRT_Encoder_Baudrate baudrate);
    void CAN_Send_SetMode(Enum_BRT_Encoder_Mode mode);
    void CAN_Send_SetAutoSendTime(uint16_t time_us);
    void CAN_Send_SetZero();
    void CAN_Send_SetDirection(Enum_BRT_Encoder_Direction direction);
    void CAN_Send_ReadAngularVelocity();
    void CAN_Send_SetVelocitySampleTime(uint16_t time_ms);
    void CAN_Send_SetMidpoint();
    void CAN_Send_SetCurrentValue(uint32_t value);
    void CAN_Send_Set5TurnValue();

    // 数据获取函数
    inline uint32_t Get_EncoderValue();
    inline float Get_CurrentAngle();

    // 物理对账域（50 圈 raw，标定/诊断/零点持久化）
    inline float Get_Wheel_Posture_radian_True() const;
    inline float Get_Wheel_Angle_True() const;
    inline int32_t Get_Raw_True_Cal() const;

    inline float Get_AngularVelocity();
    inline uint32_t Get_TotalRounds();
    inline Enum_BRT_Encoder_Status Get_Status();

    inline uint8_t Get_Can_ID();
    inline bool Has_Valid_Wheel_Posture();
    inline uint32_t Get_Rx_Count();

    // set 指令 ACK 查询
    inline uint8_t Get_Last_Ack_Cmd() const { return data.last_ack_cmd; }
    inline uint8_t Get_Last_Ack_Status() const { return data.last_ack_status; }
    inline bool Is_Mode_Ack_Ok() const { return data.mode_ack_status == 0x00; }
    inline bool Is_Zero_Ack_Ok() const { return data.zero_ack_status == 0x00; }
    inline bool Is_Midpoint_Ack_Ok() const { return data.midpoint_ack_status == 0x00; }

    // 参数设置函数
    inline void Set_SingleTurnResolution(uint16_t resolution);
    inline void Set_MaxTurns(uint32_t turns);
    inline void Set_Target_Angle(float Target_Angle);
    inline void Set_Omega(float Target_Omega);
    inline void Set_Zero_Offset(int32_t offset);
    inline int32_t Get_Zero_Offset() const;

    // ============== 多圈解包 + 掉电恢复 ==============
    // 上电恢复：用持久化的 saved_pulses、saved_raw 与首次读到的 current_raw_abs 做 diff 缝合。
    // saved_raw 为保存时刻的原始编码值，恢复算法直接计算 delta = current_raw - saved_raw。
    // 仅会被首次有效帧触发执行一次（之后由 Update_Unwrapped_Total 增量维护）。
    void Init_Restore_Position(int64_t saved_pulses, int32_t saved_raw, int32_t current_raw_abs);

    // 运行时增量解包：每次拿到新 raw_abs 调用一次。
    // 内部根据 last_raw_abs 计算 delta 并累加到 total_unwrapped_pulses。
    void Update_Unwrapped_Total(int32_t current_raw_abs);

    // 在收到首帧 raw 之前，把磁盘里读到的累计值和 raw 喂进来；
    // 首帧到达时 Update_Unwrapped_Total 会自动改走 Init_Restore_Position 路径。
    inline void Set_Pending_Restore(int64_t saved_pulses, int32_t saved_raw);
    inline bool Has_Pending_Restore() const { return pending_restore_valid; }

    // 找零（Homing）瞬间显式把逻辑累计轴归零（或设为给定值）。
    // 注意：当前主流程不再调用此接口（见 Set_Logical_Zero_Anchor）。
    // 这里保留是为了将来可能的「强制 L 归零 + 同步落盘」调试场景。
    // 重置后下一次写盘会立即触发（last_persist_ms 同步清零，绕过节流）。
    void Reset_Total_Unwrapped_Pulses(int64_t new_total = 0);

    // ============== 逻辑零位锚点（角度参考点） ==============
    // 设计理念：L 始终保持 L mod 204800 == raw_true 的不变量；
    // 校准时不去动 L，而是把当时 L 的值记到 logical_zero_anchor，
    // 角度计算用 phase = (L - logical_zero_anchor) mod 14336。
    // 这样 Init_Restore_Position / Update_Unwrapped_Total 的恢复算法
    // 和角度参考点完全解耦，不会再互相破坏。
    inline int64_t Get_Logical_Zero_Anchor() const { return logical_zero_anchor; }
    inline bool Is_Logical_Zero_Anchor_Valid() const { return logical_zero_anchor_valid; }
    // 直接设置锚点值（从磁盘加载时使用）
    inline void Set_Logical_Zero_Anchor(int64_t v)
    {
        logical_zero_anchor = v;
        logical_zero_anchor_valid = true;
    }
    // 把当前 L 记为锚点（在校准抓取瞬间调用）
    inline void Capture_Logical_Zero_Anchor()
    {
        logical_zero_anchor = total_unwrapped_pulses;
        logical_zero_anchor_valid = true;
    }
    inline void Clear_Logical_Zero_Anchor()
    {
        logical_zero_anchor = 0;
        logical_zero_anchor_valid = false;
    }

    // 查询当前累计值与状态（控制层应优先使用累计值）
    inline int64_t Get_Total_Unwrapped_Pulses() const { return total_unwrapped_pulses; }
    inline int32_t Get_Last_Raw_Abs() const { return last_raw_abs; }
    inline bool Is_Restore_Valid() const { return restore_valid; }
    inline bool Is_Position_Restore_Ok() const { return position_restore_ok; }
    inline bool Is_Degraded_Mode() const { return degraded_mode; }
    inline uint64_t Get_Last_Persist_Ms() const { return last_persist_ms; }
    inline void Set_Last_Persist_Ms(uint64_t now_ms) { last_persist_ms = now_ms; }
    // 连续舵向角（不取模）：直接由 total_unwrapped_pulses 换算得到
    inline float Get_Steer_Total_Radian() const;
    inline float Get_Steer_Total_Degree() const;
    // 相对逻辑零位锚点的连续舵向角（不取模）
    inline float Get_Steer_Relative_Radian() const;
    inline float Get_Steer_Relative_Degree() const;

    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Query_PeriodElapsedCallback();

protected:
    // 初始化相关变量
    linkx_t *LinkX_Ptr;
    uint16_t Slave_Index;
    Struct_BRT_Encoder_Config config;

    // 接收数据
    Struct_BRT_Encoder_Data data;

    // 内部变量
    uint32_t alive_flag = 0;
    uint32_t pre_alive_flag = 0;
    uint8_t tx_data[8];

    // ============== 多圈解包 + 掉电恢复 状态 ==============
    int64_t total_unwrapped_pulses = 0; // 软件解包后的全生命周期累计脉冲
    int32_t last_raw_abs = 0;           // 上一帧原始绝对值（0~ENC_MAX_PULSES-1）
    bool restore_valid = false;         // 已完成首次初始化/恢复
    bool position_restore_ok = false;   // 恢复成功且未降级（对外可读）
    bool degraded_mode = false;         // 上电 diff 超阈值，进入降级告警
    bool pending_restore_valid = false; // 是否已注入磁盘读出的 saved_pulses
    int64_t pending_saved_pulses = 0;   // 等待首帧到达时使用的待恢复累计值
    int32_t pending_saved_raw = 0;      // 等待首帧到达时使用的保存时刻 raw 值
    uint64_t last_persist_ms = 0;       // 上次成功持久化时刻（限频）

    // 逻辑零位锚点：L_at_calibration（角度计算的参考原点）
    int64_t logical_zero_anchor = 0;
    bool logical_zero_anchor_valid = false;

    // 内部函数
    void Data_Process(uint8_t *inputs_ptr);
    void CAN_Send_Command(uint8_t command, uint8_t *payload, uint8_t data_len);
};

inline uint32_t Class_Encoder_BRT::Get_EncoderValue()
{
    return data.encoder_value;
}

inline float Class_Encoder_BRT::Get_CurrentAngle()
{
    return data.wheel_posture_angle_true;
}

inline float Class_Encoder_BRT::Get_Wheel_Posture_radian_True() const
{
    return data.wheel_posture_radian_true;
}
inline float Class_Encoder_BRT::Get_Wheel_Angle_True() const
{
    return data.wheel_posture_angle_true;
}
inline int32_t Class_Encoder_BRT::Get_Raw_True_Cal() const
{
    return data.raw_true_cal;
}
inline float Class_Encoder_BRT::Get_AngularVelocity()
{
    return data.current_omega;
}

inline uint32_t Class_Encoder_BRT::Get_TotalRounds()
{
    return data.current_rounds;
}

inline Enum_BRT_Encoder_Status Class_Encoder_BRT::Get_Status()
{
    return data.status;
}

inline uint8_t Class_Encoder_BRT::Get_Can_ID()
{
    return config.can_id;
}

inline void Class_Encoder_BRT::Set_Target_Angle(float Target_Angle)
{
    data.target_radian = Target_Angle;
}

inline void Class_Encoder_BRT::Set_Omega(float Target_Omega)
{
    data.target_omega = Target_Omega;
}

// 参数设置函数
inline void Class_Encoder_BRT::Set_SingleTurnResolution(uint16_t resolution)
{
    config.single_turn_resolution = resolution;
}

inline void Class_Encoder_BRT::Set_MaxTurns(uint32_t turns)
{
    config.max_turns = turns;
}

inline bool Class_Encoder_BRT::Has_Valid_Wheel_Posture()
{
    return data.wheel_posture_valid != 0;
}

inline uint32_t Class_Encoder_BRT::Get_Rx_Count()
{
    return data.rx_count;
}

inline void Class_Encoder_BRT::Set_Zero_Offset(int32_t offset)
{
    config.zero_offset = offset;
}

inline int32_t Class_Encoder_BRT::Get_Zero_Offset() const
{
    return config.zero_offset;
}

inline void Class_Encoder_BRT::Set_Pending_Restore(int64_t saved_pulses, int32_t saved_raw)
{
    pending_saved_pulses = saved_pulses;
    pending_saved_raw = saved_raw;
    pending_restore_valid = true;
    restore_valid = false;
    position_restore_ok = false;
    degraded_mode = false;
}

inline float Class_Encoder_BRT::Get_Steer_Total_Radian() const
{
    return static_cast<float>(total_unwrapped_pulses) / STEER_PULSES_PER_TURN_F * 2.0f * PI;
}

inline float Class_Encoder_BRT::Get_Steer_Total_Degree() const
{
    return static_cast<float>(total_unwrapped_pulses) / STEER_PULSES_PER_TURN_F * 360.0f;
}

inline float Class_Encoder_BRT::Get_Steer_Relative_Radian() const
{
    const int64_t relative_pulses = total_unwrapped_pulses - logical_zero_anchor;
    return static_cast<float>(relative_pulses) / STEER_PULSES_PER_TURN_F * 2.0f * PI;
}

inline float Class_Encoder_BRT::Get_Steer_Relative_Degree() const
{
    const int64_t relative_pulses = total_unwrapped_pulses - logical_zero_anchor;
    return static_cast<float>(relative_pulses) / STEER_PULSES_PER_TURN_F * 360.0f;
}

// 持久化数据结构（含魔数与 CRC，与 ENCODER_UNWRAP_POWERLOSS_RECOVERY_STEPS.md 对齐）
struct EncoderPersistData
{
    uint32_t magic;       // 固定魔数 ENC_PERSIST_MAGIC ('ENCP')
    uint32_t can_id;      // 编码器 CAN 节点 ID（防止文件错配）
    int64_t total_pulses; // total_unwrapped_pulses
    int32_t raw_abs;      // 保存时刻的原始编码值（0~204799），恢复时直接用于 diff
    uint32_t seq;         // 递增序号（可选）
    uint32_t crc32;       // 前面字段 CRC
};

#endif // DVC_ENCODER_H
