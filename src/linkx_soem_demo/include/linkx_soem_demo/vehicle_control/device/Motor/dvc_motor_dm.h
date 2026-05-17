//
// Created by pzx on 2025/12/15.
//

#ifndef DVC_MOTOR_DM_H
#define DVC_MOTOR_DM_H
#include <cstdint>
#include <cmath>
#include "linkx.h"
#include "linkx4c_handler.h"

/**
 * @brief 达妙电机存活状态（由 TIM_Alive_PeriodElapsedCallback 维护）
 */
enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
};

/**
 * @brief 达妙电机控制状态字（来自电机反馈帧低 4 bit）
 */
enum Enum_Motor_DM_Control_Status_Normal
{
    Motor_DM_Control_Status_DISABLE = 0x00,
    Motor_DM_Control_Status_ENABLE,
    Motor_DM_Control_Status_UNDERVOLTAGE,
    Motor_DM_Control_Status_OVERCURRENT,
    Motor_DM_Control_Status_MOS_OVERTEMPERATURE,
    Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE,
    Motor_DM_Control_Status_LOSE_CONNECTION,
    Motor_DM_Control_Status_MOS_OVERLOAD,
    Motor_DM_Control_Status_OVERVOLTAGE = 0x08,
};

/**
 * @brief 达妙电机控制方式 / 报文偏移
 *        NORMAL_* 走传统 (Master_ID + 0x000/0x100/0x200/0x300) 偏移
 */
enum Enum_Motor_DM_Control_Method
{
    Motor_DM_Control_Method_NORMAL_MIT = 0,
    Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
    Motor_DM_Control_Method_NORMAL_OMEGA,
    Motor_DM_Control_Method_NORMAL_EMIT,
    Motor_DM_Control_Method_1_TO_4_CURRENT,
    Motor_DM_Control_Method_1_TO_4_OMEGA,
    Motor_DM_Control_Method_1_TO_4_ANGLE,
};

/**
 * @brief 标定模式（动摩擦 / 静摩擦 / 转动惯量），全部走 MIT 模式
 *        要求：车轮悬空 + 电机已使能
 */
enum Enum_Motor_DM_Calib_Mode
{
    Motor_DM_Calib_Mode_NONE = 0,
    Motor_DM_Calib_Mode_FRICTION,
    Motor_DM_Calib_Mode_STICTION,
    Motor_DM_Calib_Mode_INERTIA,
};

/**
 * @brief 标定结果（外部可读，运行中 phase / total_elapsed_s 实时刷新）
 */
struct Struct_Motor_DM_Calib_Result
{
    Enum_Motor_DM_Calib_Mode mode;
    bool finished;
    bool success;
    int phase;
    float total_elapsed_s;

    // FRICTION
    float friction_torque_pos_nm;
    float friction_torque_neg_nm;
    float friction_torque_avg_nm;
    float friction_omega_pos_actual;
    float friction_omega_neg_actual;

    // STICTION
    float stiction_torque_nm;
    float stiction_breakaway_omega;

    // INERTIA
    float inertia_kgm2;
    float inertia_alpha_meas;
    float inertia_torque_step_nm;
    float inertia_friction_used_nm;
};

/**
 * @brief 达妙电机反馈 CAN 帧 (8 字节, 大端编码器, 12bit ω/τ)
 */
struct Struct_Motor_DM_CAN_Rx_Data_Normal
{
    uint8_t CAN_ID : 4;
    uint8_t Control_Status_Enum : 4;
    uint16_t Angle_Reverse;
    uint8_t Omega_11_4;
    uint8_t Omega_3_0_Torque_11_8;
    uint8_t Torque_7_0;
    uint8_t MOS_Temperature;
    uint8_t Rotor_Temperature;
} __attribute__((packed));

/**
 * @brief MIT 模式发送 CAN 帧
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_MIT
{
    uint16_t Control_Angle_Reverse;
    uint8_t Control_Omega_11_4;
    uint8_t Control_Omega_3_0_K_P_11_8;
    uint8_t K_P_7_0;
    uint8_t K_D_11_4;
    uint8_t K_D_3_0_Control_Torque_11_8;
    uint8_t Control_Torque_7_0;
} __attribute__((packed));

/**
 * @brief 位置-速度模式发送 CAN 帧
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega
{
    float Control_Angle;
    float Control_Omega;
} __attribute__((packed));

/**
 * @brief 速度模式发送 CAN 帧
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_Omega
{
    float Control_Omega;
} __attribute__((packed));

/**
 * @brief EMIT 模式发送 CAN 帧（带速度限幅、电流限幅）
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT
{
    float Control_Angle;
    uint16_t Control_Omega;     // rad/s × 100
    uint16_t Control_Current;   // 占电流最大值的 1/10000
} __attribute__((packed));

/**
 * @brief 解析后的电机数据（对外通过 Get_Now_* 读取）
 */
struct Struct_Motor_DM_Rx_Data_Normal
{
    Enum_Motor_DM_Control_Status_Normal Control_Status;
    float Now_Rad;                     // 累计圈数 + 单圈位置
    float Now_Omega;
    float Now_Torque;
    float Now_MOS_Temperature;         // K
    float Now_Rotor_Temperature;       // K

    uint32_t Pre_Encoder;
    int32_t Total_Round;
    uint8_t First_Update_Flag;
};

/**
 * @brief Reusable, 达妙电机, 传统模式
 *        没有零点, 可在上位机调零点
 */
class Class_Motor_DM_Normal
{
public:
    /// 收数据绑定的 CAN ID（=上位机 Master_ID）
    uint16_t DM_CAN_Rx_ID;
    /// 发数据绑定的 CAN ID（=上位机 CAN_ID + 控制模式偏移）
    uint16_t DM_CAN_Tx_ID;

    /**
     * @brief 电机初始化（绑定 CAN/句柄、控制模式、参数幅值）
     * @param __LinkX_Handler            EtherCAT LinkX 句柄
     * @param __CAN_Channel              CAN 通道编号
     * @param __CAN_Rx_ID                收 ID（与上位机 Master_ID 一致）
     * @param __CAN_Tx_ID                发 ID 基址（CAN_ID），由控制模式追加偏移
     * @param __Motor_DM_Control_Method  控制方式 (MIT/Angle_Omega/Omega/EMIT)
     * @param __Angle_Max                位置幅值 PMAX (rad)
     * @param __Omega_Max                速度幅值 VMAX (rad/s)
     * @param __Torque_Max               力矩幅值 TMAX (Nm)
     * @param __Current_Max              电流幅值 (A)
     */
    void Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel,
              uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID,
              Enum_Motor_DM_Control_Method __Motor_DM_Control_Method,
              float __Angle_Max, float __Omega_Max,
              float __Torque_Max, float __Current_Max);

    // ---- 状态读取 ----

    /** @brief 读电机存活状态 (alive 滑动窗口结果) */
    inline Enum_Motor_DM_Status Get_Status() const           { return Motor_DM_Status; }
    /** @brief 读电机最近一次反馈的控制状态字 */
    inline float Get_Now_Control_Status() const               { return data.Control_Status; }
    /** @brief 读当前电机轴累计角度 (rad) */
    inline float Get_Now_Radian() const                       { return data.Now_Rad; }
    /** @brief 读位置幅值 PMAX (rad) — 固件单圈相位边界 */
    inline float Get_Radian_Max() const                       { return Radian_Max; }
    /** @brief 读当前电机轴角速度 (rad/s) */
    inline float Get_Now_Omega() const                        { return data.Now_Omega; }
    /** @brief 读当前电机反馈力矩 (Nm) */
    inline float Get_Now_Torque() const                       { return data.Now_Torque; }
    /** @brief 读 MOS 温度 (K) — 减 CELSIUS_TO_KELVIN 转 ℃ */
    inline float Get_Now_MOS_Temperature() const              { return data.Now_MOS_Temperature; }
    /** @brief 读转子温度 (K) — 减 CELSIUS_TO_KELVIN 转 ℃ */
    inline float Get_Now_Rotor_Temperature() const            { return data.Now_Rotor_Temperature; }

    // ---- 控制设定 ----

    /** @brief 强制设置存活状态（外部判定离线时使用） */
    inline void Set_Control_Status(Enum_Motor_DM_Status s)                      { Motor_DM_Status = s; }
    /** @brief 切换控制模式（注意：Init 已根据模式确定 Tx_ID，不会随之改变） */
    inline void Set_Control_Method(Enum_Motor_DM_Control_Method m)              { Motor_DM_Control_Method = m; }

    /**
     * @brief 一次性写入 MIT 全部 5 项目标
     * @param angle  目标角 (rad)
     * @param omega  目标速度 (rad/s)
     * @param torque 目标力矩 (Nm)
     * @param kp     位置增益 (0~500)
     * @param kd     阻尼增益 (0~5)
     */
    inline void Set_Control_Maintain_Postion(float angle, float omega, float torque, float kp, float kd)
    {
        Control_Radian = angle;
        Control_Omega  = omega;
        Control_Torque = torque;
        K_P = kp;
        K_D = kd;
    }
    /** @brief 仅写入目标 angle + omega，不改 torque/Kp/Kd */
    inline void Set_Control_Parameter_MIT(float angle, float omega)
    {
        Control_Radian = angle;
        Control_Omega  = omega;
    }
    /** @brief 写入 (torque, angle, omega)，不改 Kp/Kd */
    inline void Set_Control_Parameter_MIT(float torque, float angle, float omega)
    {
        Control_Torque = torque;
        Control_Radian = angle;
        Control_Omega  = omega;
    }
    /** @brief 写入 (torque, Kp, Kd)，不改 angle/omega */
    inline void Set_Control_Torque_P_D_MIT(float torque, float kp, float kd)
    {
        Control_Torque = torque;
        K_P = kp;
        K_D = kd;
    }
    /** @brief 仅写入 (Kp, Kd)，不改 torque/angle/omega */
    inline void Set_Control_Torque_P_D_MIT(float kp, float kd)
    {
        K_P = kp;
        K_D = kd;
    }

    /**
     * @brief CAN 接收回调 — 由 SOEM 主循环在收到本电机帧时调用
     * @param Rx_Data 8 字节原始数据
     */
    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    /** @brief 发送使能帧 0xFF...0xFC */
    void CAN_Send_Enter();
    /** @brief 发送失能帧 0xFF...0xFD */
    void CAN_Send_Exit();
    /** @brief 发送保存零点帧 0xFF...0xFE */
    void CAN_Send_Save_Zero();

    /**
     * @brief 在线检测周期回调 — 通过 Flag 增量判断本周期是否收过帧
     *        若没收到 → 标记 DISABLE
     */
    void TIM_Alive_PeriodElapsedCallback();
    /**
     * @brief 周期发送回调 — 根据 data.Control_Status 决策：
     *          ENABLE  → 限幅后调用 Output() 发控制帧
     *          DISABLE → 发使能帧
     *          其他    → 发清除错误帧
     */
    void TIM_Send_PeriodElapsedCallback();

    // ============================================================
    //   参数标定（FRICTION / STICTION / INERTIA），全程 MIT 模式
    //   主循环每个控制周期调用一次 Calibration_Tick(dt_s)
    //   ★ 开始前必须把车轮架空 ★
    // ============================================================

    /**
     * @brief 启动动摩擦标定：用 MIT 的 D 项当速度环跟住 omega_target，稳态 torque ≈ 动摩擦
     * @param omega_target_rad_s   目标速度（电机轴），建议 1.0~3.0
     * @param kd_velocity_loop     MIT D 增益，建议 1.5~3.0
     * @param warmup_s             单方向加速到位等待时间
     * @param measure_s            单方向取样时长
     */
    void Begin_Friction_Calibration(float omega_target_rad_s,
                                    float kd_velocity_loop = 2.0f,
                                    float warmup_s = 1.5f,
                                    float measure_s = 2.0f);

    /**
     * @brief 启动静摩擦标定：MIT(kp=0,kd=0)，torque 阶梯递增到 |omega| 突破阈值
     * @param torque_step_nm       每步增量（建议 0.005~0.02）
     * @param dwell_s              每步保持时长（建议 0.10~0.20）
     * @param omega_threshold_rad_s 突破判定 ω 阈值（建议 0.3）
     * @param torque_max_nm        上限保护，超出仍未动则失败
     */
    void Begin_Stiction_Calibration(float torque_step_nm = 0.01f,
                                    float dwell_s = 0.10f,
                                    float omega_threshold_rad_s = 0.3f,
                                    float torque_max_nm = 1.0f);

    /**
     * @brief 启动转动惯量标定：MIT(kp=0,kd=0) 给 torque 阶跃，由 ω-t 一次拟合得 α，J=(T-Tf)/α
     * @param torque_step_nm           阶跃力矩（建议 0.3~0.8）
     * @param friction_torque_known_nm 减除的动摩擦（先跑 Friction 标定得到，可填 0）
     * @param warmup_s                 阶跃前静止等待时间
     * @param accel_duration_s         阶跃 / 采样时长（避免 ω 撞上限）
     */
    void Begin_Inertia_Calibration(float torque_step_nm,
                                   float friction_torque_known_nm = 0.0f,
                                   float warmup_s = 0.5f,
                                   float accel_duration_s = 0.5f);

    /**
     * @brief 标定状态机推进，主循环每个控制周期调一次
     * @param dt_s 控制周期 (s)，典型 0.002
     */
    void Calibration_Tick(float dt_s);

    /** @brief 立即终止标定，恢复 0 力矩 0 增益（不会失能电机） */
    void Stop_Calibration();

    /** @brief 是否完成（成功或失败均算 finished） */
    inline bool Is_Calibration_Finished() const                          { return calib_result_.finished; }
    /** @brief 获取最近一次标定结果（运行中也可读） */
    inline Struct_Motor_DM_Calib_Result Get_Calibration_Result() const   { return calib_result_; }

protected:
    // ---- 通信 ----
    linkx_t *LinkX_Handler;
    uint8_t  CAN_Channel;

    // ---- 上位机参数幅值 ----
    float Radian_Max;
    float Omega_Max;
    float Torque_Max;
    float Current_Max;

    // ---- 发送缓冲区 ----
    uint8_t Tx_Data[8];

    // ---- 接收/状态 ----
    Enum_Motor_DM_Status Motor_DM_Status = Motor_DM_Status_DISABLE;
    Struct_Motor_DM_Rx_Data_Normal data;

    // ---- 在线检测计数 ----
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    // ---- 控制参数 ----
    Enum_Motor_DM_Control_Method Motor_DM_Control_Method = Motor_DM_Control_Method_NORMAL_MIT;
    float Control_Radian  = 0.0f;
    float Control_Omega   = 0.0f;
    float Control_Torque  = 0.0f;
    float Control_Current = 0.0f;
    float K_P = 0.0f;
    float K_D = 0.0f;

    // ---- 标定状态 ----
    Enum_Motor_DM_Calib_Mode calib_mode_ = Motor_DM_Calib_Mode_NONE;
    int   calib_phase_ = 0;
    float calib_phase_elapsed_s_ = 0.0f;
    float calib_total_elapsed_s_ = 0.0f;

    // FRICTION
    float friction_omega_target_ = 0.0f;
    float friction_kd_           = 2.0f;
    float friction_warmup_s_     = 1.5f;
    float friction_measure_s_    = 2.0f;
    double friction_acc_torque_pos_ = 0.0;
    double friction_acc_torque_neg_ = 0.0;
    double friction_acc_omega_pos_  = 0.0;
    double friction_acc_omega_neg_  = 0.0;
    uint32_t friction_n_pos_ = 0;
    uint32_t friction_n_neg_ = 0;

    // STICTION
    float stiction_torque_step_   = 0.01f;
    float stiction_dwell_s_       = 0.10f;
    float stiction_omega_thresh_  = 0.3f;
    float stiction_torque_max_    = 1.0f;
    float stiction_current_torque_ = 0.0f;

    // INERTIA  (拟合 ω = ω0 + α·t)
    float inertia_torque_step_     = 0.5f;
    float inertia_warmup_s_        = 0.5f;
    float inertia_duration_s_      = 0.5f;
    float inertia_friction_known_  = 0.0f;
    double inertia_t_sum_       = 0.0;
    double inertia_omega_sum_   = 0.0;
    double inertia_t2_sum_      = 0.0;
    double inertia_t_omega_sum_ = 0.0;
    uint32_t inertia_n_ = 0;

    /** 最近一次标定结果 */
    Struct_Motor_DM_Calib_Result calib_result_ = {};

    // ---- 内部辅助 ----

    /** @brief 用当前 Tx_ID 发送 8 字节命令帧 */
    void Send_CAN_Cmd(const uint8_t cmd[8]);
    /** @brief 发送清错帧 0xFF...0xFB（仅内部使用） */
    void CAN_Send_Clear_Error();
    /** @brief 解析反馈帧并更新 data 字段 */
    void Data_Process(uint8_t *Rx_Data);
    /** @brief 按 Motor_DM_Control_Method 打包并发送控制帧 */
    void Output();
};

#endif // DVC_MOTOR_DM_H
