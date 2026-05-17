//
// Created by pzx on 2025/12/15.
//
#include "dvc_motor_dm.h"
#include "math.h"
#include <cmath>
#include <iostream>

namespace
{
// 控制帧（传统模式有效）
constexpr uint8_t DM_Cmd_Clear_Error[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfb};
constexpr uint8_t DM_Cmd_Enter      [8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc};
constexpr uint8_t DM_Cmd_Exit       [8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd};
constexpr uint8_t DM_Cmd_Save_Zero  [8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe};
}

/* ============================================================================
 *   初始化 / CAN 发送
 * ========================================================================== */

/**
 * @brief 电机初始化：绑定 CAN 通道、设置 Tx_ID（按控制模式追加偏移）、保存幅值
 * @param __LinkX_Handler            LinkX 句柄
 * @param __CAN_Channel              CAN 通道
 * @param __CAN_Rx_ID                收 ID（与上位机 Master_ID 一致）
 * @param __CAN_Tx_ID                发 ID 基址（CAN_ID）
 * @param __Motor_DM_Control_Method  控制方式（决定 Tx_ID 偏移：MIT=0 / Angle_Omega=0x100 / Omega=0x200 / EMIT=0x300）
 * @param __Angle_Max / __Omega_Max / __Torque_Max / __Current_Max  上位机控制幅值
 */
void Class_Motor_DM_Normal::Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel,
                                 uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID,
                                 Enum_Motor_DM_Control_Method __Motor_DM_Control_Method,
                                 float __Angle_Max, float __Omega_Max,
                                 float __Torque_Max, float __Current_Max)
{
    LinkX_Handler = __LinkX_Handler;
    CAN_Channel   = __CAN_Channel;

    // NORMAL_MIT/ANGLE_OMEGA/OMEGA/EMIT 对应偏移 0/0x100/0x200/0x300
    uint16_t mode_offset = 0;
    switch (__Motor_DM_Control_Method) {
        case Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA: mode_offset = 0x100; break;
        case Motor_DM_Control_Method_NORMAL_OMEGA:       mode_offset = 0x200; break;
        case Motor_DM_Control_Method_NORMAL_EMIT:        mode_offset = 0x300; break;
        default: break;
    }
    DM_CAN_Tx_ID = __CAN_Tx_ID + mode_offset;
    DM_CAN_Rx_ID = __CAN_Rx_ID;

    Motor_DM_Control_Method = __Motor_DM_Control_Method;
    Radian_Max  = __Angle_Max;
    Omega_Max   = __Omega_Max;
    Torque_Max  = __Torque_Max;
    Current_Max = __Current_Max;
}

/**
 * @brief 通过 LinkX 用当前 Tx_ID 发送 8 字节命令帧（4 个 CAN_Send_* 的共同实现）
 */
void Class_Motor_DM_Normal::Send_CAN_Cmd(const uint8_t cmd[8])
{
    linkx_quick_can_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, const_cast<uint8_t *>(cmd));
}

/** @brief 发送清除错误帧 (FF...FB) */
void Class_Motor_DM_Normal::CAN_Send_Clear_Error() { Send_CAN_Cmd(DM_Cmd_Clear_Error); }
/** @brief 发送使能帧 (FF...FC) */
void Class_Motor_DM_Normal::CAN_Send_Enter()       { Send_CAN_Cmd(DM_Cmd_Enter);       }
/** @brief 发送失能帧 (FF...FD) */
void Class_Motor_DM_Normal::CAN_Send_Exit()        { Send_CAN_Cmd(DM_Cmd_Exit);        }
/** @brief 发送保存零点帧 (FF...FE) */
void Class_Motor_DM_Normal::CAN_Send_Save_Zero()   { Send_CAN_Cmd(DM_Cmd_Save_Zero);   }

/* ============================================================================
 *   中断 / 周期回调
 * ========================================================================== */

/**
 * @brief CAN 接收回调：累加 Flag（供存活检测），并解析数据
 */
void Class_Motor_DM_Normal::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Flag += 1;
    Data_Process(Rx_Data);
}

/**
 * @brief 存活检测回调：本周期 Flag 未变化 → 标记 DISABLE
 */
void Class_Motor_DM_Normal::TIM_Alive_PeriodElapsedCallback()
{
    Motor_DM_Status = (Flag == Pre_Flag) ? Motor_DM_Status_DISABLE : Motor_DM_Status_ENABLE;
    Pre_Flag = Flag;
}

/**
 * @brief 周期发送回调：根据反馈状态字三态决策 — 正常下发控制 / 重试使能 / 清除错误
 */
void Class_Motor_DM_Normal::TIM_Send_PeriodElapsedCallback()
{
    if (data.Control_Status == Motor_DM_Status_ENABLE) {
        // 限幅后再下发控制帧
        Math_Constrain(&Control_Radian,  -Radian_Max,  Radian_Max);
        Math_Constrain(&Control_Omega,   -Omega_Max,   Omega_Max);
        Math_Constrain(&Control_Torque,  -Torque_Max,  Torque_Max);
        Math_Constrain(&Control_Current, -Current_Max, Current_Max);
        Math_Constrain(&K_P, 0.0f, 500.0f);
        Math_Constrain(&K_D, 0.0f, 5.0f);
        Output();
    } else if (data.Control_Status == Motor_DM_Status_DISABLE) {
        CAN_Send_Enter();
    } else {
        CAN_Send_Clear_Error();
    }
}

/* ============================================================================
 *   收 / 发数据处理
 * ========================================================================== */

/**
 * @brief 解析反馈帧 — 校验 ID、解大端编码器、累计圈数、12bit ω/τ 反向映射、温度 K 化
 *
 *  数据流：
 *      Rx_Data  →  Struct_Motor_DM_CAN_Rx_Data_Normal
 *      → 反向编码器 16bit + 12bit ω + 12bit τ + 状态字
 *      → 与 Pre_Encoder 比较检测翻圈，更新 Total_Round
 *      → 单圈 rad = -Pmax + (enc/65535) * 2Pmax
 *      → Now_Rad = Total_Round * 2Pmax + 单圈 rad
 *
 *  注：Math_Int_To_Float 的输出区间为 [-Omega_Max, +Omega_Max]，
 *      解决了原协议把负值截断为 0 的问题。
 */
void Class_Motor_DM_Normal::Data_Process(uint8_t *Rx_Data)
{
    auto *rx = reinterpret_cast<Struct_Motor_DM_CAN_Rx_Data_Normal *>(Rx_Data);

    // 电机 ID 不匹配则丢弃
    if (rx->CAN_ID != (DM_CAN_Tx_ID & 0x0f)) return;

    // 大端 → 主机序
    uint16_t tmp_encoder = 0;
    Math_Endian_Reverse_16(&rx->Angle_Reverse, &tmp_encoder);
    const uint16_t tmp_omega  = (rx->Omega_11_4 << 4) | (rx->Omega_3_0_Torque_11_8 >> 4);
    const uint16_t tmp_torque = ((rx->Omega_3_0_Torque_11_8 & 0x0f) << 8) | rx->Torque_7_0;

    data.Control_Status = static_cast<Enum_Motor_DM_Control_Status_Normal>(rx->Control_Status_Enum);

    // 首帧记录基线，避免误触发翻圈
    if (data.First_Update_Flag == 0) {
        data.Pre_Encoder = tmp_encoder;
        data.First_Update_Flag = 1;
    }

    // 编码器跨 16bit 上下界检测，累计圈数
    const int32_t delta_encoder =
        static_cast<int32_t>(tmp_encoder) - static_cast<int32_t>(data.Pre_Encoder);
    if      (delta_encoder < -(1 << 15)) data.Total_Round++;
    else if (delta_encoder >  (1 << 15)) data.Total_Round--;

    // 多圈连续位置
    const float range_rad  = 2.0f * Radian_Max;
    const float single_rad = -Radian_Max + (tmp_encoder / 65535.0f) * range_rad;
    data.Now_Rad = (data.Total_Round * range_rad) + single_rad;

    // 12bit 双向映射；温度 ℃→K
    data.Now_Omega  = Math_Int_To_Float(tmp_omega,  0, (1 << 12) - 1, -Omega_Max,  Omega_Max);
    data.Now_Torque = Math_Int_To_Float(tmp_torque, 0, (1 << 12) - 1, -Torque_Max, Torque_Max);
    data.Now_MOS_Temperature   = rx->MOS_Temperature   + CELSIUS_TO_KELVIN;
    data.Now_Rotor_Temperature = rx->Rotor_Temperature + CELSIUS_TO_KELVIN;

    data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 按当前控制方式打包并发送控制帧
 *
 *  四种模式都打包到 Tx_Data[8]，最后统一调用 linkx_quick_can_send。
 *    - MIT          : 角度 16bit + 12bit ω/τ + 12bit Kp/Kd
 *    - ANGLE_OMEGA  : 8 字节 = 2 × float (angle, omega)
 *    - OMEGA        : 4 字节 = float (omega)
 *    - EMIT         : float angle + uint16 ω×100 + uint16 (I/Imax)×10000
 */
void Class_Motor_DM_Normal::Output()
{
    switch (Motor_DM_Control_Method)
    {
    case Motor_DM_Control_Method_NORMAL_MIT:
    {
        auto *b = reinterpret_cast<Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *>(Tx_Data);

        uint16_t tmp_angle  = Math_Float_To_Int(Control_Radian, -Radian_Max, Radian_Max, 0, (1 << 16) - 1);
        const uint16_t tmp_omega  = Math_Float_To_Int(Control_Omega,  -Omega_Max,  Omega_Max,  0, (1 << 12) - 1);
        const uint16_t tmp_torque = Math_Float_To_Int(Control_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);
        const uint16_t tmp_k_p    = Math_Float_To_Int(K_P, 0, 500.0f, 0, (1 << 12) - 1);
        const uint16_t tmp_k_d    = Math_Float_To_Int(K_D, 0, 5.0f,   0, (1 << 12) - 1);

        b->Control_Angle_Reverse        = Math_Endian_Reverse_16(&tmp_angle, nullptr);
        b->Control_Omega_11_4           = tmp_omega >> 4;
        b->Control_Omega_3_0_K_P_11_8   = ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
        b->K_P_7_0                      = tmp_k_p & 0xff;
        b->K_D_11_4                     = tmp_k_d >> 4;
        b->K_D_3_0_Control_Torque_11_8  = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        b->Control_Torque_7_0           = tmp_torque & 0xff;
        break;
    }
    case Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA:
    {
        auto *b = reinterpret_cast<Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *>(Tx_Data);
        b->Control_Angle = Control_Radian;
        b->Control_Omega = Control_Omega;
        break;
    }
    case Motor_DM_Control_Method_NORMAL_OMEGA:
    {
        auto *b = reinterpret_cast<Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *>(Tx_Data);
        b->Control_Omega = Control_Omega;
        break;
    }
    case Motor_DM_Control_Method_NORMAL_EMIT:
    {
        auto *b = reinterpret_cast<Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *>(Tx_Data);
        b->Control_Angle   = Control_Radian;
        b->Control_Omega   = static_cast<uint16_t>(Control_Omega * 100.0f);
        b->Control_Current = static_cast<uint16_t>(Control_Current / Current_Max * 10000.0f);
        break;
    }
    default:
        return;
    }

    linkx_quick_can_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, Tx_Data);
}

/* ============================================================================
 *   参数标定（动摩擦 / 静摩擦 / 转动惯量），全部走 MIT 模式
 * ========================================================================== */

/**
 * @brief 启动动摩擦标定。重置累加器、配置 MIT 模式 + Kd 速度环跟随 +omega_target
 */
void Class_Motor_DM_Normal::Begin_Friction_Calibration(float omega_target_rad_s,
                                                       float kd_velocity_loop,
                                                       float warmup_s,
                                                       float measure_s)
{
    calib_mode_  = Motor_DM_Calib_Mode_FRICTION;
    calib_phase_ = 0;
    calib_phase_elapsed_s_ = calib_total_elapsed_s_ = 0.0f;

    friction_omega_target_ = omega_target_rad_s;
    friction_kd_           = kd_velocity_loop;
    friction_warmup_s_     = warmup_s;
    friction_measure_s_    = measure_s;
    friction_acc_torque_pos_ = friction_acc_torque_neg_ = 0.0;
    friction_acc_omega_pos_  = friction_acc_omega_neg_  = 0.0;
    friction_n_pos_ = friction_n_neg_ = 0;

    calib_result_       = {};
    calib_result_.mode  = Motor_DM_Calib_Mode_FRICTION;

    Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
    Set_Control_Maintain_Postion(0.0f, +friction_omega_target_, 0.0f, 0.0f, friction_kd_);

    std::cout << "[CALIB] Friction: omega=" << omega_target_rad_s
              << " rad/s  kd=" << kd_velocity_loop
              << "  warmup=" << warmup_s << "s  measure=" << measure_s << "s\n";
}

/**
 * @brief 启动静摩擦标定。重置累加器、配置 MIT(kp=kd=0)，τ 从 0 开始阶梯增长
 */
void Class_Motor_DM_Normal::Begin_Stiction_Calibration(float torque_step_nm,
                                                       float dwell_s,
                                                       float omega_threshold_rad_s,
                                                       float torque_max_nm)
{
    calib_mode_  = Motor_DM_Calib_Mode_STICTION;
    calib_phase_ = 0;
    calib_phase_elapsed_s_ = calib_total_elapsed_s_ = 0.0f;

    stiction_torque_step_    = torque_step_nm;
    stiction_dwell_s_        = dwell_s;
    stiction_omega_thresh_   = omega_threshold_rad_s;
    stiction_torque_max_     = torque_max_nm;
    stiction_current_torque_ = 0.0f;

    calib_result_       = {};
    calib_result_.mode  = Motor_DM_Calib_Mode_STICTION;

    Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
    Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    std::cout << "[CALIB] Stiction: step=" << torque_step_nm
              << " Nm  dwell=" << dwell_s << "s  thresh=" << omega_threshold_rad_s
              << " rad/s  Tmax=" << torque_max_nm << " Nm\n";
}

/**
 * @brief 启动转动惯量标定。重置最小二乘累加器、配置 MIT(kp=kd=0)，先静止再 τ 阶跃
 */
void Class_Motor_DM_Normal::Begin_Inertia_Calibration(float torque_step_nm,
                                                      float friction_torque_known_nm,
                                                      float warmup_s,
                                                      float accel_duration_s)
{
    calib_mode_  = Motor_DM_Calib_Mode_INERTIA;
    calib_phase_ = 0;
    calib_phase_elapsed_s_ = calib_total_elapsed_s_ = 0.0f;

    inertia_torque_step_    = torque_step_nm;
    inertia_friction_known_ = friction_torque_known_nm;
    inertia_warmup_s_       = warmup_s;
    inertia_duration_s_     = accel_duration_s;

    inertia_t_sum_ = inertia_omega_sum_ = inertia_t2_sum_ = inertia_t_omega_sum_ = 0.0;
    inertia_n_ = 0;

    calib_result_       = {};
    calib_result_.mode  = Motor_DM_Calib_Mode_INERTIA;

    Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
    Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    std::cout << "[CALIB] Inertia: T_step=" << torque_step_nm
              << " Nm  friction=" << friction_torque_known_nm
              << " Nm  warmup=" << warmup_s << "s  accel=" << accel_duration_s << "s\n";
}

/**
 * @brief 终止当前标定 — 把模式切到 NONE、写入 finished、强制 0 力矩 0 增益
 *        ★ 不会 CAN_Send_Exit ★，电机仍处于使能状态
 */
void Class_Motor_DM_Normal::Stop_Calibration()
{
    calib_mode_ = Motor_DM_Calib_Mode_NONE;
    calib_result_.finished = true;
    Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

/**
 * @brief 标定状态机推进 — 每个控制周期调一次
 *
 *  FRICTION  : phase 0 正向 warmup → 1 正向 measure → 2 反向 warmup → 3 反向 measure
 *              结束时取 ±方向反馈 τ/ω 平均，filled into calib_result_
 *  STICTION  : 持续把 τ 阶梯递增 + 每 dwell_s 加 step；|ω|>thresh 即记录突破点
 *              τ 超 torque_max → 失败
 *  INERTIA   : phase 0 静止 warmup → 1 阶跃 + ω vs t 一次最小二乘求 α
 *              J = (T_step − T_friction_known) / α
 */
void Class_Motor_DM_Normal::Calibration_Tick(float dt_s)
{
    if (calib_mode_ == Motor_DM_Calib_Mode_NONE || calib_result_.finished)
        return;

    calib_phase_elapsed_s_ += dt_s;
    calib_total_elapsed_s_ += dt_s;
    calib_result_.phase           = calib_phase_;
    calib_result_.total_elapsed_s = calib_total_elapsed_s_;

    switch (calib_mode_)
    {
    case Motor_DM_Calib_Mode_FRICTION:
    {
        switch (calib_phase_)
        {
        case 0:
            // 正向加速预热
            Set_Control_Maintain_Postion(0.0f, +friction_omega_target_, 0.0f, 0.0f, friction_kd_);
            if (calib_phase_elapsed_s_ >= friction_warmup_s_) {
                calib_phase_ = 1; calib_phase_elapsed_s_ = 0.0f;
            }
            break;
        case 1:
            // 正向稳态采样
            Set_Control_Maintain_Postion(0.0f, +friction_omega_target_, 0.0f, 0.0f, friction_kd_);
            friction_acc_torque_pos_ += data.Now_Torque;
            friction_acc_omega_pos_  += data.Now_Omega;
            friction_n_pos_++;
            if (calib_phase_elapsed_s_ >= friction_measure_s_) {
                calib_phase_ = 2; calib_phase_elapsed_s_ = 0.0f;
            }
            break;
        case 2:
            // 反向加速预热
            Set_Control_Maintain_Postion(0.0f, -friction_omega_target_, 0.0f, 0.0f, friction_kd_);
            if (calib_phase_elapsed_s_ >= friction_warmup_s_) {
                calib_phase_ = 3; calib_phase_elapsed_s_ = 0.0f;
            }
            break;
        case 3:
            // 反向稳态采样
            Set_Control_Maintain_Postion(0.0f, -friction_omega_target_, 0.0f, 0.0f, friction_kd_);
            friction_acc_torque_neg_ += data.Now_Torque;
            friction_acc_omega_neg_  += data.Now_Omega;
            friction_n_neg_++;
            if (calib_phase_elapsed_s_ >= friction_measure_s_) {
                // 计算结果：取双向力矩绝对值平均
                const float t_pos = (friction_n_pos_ > 0)
                    ? static_cast<float>(friction_acc_torque_pos_ / friction_n_pos_) : 0.0f;
                const float t_neg = (friction_n_neg_ > 0)
                    ? static_cast<float>(friction_acc_torque_neg_ / friction_n_neg_) : 0.0f;
                const float w_pos = (friction_n_pos_ > 0)
                    ? static_cast<float>(friction_acc_omega_pos_  / friction_n_pos_) : 0.0f;
                const float w_neg = (friction_n_neg_ > 0)
                    ? static_cast<float>(friction_acc_omega_neg_  / friction_n_neg_) : 0.0f;

                calib_result_.friction_torque_pos_nm   = t_pos;
                calib_result_.friction_torque_neg_nm   = std::fabs(t_neg);
                calib_result_.friction_torque_avg_nm   = 0.5f * (std::fabs(t_pos) + std::fabs(t_neg));
                calib_result_.friction_omega_pos_actual = w_pos;
                calib_result_.friction_omega_neg_actual = w_neg;
                calib_result_.success  = (friction_n_pos_ > 50 && friction_n_neg_ > 50);
                calib_result_.finished = true;
                Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

                std::cout << "[CALIB][FRICTION] T+=" << t_pos
                          << " Nm  T-=" << t_neg
                          << " Nm  avg=" << calib_result_.friction_torque_avg_nm
                          << " Nm  (omega+=" << w_pos << " omega-=" << w_neg << ")\n";
            }
            break;
        default:
            Stop_Calibration();
            break;
        }
        break;
    }

    case Motor_DM_Calib_Mode_STICTION:
    {
        // 持续下发当前 τ
        Set_Control_Maintain_Postion(0.0f, 0.0f, stiction_current_torque_, 0.0f, 0.0f);

        // |ω|>thresh 且 τ>0 → 记录突破点
        if (std::fabs(data.Now_Omega) > stiction_omega_thresh_ && stiction_current_torque_ > 1e-6f) {
            calib_result_.stiction_torque_nm       = stiction_current_torque_;
            calib_result_.stiction_breakaway_omega = data.Now_Omega;
            calib_result_.success  = true;
            calib_result_.finished = true;
            Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            std::cout << "[CALIB][STICTION] breakaway T=" << stiction_current_torque_
                      << " Nm at omega=" << data.Now_Omega << " rad/s\n";
            break;
        }

        // 每 dwell 加一个 step；超 Tmax 视为失败
        if (calib_phase_elapsed_s_ >= stiction_dwell_s_) {
            stiction_current_torque_ += stiction_torque_step_;
            calib_phase_elapsed_s_ = 0.0f;
            if (stiction_current_torque_ > stiction_torque_max_) {
                calib_result_.stiction_torque_nm = stiction_torque_max_;
                calib_result_.success  = false;
                calib_result_.finished = true;
                Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                std::cout << "[CALIB][STICTION] FAILED: T_max=" << stiction_torque_max_
                          << " Nm reached without breakaway\n";
            }
        }
        break;
    }

    case Motor_DM_Calib_Mode_INERTIA:
    {
        switch (calib_phase_)
        {
        case 0:
            // 静止预热
            Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            if (calib_phase_elapsed_s_ >= inertia_warmup_s_) {
                calib_phase_ = 1; calib_phase_elapsed_s_ = 0.0f;
            }
            break;
        case 1:
        {
            // τ 阶跃，累加 (t, ω) 用于一次最小二乘
            Set_Control_Maintain_Postion(0.0f, 0.0f, inertia_torque_step_, 0.0f, 0.0f);
            const double t = static_cast<double>(calib_phase_elapsed_s_);
            const double w = static_cast<double>(data.Now_Omega);
            inertia_t_sum_       += t;
            inertia_omega_sum_   += w;
            inertia_t2_sum_      += t * t;
            inertia_t_omega_sum_ += t * w;
            inertia_n_++;

            if (calib_phase_elapsed_s_ >= inertia_duration_s_) {
                // α = (n·Σtω − Σt·Σω) / (n·Σt² − (Σt)²)
                const double n     = static_cast<double>(inertia_n_);
                const double denom = n * inertia_t2_sum_ - inertia_t_sum_ * inertia_t_sum_;
                float alpha = 0.0f;
                if (denom > 1e-12 && inertia_n_ > 20) {
                    alpha = static_cast<float>(
                        (n * inertia_t_omega_sum_ - inertia_t_sum_ * inertia_omega_sum_) / denom);
                }
                const float net_torque = inertia_torque_step_ - inertia_friction_known_;
                float J = 0.0f;
                if (std::fabs(alpha) > 1e-3f) J = net_torque / alpha;

                calib_result_.inertia_kgm2             = J;
                calib_result_.inertia_alpha_meas       = alpha;
                calib_result_.inertia_torque_step_nm   = inertia_torque_step_;
                calib_result_.inertia_friction_used_nm = inertia_friction_known_;
                calib_result_.success  = (J > 0.0f && std::isfinite(J));
                calib_result_.finished = true;
                Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

                std::cout << "[CALIB][INERTIA] alpha=" << alpha
                          << " rad/s²  T_net=" << net_torque
                          << " Nm  J=" << J << " kg·m²  (samples=" << inertia_n_ << ")\n";
            }
            break;
        }
        default:
            Stop_Calibration();
            break;
        }
        break;
    }

    case Motor_DM_Calib_Mode_NONE:
    default:
        break;
    }
}
