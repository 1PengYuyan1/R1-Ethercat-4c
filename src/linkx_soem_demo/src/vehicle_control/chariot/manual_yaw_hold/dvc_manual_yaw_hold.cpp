/**
 * @file  dvc_manual_yaw_hold.cpp
 * @brief 手动遥控模式下的航向慢漂纠偏实现
 *
 *  状态机:
 *    IDLE → LOCKING → HOLDING ⇄ DISARMED → IDLE
 *
 *  转移条件 (带迟滞, 避免边界抖动):
 *    - IDLE → LOCKING:        ops_ok && active && |omega| < cfg_omega_dead && vmag > cfg_vmin
 *    - LOCKING → HOLDING:     上述维持 cfg_lock_dwell_ms (200ms) → capture target_yaw, I=0
 *    - LOCKING/HOLDING → DISARMED: |omega| > cfg_omega_override (用户主动转)
 *                                   || vmag < cfg_vmin*0.5 (停车,带迟滞)
 *                                   || !ops_ok || !active
 *    - DISARMED → IDLE:       撒手 (|omega| < cfg_omega_dead) 一帧后
 *
 *  I 清零矩阵: 进入 IDLE/LOCKING/DISARMED 时强制清零, HOLDING 之外不残留。
 *
 *  OPS 帧门控: Update 每 1ms 调,但内部按 Get_Rx_Frame_Count() 是否变化 gate。
 *  帧未变 → HOLDING 沿用上次 corrected (hold-flat ZOH), 其它态返回 raw。
 *  控制环采样率严格 = OPS 物理 40Hz, 避免在 1ms 高频窗口产生 PID 步进。
 *
 *  符号约定 (与 dvc_auto_pilot.cpp:280 一致):
 *    chassis omega+ → 物理 CW → OPS yaw 减小; 与 err+("需要 yaw 增大") 反向。
 *    PID 输入 err = wrap(target - cur), 输出后翻号: corrected = -pid.Out。
 */

#include "dvc_manual_yaw_hold.h"
#include "crt_chassis.h"
#include "dvc_ops.h"

namespace
{
constexpr float kPidDt = 0.025f;   ///< 与 OPS 40Hz 节拍匹配 (auto_pilot 同款)
constexpr float kDwellHysteresisMs = 0.0f;   ///< dwell 直接计满, 无需迟滞
}

/* ============================================================
 *  生命周期 / 配置
 * ========================================================== */

void Class_Manual_Heading_Hold::Init(Class_Chassis *chassis)
{
    chassis_ = chassis;
    state_   = State::IDLE;

    target_yaw_deg_       = 0.0f;
    last_err_deg_         = 0.0f;
    locking_dwell_ms_     = 0.0f;
    last_corrected_omega_ = 0.0f;
    have_ops_frame_count_ = false;
    last_ops_frame_count_ = 0;

    Reset_PID_();
}

void Class_Manual_Heading_Hold::Reset_PID_()
{
    // Kp=0, Kd=0, Kf=0; 仅 Ki 工作。Init 同时清 Integral_Error 等内部状态。
    pid_yaw_.Init(/*K_P=*/0.0f,
                  /*K_I=*/cfg_ki_,
                  /*K_D=*/0.0f,
                  /*K_F=*/0.0f,
                  /*I_Out_Max=*/cfg_i_out_max_,
                  /*Out_Max=*/cfg_out_max_,
                  /*D_T=*/kPidDt,
                  /*Dead_Zone=*/cfg_dead_zone_deg_);
}

void Class_Manual_Heading_Hold::Set_Yaw_PID(float ki, float i_out_max,
                                            float out_max, float dead_zone_deg)
{
    cfg_ki_            = ki;
    cfg_i_out_max_     = i_out_max;
    cfg_out_max_       = out_max;
    cfg_dead_zone_deg_ = dead_zone_deg;
    Reset_PID_();   // re-Init 同时清积分
}

void Class_Manual_Heading_Hold::Set_Arming_Params(float omega_dead,
                                                  float omega_override,
                                                  float vmin,
                                                  float lock_dwell_ms)
{
    cfg_omega_dead_     = omega_dead;
    cfg_omega_override_ = omega_override;
    cfg_vmin_           = vmin;
    cfg_lock_dwell_ms_  = lock_dwell_ms;
}

void Class_Manual_Heading_Hold::Force_Disarm()
{
    Enter_State_(State::DISARMED);
}

/* ============================================================
 *  状态转移
 * ========================================================== */

void Class_Manual_Heading_Hold::Enter_State_(State new_state)
{
    // I 清零矩阵: 离开 HOLDING / 进入任何非 HOLDING 都清 I + last_corrected
    if (new_state != State::HOLDING)
    {
        pid_yaw_.Set_Integral_Error(0.0f);
        last_corrected_omega_ = 0.0f;
    }
    if (new_state == State::LOCKING)
    {
        locking_dwell_ms_ = 0.0f;
    }
    state_ = new_state;
}

/* ============================================================
 *  主循环
 * ========================================================== */

float Class_Manual_Heading_Hold::Update(float raw_omega, float vmag,
                                        bool ops_ok, bool active)
{
    // ---- 短路: 没绑底盘 / 没使能 / OPS 丢 → 强制 DISARMED, 透传 ----
    if (chassis_ == nullptr || !active || !ops_ok)
    {
        if (state_ != State::DISARMED) Enter_State_(State::DISARMED);
        return raw_omega;
    }

    const float abs_omega = (raw_omega >= 0.0f) ? raw_omega : -raw_omega;
    const bool stick_in_dead = (abs_omega < cfg_omega_dead_);
    const bool stick_override = (abs_omega > cfg_omega_override_);
    const bool moving = (vmag > cfg_vmin_);
    const bool stopped = (vmag < cfg_vmin_ * 0.5f);   // 退出迟滞: 0.5×vmin

    // ---- OPS 帧门控: 1ms 主循环 vs OPS 40Hz, 帧未变直接走 hold-flat ----
    const uint32_t cur_ops_cnt = chassis_->OPS.Get_Rx_Frame_Count();
    const bool new_ops_frame = (!have_ops_frame_count_)
                                || (cur_ops_cnt != last_ops_frame_count_);
    if (new_ops_frame)
    {
        last_ops_frame_count_ = cur_ops_cnt;
        have_ops_frame_count_ = true;
    }

    // ---- 状态机 (每 1ms 都跑, 计时累加; PID 仅在 new_ops_frame 时步进) ----
    switch (state_)
    {
        case State::IDLE:
        {
            if (stick_in_dead && moving)
                Enter_State_(State::LOCKING);
            return raw_omega;
        }

        case State::LOCKING:
        {
            // 任一条件不满足 → 退回 IDLE 或 DISARMED, dwell 计时清零
            if (stick_override)
            {
                Enter_State_(State::DISARMED);
                return raw_omega;
            }
            if (!stick_in_dead || stopped)
            {
                Enter_State_(State::IDLE);
                return raw_omega;
            }
            // 累加 dwell, 这里用主循环 1ms 节拍累加 (而非 OPS 25ms),
            // 计时更平滑, 边界判定不依赖 OPS 抖动
            locking_dwell_ms_ += 1.0f;
            if (locking_dwell_ms_ >= cfg_lock_dwell_ms_)
            {
                target_yaw_deg_ = chassis_->OPS.Get_Yaw_Deg();
                Enter_State_(State::HOLDING);
                // HOLDING 第一次进, last_corrected = 0 直到下一新 OPS 帧才 step
            }
            return raw_omega;
        }

        case State::HOLDING:
        {
            // 用户主动转 / 停车 → DISARMED
            if (stick_override)
            {
                Enter_State_(State::DISARMED);
                return raw_omega;
            }
            if (stopped)
            {
                Enter_State_(State::DISARMED);
                return raw_omega;
            }
            // OPS 帧没变, 沿用上次 corrected (ZOH 防止 1ms 注入高频)
            if (!new_ops_frame)
            {
                return last_corrected_omega_;
            }
            // OPS 新帧: 算 err, 步进 PID
            const float cur_yaw_deg = chassis_->OPS.Get_Yaw_Deg();
            const float err_deg = Wrap_Pm180_(target_yaw_deg_ - cur_yaw_deg);
            last_err_deg_ = err_deg;
            // 与 auto_pilot 同款: 把 err 当 Target, Now=0, 取出后翻号
            // (chassis omega+ → OPS yaw-, 与 err+ 反向)
            pid_yaw_.Set_Target(err_deg);
            pid_yaw_.Set_Now(0.0f);
            pid_yaw_.TIM_Calculate_PeriodElapsedCallback();
            last_corrected_omega_ = -pid_yaw_.Get_Out();
            return last_corrected_omega_;
        }

        case State::DISARMED:
        default:
        {
            // 等用户撒手 (一帧死区内即可重新 arm)
            if (stick_in_dead)
            {
                Enter_State_(State::IDLE);
            }
            return raw_omega;
        }
    }
}

/* ============================================================
 *  Yaw wrap
 * ========================================================== */

float Class_Manual_Heading_Hold::Wrap_Pm180_(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}
