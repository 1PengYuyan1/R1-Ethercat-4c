/**
 * @file  dvc_manual_yaw_hold.h
 * @brief 手动遥控模式下的航向慢漂纠偏 (yaw hold)
 *
 *  问题: 用户手动前后运动时, 多周期下车身慢慢偏一边。wheel speed calib
 *  在 ODrive 边界吃稳态速度差, 但 OPS yaw 累积漂仍可见。
 *
 *  方案: 撒手即锁。用户给 omega 时透传; omega 回到死区且车在动后维持 200ms,
 *  捕获当前 OPS yaw 作为锁定目标, 启用慢 I 控制 (Ki 极小, 25ms 节拍) 把
 *  yaw 拉回锁定值。用户重新给 omega 或停车 → 立刻退出, I 归零。
 *
 *  设计上的硬约束 (吸取 2026-05-17 失败教训, 见 memory
 *  feedback_no_manual_heading_correction):
 *    1. 禁用 P, 禁用 D, 只用 I — H_c(s)=K_i/s, 10Hz 处增益比 P 控制低 ~10^5
 *    2. 25ms 节拍, OPS 帧门控驱动 — 控制环采样率严格 = OPS 物理 40Hz
 *    3. I_Out_Max 硬 cap 0.04 rad/s — 远小于 0.3 rad/s 用户主动 omega
 *    4. 任何退出 HOLDING 的转移强制 I=0 — 切换 manual 模式不残留状态
 *
 *  参考样板: dvc_auto_pilot.cpp 的横向 I 项 (2026-05-22 上车验证 lat std 0.09mm)
 *
 *  调用契约:
 *    Init(chassis)                                      一次性绑定底盘指针
 *    Set_Yaw_PID(ki, i_max, out_max, dead_zone_deg)     调参 (会 re-init PID)
 *    Set_Arming_Params(omega_dead, omega_override,
 *                      vmin, lock_dwell_ms)             调激活/退出阈值
 *    Update(raw_omega, vmag, ops_ok, active)            主循环每帧 (1ms) 调
 *                                                       返回 corrected_omega
 *    Force_Disarm()                                     manual 失能下降沿调,清状态
 */

#ifndef DVC_MANUAL_YAW_HOLD_H
#define DVC_MANUAL_YAW_HOLD_H

#include <cstdint>
#include "alg_pid.h"

class Class_Chassis;   // 前向声明，避免头文件互相依赖

class Class_Manual_Heading_Hold
{
public:
    enum class State : int
    {
        IDLE     = 0,   ///< 待激活: 速度不足/未撒手, 输出 = raw
        LOCKING  = 1,   ///< omega 在死区, dwell 计时, 输出 = raw
        HOLDING  = 2,   ///< target 已捕获, 慢 I 工作, 输出 = raw + I
        DISARMED = 3,   ///< 用户主动转向/停车/OPS 丢, 透传, I=0
    };

    void Init(Class_Chassis *chassis);

    /**
     * @brief 重新配置 yaw PID。会 re-Init 内部 PID, 同时清零积分项。
     * @param ki              I 系数, rad/s per (deg·s)
     * @param i_out_max       I 输出限幅, rad/s
     * @param out_max         总输出限幅, rad/s (Kp=Kd=0 时 = I 上限)
     * @param dead_zone_deg   误差死区, deg
     */
    void Set_Yaw_PID(float ki, float i_out_max, float out_max,
                     float dead_zone_deg);

    /**
     * @brief 配置激活/退出阈值。
     * @param omega_dead      arming omega 死区阈值, rad/s (低于此值认为撒手)
     * @param omega_override  HOLDING 时用户主动转向的退出阈值, rad/s
     * @param vmin            最小车速激活阈值, m/s
     * @param lock_dwell_ms   omega 在死区维持此时长后捕获 target
     */
    void Set_Arming_Params(float omega_dead, float omega_override,
                           float vmin, float lock_dwell_ms);

    /**
     * @brief 主循环每帧调用。内部按 OPS frame count 门控, 真正推进 PID 的
     *        节奏 = OPS 帧率 (~40Hz)。
     * @param raw_omega   用户原始 omega 命令 (rad/s)
     * @param vmag        车体速度模 (m/s), sqrt(vx² + vy²)
     * @param ops_ok      OPS 状态 == ENABLE
     * @param active      手动模式当前是否使能 && 命令时效
     * @return corrected_omega: HOLDING 时 = I 输出, 其它态 = raw_omega
     */
    float Update(float raw_omega, float vmag, bool ops_ok, bool active);

    /**
     * @brief 强制退出 HOLDING 并清 I。manual 失能下降沿调, 防止下次再使能时
     *        从上次的 I 状态恢复 (虽然 active=false 也会自然 disarm, 但显式
     *        清更稳)。
     */
    void Force_Disarm();

    /* ---- 调试观测 ---- */
    inline State Get_State() const         { return state_; }
    inline float Get_Target_Yaw() const    { return target_yaw_deg_; }
    inline float Get_Err_Deg() const       { return last_err_deg_; }
    inline float Get_I_Value()             { return pid_yaw_.Get_Integral_Error(); }

    /// 把 State 编成整数给 CSV 用
    static inline int State_To_Int(State s) { return static_cast<int>(s); }

protected:
    Class_Chassis *chassis_ = nullptr;

    State state_ = State::IDLE;

    /* ---- PID 实例 ---- */
    Class_PID pid_yaw_;
    float cfg_ki_           = 0.0008f;
    float cfg_i_out_max_    = 0.04f;
    float cfg_out_max_      = 0.04f;
    float cfg_dead_zone_deg_ = 1.0f;

    /* ---- 激活/退出阈值 ---- */
    float cfg_omega_dead_     = 0.015f;   ///< rad/s
    float cfg_omega_override_ = 0.025f;   ///< rad/s
    float cfg_vmin_           = 0.04f;    ///< m/s
    float cfg_lock_dwell_ms_  = 200.0f;

    /* ---- 状态机内部 ---- */
    float    target_yaw_deg_      = 0.0f;
    float    last_err_deg_        = 0.0f;
    float    locking_dwell_ms_    = 0.0f;
    float    last_corrected_omega_ = 0.0f;   ///< HOLDING 时 OPS 帧间 hold-flat
    uint32_t last_ops_frame_count_ = 0;
    bool     have_ops_frame_count_ = false;

    /* ---- 内部辅助 ---- */
    void Reset_PID_();
    void Enter_State_(State new_state);
    static float Wrap_Pm180_(float deg);
};

#endif   // DVC_MANUAL_YAW_HOLD_H
