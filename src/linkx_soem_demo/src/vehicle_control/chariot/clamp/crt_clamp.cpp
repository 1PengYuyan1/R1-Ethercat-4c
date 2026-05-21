// 双 Pitch DM 电机夹爪 (R-R 串联肩+肘 2-DoF 平面臂)
// 2026-05-21 重构 Phase 1+2:
//   Phase 1: 线性斜坡 → 梯形 profile + 双轴归一化时间同步
//     - 消除起停加速度阶跃 (原 constant-velocity slew 起停瞬间 a 跳到 ±∞)
//     - 消除双轴异步到位顿挫 (原大/小轴各跑各的 v_max,到位时间差 ~70ms)
//   Phase 2: 加 FK/IK + LIN 笛卡尔直线模式
//     - L1=320mm 肩-肘, L2=160mm 肘-夹爪;零位 (θ=0) 大臂水平向后
//     - PTP 用于 POS1↔POS2 / POS4→POS1 等跨原点段 (避免直线穿肩)
//     - LIN 用于 POS2→POS3, POS3→POS4 等头顶区段 (末端走直线,长杆姿态平滑)
//   4 步 POS 序列与 dwell 时长保持 r1.2 兼容
//
// IK 选支: elbow_down (θ2 ∈ [-π, 0]),与 POS 表 θ2<0 一致
// IK unwrap: 输出经 round((q_ref - q_ik)/2π) 选最近圈,保持电机不跨整圈跳

#include "crt_clamp.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include "linkx4c_handler.h"

// ============================================================================
// CSV Trace: env CLAMP_TRACE_FILE=/path/x.csv 触发, 每 cycle (2ms = 500Hz) 写一行
// 未设 env → 永久 no-op。表头: ts_ms,seq,seg,sp_q1,act_q1,sp_q2,act_q2,
//   omega_q1,omega_q2,torque_q1,torque_q2,tau1_ff,tau2_ff
// 用 ifstream 静态,首调 init: 检查 env, 打开文件 (trunc), 写表头
// ============================================================================
namespace {
struct ClampTrace {
  bool init = false;
  bool disabled = false;
  std::ofstream stream;
  uint64_t start_ms = 0;

  void Log(float sp_q1, float ac_q1, float sp_q2, float ac_q2,
           float w_q1, float w_q2, float t_q1, float t_q2,
           float tau1_ff, float tau2_ff, uint8_t seq, uint8_t seg) {
    if (!init) {
      init = true;
      const char* path = std::getenv("CLAMP_TRACE_FILE");
      if (!path || path[0] == '\0') {
        disabled = true;
        return;
      }
      stream.open(path, std::ios::trunc);
      if (!stream.is_open()) {
        std::cerr << "[CLAMP] CLAMP_TRACE_FILE open failed: " << path
                  << std::endl;
        disabled = true;
        return;
      }
      stream << "ts_ms,seq,seg,sp_q1,act_q1,sp_q2,act_q2,"
                "omega_q1,omega_q2,torque_q1,torque_q2,tau1_ff,tau2_ff\n";
      std::cout << "[CLAMP] TRACE writing to " << path << " @500Hz"
                << std::endl;
      start_ms = 0;
    }
    if (disabled) return;
    static uint64_t tick = 0;
    const uint64_t ts_ms = (tick++) * 2;
    stream << ts_ms << "," << (int)seq << "," << (int)seg
           << "," << sp_q1 << "," << ac_q1
           << "," << sp_q2 << "," << ac_q2
           << "," << w_q1 << "," << w_q2
           << "," << t_q1 << "," << t_q2
           << "," << tau1_ff << "," << tau2_ff << "\n";
  }
};
ClampTrace g_trace;
}  // namespace

// ============================================================================
// TrapezoidProfile1D: 通用 1D 梯形速度规划 (支持初速度 v0 for blending)
// ============================================================================
// Plan: 给定段长 s_total>=0、峰值速度 v_max、加速度 a_max、初始速度 v0,
//   - 三阶段: 加速 v0→v_peak, 匀速 v_peak, 减速 v_peak→0
//   - 梯形/三角形按 (s_acc + s_dec) vs s_total 判定
//   - v0 ≤ √(2a·s_total): 标准, v_peak² = (2a·s_total + v0²)/2 (三角形)
//   - v0 > √(2a·s_total): 紧急减速, 全段减速 v0→v_end>0 在段末
//                          (允许 v_end>0 由下段衔接,不允许过冲 s_total)
// Sample(t): 三段拼接,t>=T 后钳到 s_total
// Sample_Velocity(t): 同三段速度剖面
void TrapezoidProfile1D::Plan(float s_total, float v_max, float a_max,
                              float v0) {
  s_total = std::fabs(s_total);
  v0 = std::fmax(v0, 0.0f);
  if (s_total < 1e-9f || v_max <= 0.0f || a_max <= 0.0f) {
    s_total_ = s_total;
    v0_ = v0;
    v_peak_ = a_ = 0.0f;
    t_acc_ = t_const_ = T_ = 0.0f;
    return;
  }
  s_total_ = s_total;
  v0_ = v0;
  a_ = a_max;

  // 紧急减速: v0 太大无法在 s_total 内减速到 0
  // 解 s_total = v0·t - 0.5·a·t² → t = (v0 - √(v0² - 2a·s_total)) / a (取小根)
  if (v0 * v0 > 2.0f * a_max * s_total) {
    v_peak_ = v0;
    t_acc_ = 0.0f;
    t_const_ = 0.0f;
    T_ = (v0 - std::sqrt(v0 * v0 - 2.0f * a_max * s_total)) / a_max;
    return;
  }

  const float s_acc_full = (v_max * v_max - v0 * v0) / (2.0f * a_max);
  const float s_dec_full = v_max * v_max / (2.0f * a_max);

  if (s_acc_full + s_dec_full >= s_total) {
    // 三角形: 解 v_peak; (v_peak²-v0²)/(2a) + v_peak²/(2a) = s_total
    v_peak_ =
        std::sqrt((2.0f * a_max * s_total + v0 * v0) / 2.0f);
    t_acc_ = (v_peak_ - v0) / a_max;
    t_const_ = 0.0f;
  } else {
    v_peak_ = v_max;
    t_acc_ = (v_max - v0) / a_max;
    t_const_ = (s_total - s_acc_full - s_dec_full) / v_max;
  }
  T_ = t_acc_ + t_const_ + v_peak_ / a_max;
}

float TrapezoidProfile1D::Sample(float t) const {
  if (T_ <= 0.0f || t <= 0.0f) return 0.0f;
  if (t >= T_) return s_total_;

  // 紧急减速: 全段从 v0 匀减速
  if (t_acc_ == 0.0f && t_const_ == 0.0f) {
    return v0_ * t - 0.5f * a_ * t * t;
  }

  if (t <= t_acc_) {
    return v0_ * t + 0.5f * a_ * t * t;
  }
  const float s_acc = v0_ * t_acc_ + 0.5f * a_ * t_acc_ * t_acc_;
  if (t <= t_acc_ + t_const_) {
    return s_acc + v_peak_ * (t - t_acc_);
  }
  // 减速段从末端反推 (对称无累积误差)
  const float t_dec = T_ - t;
  return s_total_ - 0.5f * a_ * t_dec * t_dec;
}

float TrapezoidProfile1D::Sample_Velocity(float t) const {
  if (T_ <= 0.0f) return 0.0f;
  if (t <= 0.0f) return v0_;
  if (t >= T_) {
    // 紧急减速末速度 v0 - a·T > 0; 否则末速度 0
    return (t_acc_ == 0.0f && t_const_ == 0.0f) ? std::fmax(v0_ - a_ * T_, 0.0f)
                                                : 0.0f;
  }

  if (t_acc_ == 0.0f && t_const_ == 0.0f) return v0_ - a_ * t;
  if (t <= t_acc_) return v0_ + a_ * t;
  if (t <= t_acc_ + t_const_) return v_peak_;
  return a_ * (T_ - t);
}

// ============================================================================
// Class_Clamp
// ============================================================================

void Class_Clamp::Init(linkx_t* __LinkX_Handler, uint8_t __CAN_Channel) {
  // 大 Pitch 轴: Tx_ID=0x01, Rx_ID=0x11; MIT 模式; PMAX=12.5 / VMAX=20 /
  // TMAX=15 / IMAX=10.26A
  Motor_Pitch_Large.Init(__LinkX_Handler, __CAN_Channel, 0x11, 0x01,
                         Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 20.0f,
                         15.0f, 10.261194f);
  // 小 Pitch 轴: Tx_ID=0x02, Rx_ID=0x12
  Motor_Pitch_Small.Init(__LinkX_Handler, __CAN_Channel, 0x12, 0x02,
                         Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 20.0f,
                         15.0f, 10.261194f);

  Motor_Pitch_Large.Set_Control_Torque_P_D_MIT(0.0f, pitch_large_kp,
                                               pitch_large_kd);
  Motor_Pitch_Small.Set_Control_Torque_P_D_MIT(0.0f, pitch_small_kp,
                                               pitch_small_kd);

  current_pitch_large_angle = pitch_large_pos1_angle;
  current_pitch_small_angle = pitch_small_pos1_angle;
  segment_active_ = false;
  segment_t_ = 0.0f;

  // 气夹爪与 DM 电机共享同一 LinkX 同一通道; 上电默认 byte0=0x00
  gripper_linkx_handler_ = __LinkX_Handler;
  gripper_can_channel_ = __CAN_Channel;
  gripper_byte0_ = 0x00U;
}

void Class_Clamp::TIM_100ms_Alive_PeriodElapsedCallback() {
  Motor_Pitch_Large.TIM_Alive_PeriodElapsedCallback();
  Motor_Pitch_Small.TIM_Alive_PeriodElapsedCallback();

  if (Clamp_Control_Type == CLAMP_CONTROL_ENABLE) {
    if (Motor_Pitch_Large.Get_Status() != Motor_DM_Control_Status_ENABLE)
      Motor_Pitch_Large.CAN_Send_Enter();
    if (Motor_Pitch_Small.Get_Status() != Motor_DM_Control_Status_ENABLE)
      Motor_Pitch_Small.CAN_Send_Enter();
  }
}

void Class_Clamp::TIM_Calculate_PeriodElapsedCallback() {
  switch (Clamp_Control_Type) {
    case CLAMP_CONTROL_DISABLE: {
      Motor_Pitch_Large.Set_Control_Status(Motor_DM_Status_DISABLE);
      Motor_Pitch_Small.Set_Control_Status(Motor_DM_Status_DISABLE);

      if (Motor_Pitch_Large.Get_Now_Control_Status() != Motor_DM_Status_DISABLE)
        Motor_Pitch_Large.CAN_Send_Exit();
      if (Motor_Pitch_Small.Get_Now_Control_Status() != Motor_DM_Status_DISABLE)
        Motor_Pitch_Small.CAN_Send_Exit();

      // 掉线: 同步当前下发角度到真实角度防切回突变;中止段与序列
      current_pitch_large_angle = Motor_Pitch_Large.Get_Now_Radian();
      current_pitch_small_angle = Motor_Pitch_Small.Get_Now_Radian();
      segment_active_ = false;
      segment_t_ = 0.0f;
      sequence_state_ = CLAMP_SEQ_IDLE;
      sequence_tick_ = 0;
      return;
    }

    case CLAMP_CONTROL_ENABLE: {
      // 推进取放序列:r1.2 UP-档 4 步流程,末态停在 POS1
      _Step_Sequence();

      // 解析当前 POS state → 关节角目标;若与当前段终点不一致(外部 Set_Pitch_*
      // 或序列切步)需要重新规划一段
      float q1_target = 0.0f, q2_target = 0.0f;
      _Resolve_Pos_Targets(q1_target, q2_target);

      const float kEps = 1e-4f;
      const bool target_changed =
          !segment_active_ ||
          std::fabs(q1_target - segment_q1_end_) > kEps ||
          std::fabs(q2_target - segment_q2_end_) > kEps;
      if (target_changed) {
        const Enum_Clamp_Interp_Mode mode = _Pick_Interp_Mode(
            current_pitch_large_angle, current_pitch_small_angle, q1_target,
            q2_target);
        _Start_Segment(q1_target, q2_target, mode);
      }

      // 按段时钟推进 setpoint
      if (segment_active_) {
        segment_t_ += kStepDt;
        const float s = segment_profile_.Sample(segment_t_);
        if (segment_mode_ == CLAMP_INTERP_PTP) {
          current_pitch_large_angle =
              segment_q1_start_ + s * (segment_q1_end_ - segment_q1_start_);
          current_pitch_small_angle =
              segment_q2_start_ + s * (segment_q2_end_ - segment_q2_start_);
        } else {  // LIN
          const float x =
              segment_x_start_ + s * (segment_x_end_ - segment_x_start_);
          const float y =
              segment_y_start_ + s * (segment_y_end_ - segment_y_start_);
          float q1_ik = 0.0f, q2_ik = 0.0f;
          // unwrap 参考用"上 cycle 下发的 setpoint",保持每步连续
          if (IK_2DoF(x, y, current_pitch_large_angle,
                      current_pitch_small_angle, q1_ik, q2_ik)) {
            current_pitch_large_angle = q1_ik;
            current_pitch_small_angle = q2_ik;
          }
          // IK 失败 (理论上不该发生,起止点已校验过) → 保持上一 cycle setpoint
        }
        if (segment_profile_.Done(segment_t_)) {
          segment_active_ = false;
          // 段末态精确钉到关节空间目标 (避免 LIN 数值累积漂移)
          current_pitch_large_angle = segment_q1_end_;
          current_pitch_small_angle = segment_q2_end_;
        }
      }

      // 下发当前 setpoint + 重力补偿前馈
      // τ_ff 用 MIT 三参接口 (torque, angle, omega), 不动 Kp/Kd (Init 时已设)
      // q1=0 (水平向后) 时 cos(q1)=1 给最大上抬扭矩; q1=π/2 顶上时 cos(q1)=0
      // sweep 模式下用候选 K 覆盖, 否则用用户设置的 K
      static constexpr float kSweepK[] = {0.0f, 0.5f, 1.0f, 1.5f, 2.0f,
                                          2.5f, 3.0f, 3.5f};
      static constexpr int kSweepN = sizeof(kSweepK) / sizeof(kSweepK[0]);
      float K1_eff = pitch_large_grav_K;
      float K2_eff = pitch_small_grav_K;
      const bool in_k1_sweep = (kCalibMode == CALIB_K1_SWEEP) && !sweep_done_ &&
                                sequence_state_ == CLAMP_SEQ_IDLE &&
                                !segment_active_;
      const bool in_k2_sweep = (kCalibMode == CALIB_K2_SWEEP) && !sweep_done_ &&
                                sequence_state_ == CLAMP_SEQ_IDLE &&
                                !segment_active_;
      if (in_k1_sweep && sweep_idx_ < (uint32_t)kSweepN) {
        K1_eff = kSweepK[sweep_idx_];
      }
      if (in_k2_sweep && sweep_idx_ < (uint32_t)kSweepN) {
        K2_eff = kSweepK[sweep_idx_];
      }
      const float tau1_ff = K1_eff * std::cos(current_pitch_large_angle);
      const float tau2_ff = K2_eff * std::cos(current_pitch_large_angle +
                                              current_pitch_small_angle);

      if (Motor_Pitch_Large.Get_Status() != Motor_DM_Control_Status_ENABLE)
        Motor_Pitch_Large.CAN_Send_Enter();
      else
        Motor_Pitch_Large.Set_Control_Parameter_MIT(
            tau1_ff, current_pitch_large_angle, 0.0f);
      Motor_Pitch_Large.TIM_Send_PeriodElapsedCallback();

      if (Motor_Pitch_Small.Get_Status() != Motor_DM_Control_Status_ENABLE)
        Motor_Pitch_Small.CAN_Send_Enter();
      else
        Motor_Pitch_Small.Set_Control_Parameter_MIT(
            tau2_ff, current_pitch_small_angle, 0.0f);
      Motor_Pitch_Small.TIM_Send_PeriodElapsedCallback();

      // CSV trace (env CLAMP_TRACE_FILE 触发, 否则 no-op) — 500Hz 全程数据
      g_trace.Log(current_pitch_large_angle,
                  Motor_Pitch_Large.Get_Now_Radian(),
                  current_pitch_small_angle,
                  Motor_Pitch_Small.Get_Now_Radian(),
                  Motor_Pitch_Large.Get_Now_Omega(),
                  Motor_Pitch_Small.Get_Now_Omega(),
                  Motor_Pitch_Large.Get_Now_Torque(),
                  Motor_Pitch_Small.Get_Now_Torque(),
                  tau1_ff, tau2_ff,
                  (uint8_t)sequence_state_,
                  segment_active_ ? 1 : 0);

      // === Calib 模式数据采集与扫描 ===
      if (kCalibMode != CALIB_OFF &&
          sequence_state_ == CLAMP_SEQ_IDLE && !segment_active_) {
        // 安全: q1 跑出 ±0.5rad 立即停 sweep
        const float q1_ac = Motor_Pitch_Large.Get_Now_Radian();
        const float q2_ac = Motor_Pitch_Small.Get_Now_Radian();
        if ((in_k1_sweep || in_k2_sweep) && std::fabs(q1_ac) > 0.5f) {
          std::printf("[CLAMP SWEEP] ABORT: q1=%.3f out of safety bound\n",
                      q1_ac);
          std::fflush(stdout);
          sweep_done_ = true;
        }

        if (kCalibMode == CALIB_PRINT_STATIC) {
          if (++calib_print_tick_ >= 500) {
            calib_print_tick_ = 0;
            const float dq1 = current_pitch_large_angle - q1_ac;
            const float dq2 = current_pitch_small_angle - q2_ac;
            const float c1 = std::cos(q1_ac);
            const float c2 = std::cos(q1_ac + q2_ac);
            const float K1_rec = (std::fabs(c1) > 0.1f)
                ? (pitch_large_grav_K + pitch_large_kp * dq1 / c1)
                : 0.0f;
            const float K2_rec = (std::fabs(c2) > 0.1f)
                ? (pitch_small_grav_K + pitch_small_kp * dq2 / c2)
                : 0.0f;
            std::printf(
                "[CLAMP CALIB] q1: sp=%+.3f act=%+.3f d=%+.4f (%+.2fdeg) "
                "| q2: sp=%+.3f act=%+.3f d=%+.4f (%+.2fdeg) "
                "| K_now=(%.3f,%.3f) K_rec=(%.3f,%.3f)\n",
                current_pitch_large_angle, q1_ac, dq1, dq1 * 57.2958f,
                current_pitch_small_angle, q2_ac, dq2, dq2 * 57.2958f,
                pitch_large_grav_K, pitch_small_grav_K, K1_rec, K2_rec);
            std::fflush(stdout);
          }
        } else if (in_k1_sweep || in_k2_sweep) {
          // sweep 节奏: 每档 1500 ticks (3s), 前 1000 ticks 稳定, 后 500 采样
          static constexpr uint32_t kPerStep = 1500U;
          static constexpr uint32_t kSettle = 1000U;
          if (sweep_tick_ == 0) {
            std::printf(
                "[CLAMP SWEEP %s] idx=%u/%d K=%.2f (waiting 2s settle...)\n",
                in_k1_sweep ? "K1" : "K2", sweep_idx_, kSweepN,
                kSweepK[sweep_idx_]);
            std::fflush(stdout);
          }
          sweep_tick_++;
          if (sweep_tick_ >= kSettle) {
            const float q_now = in_k1_sweep ? q1_ac : q2_ac;
            sweep_q_acc_ += q_now;
            sweep_q_cnt_++;
          }
          if (sweep_tick_ >= kPerStep) {
            const float q_mean = sweep_q_acc_ / std::max(1U, sweep_q_cnt_);
            sweep_results_[sweep_idx_] = q_mean;
            std::printf("[CLAMP SWEEP %s] K=%.2f → q_mean=%+.4f rad (%+.2fdeg)\n",
                        in_k1_sweep ? "K1" : "K2",
                        kSweepK[sweep_idx_], q_mean, q_mean * 57.2958f);
            std::fflush(stdout);
            sweep_idx_++;
            sweep_tick_ = 0;
            sweep_q_acc_ = 0.0f;
            sweep_q_cnt_ = 0;
            if (sweep_idx_ >= (uint32_t)kSweepN) {
              sweep_done_ = true;
              // 找 q 跨过 0 的相邻两档线性插值
              float k_rec = -1.0f;
              for (int i = 1; i < kSweepN; i++) {
                if (sweep_results_[i - 1] < 0.0f &&
                    sweep_results_[i] > 0.0f) {
                  const float q0 = sweep_results_[i - 1];
                  const float q1v = sweep_results_[i];
                  const float k0 = kSweepK[i - 1];
                  const float k1 = kSweepK[i];
                  k_rec = k0 + (0.0f - q0) * (k1 - k0) / (q1v - q0);
                  break;
                }
              }
              std::printf("[CLAMP SWEEP %s] === DONE === Recommended K=%.3f Nm\n",
                          in_k1_sweep ? "K1" : "K2", k_rec);
              std::printf("[CLAMP SWEEP] Full table:\n");
              for (int i = 0; i < kSweepN; i++) {
                std::printf("  K=%.2f → q_steady=%+.4f rad (%+.2fdeg)\n",
                            kSweepK[i], sweep_results_[i],
                            sweep_results_[i] * 57.2958f);
              }
              std::fflush(stdout);
            }
          }
        }
      } else {
        calib_print_tick_ = 0;
      }
      break;
    }
  }
}

void Class_Clamp::_Resolve_Pos_Targets(float& q1_target,
                                       float& q2_target) const {
  switch (current_pitch_large_state) {
    case L_PITCH_POS1: q1_target = pitch_large_pos1_angle; break;
    case L_PITCH_POS2: q1_target = pitch_large_pos2_angle; break;
    case L_PITCH_POS_FOLDED: q1_target = pitch_large_folded_angle; break;
  }
  switch (current_pitch_small_state) {
    case S_PITCH_POS1: q2_target = pitch_small_pos1_angle; break;
    case S_PITCH_POS2: q2_target = pitch_small_pos2_angle; break;
    case S_PITCH_POS_FOLDED: q2_target = pitch_small_folded_angle; break;
  }
}

// 启动一段双轴归一化同步规划 (PTP) 或笛卡尔直线规划 (LIN):
//   归一化 s ∈ [0, 1], v_norm = min(v_i_max / |Δqi|), a_norm = min(a_i_max / |Δqi|)
//   两轴共享同一 profile,同起同止
// LIN 模式仍按"关节空间总位移"反算 v_norm/a_norm,这是保守上限 (直线段中点
// 关节速度通常 ≤ 端点);避免每 cycle 做 Jacobian 全段扫描。
// Δq 全为 0 时不激活段 (避免除零和无意义规划)。
void Class_Clamp::_Start_Segment(float q1_target, float q2_target,
                                 Enum_Clamp_Interp_Mode mode) {
  segment_mode_ = mode;
  segment_q1_start_ = current_pitch_large_angle;
  segment_q2_start_ = current_pitch_small_angle;
  segment_q1_end_ = q1_target;
  segment_q2_end_ = q2_target;

  const float dq1 = std::fabs(q1_target - segment_q1_start_);
  const float dq2 = std::fabs(q2_target - segment_q2_start_);

  const float kMinMove = 1e-4f;
  if (dq1 < kMinMove && dq2 < kMinMove) {
    segment_active_ = false;
    segment_t_ = 0.0f;
    current_pitch_large_angle = q1_target;
    current_pitch_small_angle = q2_target;
    return;
  }

  if (mode == CLAMP_INTERP_LIN) {
    // 笛卡尔端点 + IK 反向校验:目标 q 应能由 IK 复现 (允许 0.05 rad 误差,
    // 因 POS 标定 -4.2 vs IK 单圈表示 +2.083 经 unwrap 后等价但有舍入)
    FK_2DoF(segment_q1_start_, segment_q2_start_, segment_x_start_,
            segment_y_start_);
    FK_2DoF(segment_q1_end_, segment_q2_end_, segment_x_end_, segment_y_end_);
    float q1_check = 0.0f, q2_check = 0.0f;
    const bool ik_ok = IK_2DoF(segment_x_end_, segment_y_end_,
                               segment_q1_start_, segment_q2_start_, q1_check,
                               q2_check);
    if (!ik_ok) {
      // 工作空间外 → 安全回退到 PTP
      mode = CLAMP_INTERP_PTP;
      segment_mode_ = CLAMP_INTERP_PTP;
    }
  }

  // 仅活动轴参与限速反算;另一轴视为不约束
  float v_norm = 1e9f, a_norm = 1e9f;
  if (dq1 >= kMinMove) {
    v_norm = std::fmin(v_norm, max_speed_pitch_large / dq1);
    a_norm = std::fmin(a_norm, max_accel_pitch_large / dq1);
  }
  if (dq2 >= kMinMove) {
    v_norm = std::fmin(v_norm, max_speed_pitch_small / dq2);
    a_norm = std::fmin(a_norm, max_accel_pitch_small / dq2);
  }

  segment_profile_.Plan(1.0f, v_norm * next_segment_speed_scale_,
                        a_norm * next_segment_speed_scale_,
                        std::fmin(next_segment_v0_norm_,
                                  v_norm * next_segment_speed_scale_));
  // v0 > v_max 时强制 clamp: 不 clamp 会让 Plan 算出负 t_acc, Sample(0) 返回
  // 负 s 即 setpoint 反向跳变 (抽搐 bug). 代价: 段开头有关节速度小跳 (~0.3rad/s),
  // 但比 30° setpoint 跳好太多
  next_segment_v0_norm_ = 0.0f;       // 用完即清, 默认下段从 0 起
  next_segment_speed_scale_ = 1.0f;   // 用完即清, 默认下段满速
  segment_t_ = 0.0f;
  segment_active_ = true;
}

// 段插值模式选择 (基于起止 POS 对应的笛卡尔位置与 elbow 选支):
//   1. 起止两点连线若经过肩部附近 (距原点 < 13cm) → PTP 避免穿肩
//   2. 起止跨原点 (x 异号) → PTP 同上
//   3. 起止 elbow 支不一致 (q2 mod 2π 落在 (0,π) vs (-π,0)) → PTP
//      否则 LIN 中途必穿过支切换边界, setpoint 突变
//   其它 → LIN 末端走直线
Enum_Clamp_Interp_Mode Class_Clamp::_Pick_Interp_Mode(float q1_from,
                                                      float q2_from,
                                                      float q1_to,
                                                      float q2_to) const {
  float x_from = 0.0f, y_from = 0.0f, x_to = 0.0f, y_to = 0.0f;
  FK_2DoF(q1_from, q2_from, x_from, y_from);
  FK_2DoF(q1_to, q2_to, x_to, y_to);

  // POS1 ↔ POS2 都会穿过 (0, 0) 附近 → PTP; POS4 ↔ POS1 横跨大跨度 → PTP
  const bool cross_origin = (x_from * x_to < 0.0f);
  const float kNearShoulder = 0.13f;  // m
  const bool near_shoulder =
      (std::hypot(x_from, y_from) < kNearShoulder) ||
      (std::hypot(x_to, y_to) < kNearShoulder);
  if (cross_origin || near_shoulder) return CLAMP_INTERP_PTP;

  // elbow 支判定: q2 mod 2π 化到 (-π, π],sign>0 = elbow_up,sign<0 = elbow_down
  // POS2 q2=-4.2 → +2.08 = elbow_up; POS4 q2=-2.0 → -2.0 = elbow_down
  const float kTwoPi = 2.0f * static_cast<float>(M_PI);
  const float q2_from_wrap = std::remainder(q2_from, kTwoPi);
  const float q2_to_wrap = std::remainder(q2_to, kTwoPi);
  if (q2_from_wrap * q2_to_wrap < 0.0f) return CLAMP_INTERP_PTP;

  return CLAMP_INTERP_LIN;
}

// === FK / IK 实现 ===
void Class_Clamp::FK_2DoF(float q1, float q2, float& x, float& y) const {
  // 零位 (q1=0,q2=0) 大臂指 -x;q1 增大 → 大臂顺时针绕到上/前
  // 等价表达: 用标准 2-link FK 配 (-x, y)
  x = -kL1 * std::cos(q1) - kL2 * std::cos(q1 + q2);
  y = kL1 * std::sin(q1) + kL2 * std::sin(q1 + q2);
}

bool Class_Clamp::IK_2DoF(float x, float y, float q1_ref, float q2_ref,
                          float& q1, float& q2) const {
  const float xp = -x;  // 转为"零位指 +x"的标准坐标
  const float r2 = xp * xp + y * y;
  const float L1L2 = kL1 + kL2;
  const float dL = std::fabs(kL1 - kL2);
  // 工作空间检查 (留 1mm 余量防边界舍入)
  if (r2 > (L1L2 - 1e-3f) * (L1L2 - 1e-3f) ||
      r2 < (dL + 1e-3f) * (dL + 1e-3f)) {
    return false;
  }

  float c2 = (r2 - kL1 * kL1 - kL2 * kL2) / (2.0f * kL1 * kL2);
  if (c2 > 1.0f) c2 = 1.0f;
  if (c2 < -1.0f) c2 = -1.0f;

  // 两支候选: elbow_up (q2 ∈ (0, π)) / elbow_down (q2 ∈ (-π, 0))。
  // POS2/POS3 标定 q2=-4.2/-3.5 等价单圈 +2.08/+2.78 即 elbow_up;
  // POS4 q2=-2.0 即 elbow_down。 同一 POS 表跨两支,IK 必须按 q_ref 自动选支。
  const float kTwoPi = 2.0f * static_cast<float>(M_PI);
  const float q2_up = +std::acos(c2);
  const float q2_dn = -std::acos(c2);
  auto solve_q1 = [&](float q2_raw) {
    const float s2 = std::sin(q2_raw);
    const float c2v = std::cos(q2_raw);
    return std::atan2(y, xp) - std::atan2(kL2 * s2, kL1 + kL2 * c2v);
  };
  const float q1_up = solve_q1(q2_up);
  const float q1_dn = solve_q1(q2_dn);

  // 距离用 2π wrap 后的最短差 (remainder ∈ (-π, π])
  auto wrap_dist = [&](float a, float b) {
    return std::fabs(std::remainder(a - b, kTwoPi));
  };
  const float d_up = wrap_dist(q1_up, q1_ref) + wrap_dist(q2_up, q2_ref);
  const float d_dn = wrap_dist(q1_dn, q1_ref) + wrap_dist(q2_dn, q2_ref);
  const float q1_raw = (d_up <= d_dn) ? q1_up : q1_dn;
  const float q2_raw = (d_up <= d_dn) ? q2_up : q2_dn;

  // unwrap 到参考圈 (DM 电机虽接受 ±PMAX=12.5 rad,但跨 2π 会让 setpoint 跳支)
  const float n1 = std::round((q1_ref - q1_raw) / kTwoPi);
  const float n2 = std::round((q2_ref - q2_raw) / kTwoPi);
  q1 = q1_raw + n1 * kTwoPi;
  q2 = q2_raw + n2 * kTwoPi;
  return true;
}

void Class_Clamp::Trigger_Pick_Place_Sequence() {
  if (Clamp_Control_Type != CLAMP_CONTROL_ENABLE) return;
  if (sequence_state_ != CLAMP_SEQ_IDLE) return;

  sequence_state_ = CLAMP_SEQ_STEP1;
  sequence_tick_ = 0;
  // 位置保持当前 (启动时应为 POS1),所有切换都放到 _Step_Sequence
  // 的状态转换里统一处理
}

// 2ms 节拍推进序列:
//   STEP1 (按 A 立即) / STEP2 (到 POS2 等气夹爪闭合) 用固定 dwell tick;
//   STEP3 (POS_FOLDED → POS1) 用"段进入减速即切" + blend, 全程无 0 速点。
//   抓后两段 (POS2→POS_FOLDED, POS_FOLDED→POS1) 共用 speed_scale=0.5,
//   blend 时 v0=v_max_new 完美对齐, 速度连续无跳变, 整体慢稳。
void Class_Clamp::_Step_Sequence() {
  if (sequence_state_ == CLAMP_SEQ_IDLE) return;

  constexpr uint32_t kStep1Dwell = 0U;     //  按 A 立刻起动, 无启动延迟
  constexpr uint32_t kStep2Dwell = 1500U;  // 1500ms (到 POS2 后留抓取时间)
  constexpr float kPostGraspScale = 0.5f;  // 抓后两段共用速度缩放, 慢稳

  bool advance = false;
  switch (sequence_state_) {
    case CLAMP_SEQ_STEP1:
      advance = (++sequence_tick_ >= kStep1Dwell);
      break;
    case CLAMP_SEQ_STEP2:
      advance = (++sequence_tick_ >= kStep2Dwell);
      break;
    case CLAMP_SEQ_STEP3:
      // 进入减速即切 + 记录 v0 给下段 blend
      advance = !segment_active_ ||
                segment_profile_.In_Deceleration(segment_t_);
      break;
    default:
      sequence_state_ = CLAMP_SEQ_IDLE;
      return;
  }

  if (!advance) return;
  sequence_tick_ = 0;

  // blend 衔接: 若当前段还在执行(未 done), 把当前归一化速度传给下段做初速度
  if (segment_active_ && !segment_profile_.Done(segment_t_)) {
    next_segment_v0_norm_ = segment_profile_.Sample_Velocity(segment_t_);
  } else {
    next_segment_v0_norm_ = 0.0f;
  }

  switch (sequence_state_) {
    case CLAMP_SEQ_STEP1:
      // POS1 → POS2 抓取段: 满速 (用户没抱怨这段速度)
      current_pitch_large_state = L_PITCH_POS2;
      current_pitch_small_state = S_PITCH_POS2;
      sequence_state_ = CLAMP_SEQ_STEP2;
      break;
    case CLAMP_SEQ_STEP2:
      // POS2 → POS_FOLDED 段: 半速 (跟末段同 scale, 下面 blend 时 v 对齐)
      next_segment_speed_scale_ = kPostGraspScale;
      current_pitch_large_state = L_PITCH_POS_FOLDED;
      current_pitch_small_state = S_PITCH_POS_FOLDED;
      sequence_state_ = CLAMP_SEQ_STEP3;
      break;
    case CLAMP_SEQ_STEP3:
      // POS_FOLDED → POS1 末段: 同 scale → v0=v_max_new 无速度跳变, 平稳衔接
      next_segment_speed_scale_ = kPostGraspScale;
      current_pitch_large_state = L_PITCH_POS1;
      current_pitch_small_state = S_PITCH_POS1;
      sequence_state_ = CLAMP_SEQ_IDLE;
      break;
    default:
      sequence_state_ = CLAMP_SEQ_IDLE;
      break;
  }
}

// 把当前 gripper_byte0_ 透传到 ch_/id=0x13 (整个 8B payload 其余为 0)
// LinkX 固件无 valid bit,slot 内容每 cycle 自动重发到 CAN; 上层需按节拍持续调用
// 维持状态信号 (不能用"停 push 让总线安静",一停就没法表达 byte0 的当前值)
void Class_Clamp::Push_Gripper_Frame() {
  if (gripper_linkx_handler_ == nullptr) return;
  uint8_t tx[8] = {0};
  tx[0] = gripper_byte0_;
  linkx_quick_can_send(gripper_linkx_handler_, gripper_can_channel_,
                       kGripperCanId, tx);
}
