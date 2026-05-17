//
// steer_calibration.cpp
//
// F 方案实现:MIT 位置环 + slew + done 滞回 + 持续发命令。
// 行为兼容原 Calib_* 状态机(WAIT_STABLE/CALCULATE/EXECUTING/DONE/FAIL),
// 仅 Executing 重写,Done 时序保持 100/200/500/800 tick 不变。
//

#include "steer_calibration.h"

#include <cmath>
#include <iomanip>
#include <iostream>

#include "crt_chassis.h"  // SteerWheelParams 完整定义

namespace {
constexpr float PI_F = 3.14159265358979f;
}

void Class_Steer_Calibration::Bind(Class_Motor_DM_Normal *motors,
                                   Class_Encoder_BRT *encoders,
                                   const SteerWheelParams *params,
                                   float reduction_ratio)
{
  motors_ = motors;
  encoders_ = encoders;
  params_ = params;
  reduction_ratio_ = reduction_ratio;
}

void Class_Steer_Calibration::Init()
{
  for (int i = 0; i < STEER_NUM; i++)
  {
    motors_[i].Set_Control_Torque_P_D_MIT(0.0f, 0.0f);
  }
  step_ = CALIB_STATE_WAIT_STABLE;
  all_complete_ = 0;
  wait_tick_ = 0;
  exec_tick_ = 0;
  fail_reason_ = 0;
  for (int i = 0; i < STEER_NUM; i++)
  {
    calib_done_[i] = 0;
    motor_target_slew_[i] = 0.0f;
    slew_init_[i] = false;
    done_hold_count_[i] = 0;
    prev_enc_rad_[i] = 0.0f;
    no_motion_tick_[i] = 0;
  }
}

bool Class_Steer_Calibration::Is_Wheel_Done(int i) const
{
  if (i < 0 || i >= STEER_NUM) return false;
  return calib_done_[i] != 0;
}

void Class_Steer_Calibration::Force_Recalibration()
{
  all_complete_ = 0;
  wait_tick_ = 0;
  step_ = CALIB_STATE_WAIT_STABLE;
}

uint8_t Class_Steer_Calibration::Tick()
{
  if (all_complete_) return 0;
  const uint8_t was_failed = (step_ == CALIB_STATE_FAIL) ? 1 : 0;

  switch (step_)
  {
  case CALIB_STATE_WAIT_STABLE: Wait_Stable(); break;
  case CALIB_STATE_CALCULATE:   Calculate(); break;
  case CALIB_STATE_EXECUTING:   Executing(); break;
  case CALIB_STATE_FAIL:        Fail(); break;
  case CALIB_STATE_DONE:        Done(); break;
  }

  if (all_complete_) return 1;
  if (!was_failed && step_ == CALIB_STATE_FAIL) return 2;
  return 0;
}

void Class_Steer_Calibration::Wait_Stable()
{
  for (int i = 0; i < STEER_NUM; i++)
    encoders_[i].TIM_Query_PeriodElapsedCallback();

  ++wait_tick_;

  if (wait_tick_ > (CALIB_WAIT_STABLE_MAX_MS / CALIB_TICK_MS))
  {
    fail_reason_ = 1;
    step_ = CALIB_STATE_FAIL;
    return;
  }

  if (wait_tick_ > (CALIB_WAIT_STABLE_MIN_MS / CALIB_TICK_MS))
  {
    uint8_t data_ready = 1;
    for (int i = 0; i < STEER_NUM; i++)
      if (!encoders_[i].Has_Valid_Wheel_Posture())
      {
        data_ready = 0;
        break;
      }

    if (data_ready) step_ = CALIB_STATE_CALCULATE;
  }
}

void Class_Steer_Calibration::Calculate()
{
  Print_Phase_Offsets();
  step_ = CALIB_STATE_EXECUTING;
}

void Class_Steer_Calibration::Print_Phase_Offsets()
{
  for (int i = 0; i < STEER_NUM; i++)
  {
    float phase_rad = encoders_[i].Get_Wheel_Posture_radian_True();
    float relative_rad = phase_rad;
    if (relative_rad > PI_F) relative_rad -= 2.0f * PI_F;
    calib_done_[i] = 0;
    std::cout << "[CALIB] wheel[" << i << "] phase offset: "
              << std::fixed << std::setprecision(3)
              << relative_rad << " rad ("
              << (relative_rad * 180.0f / PI_F) << " deg, "
              << (relative_rad / (2.0f * PI_F)) << " turns)" << std::endl;
  }
}

void Class_Steer_Calibration::Executing()
{
  // === F 方案常量 ===
  constexpr float MAX_CALIB_OMEGA_STEER = 3.0f;   // 舵向侧 ω 限速 [rad/s]
  constexpr float CALIB_DONE_THRESHOLD  = 0.03f;  // 收敛阈 ≈ 1.7°
  constexpr uint32_t DONE_HOLD_TICKS    = 25;     // 滞回 25 tick = 50ms
  constexpr float DT = static_cast<float>(CALIB_TICK_MS) * 0.001f;

  exec_tick_++;
  if (exec_tick_ > (CALIB_EXEC_TIMEOUT_MS / CALIB_TICK_MS))
  {
    fail_reason_ = 2;
    step_ = CALIB_STATE_FAIL;
    return;
  }

  const float slew_step_motor = MAX_CALIB_OMEGA_STEER * reduction_ratio_ * DT;

  uint8_t all_ready = 1;
  for (int i = 0; i < STEER_NUM; i++)
  {
    encoders_[i].TIM_Query_PeriodElapsedCallback();

    // 相位最短路径误差 [-pi, pi)
    float phase_rad = encoders_[i].Get_Wheel_Posture_radian_True();
    float error = phase_rad;
    if (error > PI_F) error -= 2.0f * PI_F;

    // Slew 首帧初始化:锁存当前 motor pos 作为 slew 起点
    if (!slew_init_[i])
    {
      motor_target_slew_[i] = motors_[i].Get_Now_Radian();
      slew_init_[i] = true;
    }

    // 最终目标(基于 phase 最短路径):motor_now - error × REDUCTION
    // (等价 Chassis::Steer_To_Motor_Position(0, i) 的展开)
    float final_target = motors_[i].Get_Now_Radian() - error * reduction_ratio_;

    // Slew 限步
    float delta = final_target - motor_target_slew_[i];
    if (delta >  slew_step_motor) delta =  slew_step_motor;
    if (delta < -slew_step_motor) delta = -slew_step_motor;
    motor_target_slew_[i] += delta;

    // ω 前馈固定为 0:让 Kd 项纯做阻尼。
    // steer_tuning 验证 (2026-05-13): Kp=50/Kd=1.5/ω_des=0 在 180° 大 delta 下
    // 1450ms settled、零过冲。ω_des 非零会推 Kd 项加速电机 → 惯性过冲风险
    // (见 [[feedback_steer_calib_mit_position_failure]] 上次集成失败根因)。
    const float omega_motor_ff = 0.0f;

    // 持续发 MIT 命令(不区分 done — 防止 done 后失约束飘走)
    motors_[i].Set_Control_Torque_P_D_MIT(
      0.0f, params_[i].mit_kp, params_[i].mit_kd);
    motors_[i].Set_Control_Parameter_MIT(motor_target_slew_[i], omega_motor_ff);

    // Done 滞回判定
    if (!calib_done_[i])
    {
      if (fabsf(error) < CALIB_DONE_THRESHOLD)
      {
        done_hold_count_[i]++;
        if (done_hold_count_[i] >= DONE_HOLD_TICKS)
          calib_done_[i] = 1;
        else
          all_ready = 0;
      }
      else
      {
        done_hold_count_[i] = 0;
        all_ready = 0;
      }
    }

    // 静止诊断(每 200ms 采样一次)
    if (!calib_done_[i] && (exec_tick_ % 100 == 0))
    {
      float enc_delta = fabsf(error - prev_enc_rad_[i]);
      if (enc_delta < 0.005f)
      {
        no_motion_tick_[i]++;
        std::cout << "[CALIB] wheel[" << i << "] no-motion x" << no_motion_tick_[i]
                  << " error=" << error
                  << " omega_ff=" << omega_motor_ff << std::endl;
      }
      else
      {
        no_motion_tick_[i] = 0;
      }
      prev_enc_rad_[i] = error;
    }
  }

  // 周期日志
  if (exec_tick_ % 50 == 0)
  {
    std::cout << "[CALIB] EXEC tick=" << exec_tick_;
    for (int i = 0; i < STEER_NUM; i++)
    {
      float phase_rad = encoders_[i].Get_Wheel_Posture_radian_True();
      float err = phase_rad;
      if (err > PI_F) err -= 2.0f * PI_F;
      std::cout << " | w[" << i << "] err="
                << std::fixed << std::setprecision(3) << err
                << " done=" << (int)calib_done_[i]
                << " hold=" << done_hold_count_[i];
    }
    std::cout << std::defaultfloat << std::endl;
  }

  if (all_ready)
  {
    step_ = CALIB_STATE_DONE;
    wait_tick_ = 0;
  }
}

void Class_Steer_Calibration::Fail()
{
  for (int i = 0; i < STEER_NUM; i++)
  {
    motors_[i].Set_Control_Torque_P_D_MIT(0.0f, 0.0f);
    motors_[i].Set_Control_Parameter_MIT(0.0f, 0.0f, 0.0f);
    motors_[i].CAN_Send_Exit();
  }
  static bool fail_printed = false;
  if (!fail_printed)
  {
    std::cout << "[CALIB] FAIL reason=" << (int)fail_reason_
              << " (1=enc_timeout 2=exec_timeout)"
              << " wait_tick=" << wait_tick_
              << " exec_tick=" << exec_tick_ << std::endl;
    for (int i = 0; i < STEER_NUM; i++)
      std::cout << "  wheel[" << i << "] enc_rad="
                << encoders_[i].Get_Wheel_Posture_radian_True()
                << " enc_valid=" << (int)encoders_[i].Has_Valid_Wheel_Posture()
                << " rx_count=" << encoders_[i].Get_Rx_Count()
                << " done=" << (int)calib_done_[i] << std::endl;
    fail_printed = true;
  }
}

uint8_t Class_Steer_Calibration::Done()
{
  wait_tick_++;
  if (wait_tick_ == 100)
  {
    for (int i = 0; i < STEER_NUM; i++)
      motors_[i].Set_Control_Torque_P_D_MIT(0.0f, 1.0f);
  }
  else if (wait_tick_ == 200)
  {
    for (int i = 0; i < STEER_NUM; i++) motors_[i].CAN_Send_Exit();
  }
  else if (wait_tick_ == 500)
  {
    for (int i = 0; i < STEER_NUM; i++) motors_[i].CAN_Send_Save_Zero();
  }
  else if (wait_tick_ == 800)
  {
    all_complete_ = 1;
    return 1;
  }
  return 0;
}
