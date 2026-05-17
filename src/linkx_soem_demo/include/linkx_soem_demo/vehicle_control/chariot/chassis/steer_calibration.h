//
// steer_calibration.h - 舵向校准独立模块
//
// 从 crt_chassis 抽离,实现"MIT 位置环 + slew 限速 + done 滞回 + 持续发命令"
// (F 方案,见 plans/sprightly-waddling-yao.md)。
//
// 状态机:WAIT_STABLE -> CALCULATE -> EXECUTING -> DONE (或 FAIL)
// 时序: Executing 收敛 -> Done(100/200/500/800 tick = 200/400/1000/1600 ms)
//        100: 阻尼锁位 | 200: CAN_Send_Exit | 500: CAN_Send_Save_Zero | 800: all_complete=1
//
// 资源访问:Bind() 一次性绑定 Chassis 拥有的 Motor/Encoder/Params 数组。
//

#ifndef LINKX_SOEM_DEMO_STEER_CALIBRATION_H
#define LINKX_SOEM_DEMO_STEER_CALIBRATION_H

#include <cstdint>

#include "dvc_encoder.h"
#include "dvc_motor_dm.h"

#ifndef STEER_NUM
#define STEER_NUM 4
#endif

enum Enum_Calib_State {
  CALIB_STATE_WAIT_STABLE = 0,
  CALIB_STATE_CALCULATE,
  CALIB_STATE_EXECUTING,
  CALIB_STATE_DONE,
  CALIB_STATE_FAIL  // fail_reason: 1=等待编码器超时, 2=执行超时
};

struct SteerWheelParams;  // 定义在 crt_chassis.h,cpp 内 include 后访问 mit_kp/mit_kd

class Class_Steer_Calibration {
 public:
  static constexpr uint32_t CALIB_TICK_MS = 2;
  static constexpr uint32_t CALIB_WAIT_STABLE_MIN_MS = 2000;
  static constexpr uint32_t CALIB_WAIT_STABLE_MAX_MS = 4000;
  static constexpr uint32_t CALIB_EXEC_TIMEOUT_MS = 20000;

  void Bind(Class_Motor_DM_Normal *motors,
            Class_Encoder_BRT *encoders,
            const SteerWheelParams *params,
            float reduction_ratio);

  void Init();
  // 0 = 进行中(含 idle/Already complete);1 = 本帧刚完成;2 = 本帧刚失败
  uint8_t Tick();
  bool Is_Complete() const { return all_complete_ != 0; }
  bool Is_Wheel_Done(int i) const;
  void Force_Recalibration();

 private:
  void Wait_Stable();
  void Calculate();
  void Print_Phase_Offsets();
  void Executing();
  void Fail();
  uint8_t Done();

  // 资源绑定
  Class_Motor_DM_Normal *motors_ = nullptr;
  Class_Encoder_BRT *encoders_ = nullptr;
  const SteerWheelParams *params_ = nullptr;
  float reduction_ratio_ = 1.0f;

  // 状态机
  Enum_Calib_State step_ = CALIB_STATE_WAIT_STABLE;
  uint8_t all_complete_ = 0;
  uint32_t wait_tick_ = 0;
  uint32_t exec_tick_ = 0;
  uint8_t fail_reason_ = 0;
  uint8_t calib_done_[STEER_NUM] = {0};

  // F 方案:slew + done 滞回
  float motor_target_slew_[STEER_NUM] = {0};
  bool slew_init_[STEER_NUM] = {false};
  uint32_t done_hold_count_[STEER_NUM] = {0};

  // 诊断
  float prev_enc_rad_[STEER_NUM] = {0};
  uint32_t no_motion_tick_[STEER_NUM] = {0};
};

#endif
