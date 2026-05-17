//
// SteerTrace - 500Hz CSV trace 封装
//
// 以前 chassis 内部 static ofstream + static bool init/disabled，行为不变迁移到这里。
//
// 用法：chassis 持有一个 SteerTrace 成员，每 2ms 控制周期调一次 LogTick()。
// env STEER_TRACE_FILE 指定输出 CSV 路径；未设 → 永久 noop；打开失败 → 永久 noop。
//
// CSV 列定义：
//   ts_ms, vx, vy, omega, [t_deg_i, c_deg_i, mt_rad_i, mp_rad_i, mo_i, mq_i, ss_i, bh_i] × 4
//

#ifndef LINKX_SOEM_DEMO_STEER_TRACE_H
#define LINKX_SOEM_DEMO_STEER_TRACE_H

#include <cstdint>
#include <fstream>

#ifndef STEER_NUM
#define STEER_NUM 4
#endif

// 单轮的 trace 列。chassis 每 tick 填好后传进来。
struct PerWheelTrace {
  float target_deg;        // Target_Steer_Rad * 180/PI
  float current_deg;       // Get_Now_Steer_Radian * 180/PI
  float motor_target_rad;  // Steer_To_Motor_Position
  float motor_now_rad;     // Motor.Get_Now_Radian
  float motor_omega;       // Motor.Get_Now_Omega
  float motor_torque;      // Motor.Get_Now_Torque
  int   state;             // SteerDriveState_e cast to int
  uint32_t boundary_hits;  // Steer_To_Motor_Position 被 ±PMAX 截断的累计 tick 数
};

class SteerTrace {
 public:
  // 第一次 LogTick 调用时根据 STEER_TRACE_FILE env 决定是否启用 + 打开文件 + 写表头。
  // 之后每 tick 追加一行 CSV。env 未设 / 打开失败 → 全部 noop。
  void LogTick(uint64_t ts_ms, float vx, float vy, float omega,
               const PerWheelTrace per_wheel[STEER_NUM]);

 private:
  bool init_ = false;
  bool disabled_ = false;
  std::ofstream stream_;
};

#endif
