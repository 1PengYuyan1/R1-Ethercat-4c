//
// steer_trace.cpp - 见同名 .h。函数体从 crt_chassis.cpp 的 Steer_Trace_Log 迁移。
// 默认 float 格式（无 setprecision），与原行为一致。
//

#include "steer_trace.h"

#include <cstdlib>
#include <iostream>

void SteerTrace::LogTick(uint64_t ts_ms, float vx, float vy, float omega,
                         const PerWheelTrace per_wheel[STEER_NUM])
{
  if (!init_)
  {
    init_ = true;
    const char *path = std::getenv("STEER_TRACE_FILE");
    if (!path || path[0] == '\0')
    {
      disabled_ = true;
      return;
    }
    stream_.open(path, std::ios::trunc);
    if (!stream_.is_open())
    {
      std::cerr << "[CHASSIS] STEER_TRACE_FILE open failed: " << path << std::endl;
      disabled_ = true;
      return;
    }
    stream_ << "ts_ms,vx,vy,omega";
    for (int i = 0; i < STEER_NUM; ++i)
      stream_ << ",t_deg_" << i << ",c_deg_" << i
              << ",mt_rad_" << i << ",mp_rad_" << i
              << ",mo_" << i << ",mq_" << i << ",ss_" << i
              << ",bh_" << i;
    stream_ << "\n";
    std::cout << "[CHASSIS] STEER_TRACE writing to " << path << " @500Hz" << std::endl;
  }
  if (disabled_) return;

  stream_ << ts_ms
          << "," << vx
          << "," << vy
          << "," << omega;
  for (int i = 0; i < STEER_NUM; ++i)
  {
    const PerWheelTrace &p = per_wheel[i];
    stream_ << "," << p.target_deg << "," << p.current_deg
            << "," << p.motor_target_rad << "," << p.motor_now_rad
            << "," << p.motor_omega << "," << p.motor_torque
            << "," << p.state
            << "," << p.boundary_hits;
  }
  stream_ << "\n";
}
