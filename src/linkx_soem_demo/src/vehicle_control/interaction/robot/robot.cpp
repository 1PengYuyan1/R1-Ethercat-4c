//
// Created by pzx on 2025/12/13.
//
// 机器人交互层实现
// ----------------------------------------------------------------------------
// 文件组织：
//   1. 内部常量（匿名 namespace）
//   2. 生命周期      —— Init / Start_ROS2_Remote_Bridge /
//   Stop_ROS2_Remote_Bridge
//   3. 周期回调      —— TIM_1ms / TIM_2ms / TIM_100ms
//   4. CAN 接收分发  —— CAN_Rx_Callback
//   5. 控制核心      —— _Chassis_Control 及其辅助函数
//   6. ROS2 桥接内部 —— _ROS2_Remote_Spin_Loop / _Update_Remote_*
// ----------------------------------------------------------------------------
#include "robot.h"

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/u_int16.hpp>

#include "linkx4c_handler.h"

namespace {
// 遥控命令超时阈值：超过 200ms 未刷新则视为失联，强制零速
constexpr int64_t kRemoteCmdTimeoutNs = 200LL * 1000LL * 1000LL;

/**
 * @brief 取当前 steady_clock 时间戳（纳秒）
 *        集中封装避免每次手写 duration_cast。
 */
inline int64_t Steady_Now_Ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

// ============================================================================
//  生命周期
// ============================================================================

/**
 * @brief 机器人初始化
 *        绑定底层 LinkX 通信句柄并完成底盘子系统初始化。
 * @param __LinkX_Handler     经典 LinkX(slave_id=1)句柄,挂底盘
 * DM/ODrive/Encoder/OPS
 * @param __LinkX_FD_Handler  CAN-FD LinkX(slave_id=2)句柄,挂夹爪 DM +
 * 气夹爪(均在 ch0);可为 nullptr
 */
void Class_Robot::Init(linkx_t* __LinkX_Handler, linkx_t* __LinkX_FD_Handler) {
  LinkX_Handler = __LinkX_Handler;
  LinkX_FD_Handler = __LinkX_FD_Handler;
  Chassis.Init(LinkX_Handler);
  Chassis.Init_Motor_Params();
  Chassis.Steer_Calibration_Init();

  if (LinkX_FD_Handler != nullptr) {
    Clamp.Init(LinkX_FD_Handler, /*CAN_Channel=*/0);
    std::cout << "[ROBOT] Clamp + gripper initialized on FD-LinkX ch0 "
                 "(slave_id=2, gripper id=0x13)."
              << std::endl;
  } else {
    std::cout << "[ROBOT] WARN: FD LinkX handle is null, Clamp disabled."
              << std::endl;
  }

  // ============== 半自动路径跟随（OPS 纠偏） ==============
  Auto_Pilot.Init(&Chassis);

  // 横向 PID: 启用小 I 吃静态航向偏置 (2026-05-22)
  //   实测 path0 上后半程 head_err 稳定 +0.6°、lat 卡 -3~-5mm、v_lat_corr
  //   几乎全在死区 内 0~1mm/s,形成 0.34Hz 极限环。航向 Kp 故意压死 (耦合保护,见
  //   memo
  //   [[feedback_auto_pilot_heading_lateral_coupling]]),改由横向 I 项吸收偏置。
  //   Ki=0.05 mm/s per mm·s + I_Out_Max=15mm/s:
  //     - 5mm 持续偏置攒 1s → I 出 0.25mm/s,几秒内推到 ±2mm 死区内
  //     - I 限幅 15mm/s < P_Out_Max 30mm/s,留余量给 P 项瞬态
  Auto_Pilot.Set_Lateral_PID(/*kp=*/0.5f, /*ki=*/0.05f, /*kd=*/0.0f,
                             /*out_max_mm_s=*/30.0f, /*i_out_max_mm_s=*/15.0f);

  // 航向 PID：Kp=0.01 + dead-zone 0.5°（dvc_auto_pilot 默认）→ 航向开环，接受
  // -0.6° 姿态偏置。 实测在 40Hz 有效节拍 + 舵向 150°/s slew
  // 的硬件条件下，更激进的航向闭环会通过 omega·d 项激发横向震荡（OPS 帧
  // 20260515_051204 数据）；姿态偏置应改由横向 I 项吸收。
  Auto_Pilot.Set_Heading_PID(/*kp=*/0.01f, /*ki=*/0.0f, /*kd=*/0.0f,
                             /*out_max_rad_s=*/0.05f);

  // ============== 手动模式航向慢漂纠偏 (yaw hold) ==============
  // 设计要点 (吸取 2026-05-17 P 控制失败教训): 纯 I + 25ms 节拍 + OPS 帧门控
  // Ki=0.0008 rad/s per (deg·s): 5° err 12.5s 才打满 0.04 cap, 慢得不会激发
  // 10Hz 舵向谐振; I_Out_Max=0.04 远小于 0.3 rad/s 用户主动 omega 上限。
  //
  // dead_zone=0.3°: alg_pid.cpp:84-89 死区行为是把 error 直接置 0,小于死区
  // 的 err 完全不积分。用户感受到的漂移在 0.5-1° 量级,故 dz 必须 <0.5°
  // 才能让小漂积起来; OPS 量化噪声 ~0.01° 远低于 0.3°,不会触发误纠。
  // 详见 dvc_manual_yaw_hold.h 顶部注释 + memory
  // feedback_no_manual_heading_correction
  Manual_Yaw_Hold.Init(&Chassis);
  Manual_Yaw_Hold.Set_Yaw_PID(/*ki=*/0.0008f, /*i_out_max=*/0.04f,
                              /*out_max=*/0.04f, /*dead_zone_deg=*/0.3f);
  Manual_Yaw_Hold.Set_Arming_Params(/*omega_dead=*/0.015f,
                                    /*omega_override=*/0.025f,
                                    /*vmin=*/0.04f,
                                    /*lock_dwell_ms=*/200.0f);

  // 路径 0（按键 Y 触发）：(0,0) 起 → (1640,-600) 经 4 段。
  // 2026-05-22 提速 + 终点防过冲:
  //   wp0 斜行 100 (alignment 限速反正跑不到 cmd)
  //   wp1 拐弯段 180
  //   wp2 直行段终点 1500→1300, 缩短到 300mm — 把减速距离让给 wp3
  //   wp3 终段 1300→1640 = 340mm @ cmd 50, 减速时间充足, 终点过冲 <15mm
  static const Struct_Waypoint kPath0[] = {
      // 红方
      // {400.0f, -400.0f, 00.0f, 180.0f, 30.0f},
      // {600.0f, -600.0f, 00.0f, 100.0f, 30.0f},
      // {1000.0f, -600.0f, 00.0f, 180.0f, 30.0f},
      // {1300.0f, -600.0f, 00.0f, 180.0f, 20.0f},
      // {1500.0f, -600.0f, 00.0f, 180.0f, 20.0f},
      // {1840.0f, -600.0f, 00.0f, 50.0f, 0.0f},
      // 蓝方
      {-400.0f, -400.0f, 00.0f, 180.0f, 30.0f},
      {-600.0f, -600.0f, 00.0f, 100.0f, 30.0f},
      {-1000.0f, -600.0f, 00.0f, 180.0f, 30.0f},
      {-1400.0f, -600.0f, 00.0f, 180.0f, 20.0f},
      {-1800.0f, -600.0f, 00.0f, 180.0f, 20.0f},
      {-2000.0f, -600.0f, 00.0f, 100.0f, 0.0f},
      {-2140.0f, -600.0f, 00.0f, 50.0f, 0.0f},

  };
  Auto_Pilot.Register_Path(0, kPath0, sizeof(kPath0) / sizeof(kPath0[0]));
}

/**
 * @brief 启动 ROS2 遥控桥接
 *        若 rclcpp 未初始化则在此初始化，并起后台线程订阅
 *        /cmd_vel（geometry_msgs/Twist）与 /robot_buttons（std_msgs/UInt16）。
 *        重复调用安全：bridge 已运行时直接返回。
 */
void Class_Robot::Start_ROS2_Remote_Bridge() {
  if (ros_bridge_running_.load()) return;

  if (!rclcpp::ok()) {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
    ros_initialized_here_ = true;
  }

  ros_bridge_running_.store(true);
  ros_spin_thread_ = std::thread(&Class_Robot::_ROS2_Remote_Spin_Loop, this);
  std::cout << "[ROBOT] ROS2 remote bridge started. Subscribing /cmd_vel and "
               "/robot_buttons."
            << std::endl;
}

/**
 * @brief 停止 ROS2 遥控桥接
 *        通知 spin 线程退出并 join；若 rclcpp 是本类初始化的则负责 shutdown。
 *        重复调用安全。
 */
void Class_Robot::Stop_ROS2_Remote_Bridge() {
  if (!ros_bridge_running_.load()) return;

  ros_bridge_running_.store(false);
  if (ros_spin_thread_.joinable()) ros_spin_thread_.join();

  if (ros_initialized_here_ && rclcpp::ok()) {
    rclcpp::shutdown();
  }
  ros_initialized_here_ = false;

  _Close_Manual_Correction_CSV();
  std::cout << "[ROBOT] ROS2 remote bridge stopped." << std::endl;
}

// ============================================================================
//  周期回调（由 task 层按节拍触发）
// ============================================================================

/**
 * @brief 1ms 周期回调
 *        触发底盘运动控制循环（取 ROS 命令、使能门控、下发目标速度）。
 */
void Class_Robot::TIM_1ms_Calculate_Callback() { _Chassis_Control(); }

/**
 * @brief 2ms 周期回调
 *        驱动底盘姿态解算与控制刷新，节拍由 task 层对齐。
 *        Clamp 控制(斜坡 + MIT 帧下发)也走 2ms 节拍,与 r1.2 保持一致。
 */
void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback() {
  Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
  Chassis.TIM_2ms_Control_PeriodElapsedCallback();
  Clamp.TIM_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 100ms 周期回调
 *        刷新执行器存活/心跳，做低频健康检查。
 */
void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback() {
  Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
  Clamp.TIM_100ms_Alive_PeriodElapsedCallback();
}

// ============================================================================
//  CAN 接收分发
// ============================================================================

/**
 * @brief CAN 帧接收统一入口
 *        按 (module_id, channel) 二级分发:
 *          module_id=0 (经典 LinkX, slave_id=1):
 *              ch0=DM 转向电机 / ch1=ODrive / ch2=Encoder / ch3=OPS
 *              通道映射失配时按 ID 全分发兜底,以避免硬件接线变更时漏包
 *          module_id=1 (CAN-FD LinkX, slave_id=2):
 *              ch0=Clamp DM (Tx 0x01/0x02, Rx 0x11/0x12)
 *              不参与跨模块兜底,避免与底盘 DM 同 ID 误投
 * @param Module_Id    LinkX 模块号 (0/1)
 * @param CAN_Channel  LinkX 通道号 (0..3)
 * @param CAN_ID       原始 CAN ID(可能含 LinkX 高位标志)
 * @param CAN_Data     最多 8 字节数据指针
 * @param CAN_DLen     本帧 CAN DLC
 */
void Class_Robot::CAN_Rx_Callback(uint8_t Module_Id, uint8_t CAN_Channel,
                                  uint32_t CAN_ID, uint8_t* CAN_Data,
                                  uint8_t CAN_DLen) {
  // 统一按 11-bit 标准帧 ID 做匹配，去掉 LinkX 高位标志
  const uint32_t can_id_std = (CAN_ID & 0x7FFU);

  // —— DM 转向电机：按 CAN_Rx_ID 匹配 ——
  auto dispatch_dm = [&](uint32_t id_std) -> bool {
    for (int i = 0; i < 4; i++) {
      if (id_std == Chassis.Motor_Steer[i].DM_CAN_Rx_ID) {
        Chassis.Motor_Steer[i].CAN_RxCpltCallback(CAN_Data);
        return true;
      }
    }
    return false;
  };

  // —— ODrive：按 ID 中提取的 node_id 匹配 ——
  auto dispatch_odrive = [&](uint32_t id_std) -> bool {
    const uint32_t node_id = (id_std >> 5U) & 0x3FU;
    for (int i = 0; i < 4; i++) {
      if (node_id == Chassis.ODrive_Motor_Steer[i].Get_node_id()) {
        Chassis.ODrive_Motor_Steer[i].CAN_RxCpltCallback(CAN_Data, id_std);
        return true;
      }
    }
    return false;
  };

  // —— 外部编码器：兼容 0x05~0x08 与 0x205~0x208 两种 ID 形式 ——
  auto dispatch_encoder = [&](uint32_t id_std) -> bool {
    for (int i = 0; i < 4; i++) {
      const uint32_t encoder_id = Chassis.Encoder_Steer[i].Get_Can_ID();
      if (id_std == encoder_id || id_std == (0x200U + encoder_id)) {
        Chassis.Encoder_Steer[i].CAN_RxCpltCallback(CAN_Data);
        return true;
      }
    }
    return false;
  };

  // —— OPS-9：CAN-COM 透传，所有片段共用一个 ID；DLC=4 的尾片段必须把
  //          真实长度透传给重组逻辑，否则末尾 4 个零字节会破坏帧对齐 ——
  auto dispatch_ops = [&](uint32_t /*id_std*/) -> bool {
    Chassis.OPS.CAN_RxCpltCallback(CAN_Data, CAN_DLen);
    return true;
  };

  // —— 夹爪 DM:Rx_ID = 0x11(大 Pitch) / 0x12(小 Pitch) ——
  auto dispatch_clamp = [&](uint32_t id_std) -> bool {
    if (id_std == 0x11U) {
      Clamp.Motor_Pitch_Large.CAN_RxCpltCallback(CAN_Data);
      return true;
    }
    if (id_std == 0x12U) {
      Clamp.Motor_Pitch_Small.CAN_RxCpltCallback(CAN_Data);
      return true;
    }
    return false;
  };

  if (Module_Id == 1U) {
    // 第二片 LinkX:仅 ch0 挂夹爪;其他通道暂未分配,直接丢弃避免误投
    if (CAN_Channel == 0U) (void)dispatch_clamp(can_id_std);
    return;
  }

  // 第一片 LinkX(默认 module_id==0):按通道分发
  bool handled = false;
  switch (CAN_Channel) {
    case 0:
      handled = dispatch_dm(can_id_std);
      break;
    case 1:
      handled = dispatch_odrive(can_id_std);
      break;
    case 2:
      handled = dispatch_encoder(can_id_std);
      break;
    case 3:
      handled = dispatch_ops(can_id_std);
      break;
    default:
      break;
  }

  // 通道映射失配时按 ID 全分发兜底（OPS 不参与兜底，避免误吞其他通道帧）
  if (!handled) {
    (void)(dispatch_dm(can_id_std) || dispatch_odrive(can_id_std) ||
           dispatch_encoder(can_id_std));
  }
}

// ============================================================================
//  控制核心：_Chassis_Control 与其辅助函数
//  设计目标：单一职责，方便单独替换/测试，降低与 ROS 数据源/按键逻辑的耦合。
// ============================================================================

/**
 * @brief 底盘控制主流程（每 1ms 调用一次）
 *        编排：取快照 → 更新使能 → 处理功能键 → 下发目标速度 → 刷新调试观测。
 *
 * @note  Auto-pilot（OPS 路径跟随）已启用：Y=path0 / A=path1 / LB=path2 /
 * X=停止。
 */
void Class_Robot::_Chassis_Control() {
  // 1) 取一份 ROS 命令快照，并判断时效性
  Ros_Remote_Command snapshot;
  bool is_recent = false;
  _Snapshot_Remote_Command(snapshot, is_recent);

  // 2) 根据按键长按更新手动控制使能
  const uint16_t key_code =
      snapshot.has_buttons ? snapshot.buttons : LogF710_Key_IDLE;
  const bool is_enabled = _Update_Manual_Enable_State(key_code);

  // 3) 处理功能键边沿（X/Y/A/LB/RB）
  _Process_Function_Key_Edge(key_code, is_enabled);

  // 3b) 气夹爪 (FD-LinkX ch0 / id=0x13 / byte0) 每 tick 重 push 当前状态
  //     LinkX 固件无 valid bit, slot 会被 cycle 重发, 必须用 payload 表态
  //     状态由 (1) clamp 序列 STEP2→STEP3 自动置闭 (2) Key B rising edge toggle
  //     维护
  Clamp.Push_Gripper_Frame();

  // ============== AUTO-PILOT (OPS 半自动路径跟随) ==============
  if (Auto_Pilot.Is_Active()) {
    Auto_Pilot.TIM_Tick(0.001f);  // 主循环 1ms
    debug_remote_key_code_ = key_code;
    debug_remote_is_recent_ = is_recent;
    debug_remote_is_enabled_ = is_enabled;
    debug_remote_vx_ = is_recent ? snapshot.vx : 0.0f;
    debug_remote_vy_ = is_recent ? snapshot.vy : 0.0f;
    debug_remote_omega_ = is_recent ? snapshot.omega : 0.0f;
    return;
  }
  // =============================================================

  // 4) 手动模式：航向慢漂纠偏 (yaw hold) + 速度透传
  //    Manual_Yaw_Hold 内部按 OPS 帧门控 (40Hz), 状态机自动 arm/disarm
  //    HOLDING 态以慢 I 输出 corrected_omega; 其它态透传 raw_omega
  const bool active = is_recent && is_enabled;
  const bool ops_ok = (Chassis.OPS.Get_Status() == OPS_Status_ENABLE);
  const float vmag =
      std::sqrt(snapshot.vx * snapshot.vx + snapshot.vy * snapshot.vy);
  const float corrected_omega =
      Manual_Yaw_Hold.Update(snapshot.omega, vmag, ops_ok, active);

  if (active && ops_ok) {
    _Log_Manual_Correction(snapshot.vx, snapshot.vy, snapshot.omega,
                           corrected_omega);
  }

  _Apply_Chassis_Velocity(snapshot.vx, snapshot.vy, corrected_omega, active);

  // 4b) 夹爪跟随全局手动使能;具体 POS1/POS2 由 _Process_Function_Key_Edge
  // 边沿设置
  _Clamp_Control(key_code, is_enabled);

  // 5) 调试观测（不受使能门控影响，直接显示接收到的摇杆速度）
  debug_remote_key_code_ = key_code;
  debug_remote_is_recent_ = is_recent;
  debug_remote_is_enabled_ = is_enabled;
  debug_remote_vx_ = is_recent ? snapshot.vx : 0.0f;
  debug_remote_vy_ = is_recent ? snapshot.vy : 0.0f;
  debug_remote_omega_ = is_recent ? snapshot.omega : 0.0f;
}

/**
 * @brief 取 ROS 命令快照（线程安全）
 *        仅在锁内做拷贝，时间判定放到锁外，缩短临界区。
 * @param[out] out_cmd      命令快照
 * @param[out] out_is_recent 是否在超时阈值内
 * @return out_is_recent 同值，便于链式使用
 */
bool Class_Robot::_Snapshot_Remote_Command(Ros_Remote_Command& out_cmd,
                                           bool& out_is_recent) {
  {
    std::lock_guard<std::mutex> lock(ros_cmd_mutex_);
    out_cmd = ros_cmd_;
  }
  const int64_t now_ns = Steady_Now_Ns();
  out_is_recent = out_cmd.has_twist && (out_cmd.last_update_ns > 0) &&
                  ((now_ns - out_cmd.last_update_ns) <= kRemoteCmdTimeoutNs);
  return out_is_recent;
}

/**
 * @brief 更新手动控制使能（START 长按开 / BACK 长按关）
 *        在状态翻转时打印一次日志便于现场观察。
 * @param key_code  当前按键状态字
 * @return 当前是否使能
 */
bool Class_Robot::_Update_Manual_Enable_State(uint16_t key_code) {
  const bool was_enabled = logf710_remote_.Is_Control_Enabled();
  logf710_remote_.Update_Control_Enable(key_code, 1U);
  const bool is_enabled = logf710_remote_.Is_Control_Enabled();

  if (!was_enabled && is_enabled)
    std::cout << "[ROBOT] ROS manual control enabled by START long press."
              << std::endl;
  else if (was_enabled && !is_enabled) {
    Manual_Yaw_Hold.Force_Disarm();
    std::cout << "[ROBOT] ROS manual control disabled by BACK long press."
              << std::endl;
  }

  return is_enabled;
}

/**
 * @brief 功能键边沿动作
 *        仅在控制使能时响应。三段分发:
 *          1) LB modifier 组合 (机械臂占位):LB+X/Y/A/B/Up/Down/Left/Right
 *          2) RB modifier 组合 (龙门架占位):RB+X/Y/A/B/Up/Down/Left/Right
 *          3) 单键 (LB/RB 单按已不分发,只做 modifier)
 *
 *        组合识别:rising_key 与 LogF710_Mod_LB/RB 相与判 modifier 是否按下;
 *        要求至少还有一个非 modifier 位,LB 或 RB 单按 (= 0x0008 / 0x0080)
 * 不进组合分支。
 * @param key_code   当前按键
 * @param is_enabled 是否处于手动使能态
 */
void Class_Robot::_Process_Function_Key_Edge(uint16_t key_code,
                                             bool is_enabled) {
  if (!is_enabled) return;

  uint16_t rising_key = LogF710_Key_IDLE;
  if (!logf710_remote_.Check_Key_Rising_Edge(key_code, &rising_key)) return;

  // ============ LB modifier 组合 → 机械臂 (占位) ============
  if ((rising_key & LogF710_Mod_LB) && (rising_key != LogF710_Mod_LB)) {
    const uint16_t base = static_cast<uint16_t>(rising_key & ~LogF710_Mod_LB);
    switch (base) {
      case LogF710_Key_X:
        std::cout << "[ARM] LB+X: (placeholder)" << std::endl;
        break;
      case LogF710_Key_Y:
        std::cout << "[ARM] LB+Y: (placeholder)" << std::endl;
        break;
      case LogF710_Key_A:
        std::cout << "[ARM] LB+A: (placeholder)" << std::endl;
        break;
      case LogF710_Key_B:
        std::cout << "[ARM] LB+B: (placeholder)" << std::endl;
        break;
      case LogF710_Key_Up:
        if (LinkX_FD_Handler != nullptr) {
          uint8_t buf[8] = {0};
          buf[0] = 0x01U;
          linkx_quick_can_send(LinkX_FD_Handler, 0U, 0x16U, buf);
          std::cout << "[ROBOT] LB+Up: FD-LinkX ch0 ID=0x16 payload=0x01"
                    << std::endl;
        }
        break;
      case LogF710_Key_Down:
        if (LinkX_FD_Handler != nullptr) {
          uint8_t buf[8] = {0};
          buf[0] = 0x02U;
          linkx_quick_can_send(LinkX_FD_Handler, 0U, 0x16U, buf);
          std::cout << "[ROBOT] LB+Down: FD-LinkX ch0 ID=0x16 payload=0x02"
                    << std::endl;
        }
        break;
      case LogF710_Key_Left:
        std::cout << "[ARM] LB+Left: (placeholder)" << std::endl;
        break;
      case LogF710_Key_Right:
        std::cout << "[ARM] LB+Right: (placeholder)" << std::endl;
        break;
      default:
        break;
    }
    return;
  }

  // ============ RB modifier 组合 → 夹爪手动控制 + 龙门架 (占位) ============
  if ((rising_key & LogF710_Mod_RB) && (rising_key != LogF710_Mod_RB)) {
    const uint16_t base = static_cast<uint16_t>(rising_key & ~LogF710_Mod_RB);
    switch (base) {
      case LogF710_Key_X:
        Clamp.Set_Gripper_Byte0(0x01U);
        std::cout << "[CLAMP] RB+Y: gripper CLOSE (byte0=0x01)" << std::endl;

        break;
      case LogF710_Key_Y:
        Clamp.Set_Gripper_Byte0(0x00U);
        std::cout << "[CLAMP] RB+X: gripper OPEN (byte0=0x00)" << std::endl;
        break;
      case LogF710_Key_A:
        Clamp.Set_Pitch_Large_State(L_PITCH_POS2);
        Clamp.Set_Pitch_Small_State(S_PITCH_POS2);
        std::cout << "[CLAMP] RB+A: arm -> POS2 (pick pose)" << std::endl;
        break;
      case LogF710_Key_B:
        Clamp.Set_Pitch_Large_State(L_PITCH_POS1);
        Clamp.Set_Pitch_Small_State(S_PITCH_POS1);
        std::cout << "[CLAMP] RB+B: arm -> POS1 (home pose)" << std::endl;
        break;
      case LogF710_Key_Up:
        if (LinkX_FD_Handler != nullptr) {
          uint8_t buf[8] = {0};
          buf[0] = 0x01U;
          linkx_quick_can_send(LinkX_FD_Handler, 0U, 0x15U, buf);
          std::cout << "[ROBOT] RB+Up: FD-LinkX ch0 ID=0x15 payload=0x01"
                    << std::endl;
        }
        break;
      case LogF710_Key_Down:
        if (LinkX_FD_Handler != nullptr) {
          uint8_t buf[8] = {0};
          buf[0] = 0x02U;
          linkx_quick_can_send(LinkX_FD_Handler, 0U, 0x15U, buf);
          std::cout << "[ROBOT] RB+Down: FD-LinkX ch0 ID=0x15 payload=0x02"
                    << std::endl;
        }
        break;
      case LogF710_Key_Left:
        std::cout << "[GANTRY] RB+Left: (placeholder)" << std::endl;
        break;
      case LogF710_Key_Right:
        Clamp.Set_Pitch_Large_State(L_PITCH_POS2);
        Clamp.Set_Pitch_Small_State(S_PITCH_POS3);
        std::cout << "[CLAMP] RB+Right: arm -> POS2/POS3 (place pose, "
                     "q1=3.520, q2=-5.084)"
                  << std::endl;
        break;
      default:
        break;
    }
    return;
  }

  // ============ 单键分发 (LB/RB 单按已不再触发任何动作,纯 modifier)
  // ============
  switch (rising_key) {
    // ---- AUTO-PILOT TRIGGERS ----
    case LogF710_Key_X:
    case LogF710_Key_Y:
    case LogF710_Key_B: {
      if (LinkX_FD_Handler != nullptr) {
        uint8_t buf[8] = {0};
        buf[0] = (rising_key == LogF710_Key_X)   ? 0x01U
                 : (rising_key == LogF710_Key_Y) ? 0x02U
                                                 : 0x03U;
        linkx_quick_can_send(LinkX_FD_Handler, 0U, 0x14U, buf);
        std::cout << "[ROBOT] Key "
                  << (rising_key == LogF710_Key_X   ? "X"
                      : rising_key == LogF710_Key_Y ? "Y"
                                                    : "B")
                  << ": FD-LinkX ch0 ID=0x14 payload=0x" << std::hex
                  << static_cast<int>(buf[0]) << std::dec << std::endl;
      }
      break;
    }
    case LogF710_Key_A:
      // 已迁到 RB+A (去 POS2) / RB+B (回 POS1); 单按 A 不再触发自动序列
      break;
    case LogF710_Key_LT:
      // 夹爪复位:大+小 Pitch 同时回 POS1
      // 注意:LT 仅在 DInput 布局下作为按键被解析,XInput 模式下变 axis 不响应
      Clamp.Set_Pitch_Large_State(L_PITCH_POS1);
      Clamp.Set_Pitch_Small_State(S_PITCH_POS1);
      std::cout << "[ROBOT] Key LT: Clamp -> POS1 (open)" << std::endl;
      break;
    // ---- 预留按键 (无任务,仅打印边沿) ----
    // RT: XInput 下为 axis,仅 DInput 布局下作为按键被解析
    // Back/Start: 长按已被 _Update_Manual_Enable 消费
    // (disable/enable);短按时才进入这里
    case LogF710_Key_RT:
      std::cout << "[ROBOT] Key RT: (unassigned)" << std::endl;
      break;
    case LogF710_Key_Back:
      std::cout << "[ROBOT] Key Back: (unassigned)" << std::endl;
      break;
    case LogF710_Key_Start:
      std::cout << "[ROBOT] Key Start: (unassigned)" << std::endl;
      break;
    case LogF710_Key_Up:
      Auto_Pilot.Start(0);
      std::cout << "[ROBOT] Key Up: gripper byte0 -> 0x01" << std::endl;
      break;
    case LogF710_Key_Down:
      Auto_Pilot.Stop();

      break;
    case LogF710_Key_Left:
      std::cout << "[ROBOT] Key Left: (unassigned)" << std::endl;
      break;
    case LogF710_Key_Right:
      Chassis.OPS.Send_Zero();
      std::cout << "[ROBOT] Key Down: Auto-Pilot STOP + OPS zero (ACT0)"
                << std::endl;
      // std::cout << "[ROBOT] Key Right: (unassigned)" << std::endl;
      break;
    default:
      break;
  }
}

/**
 * @brief 夹爪使能跟随全局手动控制状态
 *        Start 长按 -> is_enabled=true -> Clamp ENABLE;
 *        Back 长按或控制超时 -> is_enabled=false -> Clamp DISABLE。
 *        具体 POS1/POS2 切换由 _Process_Function_Key_Edge 处理。
 */
void Class_Robot::_Clamp_Control(uint16_t /*key_code*/, bool is_enabled) {
  Clamp.Set_Clamp_Control_Type(is_enabled ? CLAMP_CONTROL_ENABLE
                                          : CLAMP_CONTROL_DISABLE);
}

/**
 * @brief 将目标速度下发到底盘
 *        active=true 时透传速度并 ENABLE；否则零速 + DISABLE，
 *        统一封装确保任何控制源失效时都进入安全状态。
 */
void Class_Robot::_Apply_Chassis_Velocity(float vx, float vy, float omega,
                                          bool active) {
  if (active) {
    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ENABLE);
    Chassis.Set_Target_Velocity_X(vx);
    Chassis.Set_Target_Velocity_Y(vy);
    Chassis.Set_Target_Omega(omega);
  } else {
    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    Chassis.Set_Target_Velocity_X(0.0f);
    Chassis.Set_Target_Velocity_Y(0.0f);
    Chassis.Set_Target_Omega(0.0f);
  }
}

// ============================================================================
//  ROS2 桥接内部
// ============================================================================

/**
 * @brief ROS2 spin 后台线程主循环
 *        在 200Hz 节拍下 spin_some，把 /cmd_vel 与 /robot_buttons
 *        通过 _Update_Remote_* 写入 ros_cmd_ 快照。
 */
void Class_Robot::_ROS2_Remote_Spin_Loop() {
  auto node = std::make_shared<rclcpp::Node>("soem_remote_bridge_node");

  auto sub_cmd = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 20, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        _Update_Remote_Twist(static_cast<float>(msg->linear.x),
                             static_cast<float>(msg->linear.y),
                             static_cast<float>(msg->angular.z));
      });

  auto sub_buttons = node->create_subscription<std_msgs::msg::UInt16>(
      "/robot_buttons", 20, [this](const std_msgs::msg::UInt16::SharedPtr msg) {
        _Update_Remote_Buttons(msg->data);
      });

  // /ops 旁路: linear.x=pos_x_mm  linear.y=pos_y_mm  angular.z=yaw_deg
  auto pub_ops = node->create_publisher<geometry_msgs::msg::Twist>("/ops", 10);

  // sub_* 必须保活到循环结束，否则订阅会被析构
  (void)sub_cmd;
  (void)sub_buttons;

  rclcpp::WallRate rate(200);
  while (ros_bridge_running_.load() && rclcpp::ok()) {
    rclcpp::spin_some(node);

    geometry_msgs::msg::Twist ops_msg;
    ops_msg.linear.x = Chassis.OPS.Get_Pos_X_Mm();
    ops_msg.linear.y = Chassis.OPS.Get_Pos_Y_Mm();
    ops_msg.angular.z = Chassis.OPS.Get_Yaw_Deg();
    pub_ops->publish(ops_msg);

    rate.sleep();
  }
}

/**
 * @brief 写入最新速度命令快照（线程安全）
 *        ROS spin 线程调用，更新时间戳供主线程做超时判定。
 */
void Class_Robot::_Update_Remote_Twist(float vx, float vy, float omega) {
  std::lock_guard<std::mutex> lock(ros_cmd_mutex_);
  ros_cmd_.vx = vx;
  ros_cmd_.vy = vy;
  ros_cmd_.omega = omega;
  ros_cmd_.has_twist = true;
  ros_cmd_.last_update_ns = Steady_Now_Ns();
}

/**
 * @brief 写入最新按键状态快照（线程安全）
 */
void Class_Robot::_Update_Remote_Buttons(uint16_t buttons) {
  std::lock_guard<std::mutex> lock(ros_cmd_mutex_);
  ros_cmd_.buttons = buttons;
  ros_cmd_.has_buttons = true;
  ros_cmd_.last_update_ns = Steady_Now_Ns();
}

// ============================================================================
//  手动修正 CSV 日志
// ============================================================================

void Class_Robot::_Open_Manual_Correction_CSV() {
  if (manual_corr_csv_opened_) return;

  auto now = std::chrono::system_clock::now();
  auto tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  localtime_r(&tt, &tm_buf);

  std::ostringstream fname;
  fname << "var_data/manual_correction_"
        << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".csv";

  manual_corr_csv_.open(fname.str(), std::ios::out | std::ios::trunc);
  if (!manual_corr_csv_.is_open()) {
    std::cerr << "[ROBOT] WARN: failed to open manual correction CSV: "
              << fname.str() << std::endl;
    return;
  }

  manual_corr_csv_ << std::fixed << std::setprecision(3);
  manual_corr_csv_
      << "tick,"
         "ops_x_mm,ops_y_mm,ops_yaw_deg,"
         "raw_vx,raw_vy,raw_omega,"
         "corrected_omega,"
         "yhold_state,yhold_target_deg,yhold_err_deg,yhold_i_value,"
         "now_vx,now_vy,now_omega,"
         "enc_deg_0,enc_deg_1,enc_deg_2,enc_deg_3,"
         "odrive_omega_0,odrive_omega_1,odrive_omega_2,odrive_omega_3,"
         "dm_omega_0,dm_omega_1,dm_omega_2,dm_omega_3,"
         "dm_torque_0,dm_torque_1,dm_torque_2,dm_torque_3\n";

  manual_corr_csv_opened_ = true;
  manual_corr_tick_ = 0;
  std::cout << "[ROBOT] Manual correction CSV started: " << fname.str()
            << std::endl;
}

void Class_Robot::_Close_Manual_Correction_CSV() {
  if (manual_corr_csv_.is_open()) {
    manual_corr_csv_.flush();
    manual_corr_csv_.close();
    std::cout << "[ROBOT] Manual correction CSV closed (" << manual_corr_tick_
              << " ticks)" << std::endl;
  }
  manual_corr_csv_opened_ = false;
}

void Class_Robot::_Log_Manual_Correction(float raw_vx, float raw_vy,
                                         float raw_omega,
                                         float corrected_omega) {
  if (!manual_corr_csv_opened_) _Open_Manual_Correction_CSV();

  if (!manual_corr_csv_.is_open()) return;

  if ((manual_corr_tick_ % kManualCorrCsvDecimate_) == 0) {
    manual_corr_csv_ << manual_corr_tick_ << "," << Chassis.OPS.Get_Pos_X_Mm()
                     << "," << Chassis.OPS.Get_Pos_Y_Mm() << ","
                     << Chassis.OPS.Get_Yaw_Deg() << "," << raw_vx << ","
                     << raw_vy << "," << raw_omega << "," << corrected_omega
                     << ","
                     << Class_Manual_Heading_Hold::State_To_Int(
                            Manual_Yaw_Hold.Get_State())
                     << "," << Manual_Yaw_Hold.Get_Target_Yaw() << ","
                     << Manual_Yaw_Hold.Get_Err_Deg() << ","
                     << Manual_Yaw_Hold.Get_I_Value() << ","
                     << Chassis.Get_Now_Velocity_X() << ","
                     << Chassis.Get_Now_Velocity_Y() << ","
                     << Chassis.Get_Now_Omega();

    for (int i = 0; i < 4; ++i)
      manual_corr_csv_ << ","
                       << Chassis.Encoder_Steer[i].Get_Wheel_Angle_True();

    for (int i = 0; i < 4; ++i)
      manual_corr_csv_ << "," << Chassis.ODrive_Motor_Steer[i].Get_Omega();

    for (int i = 0; i < 4; ++i)
      manual_corr_csv_ << "," << Chassis.Motor_Steer[i].Get_Now_Omega();

    for (int i = 0; i < 4; ++i)
      manual_corr_csv_ << "," << Chassis.Motor_Steer[i].Get_Now_Torque();

    manual_corr_csv_ << "\n";
  }
  ++manual_corr_tick_;
}
