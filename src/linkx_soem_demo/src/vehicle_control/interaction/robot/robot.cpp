//
// Created by pzx on 2025/12/13.
//
// 机器人交互层实现
// ----------------------------------------------------------------------------
// 文件组织：
//   1. 内部常量（匿名 namespace）
//   2. 生命周期      —— Init / Start_ROS2_Remote_Bridge / Stop_ROS2_Remote_Bridge
//   3. 周期回调      —— TIM_1ms / TIM_2ms / TIM_100ms
//   4. CAN 接收分发  —— CAN_Rx_Callback
//   5. 控制核心      —— _Chassis_Control 及其辅助函数
//   6. ROS2 桥接内部 —— _ROS2_Remote_Spin_Loop / _Update_Remote_*
// ----------------------------------------------------------------------------
#include "robot.h"

#include "linkx4c_handler.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace
{
// 遥控命令超时阈值：超过 200ms 未刷新则视为失联，强制零速
constexpr int64_t kRemoteCmdTimeoutNs = 200LL * 1000LL * 1000LL;

/**
 * @brief 取当前 steady_clock 时间戳（纳秒）
 *        集中封装避免每次手写 duration_cast。
 */
inline int64_t Steady_Now_Ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
} // namespace


// ============================================================================
//  生命周期
// ============================================================================

/**
 * @brief 机器人初始化
 *        绑定底层 LinkX 通信句柄并完成底盘子系统初始化。
 * @param __LinkX_Handler     经典 LinkX(slave_id=1)句柄,挂底盘 DM/ODrive/Encoder/OPS
 * @param __LinkX_FD_Handler  CAN-FD LinkX(slave_id=2)句柄,挂夹爪 DM(ch0);可为 nullptr
 */
void Class_Robot::Init(linkx_t *__LinkX_Handler, linkx_t *__LinkX_FD_Handler)
{
    LinkX_Handler    = __LinkX_Handler;
    LinkX_FD_Handler = __LinkX_FD_Handler;
    Chassis.Init(LinkX_Handler);
    Chassis.Init_Motor_Params();
    Chassis.Steer_Calibration_Init();

    if (LinkX_FD_Handler != nullptr)
    {
        Clamp.Init(LinkX_FD_Handler, /*CAN_Channel=*/0);
        std::cout << "[ROBOT] Clamp initialized on FD-LinkX ch0 (slave_id=2)." << std::endl;
    }
    else
    {
        std::cout << "[ROBOT] WARN: FD LinkX handle is null, Clamp disabled." << std::endl;
    }

    // ============== 半自动路径跟随（OPS 纠偏） ==============
    Auto_Pilot.Init(&Chassis);

    // 航向 PID：Kp=0.01 + dead-zone 0.5°（dvc_auto_pilot 默认）→ 航向开环，接受 -0.6° 姿态偏置。
    // 实测在 40Hz 有效节拍 + 舵向 150°/s slew 的硬件条件下，更激进的航向闭环会通过
    // omega·d 项激发横向震荡（OPS 帧 20260515_051204 数据）；姿态偏置应改由横向 I 项吸收。
    Auto_Pilot.Set_Heading_PID(/*kp=*/0.01f, /*ki=*/0.0f, /*kd=*/0.0f,
                               /*out_max_rad_s=*/0.05f);

    // 路径 0（按键 Y 触发）：单段直行 1m (+Y 方向),先验证 auto_pilot 闭环走直
    // speed=40mm/s 配合 MAX_CHASSIS_SPEED=0.05m/s,留 10mm/s 给 PID 横向修正
    static const Struct_Waypoint kPath0[] = {
        {      0.0f, -500.0f,  00.0f, 40.0f, 30.0f},
        {    500.0f, -500.0f,  00.0f, 40.0f, 30.0f},
        {    500.0f, -800.0f,  00.0f, 40.0f, 30.0f},

    };
    Auto_Pilot.Register_Path(0, kPath0, sizeof(kPath0)/sizeof(kPath0[0]));
}

/**
 * @brief 启动 ROS2 遥控桥接
 *        若 rclcpp 未初始化则在此初始化，并起后台线程订阅
 *        /cmd_vel（geometry_msgs/Twist）与 /robot_buttons（std_msgs/UInt16）。
 *        重复调用安全：bridge 已运行时直接返回。
 */
void Class_Robot::Start_ROS2_Remote_Bridge()
{
    if (ros_bridge_running_.load()) return;

    if (!rclcpp::ok())
    {
        int    argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
        ros_initialized_here_ = true;
    }

    ros_bridge_running_.store(true);
    ros_spin_thread_ = std::thread(&Class_Robot::_ROS2_Remote_Spin_Loop, this);
    std::cout << "[ROBOT] ROS2 remote bridge started. Subscribing /cmd_vel and /robot_buttons." << std::endl;
}

/**
 * @brief 停止 ROS2 遥控桥接
 *        通知 spin 线程退出并 join；若 rclcpp 是本类初始化的则负责 shutdown。
 *        重复调用安全。
 */
void Class_Robot::Stop_ROS2_Remote_Bridge()
{
    if (!ros_bridge_running_.load()) return;

    ros_bridge_running_.store(false);
    if (ros_spin_thread_.joinable()) ros_spin_thread_.join();

    if (ros_initialized_here_ && rclcpp::ok())
    {
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
void Class_Robot::TIM_1ms_Calculate_Callback()
{
    _Chassis_Control();
}

/**
 * @brief 2ms 周期回调
 *        驱动底盘姿态解算与控制刷新，节拍由 task 层对齐。
 *        Clamp 控制(斜坡 + MIT 帧下发)也走 2ms 节拍,与 r1.2 保持一致。
 */
void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback()
{
    Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
    Clamp.TIM_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 100ms 周期回调
 *        刷新执行器存活/心跳，做低频健康检查。
 */
void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback()
{
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
void Class_Robot::CAN_Rx_Callback(uint8_t Module_Id, uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data, uint8_t CAN_DLen)
{
    // 统一按 11-bit 标准帧 ID 做匹配，去掉 LinkX 高位标志
    const uint32_t can_id_std = (CAN_ID & 0x7FFU);

    // —— DM 转向电机：按 CAN_Rx_ID 匹配 ——
    auto dispatch_dm = [&](uint32_t id_std) -> bool
    {
        for (int i = 0; i < 4; i++)
        {
            if (id_std == Chassis.Motor_Steer[i].DM_CAN_Rx_ID)
            {
                Chassis.Motor_Steer[i].CAN_RxCpltCallback(CAN_Data);
                return true;
            }
        }
        return false;
    };

    // —— ODrive：按 ID 中提取的 node_id 匹配 ——
    auto dispatch_odrive = [&](uint32_t id_std) -> bool
    {
        const uint32_t node_id = (id_std >> 5U) & 0x3FU;
        for (int i = 0; i < 4; i++)
        {
            if (node_id == Chassis.ODrive_Motor_Steer[i].Get_node_id())
            {
                Chassis.ODrive_Motor_Steer[i].CAN_RxCpltCallback(CAN_Data, id_std);
                return true;
            }
        }
        return false;
    };

    // —— 外部编码器：兼容 0x05~0x08 与 0x205~0x208 两种 ID 形式 ——
    auto dispatch_encoder = [&](uint32_t id_std) -> bool
    {
        for (int i = 0; i < 4; i++)
        {
            const uint32_t encoder_id = Chassis.Encoder_Steer[i].Get_Can_ID();
            if (id_std == encoder_id || id_std == (0x200U + encoder_id))
            {
                Chassis.Encoder_Steer[i].CAN_RxCpltCallback(CAN_Data);
                return true;
            }
        }
        return false;
    };

    // —— OPS-9：CAN-COM 透传，所有片段共用一个 ID；DLC=4 的尾片段必须把
    //          真实长度透传给重组逻辑，否则末尾 4 个零字节会破坏帧对齐 ——
    auto dispatch_ops = [&](uint32_t /*id_std*/) -> bool
    {
        Chassis.OPS.CAN_RxCpltCallback(CAN_Data, CAN_DLen);
        return true;
    };

    // —— 夹爪 DM:Rx_ID = 0x11(大 Pitch) / 0x12(小 Pitch) ——
    auto dispatch_clamp = [&](uint32_t id_std) -> bool
    {
        if (id_std == 0x11U) { Clamp.Motor_Pitch_Large.CAN_RxCpltCallback(CAN_Data); return true; }
        if (id_std == 0x12U) { Clamp.Motor_Pitch_Small.CAN_RxCpltCallback(CAN_Data); return true; }
        return false;
    };

    if (Module_Id == 1U)
    {
        // 第二片 LinkX:仅 ch0 挂夹爪;其他通道暂未分配,直接丢弃避免误投
        if (CAN_Channel == 0U)
            (void)dispatch_clamp(can_id_std);
        return;
    }

    // 第一片 LinkX(默认 module_id==0):按通道分发
    bool handled = false;
    switch (CAN_Channel)
    {
    case 0: handled = dispatch_dm(can_id_std);      break;
    case 1: handled = dispatch_odrive(can_id_std);  break;
    case 2: handled = dispatch_encoder(can_id_std); break;
    case 3: handled = dispatch_ops(can_id_std);     break;
    default: break;
    }

    // 通道映射失配时按 ID 全分发兜底（OPS 不参与兜底，避免误吞其他通道帧）
    if (!handled)
    {
        (void)(dispatch_dm(can_id_std) ||
               dispatch_odrive(can_id_std) ||
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
 * @note  Auto-pilot（OPS 路径跟随）已启用：Y=path0 / A=path1 / LB=path2 / X=停止。
 */
void Class_Robot::_Chassis_Control()
{
    // 1) 取一份 ROS 命令快照，并判断时效性
    Ros_Remote_Command snapshot;
    bool is_recent = false;
    _Snapshot_Remote_Command(snapshot, is_recent);

    // 2) 根据按键长按更新手动控制使能
    const uint16_t key_code  = snapshot.has_buttons ? snapshot.buttons : LogF710_Key_IDLE;
    const bool     is_enabled = _Update_Manual_Enable_State(key_code);

    // 3) 处理功能键边沿（X/Y/A/LB/RB）
    _Process_Function_Key_Edge(key_code, is_enabled);

    // 3b) Key B 状态透传:每 tick 都重入队 FD-LinkX ch1 ID=0x13
    //     payload byte 0 = B 是否按住 (0x01=held, 0x00=released)
    //     LinkX 固件会每 cycle 把 slot 内容重发到 CAN, 下游 DM-MC 读 byte0 取状态
    //     不能用 "停 push 让总线安静": 固件无 valid bit, 槽位会一直被重发, 必须靠 payload 表态
    {
        const bool b_held = is_enabled && (key_code == LogF710_Key_B);
        if (LinkX_FD_Handler != nullptr)
        {
            uint8_t tx[8] = {0};
            tx[0] = b_held ? 0x01U : 0x00U;
            linkx_quick_can_send(LinkX_FD_Handler, /*ch=*/1U, /*id=*/0x13U, tx);
        }
        if (b_held != b_key_was_held_)
        {
            std::cout << "[ROBOT] Key B " << (b_held ? "HELD   -> 0x13 byte0=0x01"
                                                    : "released -> 0x13 byte0=0x00") << std::endl;
            b_key_was_held_ = b_held;
        }
    }

    // ============== AUTO-PILOT (OPS 半自动路径跟随) ==============
    if (Auto_Pilot.Is_Active())
    {
        Auto_Pilot.TIM_Tick(0.001f);   // 主循环 1ms
        debug_remote_key_code_   = key_code;
        debug_remote_is_recent_  = is_recent;
        debug_remote_is_enabled_ = is_enabled;
        debug_remote_vx_         = is_recent ? snapshot.vx    : 0.0f;
        debug_remote_vy_         = is_recent ? snapshot.vy    : 0.0f;
        debug_remote_omega_      = is_recent ? snapshot.omega : 0.0f;
        return;
    }
    // =============================================================

    // 4) 手动模式：遥控直接透传(航向纠偏已删除,稳态漂移靠 wheel speed calib 在底盘端吸收)
    const bool active = is_recent && is_enabled;

    if (active && Chassis.OPS.Get_Status() == OPS_Status_ENABLE)
    {
        _Log_Manual_Correction(snapshot.vx, snapshot.vy, snapshot.omega, snapshot.omega);
    }

    _Apply_Chassis_Velocity(snapshot.vx, snapshot.vy, snapshot.omega, active);

    // 4b) 夹爪跟随全局手动使能;具体 POS1/POS2 由 _Process_Function_Key_Edge 边沿设置
    _Clamp_Control(key_code, is_enabled);

    // 5) 调试观测（不受使能门控影响，直接显示接收到的摇杆速度）
    debug_remote_key_code_   = key_code;
    debug_remote_is_recent_  = is_recent;
    debug_remote_is_enabled_ = is_enabled;
    debug_remote_vx_         = is_recent ? snapshot.vx    : 0.0f;
    debug_remote_vy_         = is_recent ? snapshot.vy    : 0.0f;
    debug_remote_omega_      = is_recent ? snapshot.omega : 0.0f;
}

/**
 * @brief 取 ROS 命令快照（线程安全）
 *        仅在锁内做拷贝，时间判定放到锁外，缩短临界区。
 * @param[out] out_cmd      命令快照
 * @param[out] out_is_recent 是否在超时阈值内
 * @return out_is_recent 同值，便于链式使用
 */
bool Class_Robot::_Snapshot_Remote_Command(Ros_Remote_Command &out_cmd, bool &out_is_recent)
{
    {
        std::lock_guard<std::mutex> lock(ros_cmd_mutex_);
        out_cmd = ros_cmd_;
    }
    const int64_t now_ns = Steady_Now_Ns();
    out_is_recent = out_cmd.has_twist &&
                    (out_cmd.last_update_ns > 0) &&
                    ((now_ns - out_cmd.last_update_ns) <= kRemoteCmdTimeoutNs);
    return out_is_recent;
}

/**
 * @brief 更新手动控制使能（START 长按开 / BACK 长按关）
 *        在状态翻转时打印一次日志便于现场观察。
 * @param key_code  当前按键状态字
 * @return 当前是否使能
 */
bool Class_Robot::_Update_Manual_Enable_State(uint16_t key_code)
{
    const bool was_enabled = logf710_remote_.Is_Control_Enabled();
    logf710_remote_.Update_Control_Enable(key_code, 1U);
    const bool is_enabled = logf710_remote_.Is_Control_Enabled();

    if (!was_enabled && is_enabled)
        std::cout << "[ROBOT] ROS manual control enabled by START long press." << std::endl;
    else if (was_enabled && !is_enabled)
        std::cout << "[ROBOT] ROS manual control disabled by BACK long press." << std::endl;

    return is_enabled;
}

/**
 * @brief 功能键边沿动作
 *        仅在控制使能时响应，目前为预留 hook（导航/机构动作）。
 * @param key_code   当前按键
 * @param is_enabled 是否处于手动使能态
 */
void Class_Robot::_Process_Function_Key_Edge(uint16_t key_code, bool is_enabled)
{
    if (!is_enabled) return;

    uint16_t rising_key = LogF710_Key_IDLE;
    if (!logf710_remote_.Check_Key_Rising_Edge(key_code, &rising_key)) return;

    switch (rising_key)
    {
    // ============ AUTO-PILOT TRIGGERS ============
    case LogF710_Key_X:  Auto_Pilot.Stop();    std::cout << "[ROBOT] Key X: Auto-Pilot STOP"     << std::endl; break;
    case LogF710_Key_Y:  Auto_Pilot.Start(0);  std::cout << "[ROBOT] Key Y: Auto-Pilot path 0"   << std::endl; break;
    case LogF710_Key_A:
        // 夹爪取放序列:闭合 → 等 → 张开 → 等 → 闭合(末态);仅在 Clamp ENABLE 下生效
        Clamp.Trigger_Pick_Place_Sequence();
        std::cout << "[ROBOT] Key A: Clamp pick-place sequence triggered" << std::endl;
        break;
    case LogF710_Key_B:
        // 已迁移到 _Chassis_Control() 中按住持续 push (LinkX TX slot 不自清,边沿单 push 会被
        // 固件每 cycle 重发到总线; 改为按住才入队,新鲜帧覆盖旧帧)
        break;
    case LogF710_Key_LB: Auto_Pilot.Start(2);  std::cout << "[ROBOT] Key LB: Auto-Pilot path 2"  << std::endl; break;
    case LogF710_Key_RB:
        // 夹爪压合:大+小 Pitch 同时去 POS2
        Clamp.Set_Pitch_Large_State(L_PITCH_POS2);
        Clamp.Set_Pitch_Small_State(S_PITCH_POS2);
        std::cout << "[ROBOT] Key RB: Clamp -> POS2 (close)" << std::endl;
        break;
    case LogF710_Key_LT:
        // 夹爪复位:大+小 Pitch 同时回 POS1
        // 注意:LT 仅在 DInput 布局下作为按键被解析,XInput 模式下变 axis 不响应
        Clamp.Set_Pitch_Large_State(L_PITCH_POS1);
        Clamp.Set_Pitch_Small_State(S_PITCH_POS1);
        std::cout << "[ROBOT] Key LT: Clamp -> POS1 (open)" << std::endl;
        break;
    // ============ 预留按键 (无任务,仅打印边沿) ============
    // RT: XInput 下为 axis,仅 DInput 布局下作为按键被解析
    // Back/Start: 长按已被 _Update_Manual_Enable 消费 (disable/enable);短按时才进入这里
    case LogF710_Key_RT:    std::cout << "[ROBOT] Key RT: (unassigned)"    << std::endl; break;
    case LogF710_Key_Back:  std::cout << "[ROBOT] Key Back: (unassigned)"  << std::endl; break;
    case LogF710_Key_Start: std::cout << "[ROBOT] Key Start: (unassigned)" << std::endl; break;
    case LogF710_Key_Up:    std::cout << "[ROBOT] Key Up: (unassigned)"    << std::endl; break;
    case LogF710_Key_Down:  std::cout << "[ROBOT] Key Down: (unassigned)"  << std::endl; break;
    case LogF710_Key_Left:  std::cout << "[ROBOT] Key Left: (unassigned)"  << std::endl; break;
    case LogF710_Key_Right: std::cout << "[ROBOT] Key Right: (unassigned)" << std::endl; break;
    default: break;
    }
}

/**
 * @brief 夹爪使能跟随全局手动控制状态
 *        Start 长按 -> is_enabled=true -> Clamp ENABLE;
 *        Back 长按或控制超时 -> is_enabled=false -> Clamp DISABLE。
 *        具体 POS1/POS2 切换由 _Process_Function_Key_Edge 处理。
 */
void Class_Robot::_Clamp_Control(uint16_t /*key_code*/, bool is_enabled)
{
    // Clamp.Set_Clamp_Control_Type(is_enabled ? CLAMP_CONTROL_ENABLE : CLAMP_CONTROL_DISABLE);
}

/**
 * @brief 将目标速度下发到底盘
 *        active=true 时透传速度并 ENABLE；否则零速 + DISABLE，
 *        统一封装确保任何控制源失效时都进入安全状态。
 */
void Class_Robot::_Apply_Chassis_Velocity(float vx, float vy, float omega, bool active)
{
    if (active)
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ENABLE);
        Chassis.Set_Target_Velocity_X(vx);
        Chassis.Set_Target_Velocity_Y(vy);
        Chassis.Set_Target_Omega(omega);
    }
    else
    {
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
void Class_Robot::_ROS2_Remote_Spin_Loop()
{
    auto node = std::make_shared<rclcpp::Node>("soem_remote_bridge_node");

    auto sub_cmd = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 20,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            _Update_Remote_Twist(static_cast<float>(msg->linear.x),
                                 static_cast<float>(msg->linear.y),
                                 static_cast<float>(msg->angular.z));
        });

    auto sub_buttons = node->create_subscription<std_msgs::msg::UInt16>(
        "/robot_buttons", 20,
        [this](const std_msgs::msg::UInt16::SharedPtr msg)
        {
            _Update_Remote_Buttons(msg->data);
        });

    // sub_* 必须保活到循环结束，否则订阅会被析构
    (void)sub_cmd;
    (void)sub_buttons;

    rclcpp::WallRate rate(200);
    while (ros_bridge_running_.load() && rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

/**
 * @brief 写入最新速度命令快照（线程安全）
 *        ROS spin 线程调用，更新时间戳供主线程做超时判定。
 */
void Class_Robot::_Update_Remote_Twist(float vx, float vy, float omega)
{
    std::lock_guard<std::mutex> lock(ros_cmd_mutex_);
    ros_cmd_.vx             = vx;
    ros_cmd_.vy             = vy;
    ros_cmd_.omega          = omega;
    ros_cmd_.has_twist      = true;
    ros_cmd_.last_update_ns = Steady_Now_Ns();
}

/**
 * @brief 写入最新按键状态快照（线程安全）
 */
void Class_Robot::_Update_Remote_Buttons(uint16_t buttons)
{
    std::lock_guard<std::mutex> lock(ros_cmd_mutex_);
    ros_cmd_.buttons        = buttons;
    ros_cmd_.has_buttons    = true;
    ros_cmd_.last_update_ns = Steady_Now_Ns();
}


// ============================================================================
//  手动修正 CSV 日志
// ============================================================================

void Class_Robot::_Open_Manual_Correction_CSV()
{
    if (manual_corr_csv_opened_) return;

    auto now = std::chrono::system_clock::now();
    auto tt  = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&tt, &tm_buf);

    std::ostringstream fname;
    fname << "var_data/manual_correction_"
          << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".csv";

    manual_corr_csv_.open(fname.str(), std::ios::out | std::ios::trunc);
    if (!manual_corr_csv_.is_open())
    {
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
           "now_vx,now_vy,now_omega,"
           "enc_deg_0,enc_deg_1,enc_deg_2,enc_deg_3,"
           "odrive_omega_0,odrive_omega_1,odrive_omega_2,odrive_omega_3,"
           "dm_omega_0,dm_omega_1,dm_omega_2,dm_omega_3,"
           "dm_torque_0,dm_torque_1,dm_torque_2,dm_torque_3\n";

    manual_corr_csv_opened_ = true;
    manual_corr_tick_ = 0;
    std::cout << "[ROBOT] Manual correction CSV started: " << fname.str() << std::endl;
}

void Class_Robot::_Close_Manual_Correction_CSV()
{
    if (manual_corr_csv_.is_open())
    {
        manual_corr_csv_.flush();
        manual_corr_csv_.close();
        std::cout << "[ROBOT] Manual correction CSV closed (" << manual_corr_tick_ << " ticks)" << std::endl;
    }
    manual_corr_csv_opened_ = false;
}

void Class_Robot::_Log_Manual_Correction(float raw_vx, float raw_vy, float raw_omega,
                                         float corrected_omega)
{
    if (!manual_corr_csv_opened_)
        _Open_Manual_Correction_CSV();

    if (!manual_corr_csv_.is_open()) return;

    if ((manual_corr_tick_ % kManualCorrCsvDecimate_) == 0)
    {
        manual_corr_csv_
            << manual_corr_tick_
            << "," << Chassis.OPS.Get_Pos_X_Mm()
            << "," << Chassis.OPS.Get_Pos_Y_Mm()
            << "," << Chassis.OPS.Get_Yaw_Deg()
            << "," << raw_vx
            << "," << raw_vy
            << "," << raw_omega
            << "," << corrected_omega
            << "," << Chassis.Get_Now_Velocity_X()
            << "," << Chassis.Get_Now_Velocity_Y()
            << "," << Chassis.Get_Now_Omega();

        for (int i = 0; i < 4; ++i)
            manual_corr_csv_ << "," << Chassis.Encoder_Steer[i].Get_Wheel_Angle_True();

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
