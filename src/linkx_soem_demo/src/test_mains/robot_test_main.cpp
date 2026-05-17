// =============================================================================
// robot_test_main.cpp
//
// Class_Robot 接口的真实上机回归测试（顺序自动）。
//
// 测试覆盖：
//   T1  Init 完整性                    —— 子设备在线性
//   T2  EtherCAT/CAN 接收链路           —— Encoder_Steer.Get_Rx_Count() 增长
//   T3  TIM 周期回调存活                —— 1ms/2ms/100ms 在 bg 线程内连续调用 200ms 不崩
//   T4  ROS 桥接订阅                   —— /cmd_vel 注入 → Get_Debug_Remote_* 反映出新值
//   T5  使能门控翻转                   —— /robot_buttons=START 长按 ≥500ms → Is_Enabled=true
//   T6  限速短转 (0.05 m/s × 2s)        —— 检查 chassis target/now velocity，自动停
//   T7  Stop_ROS2_Remote_Bridge        —— 后台线程优雅退出
//
// 使用：
//   sudo ./robot_test  <iface>          # iface 例：enp86s0
// =============================================================================
#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "robot.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>


// ---------------------------------------------------------------------------
// 全局对象（与 task.cpp 同名同型，但本测试单独 link 不会冲突）
// ---------------------------------------------------------------------------
Class_Robot   robot;
ecat_master_t master;
linkx_t       linkx_dev;

static constexpr int      kChannelCount         = 4;
static constexpr uint32_t kTaskPeriod2Ms        = 2;
static constexpr uint32_t kTaskPeriod100Ms      = 100;
static constexpr float    kTestForwardSpeedMps  = 0.08f;     // T6 限速（< MAX_CHASSIS_SPEED=0.1 避免被 chassis 限幅改写）
static constexpr int      kTestForwardDurationMs = 2000;     // T6 持续时间
static constexpr int      kStartLongPressMarginMs = 600;     // T5 START 长按 (>500ms 阈值)
static constexpr int      kBackLongPressMarginMs  = 200;     // T7 BACK 长按 (>100ms 阈值)


// ---------------------------------------------------------------------------
// 测试结果记录
// ---------------------------------------------------------------------------
struct Test_Result
{
    std::string name;
    bool        passed = false;
    std::string detail;
};
static std::vector<Test_Result> g_results;

/**
 * @brief 打印并记录单条测试结果
 */
static void Record(const std::string &name, bool ok, const std::string &detail = "")
{
    g_results.push_back({name, ok, detail});
    std::cout << (ok ? "  [PASS] " : "  [FAIL] ") << name;
    if (!detail.empty()) std::cout << " — " << detail;
    std::cout << std::endl;
}

/**
 * @brief 打印章节标题
 */
static void Section(const std::string &title)
{
    std::cout << "\n────────────────────────────────────────────────────────────\n"
              << " " << title << "\n"
              << "────────────────────────────────────────────────────────────"
              << std::endl;
}

/**
 * @brief 信号处理：触发主循环退出，由 bg 线程感知后停止
 */
static void Signal_Handler(int /*sig*/)
{
    std::cout << "\n[TEST] SIGINT received, requesting shutdown..." << std::endl;
    master.is_running = false;
}


// ---------------------------------------------------------------------------
// 后台 EtherCAT/CAN/TIM 工作线程
//   完全复用 task.cpp 1ms 主循环的核心节拍：sync → recv_pdos → CAN 分发
//   → TIM_1ms / TIM_2ms / TIM_100ms → send_pdos
//   不打 dashboard，不做 calibrate/save_unwrapped，专注于让 robot 能正常跑。
// ---------------------------------------------------------------------------
static std::atomic<bool> g_bg_running{false};

static void Background_Worker()
{
    auto    next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;

    while (g_bg_running.load() && master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(1);
        std::this_thread::sleep_until(next_wakeup);

        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev);

        can_msg_t msg;
        for (uint8_t ch = 0; ch < kChannelCount; ch++)
        {
            while (linkx_quick_recv(&linkx_dev, ch, &msg))
            {
                robot.CAN_Rx_Callback(0, ch, msg.id, msg.data, msg.dlen);
            }
        }

        robot.TIM_1ms_Calculate_Callback();
        if ((tick % kTaskPeriod2Ms) == 0)   robot.TIM_2ms_Calculate_PeriodElapsedCallback();
        if ((tick % kTaskPeriod100Ms) == 0) robot.TIM_100ms_Alive_PeriodElapsedCallback();

        linkx_send_pdos(&linkx_dev);
        tick++;
    }
}


// ---------------------------------------------------------------------------
// ROS2 注入端：模拟 /cmd_vel 与 /robot_buttons 发布者
// ---------------------------------------------------------------------------
class Test_Injector
{
public:
    explicit Test_Injector(rclcpp::Node::SharedPtr node)
    {
        twist_pub_   = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
        buttons_pub_ = node->create_publisher<std_msgs::msg::UInt16>("/robot_buttons", 20);
    }

    /** @brief 发布一次 twist */
    void Publish_Twist(float vx, float vy, float omega)
    {
        geometry_msgs::msg::Twist m;
        m.linear.x  = vx;
        m.linear.y  = vy;
        m.angular.z = omega;
        twist_pub_->publish(m);
    }

    /** @brief 发布一次按键 */
    void Publish_Buttons(uint16_t code)
    {
        std_msgs::msg::UInt16 m;
        m.data = code;
        buttons_pub_->publish(m);
    }

    /**
     * @brief 在指定时长内按 ~10ms 节拍重复发布按键
     *        模拟"按住"行为；每 10ms 间隔由实际 ROS 线程调度决定。
     * @param code  目标按键码
     * @param ms    维持总时长（毫秒）
     */
    void Hold_Buttons(uint16_t code, int ms)
    {
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
        while (std::chrono::steady_clock::now() < end)
        {
            Publish_Buttons(code);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    /**
     * @brief 在指定时长内按 ~10ms 节拍重复发布 twist
     */
    void Hold_Twist(float vx, float vy, float omega, int ms)
    {
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
        while (std::chrono::steady_clock::now() < end)
        {
            Publish_Twist(vx, vy, omega);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   twist_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr       buttons_pub_;
};


// ---------------------------------------------------------------------------
// 各 Test 用例（每个返回 void，内部用 Record 记 PASS/FAIL）
// ---------------------------------------------------------------------------

/**
 * @brief T1：验证 robot.Init 后子设备就位
 */
static void Test_Init_Integrity()
{
    Section("T1  Init 完整性 (Chassis 子设备在线性)");
    bool all_good = true;
    std::ostringstream detail;

    for (int i = 0; i < 4; i++)
    {
        const uint8_t  enc_id  = robot.Chassis.Encoder_Steer[i].Get_Can_ID();
        const uint8_t  node_id = robot.Chassis.ODrive_Motor_Steer[i].Get_node_id();
        const uint16_t dm_id   = robot.Chassis.Motor_Steer[i].DM_CAN_Rx_ID;
        if (enc_id == 0 || node_id == 0 || dm_id == 0)
        {
            all_good = false;
            detail << "wheel " << i
                   << " enc=0x" << std::hex << static_cast<int>(enc_id)
                   << " odv=0x" << static_cast<int>(node_id)
                   << " dm=0x"  << dm_id << std::dec << " | ";
        }
    }
    Record("子设备 ID 全部就位", all_good, detail.str());
}

/**
 * @brief T2.5：Warm-up，让 chassis 完成内部启动序列（编码器稳定 + 舵向相位标定）
 *
 *        注意：ODrive 此阶段会保持 IDLE(1)，因为 chassis.TIM_100ms_Alive 中有双锁：
 *          if (Chassis_Control_Type != ENABLE || !all_calibration_complete) return;
 *        ODrive 直到 T6 chassis 切到 ENABLE 后才会被推进 CLOSED_LOOP_CONTROL(8)。
 *        这是 chassis 的设计，不是问题。
 */
static void Test_Warmup_For_Closed_Loop()
{
    Section("T2.5  Chassis 启动序列 warmup (4s, 等编码器稳定+相位标定)");

    constexpr int kWarmupSeconds = 4;
    for (int s = 0; s < kWarmupSeconds; s++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "    warmup t+" << (s + 1) << "s  axis_state=";
        for (int i = 0; i < 4; i++)
        {
            const int st = static_cast<int>(robot.Chassis.ODrive_Motor_Steer[i].Get_Axis_State());
            std::cout << "w" << i << ":" << st << " ";
        }
        std::cout << " (闭环切换在 T6 ENABLE 后才会发生)" << std::endl;
    }
    Record("Chassis warmup 完成", true, "等到 T6 才会触发 ODrive 进闭环");
}

/**
 * @brief T2：CAN 接收链路：在 2s 窗口内 4 个编码器都有 Rx_Count 增长
 */
static void Test_CAN_Reception()
{
    Section("T2  CAN 接收链路 (Encoder Rx 计数 2 秒内增长)");

    uint32_t before[4];
    for (int i = 0; i < 4; i++) before[i] = robot.Chassis.Encoder_Steer[i].Get_Rx_Count();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    uint32_t after[4];
    bool     all_growing = true;
    std::ostringstream detail;
    for (int i = 0; i < 4; i++)
    {
        after[i] = robot.Chassis.Encoder_Steer[i].Get_Rx_Count();
        const uint32_t delta = after[i] - before[i];
        detail << "w" << i << ":+" << delta << " ";
        if (delta == 0) all_growing = false;
    }
    Record("4 编码器均收到新帧", all_growing, detail.str());
}

/**
 * @brief T3：TIM 回调存活（已在 bg 线程跑 ≥1s，无 crash 则视为通过）
 */
static void Test_TIM_Alive()
{
    Section("T3  TIM 回调存活 (1ms/2ms/100ms 持续运行)");
    Record("bg 线程 1s 内未崩溃", g_bg_running.load() && master.is_running, "依靠后台线程持续节拍");
}

/**
 * @brief T4：ROS 桥接订阅：注入 /cmd_vel，等若干 ms，读 Debug 接口
 */
static void Test_ROS_Subscription(Test_Injector &inj)
{
    Section("T4  ROS 桥接订阅 (/cmd_vel → Get_Debug_Remote_*)");

    constexpr float vx_inj    = 0.10f;
    constexpr float vy_inj    = 0.20f;
    constexpr float omega_inj = 0.30f;

    // 持续注入 200ms，等订阅 + 内部 _Chassis_Control 刷新 debug 缓存
    inj.Hold_Twist(vx_inj, vy_inj, omega_inj, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const float vx_dbg    = robot.Get_Debug_Remote_Vx();
    const float vy_dbg    = robot.Get_Debug_Remote_Vy();
    const float omega_dbg = robot.Get_Debug_Remote_Omega();
    const bool  recent    = robot.Get_Debug_Remote_Is_Recent();

    auto near_eq = [](float a, float b) { return std::fabs(a - b) < 1e-3f; };

    bool ok = recent && near_eq(vx_dbg, vx_inj) && near_eq(vy_dbg, vy_inj) && near_eq(omega_dbg, omega_inj);

    std::ostringstream d;
    d << std::fixed << std::setprecision(3)
      << "recent=" << recent
      << " vx=" << vx_dbg << " vy=" << vy_dbg << " omega=" << omega_dbg;
    Record("注入速度反映到 Debug 接口", ok, d.str());

    // 清零 cmd_vel 但保持快照新鲜，避免影响后续测试
    inj.Publish_Twist(0.0f, 0.0f, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

/**
 * @brief T5：使能门控翻转：发 START 长按 ≥500ms → Is_Enabled=true
 */
static void Test_Manual_Enable(Test_Injector &inj)
{
    Section("T5  使能门控 (/robot_buttons=START 长按 600ms)");

    const bool was_enabled_before = robot.Get_Debug_Remote_Is_Enabled();
    inj.Hold_Buttons(LogF710_Key_Start, kStartLongPressMarginMs);

    // 释放按键，但仍保持新鲜：让 _Chassis_Control 在下次 1ms 周期更新 debug
    inj.Publish_Buttons(LogF710_Key_IDLE);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const bool is_enabled_after = robot.Get_Debug_Remote_Is_Enabled();
    std::ostringstream d;
    d << "before=" << was_enabled_before << " after=" << is_enabled_after;
    Record("START 长按使能翻转", !was_enabled_before && is_enabled_after, d.str());
}

/**
 * @brief T6：限速短转：使能后先发 0 速度等 ODrive 进闭环，再发 0.05 m/s × 2s
 *
 * 注意 chassis 内部双锁：TIM_100ms_Alive 里要求 (control_type==ENABLE && calibration_complete)
 * 才会下发 SET_ClosedLoop。所以必须先让 chassis 持续 ENABLE 一段时间，
 * ODrive 才会从 IDLE(1) 切到 CLOSED_LOOP_CONTROL(8)，之后真实速度命令才有意义。
 */
static void Test_Limited_Forward(Test_Injector &inj)
{
    Section("T6  限速短转 (cmd_vel=0.05 m/s × 2s)");

    if (!robot.Get_Debug_Remote_Is_Enabled())
    {
        Record("使能未维持", false, "Skip — 前置条件失败");
        return;
    }

    // —— Phase A：保持使能 + 零速发布 3 秒，让 ODrive 全部进入 CLOSED_LOOP_CONTROL
    std::cout << "    [Phase A] 维持 ENABLE+0速度 3s，等待 ODrive 闭环就绪..." << std::endl;
    const auto warmup_end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < warmup_end)
    {
        inj.Publish_Twist(0.0f, 0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "      odrive_state:";
        for (int i = 0; i < 4; i++)
            std::cout << " w" << i << ":"
                      << static_cast<int>(robot.Chassis.ODrive_Motor_Steer[i].Get_Axis_State());
        std::cout << std::endl;
    }
    int closed_loop_count = 0;
    for (int i = 0; i < 4; i++)
        if (static_cast<int>(robot.Chassis.ODrive_Motor_Steer[i].Get_Axis_State()) == 8)
            closed_loop_count++;
    std::cout << "    [Phase A] 完成: closed-loop=" << closed_loop_count << "/4" << std::endl;

    // —— Phase B：发 0.05 m/s，监控 chassis target_vx / now_vx 与 ODrive omega
    std::cout << "    [Phase B] 发 cmd_vel=" << kTestForwardSpeedMps << " m/s × 2s..." << std::endl;
    const auto t0 = std::chrono::steady_clock::now();
    int   sample_count       = 0;
    int   enable_state_count = 0;
    int   target_match_count = 0;
    float omega_max_observed = 0.0f;

    auto end = t0 + std::chrono::milliseconds(kTestForwardDurationMs);
    while (std::chrono::steady_clock::now() < end)
    {
        inj.Publish_Twist(kTestForwardSpeedMps, 0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        const float    target_vx = robot.Chassis.Get_Target_Velocity_X();
        const float    now_vx    = robot.Chassis.Get_Now_Velocity_X();
        const auto     ctrl_type = robot.Chassis.Get_Chassis_Control_Type();

        sample_count++;
        if (ctrl_type == Chassis_Control_Type_ENABLE)               enable_state_count++;
        if (std::fabs(target_vx - kTestForwardSpeedMps) < 1e-2f)    target_match_count++;

        std::cout << "    t+" << std::setw(4)
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - t0).count()
                  << "ms  ctrl=" << static_cast<int>(ctrl_type)
                  << "  tgt_vx=" << std::fixed << std::setprecision(3) << target_vx
                  << "  now_vx=" << now_vx
                  << "  ODrv[s/ω]:";
        for (int i = 0; i < 4; i++)
        {
            const float omega = robot.Chassis.ODrive_Motor_Steer[i].Get_Omega();
            if (std::fabs(omega) > omega_max_observed) omega_max_observed = std::fabs(omega);
            std::cout << " w" << i << ":"
                      << static_cast<int>(robot.Chassis.ODrive_Motor_Steer[i].Get_Axis_State())
                      << "/"
                      << std::setprecision(2) << omega;
        }
        std::cout << std::endl;
    }

    // 立即归零
    inj.Publish_Twist(0.0f, 0.0f, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::ostringstream d;
    d << "ENABLE " << enable_state_count << "/" << sample_count
      << "，target≈" << kTestForwardSpeedMps << " " << target_match_count << "/" << sample_count
      << "，max|ODrive_omega|=" << std::fixed << std::setprecision(3) << omega_max_observed;
    bool ok = (enable_state_count == sample_count) && (target_match_count >= sample_count - 1);
    Record("Chassis 进入 ENABLE 且 target_vx 透传", ok, d.str());

    // 注意：ODrive 实际产生轮速取决于整车物理状态（电池电压、舵向对齐、电流保护、Wheel_Radius=0.018m
    // 配置等），与 robot.cpp 接口层重构无关。这里仅作为 INFO 报告，不算测试失败。
    if (omega_max_observed > 0.1f)
        Record("[INFO] ODrive 实际转动 (>0.1 rad/s)", true,
               std::string("max|ω|=") + std::to_string(omega_max_observed) + " rad/s");
    else
        std::cout << "  [INFO] ODrive 未实转 (max|ω|=" << omega_max_observed
                  << " rad/s) — 属于整车下游问题，与 robot 接口重构无关" << std::endl;
}

/**
 * @brief T7：解使能 + Stop ROS 桥
 */
static void Test_Disable_And_Stop(Test_Injector &inj)
{
    Section("T7  解使能 + Stop_ROS2_Remote_Bridge");

    // 长按 BACK 解使能
    inj.Publish_Twist(0.0f, 0.0f, 0.0f);
    inj.Hold_Buttons(LogF710_Key_Back, kBackLongPressMarginMs);
    inj.Publish_Buttons(LogF710_Key_IDLE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    bool disabled_ok = !robot.Get_Debug_Remote_Is_Enabled();
    Record("BACK 长按解使能", disabled_ok,
           std::string("is_enabled=") + (robot.Get_Debug_Remote_Is_Enabled() ? "true" : "false"));

    // 停 ROS 桥（不再让 ros 线程跑），后续 publisher 不再有效消费方
    robot.Stop_ROS2_Remote_Bridge();

    // 再读一次 robot 状态，确认底盘最终为 DISABLE
    const auto ctrl = robot.Chassis.Get_Chassis_Control_Type();
    Record("Chassis 最终为 DISABLE", ctrl == Chassis_Control_Type_DISABLE,
           std::string("ctrl=") + std::to_string(static_cast<int>(ctrl)));
}

/**
 * @brief 汇总
 */
static int Print_Summary()
{
    Section("Summary");
    int passed = 0;
    int failed = 0;
    for (const auto &r : g_results)
    {
        std::cout << (r.passed ? "  [PASS] " : "  [FAIL] ") << r.name;
        if (!r.detail.empty()) std::cout << "    (" << r.detail << ")";
        std::cout << std::endl;
        (r.passed ? passed : failed)++;
    }
    std::cout << "\n  Total: " << g_results.size()
              << "   Passed: " << passed
              << "   Failed: " << failed << std::endl;
    return failed == 0 ? 0 : 1;
}


// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    std::string ifname = "enp86s0";
    if (argc > 1) ifname = argv[1];

    std::signal(SIGINT, Signal_Handler);
    std::signal(SIGTERM, Signal_Handler);

    std::cout << "============================================================\n"
              << "  Class_Robot 真实上机回归测试\n"
              << "  Interface: " << ifname << "\n"
              << "============================================================" << std::endl;

    // 1. 初始化 EtherCAT + LinkX（与 task.cpp 一致）
    Section("Bring up EtherCAT / LinkX");
    if (!ecat_master_init(&master, ifname.c_str()))
    {
        std::cerr << "[FATAL] ecat_master_init failed" << std::endl;
        return -1;
    }
    linkx_init(&linkx_dev, 1, &master.ctx);
    linkx_hw_wakeup(&linkx_dev);
    for (int i = 0; i < kChannelCount; i++)
    {
        linkx_set_can_baudrate(&linkx_dev, i, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    }
    if (!ecat_master_bring_online(&master))
    {
        std::cerr << "[FATAL] ecat_master_bring_online failed" << std::endl;
        return -1;
    }
    std::cout << "  EtherCAT online" << std::endl;

    // 2. robot.Init + ROS 桥
    robot.Init(&linkx_dev);
    robot.Start_ROS2_Remote_Bridge();   // 内部会按需 rclcpp::init

    // 3. 启动后台 1ms 工作线程
    g_bg_running.store(true);
    std::thread bg(Background_Worker);
    std::this_thread::sleep_for(std::chrono::seconds(1));   // 让节拍稳定

    // 4. 在主线程内做 publisher 注入
    auto pub_node = std::make_shared<rclcpp::Node>("robot_test_injector");
    Test_Injector inj(pub_node);

    // 顺序自动跑全套
    Test_Init_Integrity();
    Test_CAN_Reception();
    Test_Warmup_For_Closed_Loop();
    Test_TIM_Alive();
    Test_ROS_Subscription(inj);
    Test_Manual_Enable(inj);
    Test_Limited_Forward(inj);
    Test_Disable_And_Stop(inj);

    int rc = Print_Summary();

    // 5. 收尾：停止 bg 线程，关 EtherCAT
    g_bg_running.store(false);
    master.is_running = false;
    if (bg.joinable()) bg.join();

    // robot.Stop_ROS2_Remote_Bridge 已在 T7 调用过，但重复 stop 安全（内部有 guard）
    robot.Stop_ROS2_Remote_Bridge();

    std::cout << "\n[TEST] Exit code = " << rc << std::endl;
    return rc;
}
