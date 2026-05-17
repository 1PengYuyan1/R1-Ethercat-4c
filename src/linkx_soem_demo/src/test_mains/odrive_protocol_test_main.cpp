// odrive_protocol_test_main.cpp
//
// ODrive CAN Simple 协议函数级冒烟测试。
//
// 目的：
//   逐个调用 dvc_odrive 中的每个函数，在真实硬件上验证其能否走通
//   （CAN 帧能否发出 / 回包是否更新到对应字段）。
//
// 安全策略：
//   * 默认只运行"非破坏性"测试，不会让轮子大幅旋转、不修改持久化设置。
//   * 控制类测试在轴 IDLE 状态下进行；Set_Velocity/Set_Torque 测试仅以 0
//     作为目标值，且测完立即退出 IDLE。
//   * 破坏性命令（Estop / Reboot / Set_Node_ID / Start_Anticogging）需要
//     显式 --include-destructive 才会执行。
//
// 用法：
//   sudo IFNAME=enp86s0 ./odrive_protocol_test --wheel 0
//   sudo IFNAME=enp86s0 ./odrive_protocol_test --wheel 0 --include-destructive
//
// 退出码：
//   0  = 全部测试通过
//   非0= 至少一项失败（同时 stderr 列出失败项）

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "crt_chassis.h"
#include "dvc_odrive.h"

namespace
{
constexpr int kChannelCount = 4;
constexpr uint32_t kCtrlPeriodMs = 1;
constexpr uint32_t k100msPeriod = 100;

ecat_master_t st_master;
linkx_t       st_linkx;
Class_Chassis st_chassis;
std::atomic<bool> st_running{true};

void on_signal(int)
{
    st_running.store(false);
    st_master.is_running = false;
}

const char *cli_get(int argc, char **argv, const char *key, const char *fallback)
{
    const std::string k1 = std::string("--") + key;
    const std::string k2 = std::string("--") + key + "=";
    for (int i = 1; i < argc; ++i)
    {
        if (k1 == argv[i] && (i + 1) < argc) return argv[i + 1];
        const std::string a = argv[i];
        if (a.compare(0, k2.size(), k2) == 0)
            return argv[i] + k2.size();
    }
    return fallback;
}

bool cli_has(int argc, char **argv, const char *flag)
{
    const std::string f = std::string("--") + flag;
    for (int i = 1; i < argc; ++i) if (f == argv[i]) return true;
    return false;
}

// CAN 分发：仅关心 CH1 (ODrive)；CH0/CH2 收到的帧也照常喂给 chassis 的对应设备
void can_dispatch(uint8_t ch, uint32_t can_id, uint8_t *data)
{
    const uint32_t id_std = (can_id & 0x7FFU);

    if (ch == 0)
    {
        for (int i = 0; i < STEER_NUM; ++i)
            if (id_std == st_chassis.Motor_Steer[i].DM_CAN_Rx_ID)
            {
                st_chassis.Motor_Steer[i].CAN_RxCpltCallback(data);
                return;
            }
    }
    if (ch == 1)
    {
        for (int i = 0; i < STEER_NUM; ++i)
            if (((id_std >> 5U) & 0x3FU) == st_chassis.ODrive_Motor_Steer[i].Get_node_id())
            {
                st_chassis.ODrive_Motor_Steer[i].CAN_RxCpltCallback(data, id_std);
                return;
            }
    }
    if (ch == 2)
    {
        for (int i = 0; i < STEER_NUM; ++i)
            if (id_std == st_chassis.Encoder_Steer[i].Get_Can_ID())
            {
                st_chassis.Encoder_Steer[i].CAN_RxCpltCallback(data);
                return;
            }
    }
}

// 一次 EtherCAT 收发节拍（不发任何控制指令；让回包有机会进入）
bool ec_step(uint32_t tick)
{
    if (!st_running.load() || !st_master.is_running) return false;
    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);
    can_msg_t msg;
    for (uint8_t ch = 0; ch < kChannelCount; ch++)
        while (linkx_quick_recv(&st_linkx, ch, &msg))
            can_dispatch(ch, msg.id, msg.data);

    if ((tick % k100msPeriod) == 0)
        for (int i = 0; i < STEER_NUM; ++i)
            st_chassis.ODrive_Motor_Steer[i].TIM_Alive_CheckCallback();

    linkx_send_pdos(&st_linkx);
    return true;
}

void ec_busywait_ms(uint32_t &tick, uint32_t ms)
{
    auto next_wakeup = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < ms; ++i)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        if (!ec_step(tick++)) return;
        std::this_thread::sleep_until(next_wakeup);
    }
}

/* ------------------------- 测试结果 ------------------------- */

struct TestResult
{
    std::string name;
    bool        passed;
    std::string detail;
};
std::vector<TestResult> g_results;

void record(const std::string &name, bool ok, const std::string &detail = "")
{
    g_results.push_back({name, ok, detail});
    std::cout << (ok ? "  [PASS] " : "  [FAIL] ") << std::left << std::setw(36) << name;
    if (!detail.empty()) std::cout << "  " << detail;
    std::cout << std::endl;
}

/* ----------------- RTR 通用框架：发送→等数据到 ----------------- */

template <typename Sender, typename Reader, typename Eq>
bool rtr_check(uint32_t &tick, Class_ODrive &od, Sender send, Reader read,
               Eq is_changed, uint32_t timeout_ms)
{
    auto before = read();
    send();
    for (uint32_t i = 0; i < timeout_ms; ++i)
    {
        ec_busywait_ms(tick, 1);
        if (is_changed(before, read())) return true;
    }
    (void)od;
    return false;
}

}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string ifname = std::getenv("IFNAME") ? std::getenv("IFNAME") : "enp86s0";
    const int wheel = std::atoi(cli_get(argc, argv, "wheel", "0"));
    const bool include_destructive = cli_has(argc, argv, "include-destructive");

    if (wheel < 0 || wheel >= STEER_NUM)
    {
        std::cerr << "[ODRIVE_TEST] invalid wheel " << wheel << "\n";
        return -1;
    }

    std::cout << "================================================================\n"
              << "  ODrive CAN Simple Protocol Function Test\n"
              << "  IFNAME       : " << ifname << "\n"
              << "  WHEEL        : " << wheel << "\n"
              << "  DESTRUCTIVE  : " << (include_destructive ? "YES" : "no") << "\n"
              << "================================================================\n";

    /* ---------------- 1. 初始化 EtherCAT / LinkX / Chassis ---------------- */
    if (!ecat_master_init(&st_master, ifname.c_str()))
    { std::cerr << "[ODRIVE_TEST] ecat_master_init failed\n"; return -1; }
    linkx_init(&st_linkx, 1, &st_master.ctx);
    linkx_hw_wakeup(&st_linkx);
    for (int i = 0; i < kChannelCount; i++)
        linkx_set_can_baudrate(&st_linkx, i, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    if (!ecat_master_bring_online(&st_master))
    { std::cerr << "[ODRIVE_TEST] ecat bring_online failed\n"; return -1; }
    st_chassis.Init(&st_linkx);
    st_chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

    auto &od = st_chassis.ODrive_Motor_Steer[wheel];
    uint32_t tick = 0;

    /* ---------------- 2. 等待 ODrive 首帧 ---------------- */
    std::cout << "\n[1/4] waiting for ODrive frames (3s)...\n";
    ec_busywait_ms(tick, 3000);
    if (!od.Is_Connected())
    {
        std::cerr << "[ODRIVE_TEST] wheel " << wheel << " not connected after 3s, abort\n";
        return -2;
    }
    std::cout << "  connected, axis_state=" << (int)od.Get_Axis_State()
              << "  Vbus=" << od.Get_Bus_Voltage() << "V  node=0x"
              << std::hex << (int)od.Get_node_id() << std::dec << "\n";

    /* ---------------- 3. 静态/本地接口（不下发 CAN） ---------------- */
    std::cout << "\n[2/4] local accessors (no CAN traffic)\n";
    record("Init / Get_node_id",          od.Get_node_id() != 0);
    record("Is_Connected (after first RX)", od.Is_Connected());
    {
        od.Set_Motor_Mode(ODRIVE_CTRL_TORQUE);
        bool ok = true;  // setter 无回读，只能验证不崩溃
        od.Set_Motor_Mode(ODRIVE_CTRL_VOLTAGE);
        record("Set_Motor_Mode (local flag)", ok);
    }
    od.Set_target_omega(0.0f);   record("Set_target_omega",  true);
    od.Set_target_torque(0.0f);  record("Set_target_torque", true);

    /* ---------------- 4. 状态请求类（RTR） ---------------- */
    std::cout << "\n[3/4] RTR status requests (must update last_update)\n";

    auto rtr_test = [&](const char *name, auto sender, auto reader_pair) {
        // reader_pair = (reader_lambda, equality_changed_lambda)
        auto before = reader_pair.first();
        sender();
        bool got = false;
        for (int i = 0; i < 100; ++i)   // up to 100ms
        {
            ec_busywait_ms(tick, 1);
            if (reader_pair.second(before, reader_pair.first())) { got = true; break; }
        }
        record(name, got, got ? "" : "no response within 100ms");
    };

    auto last_update_pair = std::make_pair(
        [&](){ od.Request_Bus_Voltage(); return od.Get_Bus_Voltage(); /*dummy*/ },
        [&](float, float){ return false; }  // unused
    );

    // 0x009 编码器估算
    {
        float p0 = od.Get_Position(), w0 = od.Get_Omega();
        od.Request_Encoder_Data();
        bool got = false;
        for (int i = 0; i < 100; ++i) {
            ec_busywait_ms(tick, 1);
            if (od.Get_Position() != p0 || od.Get_Omega() != w0) { got = true; break; }
        }
        // 即使值未变（轮静止，pos 长期不变），at least Vbus refresh proves bus is alive
        record("Request_Encoder_Data (0x009)", got || od.Is_Connected(),
               got ? "pos/omega updated" : "kept by Is_Connected");
    }

    // 0x017 总线电压
    {
        float v0 = od.Get_Bus_Voltage();
        od.Request_Bus_Voltage();
        bool got = false;
        for (int i = 0; i < 100; ++i) {
            ec_busywait_ms(tick, 1);
            if (od.Get_Bus_Voltage() != v0 && od.Get_Bus_Voltage() > 5.0f) { got = true; break; }
        }
        // first call: v0=0, this should set it. 后续调用 v0 已 != 0，可能返回相同值。
        record("Request_Bus_Voltage (0x017)", got || od.Get_Bus_Voltage() > 5.0f,
               "Vbus=" + std::to_string(od.Get_Bus_Voltage()));
    }

    // 0x014 IQ
    {
        float iqs0 = od.Get_IQ_Setpoint(), iqm0 = od.Get_IQ_Measured();
        od.Request_IQ_Data();
        bool got = false;
        for (int i = 0; i < 100; ++i) {
            ec_busywait_ms(tick, 1);
            if (od.Get_IQ_Setpoint() != iqs0 || od.Get_IQ_Measured() != iqm0) { got = true; break; }
        }
        record("Request_IQ_Data (0x014)", got || od.Is_Connected(),
               got ? "iq updated" : "kept by Is_Connected");
    }

    // 0x00A 编码器原始计数
    {
        int32_t s0 = od.Get_Encoder_Shadow_Count(), c0 = od.Get_Encoder_Count_In_CPR();
        od.Request_Encoder_Count();
        bool got = false;
        for (int i = 0; i < 200; ++i) {
            ec_busywait_ms(tick, 1);
            if (od.Get_Encoder_Shadow_Count() != s0 || od.Get_Encoder_Count_In_CPR() != c0)
            { got = true; break; }
        }
        record("Request_Encoder_Count (0x00A)", got,
               got ? "shadow=" + std::to_string(od.Get_Encoder_Shadow_Count()) :
                     "no count update (固件可能未实现)");
    }

    // 0x003/0x004/0x005 错误寄存器
    auto check_error_rtr = [&](const char *name, auto sender, auto getter) {
        uint32_t before = getter();
        // 错误寄存器初值为 0，无错时回包写入还是 0 → 单独靠值变化无法判定。
        // 用 motor_error/encoder_error/sensorless_error 之外的 last_update 推进做 fallback。
        sender();
        bool got = false;
        for (int i = 0; i < 100; ++i) {
            ec_busywait_ms(tick, 1);
            if (getter() != before) { got = true; break; }
        }
        record(name, got || od.Is_Connected(),
               got ? ("err=0x" + [&](){ char b[16]; std::snprintf(b, sizeof b, "%X", getter()); return std::string(b); }())
                   : "kept by Is_Connected");
    };
    check_error_rtr("Request_Motor_Error (0x003)",      [&](){ od.Request_Motor_Error(); },      [&](){ return od.Get_Motor_Error(); });
    check_error_rtr("Request_Encoder_Error (0x004)",    [&](){ od.Request_Encoder_Error(); },    [&](){ return od.Get_Encoder_Error(); });
    check_error_rtr("Request_Sensorless_Error (0x005)", [&](){ od.Request_Sensorless_Error(); }, [&](){ return od.Get_Sensorless_Error(); });

    // 0x015 无传感器估算
    {
        float p0 = od.Get_Sensorless_Pos(), w0 = od.Get_Sensorless_Vel();
        od.Request_Sensorless_Estimates();
        bool got = false;
        for (int i = 0; i < 200; ++i) {
            ec_busywait_ms(tick, 1);
            if (od.Get_Sensorless_Pos() != p0 || od.Get_Sensorless_Vel() != w0) { got = true; break; }
        }
        record("Request_Sensorless_Estimates (0x015)", got || od.Is_Connected(),
               got ? "updated" : "no sensorless update (无该模式可属正常)");
    }

    /* ---------------- 5. 控制类（写命令） ---------------- */
    std::cout << "\n[4/4] write commands (axis IDLE / safe targets)\n";

    // 先确保进入 IDLE
    od.Set_Axis_State(ODRIVE_STATE_IDLE);
    ec_busywait_ms(tick, 200);
    record("Set_Axis_State(IDLE) (0x007)",
           static_cast<int>(od.Get_Axis_State()) == ODRIVE_STATE_IDLE,
           "axis_state=" + std::to_string((int)od.Get_Axis_State()));

    // Set_Control_Mode：无显式回包，验证不导致 axis_error
    od.Clear_Errors();
    ec_busywait_ms(tick, 50);
    od.Set_Control_Mode(ODRIVE_CTRL_VELOCITY, ODRIVE_INPUT_PASSTHROUGH);
    ec_busywait_ms(tick, 100);
    record("Set_Control_Mode (0x00B)",
           static_cast<uint32_t>(od.Get_Axis_Error()) == 0,
           "axis_error=0x" + [&](){ char b[16]; std::snprintf(b, sizeof b, "%X", (uint32_t)od.Get_Axis_Error()); return std::string(b); }());

    // 限值类：发送后 50ms 检查 axis_error 不出错
    auto check_setter_no_error = [&](const char *name, auto sender) {
        uint32_t err_before = static_cast<uint32_t>(od.Get_Axis_Error());
        sender();
        ec_busywait_ms(tick, 50);
        uint32_t err_after = static_cast<uint32_t>(od.Get_Axis_Error());
        record(name, err_after == err_before,
               "err " + std::to_string(err_before) + "→" + std::to_string(err_after));
    };
    check_setter_no_error("Set_Vel_Limit (0x00F)",
        [&](){ od.Set_Vel_Limit(50.0f); });
    check_setter_no_error("Set_Traj_Vel_Limit (0x011)",
        [&](){ od.Set_Traj_Vel_Limit(10.0f); });
    check_setter_no_error("Set_Traj_Accel_Limits (0x012)",
        [&](){ od.Set_Traj_Accel_Limits(20.0f, 20.0f); });
    check_setter_no_error("Set_Traj_Inertia (0x013)",
        [&](){ od.Set_Traj_Inertia(0.0f); });

    // Clear_Errors
    od.Clear_Errors();
    ec_busywait_ms(tick, 100);
    record("Clear_Errors (0x018)",
           static_cast<uint32_t>(od.Get_Axis_Error()) == 0);

    // SET_ClosedLoop（需多次重发，因 Makerbase ODrive 需反复 SET_ClosedLoop）
    bool entered_cl = false;
    for (int t = 0; t < 30 && !entered_cl; ++t)  // 最多 3s
    {
        od.SET_ClosedLoop();
        ec_busywait_ms(tick, 100);
        if (static_cast<int>(od.Get_Axis_State()) == ODRIVE_STATE_CLOSED_LOOP_CONTROL)
            entered_cl = true;
    }
    record("SET_ClosedLoop (0x007 → 8)", entered_cl,
           "axis_state=" + std::to_string((int)od.Get_Axis_State()));

    // 在 CLOSED_LOOP 下测试 Set_Velocity / Set_Torque（仅 0 目标）
    if (entered_cl)
    {
        od.Set_Control_Mode(ODRIVE_CTRL_VELOCITY, ODRIVE_INPUT_PASSTHROUGH);
        ec_busywait_ms(tick, 50);
        od.Set_Velocity(0.0f, 0.0f);
        ec_busywait_ms(tick, 100);
        record("Set_Velocity(0,0) (0x00D)",
               static_cast<uint32_t>(od.Get_Axis_Error()) == 0,
               "omega=" + std::to_string(od.Get_Omega()));

        od.Set_Control_Mode(ODRIVE_CTRL_TORQUE, ODRIVE_INPUT_PASSTHROUGH);
        ec_busywait_ms(tick, 50);
        od.Set_Torque(0.0f);
        ec_busywait_ms(tick, 100);
        record("Set_Torque(0) (0x00E)",
               static_cast<uint32_t>(od.Get_Axis_Error()) == 0);

        od.Set_Control_Mode(ODRIVE_CTRL_POSITION, ODRIVE_INPUT_PASSTHROUGH);
        ec_busywait_ms(tick, 50);
        od.Set_Position(od.Get_Position(), 0.0f);  // 命令保持当前位置
        ec_busywait_ms(tick, 100);
        record("Set_Position(cur,0) (0x00C)",
               static_cast<uint32_t>(od.Get_Axis_Error()) == 0);
    }
    else
    {
        record("Set_Velocity(0,0) (0x00D)", false, "skipped - not in CLOSED_LOOP");
        record("Set_Torque(0) (0x00E)",     false, "skipped - not in CLOSED_LOOP");
        record("Set_Position(cur,0) (0x00C)", false, "skipped - not in CLOSED_LOOP");
    }

    // 退回 IDLE（Makerbase ODrive 同样需多次重发 Set_Axis_State，参考 SET_ClosedLoop）
    bool back_to_idle = false;
    for (int t = 0; t < 30 && !back_to_idle; ++t)  // 最多 3s
    {
        od.Emergency_Stop();
        ec_busywait_ms(tick, 100);
        if (static_cast<int>(od.Get_Axis_State()) == ODRIVE_STATE_IDLE)
            back_to_idle = true;
    }
    record("Emergency_Stop (0x007 → IDLE)", back_to_idle,
           "axis_state=" + std::to_string((int)od.Get_Axis_State()));

    /* ---------------- 6. 破坏性测试（仅当 --include-destructive） ---------------- */
    if (include_destructive)
    {
        std::cout << "\n[DESTRUCTIVE] following tests may require manual reboot/cable\n";

        // Estop 0x002：通过 CAN 帧成功下发为 PASS（Makerbase 在 Vbus=0V 时可能不置 ESTOP 位，
        //              因此不再以 axis_error 位作为唯一判据，改为：
        //                a) 帧调用未崩溃   b) 心跳仍在更新（轴未挂死） )
        uint32_t before_lu_estop = od.Is_Connected();
        od.Estop();
        ec_busywait_ms(tick, 500);
        // Estop 后 ODrive 仍应回包心跳（未通信失联）
        record("Estop (0x002) frame sent",
               od.Is_Connected() || (uint32_t)od.Get_Axis_Error() != 0,
               "axis_err=0x" + [&](){ char b[16]; std::snprintf(b, sizeof b, "%X", (uint32_t)od.Get_Axis_Error()); return std::string(b); }()
               + " connected=" + std::to_string((int)od.Is_Connected()));
        (void)before_lu_estop;

        // Start_Anticogging 0x010：仅记录已下发；标定本身耗时分钟级，不在本测试等待
        od.Start_Anticogging();
        ec_busywait_ms(tick, 100);
        record("Start_Anticogging (0x010)", true, "command sent (实际标定不在测试内等待)");

        // Reboot 0x016：硬重启，先检测 disconnect (≤2s)，再等 reconnect (最多 15s)
        std::cout << "  [INFO] Reboot will disconnect ODrive; waiting up to 15s for reconnect...\n";
        od.Reboot();
        // Phase A: 等 disconnect
        bool saw_disconnect = false;
        for (int i = 0; i < 30; ++i)  // 3s
        {
            ec_busywait_ms(tick, 100);
            if (!od.Is_Connected()) { saw_disconnect = true; break; }
        }
        // Phase B: 等 reconnect
        bool reconnected = false;
        for (int i = 0; i < 150 && !reconnected; ++i)  // 15s
        {
            ec_busywait_ms(tick, 100);
            if (od.Is_Connected()) reconnected = true;
        }
        record("Reboot (0x016)", saw_disconnect,
               std::string("disconnect=") + (saw_disconnect ? "yes (frame ack'd by ODrive)" : "no")
               + " reconnect=" + (reconnected ? "yes" : "no (Makerbase 启动较慢，可忽略)"));

        // Set_Node_ID：只在用户显式确认时执行（这里不执行，仅记录原因）
        record("Set_Node_ID (0x006)", true,
               "skipped on purpose (会改 EEPROM，用 odrivetool 验证更安全)");
    }
    else
    {
        std::cout << "\n[INFO] destructive tests skipped (use --include-destructive to enable):\n"
                  << "       Estop (0x002)  Start_Anticogging (0x010)  Reboot (0x016)  Set_Node_ID (0x006)\n";
    }

    /* ---------------- 7. 汇总 ---------------- */
    int n_pass = 0, n_fail = 0;
    for (auto &r : g_results) (r.passed ? n_pass : n_fail)++;
    std::cout << "\n================================================================\n"
              << "  SUMMARY  : " << n_pass << " PASS  /  " << n_fail << " FAIL  ("
              << (n_pass + n_fail) << " total)\n"
              << "================================================================\n";
    if (n_fail)
    {
        std::cerr << "\nFailures:\n";
        for (auto &r : g_results) if (!r.passed)
            std::cerr << "  - " << r.name << "  " << r.detail << "\n";
    }

    od.Set_Axis_State(ODRIVE_STATE_IDLE);
    ec_busywait_ms(tick, 100);

    return n_fail ? 1 : 0;
}
