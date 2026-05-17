// motor_calib_main.cpp
//
// 单轮参数标定工具：动摩擦力 / 静摩擦力 / 转动惯量。
// 全部走 MIT 模式，调用 Class_Motor_DM_Normal::Begin_*_Calibration() +
// Calibration_Tick(dt) 完成；数据采集与平均都在 motor 类内部完成。
//
// ⚠️ 测试前请把目标轮架空（轮胎不接触地面），否则地面摩擦会污染测量。
//
// 用法（必须 sudo）：
//   sudo IFNAME=enp86s0 ./motor_calib --wheel 0 --test friction
//   sudo IFNAME=enp86s0 ./motor_calib --wheel 0 --test stiction
//   sudo IFNAME=enp86s0 ./motor_calib --wheel 0 --test inertia --tstep 0.5 --friction 0.05
//   sudo IFNAME=enp86s0 ./motor_calib --wheel 0 --test all
//
// 环境变量与 CLI 等价（CLI 优先）：
//   CALIB_WHEEL=0..3
//   CALIB_TEST=friction|stiction|inertia|all
//   CALIB_OMEGA=2.0      Friction 测试速度 (rad/s, 电机轴)
//   CALIB_KD=2.0         Friction MIT D 增益
//   CALIB_WARMUP=1.5     Friction 单方向 warmup 时长 (s)
//   CALIB_MEASURE=2.0    Friction 单方向取样时长 (s)
//   CALIB_TSTEP=0.01     Stiction 力矩步进 (Nm)
//   CALIB_DWELL=0.10     Stiction 每步保持时长 (s)
//   CALIB_THRESH=0.3     Stiction 突破阈值 (rad/s)
//   CALIB_TMAX=1.0       Stiction 上限保护 (Nm)
//   CALIB_INERTIA_T=0.5  Inertia 阶跃力矩 (Nm)
//   CALIB_INERTIA_DUR=0.5 Inertia 加速段时长 (s)
//   CALIB_FRICTION=0.0   Inertia 减除的动摩擦力 (Nm)
//
// 温度保护：每个测试间检查 MOS / Rotor 温度，> 70°C 直接退出。
//

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "crt_chassis.h"
#include "dvc_motor_dm.h"
#include "dvc_encoder.h"
#include "math.h"

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

float env_f(const char *name, float fallback)
{
    const char *v = std::getenv(name);
    if (!v || v[0] == '\0') return fallback;
    char *end = nullptr;
    float r = std::strtof(v, &end);
    return (end == v) ? fallback : r;
}

int env_i(const char *name, int fallback)
{
    const char *v = std::getenv(name);
    if (!v || v[0] == '\0') return fallback;
    return std::atoi(v);
}

// 命令行参数解析：--key value 或 --key=value
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

void can_dispatch(uint8_t ch, uint32_t can_id, uint8_t *data)
{
    const uint32_t id_std = (can_id & 0x7FFU);
    if (ch == 0)
    {
        for (int i = 0; i < STEER_NUM; ++i)
        {
            if (id_std == st_chassis.Motor_Steer[i].DM_CAN_Rx_ID)
            {
                st_chassis.Motor_Steer[i].CAN_RxCpltCallback(data);
                return;
            }
        }
        for (int i = 0; i < STEER_NUM; ++i)
        {
            const uint32_t eid = st_chassis.Encoder_Steer[i].Get_Can_ID();
            if (id_std == eid)
            {
                st_chassis.Encoder_Steer[i].CAN_RxCpltCallback(data);
                return;
            }
        }
    }
}

// 一次温度检查：> 70°C 立即终止
bool temperature_safe(float limit_c = 70.0f)
{
    for (int i = 0; i < STEER_NUM; ++i)
    {
        const float mos_c   = st_chassis.Motor_Steer[i].Get_Now_MOS_Temperature()   - CELSIUS_TO_KELVIN;
        const float rotor_c = st_chassis.Motor_Steer[i].Get_Now_Rotor_Temperature() - CELSIUS_TO_KELVIN;
        if ((mos_c > -50.0f && mos_c < 200.0f && mos_c >= limit_c) ||
            (rotor_c > -50.0f && rotor_c < 200.0f && rotor_c >= limit_c))
        {
            std::cerr << "\n[CALIB][THERMAL] wheel=" << i
                      << " MOS=" << mos_c << "°C  Rotor=" << rotor_c
                      << "°C  >= " << limit_c << "°C  abort.\n";
            return false;
        }
    }
    return true;
}

// 把 4 轮（除目标 wheel 外）维持为零力矩松软（MIT, 全 0）
void hold_other_wheels_neutral(int target_wheel)
{
    for (int i = 0; i < STEER_NUM; ++i)
    {
        if (i == target_wheel) continue;
        auto &m = st_chassis.Motor_Steer[i];
        m.Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        m.Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}

// 单步：EtherCAT 一次收发 + 100ms 心跳维持电机使能。返回 false 表示请求退出。
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
    {
        for (int i = 0; i < STEER_NUM; ++i)
        {
            st_chassis.Motor_Steer[i].TIM_Alive_PeriodElapsedCallback();
            st_chassis.Encoder_Steer[i].TIM_Alive_PeriodElapsedCallback();
            if (st_chassis.Motor_Steer[i].Get_Status() != Motor_DM_Status_ENABLE)
                st_chassis.Motor_Steer[i].CAN_Send_Enter();
        }
    }

    for (int i = 0; i < STEER_NUM; ++i)
        st_chassis.Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
    linkx_send_pdos(&st_linkx);
    return true;
}

// 不阻塞 EtherCAT 链路地等待若干毫秒（一直在 ec_step 主节奏运行，避免 RX buffer 溢出）
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

// 阻塞执行一次标定直到完成；过程中维持其他轮零力矩、检查温度。
//   wheel  : 目标轮
//   timeout_s: 强制超时（防 stiction 死循环）
// 返回 calib_result（finished=true 后）
Struct_Motor_DM_Calib_Result run_calibration_blocking(int wheel, float timeout_s)
{
    auto &mot = st_chassis.Motor_Steer[wheel];
    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;
    const float dt = static_cast<float>(kCtrlPeriodMs) / 1000.0f;
    float elapsed = 0.0f;
    uint32_t print_div = 0;

    while (st_running.load() && st_master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        if (!ec_step(tick)) break;

        hold_other_wheels_neutral(wheel);
        mot.Calibration_Tick(dt);

        if (mot.Is_Calibration_Finished()) break;

        if ((print_div++ % 200) == 0)  // 5 Hz 打点
        {
            const auto r = mot.Get_Calibration_Result();
            std::cout << "  [t=" << std::fixed << std::setprecision(2) << elapsed
                      << "s phase=" << r.phase
                      << "  status=" << static_cast<int>(mot.Get_Status())
                      << "  ctrl_st=" << static_cast<int>(mot.Get_Now_Control_Status())
                      << "  rad=" << mot.Get_Now_Radian()
                      << "  omega=" << mot.Get_Now_Omega()
                      << "  torque=" << mot.Get_Now_Torque() << "]\r" << std::flush;
        }
        if (!temperature_safe(70.0f)) { mot.Stop_Calibration(); break; }

        elapsed += dt;
        if (elapsed > timeout_s)
        {
            std::cerr << "\n[CALIB][TIMEOUT] " << timeout_s << "s reached, abort.\n";
            mot.Stop_Calibration();
            break;
        }

        tick++;
        std::this_thread::sleep_until(next_wakeup);
    }
    std::cout << "\n";
    return mot.Get_Calibration_Result();
}

}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string ifname = std::getenv("IFNAME") ? std::getenv("IFNAME") : "enp86s0";

    // ---- 解析参数（CLI 优先）----
    const int wheel = std::atoi(cli_get(argc, argv, "wheel",
                                        std::to_string(env_i("CALIB_WHEEL", 0)).c_str()));
    if (wheel < 0 || wheel >= STEER_NUM)
    {
        std::cerr << "[CALIB] invalid wheel " << wheel << " (0~" << (STEER_NUM - 1) << ")\n";
        return -1;
    }

    const std::string test = cli_get(argc, argv, "test",
                                     std::getenv("CALIB_TEST") ? std::getenv("CALIB_TEST") : "all");

    const float omega_target  = std::strtof(cli_get(argc, argv, "omega",
                                std::to_string(env_f("CALIB_OMEGA", 2.0f)).c_str()), nullptr);
    const float kd_loop       = std::strtof(cli_get(argc, argv, "kd",
                                std::to_string(env_f("CALIB_KD", 2.0f)).c_str()), nullptr);
    const float warmup        = std::strtof(cli_get(argc, argv, "warmup",
                                std::to_string(env_f("CALIB_WARMUP", 1.5f)).c_str()), nullptr);
    const float measure       = std::strtof(cli_get(argc, argv, "measure",
                                std::to_string(env_f("CALIB_MEASURE", 2.0f)).c_str()), nullptr);

    const float t_step        = std::strtof(cli_get(argc, argv, "tstep",
                                std::to_string(env_f("CALIB_TSTEP", 0.01f)).c_str()), nullptr);
    const float dwell         = std::strtof(cli_get(argc, argv, "dwell",
                                std::to_string(env_f("CALIB_DWELL", 0.10f)).c_str()), nullptr);
    const float thresh        = std::strtof(cli_get(argc, argv, "thresh",
                                std::to_string(env_f("CALIB_THRESH", 0.3f)).c_str()), nullptr);
    const float t_max         = std::strtof(cli_get(argc, argv, "tmax",
                                std::to_string(env_f("CALIB_TMAX", 1.0f)).c_str()), nullptr);

    const float inertia_T     = std::strtof(cli_get(argc, argv, "inertia_t",
                                std::to_string(env_f("CALIB_INERTIA_T", 0.5f)).c_str()), nullptr);
    const float inertia_dur   = std::strtof(cli_get(argc, argv, "inertia_dur",
                                std::to_string(env_f("CALIB_INERTIA_DUR", 0.5f)).c_str()), nullptr);
    float friction_known      = std::strtof(cli_get(argc, argv, "friction",
                                std::to_string(env_f("CALIB_FRICTION", 0.0f)).c_str()), nullptr);

    std::cout << "===============================================\n"
              << "  Motor Parameter Calibration  (DM Normal MIT)\n"
              << "  IFNAME : " << ifname << "\n"
              << "  WHEEL  : " << wheel << "  (架空提示: 务必让该轮悬空!)\n"
              << "  TEST   : " << test << "\n"
              << "===============================================\n";

    // ---- EtherCAT / LinkX / Chassis 初始化 ----
    if (!ecat_master_init(&st_master, ifname.c_str()))
    {
        std::cerr << "[CALIB] ecat_master_init failed\n";
        return -1;
    }
    linkx_init(&st_linkx, 1, &st_master.ctx);
    linkx_hw_wakeup(&st_linkx);
    for (int i = 0; i < kChannelCount; i++)
        linkx_set_can_baudrate(&st_linkx, i, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    if (!ecat_master_bring_online(&st_master))
    {
        std::cerr << "[CALIB] ecat bring_online failed\n";
        return -1;
    }
    st_chassis.Init(&st_linkx);
    st_chassis.Init_Motor_Params();
    st_chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

    // ---- 等待编码器首帧（最多 3 秒）----
    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;
    while (st_running.load() && st_master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        ec_step(tick);
        if ((tick % 100) == 0)
        {
            bool ok = true;
            for (int i = 0; i < STEER_NUM; ++i)
                if (!st_chassis.Encoder_Steer[i].Has_Valid_Wheel_Posture())
                { ok = false; break; }
            if (ok) break;
        }
        if (tick > 3000) break;
        tick++;
        std::this_thread::sleep_until(next_wakeup);
    }

    // ---- 使能电机 ----
    std::cout << "[CALIB] enabling motors...\n";
    for (int i = 0; i < STEER_NUM; ++i) st_chassis.Motor_Steer[i].CAN_Send_Enter();
    linkx_send_pdos(&st_linkx);
    ec_busywait_ms(tick, 150);

    // ---- 把全部轮先压成"零力矩松软" ----
    for (int i = 0; i < STEER_NUM; ++i)
    {
        auto &m = st_chassis.Motor_Steer[i];
        m.Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        m.Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    // ---- 提示用户最后确认架空 ----
    std::cout << "\n>>> 请确认 wheel " << wheel << " 已架空。3 秒后开始测试...\n";
    ec_busywait_ms(tick, 3000);

    auto &mot = st_chassis.Motor_Steer[wheel];
    std::ofstream summary("/home/rc/Ethercat_R1/var_data/motor_calib_result.txt", std::ios::app);

    auto write_summary_header = [&]() {
        if (summary.is_open())
        {
            std::time_t now = std::time(nullptr);
            summary << "\n===== "
                    << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S")
                    << "  wheel=" << wheel << "  test=" << test << " =====\n";
        }
    };
    write_summary_header();

    // ---- 测试 1：动摩擦 ----
    if (test == "friction" || test == "all")
    {
        std::cout << "\n--- [1/3] FRICTION calibration ---\n";
        mot.Begin_Friction_Calibration(omega_target, kd_loop, warmup, measure);
        const float timeout = (warmup + measure) * 2.0f + 2.0f;
        auto r = run_calibration_blocking(wheel, timeout);
        std::cout << std::fixed << std::setprecision(4)
                  << "  result: T+=" << r.friction_torque_pos_nm
                  << "  T-=" << r.friction_torque_neg_nm
                  << "  AVG=" << r.friction_torque_avg_nm
                  << " Nm  (omega+=" << std::setprecision(3) << r.friction_omega_pos_actual
                  << "  omega-=" << r.friction_omega_neg_actual << ")\n"
                  << std::defaultfloat;
        if (summary.is_open())
            summary << std::fixed << std::setprecision(4)
                    << "FRICTION: T+=" << r.friction_torque_pos_nm
                    << "  T-=" << r.friction_torque_neg_nm
                    << "  AVG=" << r.friction_torque_avg_nm
                    << "  omega+=" << r.friction_omega_pos_actual
                    << "  omega-=" << r.friction_omega_neg_actual << "\n"
                    << std::defaultfloat;
        // 用动摩擦平均值给 inertia 测试自动用上
        if (r.success && (test == "all") && friction_known < 1e-6f)
            friction_known = r.friction_torque_avg_nm;
        // 冷却（持续 ec_step 不间断 EtherCAT 链路）
        ec_busywait_ms(tick, 800);
    }

    // ---- 测试 2：静摩擦 ----
    if ((test == "stiction" || test == "all") && st_running.load())
    {
        std::cout << "\n--- [2/3] STICTION calibration ---\n";
        mot.Begin_Stiction_Calibration(t_step, dwell, thresh, t_max);
        // 最坏 (T_max / step) * dwell
        const float timeout = (t_max / std::max(1e-3f, t_step)) * dwell + 2.0f;
        auto r = run_calibration_blocking(wheel, timeout);
        std::cout << std::fixed << std::setprecision(4)
                  << "  result: stiction T=" << r.stiction_torque_nm
                  << " Nm  breakaway omega=" << std::setprecision(3) << r.stiction_breakaway_omega
                  << " rad/s  success=" << (r.success ? "YES" : "NO") << "\n"
                  << std::defaultfloat;
        if (summary.is_open())
            summary << std::fixed << std::setprecision(4)
                    << "STICTION: T=" << r.stiction_torque_nm
                    << "  breakaway omega=" << r.stiction_breakaway_omega
                    << "  success=" << r.success << "\n"
                    << std::defaultfloat;
        ec_busywait_ms(tick, 800);
    }

    // ---- 测试 3：转动惯量 ----
    if ((test == "inertia" || test == "all") && st_running.load())
    {
        std::cout << "\n--- [3/3] INERTIA calibration ---\n";
        std::cout << "  using friction_known=" << friction_known << " Nm\n";
        mot.Begin_Inertia_Calibration(inertia_T, friction_known, /*warmup=*/0.5f, inertia_dur);
        const float timeout = inertia_dur + 1.5f;
        auto r = run_calibration_blocking(wheel, timeout);
        std::cout << std::fixed << std::setprecision(6)
                  << "  result: J=" << r.inertia_kgm2
                  << " kg·m²  alpha=" << std::setprecision(3) << r.inertia_alpha_meas
                  << " rad/s²  T_step=" << std::setprecision(4) << r.inertia_torque_step_nm
                  << "  friction_used=" << r.inertia_friction_used_nm
                  << "  success=" << (r.success ? "YES" : "NO") << "\n"
                  << std::defaultfloat;
        if (summary.is_open())
            summary << std::fixed << std::setprecision(6)
                    << "INERTIA: J=" << r.inertia_kgm2
                    << "  alpha=" << std::setprecision(3) << r.inertia_alpha_meas
                    << "  T_step=" << std::setprecision(4) << r.inertia_torque_step_nm
                    << "  friction_used=" << r.inertia_friction_used_nm
                    << "  success=" << r.success << "\n"
                    << std::defaultfloat;
    }

    // ---- 收尾 ----
    std::cout << "\n[CALIB] disabling motors...\n";
    for (int i = 0; i < STEER_NUM; ++i)
    {
        st_chassis.Motor_Steer[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        st_chassis.Motor_Steer[i].CAN_Send_Exit();
    }
    linkx_send_pdos(&st_linkx);
    ec_busywait_ms(tick, 100);
    std::cout << "[CALIB] result also appended to var_data/motor_calib_result.txt\n[CALIB] bye.\n";
    return 0;
}
