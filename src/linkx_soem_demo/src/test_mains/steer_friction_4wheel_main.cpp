// =============================================================================
// 4 舵向电机并行 摩擦力测试 (静摩擦 + 动摩擦)
//
// 用途：
//   一次性同时测 4 个舵向电机 (DM, CAN_CH=0, Master_ID 0x11..0x14) 的：
//     - Stiction (静摩擦)  ：MIT(kp=0,kd=0)，τ 阶梯递增到 |ω|>thresh 即记录
//     - Friction (动摩擦)  ：MIT 速度环 (D 项)，稳态 τ 平均
//
// 设计约束 (与用户口头要求一致)：
//   - 4 个电机并行测试，互不影响 (每个 motor 维持独立的 calib state machine)；
//   - 单轮转动时间 ≤ 10s：
//       Stiction 最坏 = (Tmax / step) * dwell = (1.0/0.02)*0.05 = 2.5s
//       Friction       = (warmup + measure) * 2 = (0.5 + 1.5) * 2 = 4s
//   - 多轮间停稳：每轮结束后 SETTLE_S 秒 0 力矩冷却，所有电机 |ω|<0.1 才进入下一轮。
//
// 警告 / 前置条件：
//   - 这测的是 ★ 舵向 ★ (steering 转向机构) 的摩擦力，不是行驶轮 (ODrive)；
//   - 不需要悬空 (steer shaft 转动只受 bearing + 接地胎面阻力)；
//     ★ 但请确保 4 个舵向有足够 ±70° 转动空间，避免撞限位。
//   - 若需测纯机械摩擦 (无轮胎接地)，请把车架空。
//
// 用法 (必须 sudo)：
//   sudo IFNAME=enp86s0 ./install/linkx_soem_demo/lib/linkx_soem_demo/steer_friction_4wheel
//
// 环境变量：
//   IFNAME      默认 enp86s0
//   ROUNDS      重复整套 sequence 次数，默认 1 (设 3 可做平均)
//   FRIC_OMEGA  动摩擦驱动速度 (rad/s, 电机轴)，默认 2.0  (steer 侧 ≈ 0.57 rad/s = 33°/s)
//   FRIC_KD     动摩擦 MIT D 增益，默认 2.0
//   FRIC_WARM   动摩擦每方向 warmup (s)，默认 0.5
//   FRIC_MEAS   动摩擦每方向 measure (s)，默认 1.5  ⇒ 单轮 4s
//   STIC_STEP   静摩擦力矩步进 (Nm)，默认 0.02
//   STIC_DWELL  静摩擦每步保持 (s)，默认 0.05      ⇒ 最坏 2.5s
//   STIC_THRESH 静摩擦突破阈值 (rad/s)，默认 0.3
//   STIC_TMAX   静摩擦上限 (Nm)，默认 1.0
//   SETTLE_S    轮间静止冷却时长 (s)，默认 2.0
//
// 输出：
//   var_data/steer_friction_4wheel_result.txt  (append；含时间戳 + 每轮 4 行)
// =============================================================================

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "crt_chassis.h"
#include "dvc_motor_dm.h"
#include "dvc_encoder.h"
#include "math.h"

namespace
{
constexpr int      kChannelCount    = 4;
constexpr uint32_t kCtrlPeriodMs    = 1;
constexpr uint32_t k100msPeriod     = 100;
constexpr float    kSettleOmegaThr  = 0.1f;   // settle 判定：|ω| < 0.1 rad/s
constexpr float    kTempLimitC      = 70.0f;

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
    }
    else if (ch == 2)
    {
        for (int i = 0; i < STEER_NUM; ++i)
        {
            const uint32_t eid = st_chassis.Encoder_Steer[i].Get_Can_ID();
            if (id_std == eid || id_std == (0x200U + eid))
            {
                st_chassis.Encoder_Steer[i].CAN_RxCpltCallback(data);
                return;
            }
        }
    }
    // ch 1 (ODrive) / ch 3 (OPS) 本测试忽略
}

// EtherCAT 一次收发 + 心跳维持电机使能。返回 false 表示请求退出。
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

// 温度安全检查：任一 wheel 的 MOS/Rotor 超 70°C 则 abort
bool temperature_safe()
{
    for (int i = 0; i < STEER_NUM; ++i)
    {
        const float mos_c   = st_chassis.Motor_Steer[i].Get_Now_MOS_Temperature()   - CELSIUS_TO_KELVIN;
        const float rotor_c = st_chassis.Motor_Steer[i].Get_Now_Rotor_Temperature() - CELSIUS_TO_KELVIN;
        if ((mos_c > -50.0f && mos_c < 200.0f && mos_c >= kTempLimitC) ||
            (rotor_c > -50.0f && rotor_c < 200.0f && rotor_c >= kTempLimitC))
        {
            std::cerr << "\n[FRIC4][THERMAL] wheel=" << i
                      << " MOS=" << mos_c << "°C  Rotor=" << rotor_c
                      << "°C  >= " << kTempLimitC << "°C  abort.\n";
            return false;
        }
    }
    return true;
}

// 强制所有 4 个电机 0 力矩 0 增益（settle / e-stop 用）
void force_all_zero()
{
    for (int i = 0; i < STEER_NUM; ++i)
        st_chassis.Motor_Steer[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

// 4 轮同时以速度环 (MIT, kp=0, kd) 跑 duration_s 秒
// motor_omega: 电机轴 rad/s (steer 侧 = motor_omega / 3.5)
// 用于"预热转一圈"使减速器啮合、轴承润滑分布均匀
void prerotate_all_wheels(uint32_t &tick, float motor_omega, float kd, float duration_s)
{
    auto next_wakeup = std::chrono::steady_clock::now();
    const float dt = static_cast<float>(kCtrlPeriodMs) / 1000.0f;
    float elapsed = 0.0f;
    uint32_t print_div = 0;
    while (st_running.load() && st_master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        for (int i = 0; i < STEER_NUM; ++i)
        {
            st_chassis.Motor_Steer[i].Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
            st_chassis.Motor_Steer[i].Set_Control_Maintain_Postion(0.0f, motor_omega, 0.0f, 0.0f, kd);
        }
        if (!ec_step(tick++)) break;

        if (!temperature_safe()) break;
        elapsed += dt;
        if (elapsed >= duration_s) break;

        if ((print_div++ % 200) == 0)  // 5 Hz 进度
        {
            std::cout << "  [prerot t=" << std::fixed << std::setprecision(2) << elapsed
                      << "/" << duration_s << "s  ";
            for (int i = 0; i < STEER_NUM; ++i)
            {
                std::cout << "W" << i << ":ω="
                          << std::setprecision(2)
                          << st_chassis.Motor_Steer[i].Get_Now_Omega() << " ";
            }
            std::cout << "]    \r" << std::flush;
        }
        std::this_thread::sleep_until(next_wakeup);
    }
    std::cout << "\n";
    force_all_zero();
}

// 等到 4 轮全部稳定 (|ω|<thr 持续 0.5s) 或 timeout
bool wait_until_settled(uint32_t &tick, float min_seconds, float timeout_seconds)
{
    auto next_wakeup = std::chrono::steady_clock::now();
    float t_calm = 0.0f;
    const float dt = 0.001f;
    float t_total = 0.0f;
    while (st_running.load() && st_master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        force_all_zero();
        if (!ec_step(tick++)) return false;

        bool all_calm = true;
        for (int i = 0; i < STEER_NUM; ++i)
        {
            if (std::fabs(st_chassis.Motor_Steer[i].Get_Now_Omega()) > kSettleOmegaThr)
            { all_calm = false; break; }
        }
        t_calm = all_calm ? (t_calm + dt) : 0.0f;
        t_total += dt;

        if (t_calm >= min_seconds) return true;
        if (t_total >= timeout_seconds)
        {
            std::cerr << "[FRIC4][SETTLE] timeout " << timeout_seconds
                      << "s reached, omega not below " << kSettleOmegaThr << "; continue anyway.\n";
            return false;
        }
        std::this_thread::sleep_until(next_wakeup);
    }
    return false;
}

// 并行运行一组标定 (4 motors 各自的 state machine 已 Begin_*_Calibration 启动好)。
// 等到 4 个 motor 都 finished 或 timeout 强制结束。
void run_parallel_calibration(uint32_t &tick, float timeout_s)
{
    auto next_wakeup = std::chrono::steady_clock::now();
    const float dt = static_cast<float>(kCtrlPeriodMs) / 1000.0f;
    float elapsed = 0.0f;
    uint32_t print_div = 0;
    while (st_running.load() && st_master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        if (!ec_step(tick++)) break;

        // 给 4 个 motor 各自 tick 标定
        for (int i = 0; i < STEER_NUM; ++i)
            st_chassis.Motor_Steer[i].Calibration_Tick(dt);

        // 检查是否 4 个都 finished
        bool all_done = true;
        for (int i = 0; i < STEER_NUM; ++i)
            if (!st_chassis.Motor_Steer[i].Is_Calibration_Finished())
            { all_done = false; break; }
        if (all_done) break;

        if (!temperature_safe())
        {
            for (int i = 0; i < STEER_NUM; ++i) st_chassis.Motor_Steer[i].Stop_Calibration();
            break;
        }

        elapsed += dt;
        if (elapsed > timeout_s)
        {
            std::cerr << "\n[FRIC4][TIMEOUT] " << timeout_s
                      << "s reached, forcing stop on incomplete wheels.\n";
            for (int i = 0; i < STEER_NUM; ++i)
                if (!st_chassis.Motor_Steer[i].Is_Calibration_Finished())
                    st_chassis.Motor_Steer[i].Stop_Calibration();
            break;
        }

        if ((print_div++ % 100) == 0)  // 10 Hz 进度
        {
            std::cout << "  [t=" << std::fixed << std::setprecision(2) << elapsed << "s ";
            for (int i = 0; i < STEER_NUM; ++i)
            {
                const auto r = st_chassis.Motor_Steer[i].Get_Calibration_Result();
                std::cout << "W" << i << "(ph" << r.phase
                          << "/done=" << (r.finished ? "Y" : "N") << ") ";
            }
            std::cout << "]      \r" << std::flush;
        }
        std::this_thread::sleep_until(next_wakeup);
    }
    std::cout << "\n";
}

struct WheelAccum
{
    int      n_stic       = 0;
    int      n_stic_ok    = 0;
    double   sum_stic_nm  = 0.0;
    double   sum_stic_omg = 0.0;

    int      n_fric       = 0;
    int      n_fric_ok    = 0;
    double   sum_fric_pos = 0.0;
    double   sum_fric_neg = 0.0;
    double   sum_fric_avg = 0.0;
    double   sum_fric_omg_pos = 0.0;
    double   sum_fric_omg_neg = 0.0;
};

}  // namespace

int main(int /*argc*/, char ** /*argv*/)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string ifname = std::getenv("IFNAME") ? std::getenv("IFNAME") : "enp86s0";

    const int   rounds       = std::max(1, env_i("ROUNDS", 1));

    const float fric_omega   = env_f("FRIC_OMEGA",   2.0f);
    const float fric_kd      = env_f("FRIC_KD",      2.0f);
    const float fric_warm    = env_f("FRIC_WARM",    0.5f);
    const float fric_meas    = env_f("FRIC_MEAS",    1.5f);

    const float stic_step    = env_f("STIC_STEP",    0.02f);
    const float stic_dwell   = env_f("STIC_DWELL",   0.05f);
    const float stic_thresh  = env_f("STIC_THRESH",  0.3f);
    const float stic_tmax    = env_f("STIC_TMAX",    1.0f);

    const float settle_s     = env_f("SETTLE_S",     2.0f);

    // 预热: 4 轮同步以速度环转 N 整圈 (steer 侧), 让减速器/轴承润滑均匀
    // PREROTATE_TURNS = 0 跳过预热; 默认 1 圈 (steer 侧 360°)
    const float prerot_turns = env_f("PREROTATE_TURNS", 1.0f);
    const float prerot_omega = env_f("PREROTATE_OMEGA", 3.0f);   // 电机轴 rad/s
    const float prerot_kd    = env_f("PREROTATE_KD",    1.5f);
    // 一圈 steer = 2π × REDUCTION (3.5) rad on motor axis = 21.99 rad
    const float prerot_dur   = (prerot_turns > 0.0f)
        ? (prerot_turns * 2.0f * static_cast<float>(M_PI) * REDUCTION_RATIO
           / std::max(0.5f, prerot_omega))
        : 0.0f;

    // 每轮最大转动时间 (用户约束 ≤10s)
    const float fric_round_t = (fric_warm + fric_meas) * 2.0f + 0.5f;  // = 4.5s
    const float stic_round_t = (stic_tmax / std::max(1e-3f, stic_step)) * stic_dwell + 0.5f;

    std::cout << "===============================================================\n"
              << "  4-Wheel Steer Friction Calibration\n"
              << "  IFNAME : " << ifname << "\n"
              << "  ROUNDS : " << rounds << "\n"
              << "  Stiction: step=" << stic_step << "Nm  dwell=" << stic_dwell
              << "s  thresh=" << stic_thresh << "rad/s  Tmax=" << stic_tmax
              << "Nm  (~" << stic_round_t << "s/round)\n"
              << "  Friction: omega=" << fric_omega << "rad/s  kd=" << fric_kd
              << "  warm=" << fric_warm << "s  meas=" << fric_meas
              << "s  (=" << fric_round_t << "s/round)\n"
              << "  Prerotate: " << prerot_turns << "turn(s) @"
              << prerot_omega << "rad/s motor (~" << prerot_dur << "s, kd=" << prerot_kd << ")\n"
              << "  Settle  : " << settle_s << "s between rounds\n"
              << "===============================================================\n";

    if (stic_round_t > 10.5f || fric_round_t > 10.5f)
    {
        std::cerr << "[FRIC4][WARN] one round exceeds 10s constraint; please tighten params.\n";
    }

    // ---- EtherCAT / LinkX 初始化 ----
    if (!ecat_master_init(&st_master, ifname.c_str()))
    {
        std::cerr << "[FRIC4] ecat_master_init failed\n";
        return -1;
    }

    // dual-LinkX：alias=2 (slave_id=2) = classic LinkX，挂舵电机+编码器；
    // alias=1 = FD LinkX (夹爪)，本测试不用。优先按 alias，否则回退到固定拓扑 (classic=slave2)。
    int classic_slave_id = 2;
    {
        constexpr uint16_t kAliasClassic = 2;
        int found = 0;
        for (int i = 1; i <= st_master.ctx.slavecount; ++i)
            if (st_master.ctx.slavelist[i].aliasadr == kAliasClassic)
            { found = i; break; }
        if (found > 0) classic_slave_id = found;
    }
    linkx_init(&st_linkx, static_cast<uint32_t>(classic_slave_id), &st_master.ctx);
    linkx_hw_wakeup(&st_linkx);
    for (int i = 0; i < kChannelCount; i++)
        linkx_set_can_baudrate(&st_linkx, i, 0, 2, 31, 8, 8, 1, 31, 8, 8);

    if (!ecat_master_bring_online(&st_master))
    {
        std::cerr << "[FRIC4] ecat bring_online failed\n";
        return -1;
    }

    st_chassis.Init(&st_linkx);
    st_chassis.Init_Motor_Params();
    st_chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

    // ---- 等待编码器首帧（最多 3 秒）----
    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;
    std::cout << "[FRIC4] waiting for encoders (timeout 3s)...\n";
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
            if (ok) { std::cout << "[FRIC4] encoders ready\n"; break; }
        }
        if (tick > 3000) { std::cerr << "[FRIC4][WARN] encoder timeout\n"; break; }
        tick++;
        std::this_thread::sleep_until(next_wakeup);
    }

    // ---- 使能 4 个 DM 电机 ----
    std::cout << "[FRIC4] enabling 4 DM steer motors...\n";
    for (int i = 0; i < STEER_NUM; ++i)
        st_chassis.Motor_Steer[i].CAN_Send_Enter();
    linkx_send_pdos(&st_linkx);
    ec_busywait_ms(tick, 200);

    // 全部进入 0 力矩，稳定 0.5s
    for (int i = 0; i < STEER_NUM; ++i)
    {
        st_chassis.Motor_Steer[i].Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        st_chassis.Motor_Steer[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    ec_busywait_ms(tick, 500);

    // 倒计时
    if (prerot_turns > 0.0f)
        std::cout << "\n>>> 3 秒后开始测试，请确保 4 个舵向有 "
                  << (prerot_turns >= 1.0f ? "360°" : "±70°")
                  << " 转动空间 (PREROTATE_TURNS=" << prerot_turns << ")...\n";
    else
        std::cout << "\n>>> 3 秒后开始测试，请确保 4 个舵向有 ±70° 转动空间...\n";
    for (int s = 3; s >= 1; --s) { std::cout << "  " << s << "...\n"; ec_busywait_ms(tick, 1000); }

    // ---------------- 预热: 4 轮同步转 N 整圈 ----------------
    if (prerot_turns > 0.0f && st_running.load())
    {
        std::cout << "\n############## PREROTATE: " << prerot_turns
                  << " turn(s) all wheels @ " << prerot_omega
                  << " rad/s motor (~" << prerot_dur << "s) ##############\n";
        prerotate_all_wheels(tick, prerot_omega, prerot_kd, prerot_dur);
        std::cout << "  [prerot done] settling " << settle_s << "s before test...\n";
        wait_until_settled(tick, settle_s, settle_s + 2.0f);
    }

    // 累加器（多轮平均）
    WheelAccum acc[STEER_NUM];

    // 结果文件 (append, 含时间戳)
    std::ofstream summary("/home/rc/Ethercat_R1/var_data/steer_friction_4wheel_result.txt", std::ios::app);
    if (summary.is_open())
    {
        std::time_t now_t = std::time(nullptr);
        summary << "\n=========== "
                << std::put_time(std::localtime(&now_t), "%Y-%m-%d %H:%M:%S")
                << "  rounds=" << rounds << " ===========\n";
    }

    // ---------------- 主循环：ROUNDS 轮 ----------------
    for (int r = 0; r < rounds && st_running.load(); ++r)
    {
        std::cout << "\n############## ROUND " << (r + 1) << " / " << rounds << " ##############\n";

        // ---- (A) 4 轮并行 Stiction ----
        std::cout << "\n--- [A] Stiction (all 4 wheels in parallel) ---\n";
        for (int i = 0; i < STEER_NUM; ++i)
            st_chassis.Motor_Steer[i].Begin_Stiction_Calibration(
                stic_step, stic_dwell, stic_thresh, stic_tmax);
        run_parallel_calibration(tick, stic_round_t);

        for (int i = 0; i < STEER_NUM; ++i)
        {
            const auto r_i = st_chassis.Motor_Steer[i].Get_Calibration_Result();
            std::cout << "  W" << i << ": stiction T="
                      << std::fixed << std::setprecision(4) << r_i.stiction_torque_nm
                      << "Nm  breakaway omega=" << std::setprecision(3) << r_i.stiction_breakaway_omega
                      << "rad/s  " << (r_i.success ? "OK" : "FAIL") << "\n";
            acc[i].n_stic++;
            if (r_i.success)
            {
                acc[i].n_stic_ok++;
                acc[i].sum_stic_nm  += r_i.stiction_torque_nm;
                acc[i].sum_stic_omg += r_i.stiction_breakaway_omega;
            }
        }

        // ---- Settle 1 ----
        std::cout << "  [settle] holding 0 torque for " << settle_s << "s...\n";
        wait_until_settled(tick, settle_s, settle_s + 2.0f);

        if (!st_running.load()) break;

        // ---- (B) 4 轮并行 Friction ----
        std::cout << "\n--- [B] Friction (all 4 wheels in parallel) ---\n";
        for (int i = 0; i < STEER_NUM; ++i)
            st_chassis.Motor_Steer[i].Begin_Friction_Calibration(
                fric_omega, fric_kd, fric_warm, fric_meas);
        run_parallel_calibration(tick, fric_round_t);

        for (int i = 0; i < STEER_NUM; ++i)
        {
            const auto r_i = st_chassis.Motor_Steer[i].Get_Calibration_Result();
            std::cout << "  W" << i << ": T+=" << std::fixed << std::setprecision(4)
                      << r_i.friction_torque_pos_nm
                      << "  T-=" << r_i.friction_torque_neg_nm
                      << "  AVG=" << r_i.friction_torque_avg_nm
                      << "Nm  (omega+=" << std::setprecision(3) << r_i.friction_omega_pos_actual
                      << " omega-=" << r_i.friction_omega_neg_actual << ")  "
                      << (r_i.success ? "OK" : "FAIL") << "\n";
            acc[i].n_fric++;
            if (r_i.success)
            {
                acc[i].n_fric_ok++;
                acc[i].sum_fric_pos += r_i.friction_torque_pos_nm;
                acc[i].sum_fric_neg += r_i.friction_torque_neg_nm;
                acc[i].sum_fric_avg += r_i.friction_torque_avg_nm;
                acc[i].sum_fric_omg_pos += r_i.friction_omega_pos_actual;
                acc[i].sum_fric_omg_neg += r_i.friction_omega_neg_actual;
            }
        }

        // ---- Settle 2 (只有还要进入下一轮才需要) ----
        if (r + 1 < rounds)
        {
            std::cout << "  [settle] holding 0 torque for " << settle_s << "s...\n";
            wait_until_settled(tick, settle_s, settle_s + 2.0f);
        }
    }

    // ---------------- 汇总 ----------------
    std::cout << "\n=========== SUMMARY (avg over " << rounds << " rounds) ===========\n";
    std::cout << "  Wheel | Stiction Ts (Nm)  ω_break (rad/s) | Friction T+ (Nm)  T- (Nm)  Tc avg (Nm)\n"
              << "  ------+------------------------------------+----------------------------------------\n";
    for (int i = 0; i < STEER_NUM; ++i)
    {
        const auto &a = acc[i];
        float avg_stic_nm  = (a.n_stic_ok > 0) ? (a.sum_stic_nm  / a.n_stic_ok) : 0.0f;
        float avg_stic_omg = (a.n_stic_ok > 0) ? (a.sum_stic_omg / a.n_stic_ok) : 0.0f;
        float avg_fric_pos = (a.n_fric_ok > 0) ? (a.sum_fric_pos / a.n_fric_ok) : 0.0f;
        float avg_fric_neg = (a.n_fric_ok > 0) ? (a.sum_fric_neg / a.n_fric_ok) : 0.0f;
        float avg_fric_avg = (a.n_fric_ok > 0) ? (a.sum_fric_avg / a.n_fric_ok) : 0.0f;
        std::cout << "    " << i << "   | "
                  << std::fixed << std::setprecision(4)
                  << std::setw(11) << avg_stic_nm << "    "
                  << std::setw(10) << std::setprecision(3) << avg_stic_omg
                  << "      |  "
                  << std::setprecision(4)
                  << std::setw(8) << avg_fric_pos << "  "
                  << std::setw(8) << avg_fric_neg << "  "
                  << std::setw(8) << avg_fric_avg
                  << "   (OK " << a.n_stic_ok << "/" << a.n_stic
                  << " | " << a.n_fric_ok << "/" << a.n_fric << ")\n";

        if (summary.is_open())
        {
            summary << std::fixed << std::setprecision(4)
                    << "W" << i << " STIC: avg=" << avg_stic_nm
                    << "Nm  breakaway_omega=" << std::setprecision(3) << avg_stic_omg
                    << "  (ok " << a.n_stic_ok << "/" << a.n_stic << ")  "
                    << "FRIC: T+=" << std::setprecision(4) << avg_fric_pos
                    << "  T-=" << avg_fric_neg
                    << "  AVG=" << avg_fric_avg
                    << "  (ok " << a.n_fric_ok << "/" << a.n_fric << ")\n";
        }
    }

    // ---- 收尾 ----
    std::cout << "\n[FRIC4] disabling motors...\n";
    for (int i = 0; i < STEER_NUM; ++i)
    {
        st_chassis.Motor_Steer[i].Set_Control_Status(Motor_DM_Status_DISABLE);
        st_chassis.Motor_Steer[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        st_chassis.Motor_Steer[i].CAN_Send_Exit();
    }
    linkx_send_pdos(&st_linkx);
    ec_busywait_ms(tick, 150);

    if (summary.is_open()) summary.close();
    std::cout << "[FRIC4] result appended to var_data/steer_friction_4wheel_result.txt\n"
              << "[FRIC4] bye.\n";
    return 0;
}
