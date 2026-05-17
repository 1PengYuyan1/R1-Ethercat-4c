// odrive_calib_main.cpp
//
// ODrive 驱动轮参数标定工具：动摩擦力 / 静摩擦力 / 转动惯量。
// 通过 CAN 控制 ODrive 的速度模式和力矩模式，读取编码器和 IQ 反馈完成标定。
// 标定状态机在本文件实现（ODrive 为外部控制，不像 DM 有内部标定方法）。
//
// ⚠️ 测试前请把目标轮架空（轮胎不接触地面），否则地面摩擦会污染测量。
//
// 用法（必须 sudo）：
//   sudo IFNAME=enp86s0 ./odrive_calib --wheel 0 --test friction
//   sudo IFNAME=enp86s0 ./odrive_calib --wheel 0 --test stiction
//   sudo IFNAME=enp86s0 ./odrive_calib --wheel 0 --test inertia --friction 0.03
//   sudo IFNAME=enp86s0 ./odrive_calib --wheel 0 --test all
//
// 环境变量与 CLI 等价（CLI 优先）：
//   ODRIVE_CALIB_WHEEL=0..3
//   ODRIVE_CALIB_TEST=friction|stiction|inertia|all
//   ODRIVE_CALIB_OMEGA=5.0      Friction 测试速度 (rad/s)
//   ODRIVE_CALIB_WARMUP=1.5     Friction 单方向 warmup 时长 (s)
//   ODRIVE_CALIB_MEASURE=2.0    Friction 单方向取样时长 (s)
//   ODRIVE_CALIB_TSTEP=0.005    Stiction 力矩步进 (Nm)
//   ODRIVE_CALIB_DWELL=0.10     Stiction 每步保持时长 (s)
//   ODRIVE_CALIB_THRESH=0.3     Stiction 突破阈值 (rad/s)
//   ODRIVE_CALIB_TMAX=0.5       Stiction 上限保护 (Nm)
//   ODRIVE_CALIB_INERTIA_T=0.15 Inertia 阶跃力矩 (Nm)
//   ODRIVE_CALIB_INERTIA_DUR=0.5 Inertia 加速段时长 (s)
//   ODRIVE_CALIB_FRICTION=0.0   Inertia 减除的动摩擦力 (Nm)

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "crt_chassis.h"
#include "dvc_odrive.h"
#include "dvc_encoder.h"
#include "dvc_motor_dm.h"
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
bool g_debug = false;  // --debug 启用详细 CAN 日志

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

// CAN 消息分发：CH0=DM电机, CH1=ODrive, CH2=编码器
void can_dispatch(uint8_t ch, uint32_t can_id, uint8_t *data)
{
    const uint32_t id_std = (can_id & 0x7FFU);

    if (g_debug && ch == 1)
    {
        uint32_t node = (id_std >> 5U) & 0x3FU;
        uint32_t cmd  = id_std & 0x1FU;
        printf("[CAN_RX] CH1 id=0x%03X node=0x%02X cmd=0x%02X data=",
               id_std, node, cmd);
        for (int b = 0; b < 8; ++b) printf("%02X ", data[b]);
        printf("\n");
    }

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
        if (g_debug)
            printf("[CAN_RX] CH1 id=0x%03X UNMATCHED (no ODrive node match)\n", id_std);
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

// 单步 EtherCAT 收发
bool ec_step(uint32_t tick, int target_wheel = -1)
{
    if (!st_running.load() || !st_master.is_running) return false;
    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);
    can_msg_t msg;
    for (uint8_t ch = 0; ch < kChannelCount; ch++)
        while (linkx_quick_recv(&st_linkx, ch, &msg))
            can_dispatch(ch, msg.id, msg.data);

    // 100ms 周期：DM 电机心跳维持 + 编码器 + ODrive 存活检查
    if ((tick % k100msPeriod) == 0)
    {
        for (int i = 0; i < STEER_NUM; ++i)
        {
            st_chassis.Motor_Steer[i].TIM_Alive_PeriodElapsedCallback();
            st_chassis.Encoder_Steer[i].TIM_Alive_PeriodElapsedCallback();
            st_chassis.ODrive_Motor_Steer[i].TIM_Alive_CheckCallback();
        }
    }

    // ODrive 周期发送（与主程序一致：TIM_Send_PeriodElapsedCallback 发速度命令+请求 IQ）
    if (target_wheel >= 0 && target_wheel < STEER_NUM)
    {
        auto &od = st_chassis.ODrive_Motor_Steer[target_wheel];
        od.TIM_Send_PeriodElapsedCallback();
        if ((tick % 2) == 0) od.Request_Encoder_Data();
        if ((tick % 500) == 0) od.Request_Bus_Voltage();
    }

    // DM 电机发送
    for (int i = 0; i < STEER_NUM; ++i)
        st_chassis.Motor_Steer[i].TIM_Send_PeriodElapsedCallback();

    linkx_send_pdos(&st_linkx);
    return true;
}

void ec_busywait_ms(uint32_t &tick, uint32_t ms, int target_wheel = -1)
{
    auto next_wakeup = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < ms; ++i)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        if (!ec_step(tick++, target_wheel)) return;
        std::this_thread::sleep_until(next_wakeup);
    }
}

// 将所有 ODrive 设为 IDLE
void odrive_all_idle()
{
    for (int i = 0; i < STEER_NUM; ++i)
        st_chassis.ODrive_Motor_Steer[i].Set_Axis_State(ODRIVE_STATE_IDLE);
}

// 将所有 DM 电机置零力矩
void dm_all_neutral()
{
    for (int i = 0; i < STEER_NUM; ++i)
    {
        st_chassis.Motor_Steer[i].Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        st_chassis.Motor_Steer[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}

// ODrive 使能序列
// 每 100ms 重发 SET_ClosedLoop 直到 axis_state=8
// 然后发 Set_Control_Mode 切换到指定模式（Makerbase ODrive NVM 可能不是速度模式）
bool odrive_enable(int wheel, Enum_ODrive_Control_Mode ctrl_mode, uint32_t &tick)
{
    auto &od = st_chassis.ODrive_Motor_Steer[wheel];

    // 1. 清除错误
    if (g_debug) printf("[ENABLE] step 1: Clear_Errors  node=0x%02X\n", od.Get_node_id());
    od.Clear_Errors();
    ec_busywait_ms(tick, 100, wheel);

    // 2. 持续重发 SET_ClosedLoop，模仿主程序的 100ms 回调
    if (g_debug) printf("[ENABLE] step 2: loop SET_ClosedLoop until axis_state=8 (max 3s)\n");
    {
        auto next = std::chrono::steady_clock::now();
        uint32_t resend_counter = 0;
        for (uint32_t i = 0; i < 3000 && st_running.load(); ++i)
        {
            next += std::chrono::milliseconds(kCtrlPeriodMs);

            // 每 100ms 重发 CLOSED_LOOP 命令（与主程序 100ms 回调一致）
            if ((resend_counter % 100) == 0)
            {
                uint32_t err = static_cast<uint32_t>(od.Get_Axis_Error());
                if (err != AXIS_ERROR_NONE)
                    od.Clear_Errors();
                else
                    od.SET_ClosedLoop();
                if (g_debug)
                    printf("[ENABLE] tick=%u: re-send SET_ClosedLoop  axis_state=%.0f  err=0x%X\n",
                           resend_counter, od.Get_Axis_State(), err);
            }
            resend_counter++;

            // 设置零速度（通过 target_omega → TIM_Send 发出）
            od.Set_target_omega(0.0f);
            od.Set_target_torque(0.0f);

            ec_step(tick++, wheel);
            std::this_thread::sleep_until(next);

            // 检测是否成功进入闭环
            if (static_cast<int>(od.Get_Axis_State()) == ODRIVE_STATE_CLOSED_LOOP_CONTROL)
            {
                if (g_debug)
                    printf("[ENABLE] CLOSED_LOOP achieved at tick=%u\n", resend_counter);
                // 继续保活 200ms
                for (uint32_t j = 0; j < 200 && st_running.load(); ++j)
                {
                    next += std::chrono::milliseconds(kCtrlPeriodMs);
                    od.Set_target_omega(0.0f);
                    od.Set_target_torque(0.0f);
                    ec_step(tick++, wheel);
                    std::this_thread::sleep_until(next);
                }
                break;
            }
        }
    }

    float final_state = od.Get_Axis_State();
    if (static_cast<int>(final_state) != ODRIVE_STATE_CLOSED_LOOP_CONTROL)
    {
        printf("[ENABLE] WARNING: Failed to enter CLOSED_LOOP (state=%.0f)\n", final_state);
        return false;
    }

    // 3. 切换控制模式（Makerbase ODrive NVM 可能不是速度模式）
    if (g_debug) printf("[ENABLE] step 3: Set_Control_Mode=%d  PASSTHROUGH\n", (int)ctrl_mode);
    od.Set_Control_Mode(ctrl_mode, ODRIVE_INPUT_PASSTHROUGH);
    {
        auto next = std::chrono::steady_clock::now();
        for (uint32_t i = 0; i < 300 && st_running.load(); ++i)
        {
            next += std::chrono::milliseconds(kCtrlPeriodMs);
            if (ctrl_mode == ODRIVE_CTRL_VELOCITY || ctrl_mode == ODRIVE_CTRL_VOLTAGE)
            {
                od.Set_target_omega(0.0f);
                od.Set_target_torque(0.0f);
            }
            else if (ctrl_mode == ODRIVE_CTRL_TORQUE)
                od.Set_target_torque(0.0f);
            ec_step(tick++, wheel);
            std::this_thread::sleep_until(next);
        }
    }

    if (g_debug)
    {
        printf("[ENABLE] done. omega=%.3f  Vbus=%.1f  connected=%d  axis_state=%.0f  axis_err=%.0f\n",
               od.Get_Omega(), od.Get_Bus_Voltage(),
               od.Is_Connected(), od.Get_Axis_State(), od.Get_Axis_Error());
    }
    return true;
}

// 总线电压检查
bool bus_voltage_safe(int wheel, float min_v = 12.0f)
{
    float v = st_chassis.ODrive_Motor_Steer[wheel].Get_Bus_Voltage();
    if (v > 1.0f && v < min_v)
    {
        std::cerr << "\n[ODRIVE_CALIB][LOW_VOLTAGE] Vbus=" << v
                  << "V < " << min_v << "V  abort.\n";
        return false;
    }
    return true;
}

// ======================================================
//  标定结果结构
// ======================================================
struct ODriveCalibResult
{
    // Friction
    float friction_torque_pos = 0;
    float friction_torque_neg = 0;
    float friction_torque_avg = 0;
    float friction_omega_pos = 0;
    float friction_omega_neg = 0;
    bool friction_success = false;

    // Stiction
    float stiction_torque = 0;
    float stiction_omega = 0;
    bool stiction_success = false;

    // Inertia
    float inertia_J = 0;
    float inertia_alpha = 0;
    float inertia_torque_step = 0;
    float inertia_friction_used = 0;
    bool inertia_success = false;
};

// ======================================================
//  标定阶段 1：动摩擦力
// ======================================================
ODriveCalibResult run_friction_calibration(int wheel, float omega_target,
                                           float warmup_s, float measure_s,
                                           uint32_t &tick)
{
    ODriveCalibResult result;
    auto &od = st_chassis.ODrive_Motor_Steer[wheel];
    const float dt = kCtrlPeriodMs / 1000.0f;
    const float Kt = Class_ODrive::Kt;

    std::cout << "[ODRIVE_CALIB][FRICTION] omega=" << omega_target
              << " rad/s  warmup=" << warmup_s << "s  measure=" << measure_s << "s\n";

    // 切到速度模式
    odrive_enable(wheel, ODRIVE_CTRL_VELOCITY, tick);

    auto run_one_direction = [&](float omega, double &acc_iq, double &acc_omega, int &count)
    {
        float elapsed = 0;
        auto next = std::chrono::steady_clock::now();

        // Warmup
        while (elapsed < warmup_s && st_running.load())
        {
            next += std::chrono::milliseconds(kCtrlPeriodMs);
            od.Set_target_omega(omega);
            od.Set_target_torque(0.0f);
            ec_step(tick++, wheel);
            elapsed += dt;
            std::this_thread::sleep_until(next);
        }

        // Measure
        elapsed = 0;
        acc_iq = acc_omega = 0;
        count = 0;
        while (elapsed < measure_s && st_running.load())
        {
            next += std::chrono::milliseconds(kCtrlPeriodMs);
            od.Set_target_omega(omega);
            od.Set_target_torque(0.0f);
            ec_step(tick++, wheel);

            acc_iq += static_cast<double>(od.Get_IQ_Measured());
            acc_omega += static_cast<double>(od.Get_Omega());
            count++;

            if ((count % 200) == 0)
            {
                std::cout << "  [t=" << std::fixed << std::setprecision(2) << elapsed
                          << "s  iq=" << std::setprecision(4) << od.Get_IQ_Measured()
                          << "A  omega=" << std::setprecision(2) << od.Get_Omega()
                          << "]\r" << std::flush;
            }
            elapsed += dt;
            std::this_thread::sleep_until(next);
        }
        std::cout << "\n";
    };

    // 正向
    double acc_iq_pos = 0, acc_omega_pos = 0;
    int n_pos = 0;
    std::cout << "  >> 正向 +" << omega_target << " rad/s\n";
    run_one_direction(+omega_target, acc_iq_pos, acc_omega_pos, n_pos);

    // 反向
    double acc_iq_neg = 0, acc_omega_neg = 0;
    int n_neg = 0;
    std::cout << "  >> 反向 -" << omega_target << " rad/s\n";
    run_one_direction(-omega_target, acc_iq_neg, acc_omega_neg, n_neg);

    // 停止
    od.Set_target_omega(0.0f);
    od.Set_target_torque(0.0f);

    // 计算结果
    float iq_pos = (n_pos > 0) ? static_cast<float>(acc_iq_pos / n_pos) : 0;
    float iq_neg = (n_neg > 0) ? static_cast<float>(acc_iq_neg / n_neg) : 0;
    result.friction_torque_pos = Kt * std::fabs(iq_pos);
    result.friction_torque_neg = Kt * std::fabs(iq_neg);
    result.friction_torque_avg = 0.5f * (result.friction_torque_pos + result.friction_torque_neg);
    result.friction_omega_pos = (n_pos > 0) ? static_cast<float>(acc_omega_pos / n_pos) : 0;
    result.friction_omega_neg = (n_neg > 0) ? static_cast<float>(acc_omega_neg / n_neg) : 0;
    result.friction_success = (n_pos > 50 && n_neg > 50);

    std::cout << "[ODRIVE_CALIB][FRICTION] T+=" << result.friction_torque_pos
              << " Nm  T-=" << result.friction_torque_neg
              << " Nm  AVG=" << result.friction_torque_avg
              << " Nm  (omega+=" << result.friction_omega_pos
              << "  omega-=" << result.friction_omega_neg << ")\n";

    return result;
}

// ======================================================
//  标定阶段 2：静摩擦力
// ======================================================
ODriveCalibResult run_stiction_calibration(int wheel, float torque_step,
                                           float dwell_s, float omega_thresh,
                                           float torque_max, uint32_t &tick)
{
    ODriveCalibResult result;
    auto &od = st_chassis.ODrive_Motor_Steer[wheel];
    const float dt = kCtrlPeriodMs / 1000.0f;

    std::cout << "[ODRIVE_CALIB][STICTION] step=" << torque_step
              << " Nm  dwell=" << dwell_s << "s  thresh=" << omega_thresh
              << " rad/s  Tmax=" << torque_max << " Nm\n";

    // 切到力矩模式
    odrive_enable(wheel, ODRIVE_CTRL_TORQUE, tick);
    od.Set_Motor_Mode(ODRIVE_CTRL_TORQUE);

    float current_torque = 0.0f;
    float phase_elapsed = 0.0f;
    bool found = false;
    auto next = std::chrono::steady_clock::now();

    while (st_running.load())
    {
        next += std::chrono::milliseconds(kCtrlPeriodMs);
        od.Set_target_torque(current_torque);
        ec_step(tick++, wheel);

        float omega = od.Get_Omega();

        // 检测突破
        if (std::fabs(omega) > omega_thresh && current_torque > 1e-6f)
        {
            result.stiction_torque = current_torque;
            result.stiction_omega = omega;
            result.stiction_success = true;
            found = true;
            break;
        }

        phase_elapsed += dt;
        if (phase_elapsed >= dwell_s)
        {
            current_torque += torque_step;
            phase_elapsed = 0.0f;

            std::cout << "  [T=" << std::fixed << std::setprecision(4) << current_torque
                      << " Nm  omega=" << std::setprecision(3) << omega << "]\r" << std::flush;

            if (current_torque > torque_max)
            {
                result.stiction_torque = torque_max;
                result.stiction_success = false;
                std::cerr << "\n[ODRIVE_CALIB][STICTION] FAILED: Tmax reached\n";
                break;
            }
        }

        if (!bus_voltage_safe(wheel)) break;
        std::this_thread::sleep_until(next);
    }
    std::cout << "\n";

    // 停止
    od.Set_target_torque(0.0f);
    od.Set_Motor_Mode(ODRIVE_CTRL_VOLTAGE);

    if (found)
        std::cout << "[ODRIVE_CALIB][STICTION] breakaway T=" << result.stiction_torque
                  << " Nm at omega=" << result.stiction_omega << " rad/s\n";

    return result;
}

// ======================================================
//  标定阶段 3：转动惯量
// ======================================================
ODriveCalibResult run_inertia_calibration(int wheel, float torque_step,
                                          float friction_known, float warmup_s,
                                          float accel_dur_s, uint32_t &tick)
{
    ODriveCalibResult result;
    auto &od = st_chassis.ODrive_Motor_Steer[wheel];
    const float dt = kCtrlPeriodMs / 1000.0f;

    std::cout << "[ODRIVE_CALIB][INERTIA] T_step=" << torque_step
              << " Nm  friction=" << friction_known
              << " Nm  warmup=" << warmup_s << "s  accel=" << accel_dur_s << "s\n";

    // 切到力矩模式
    odrive_enable(wheel, ODRIVE_CTRL_TORQUE, tick);
    od.Set_Motor_Mode(ODRIVE_CTRL_TORQUE);

    // Warmup：零力矩让转子静止
    auto next = std::chrono::steady_clock::now();
    float elapsed = 0;
    while (elapsed < warmup_s && st_running.load())
    {
        next += std::chrono::milliseconds(kCtrlPeriodMs);
        od.Set_target_torque(0.0f);
        ec_step(tick++, wheel);
        elapsed += dt;
        std::this_thread::sleep_until(next);
    }

    // 阶跃力矩：采集 (t, ω) 数据
    double sum_t = 0, sum_o = 0, sum_to = 0, sum_tt = 0;
    int n_samples = 0;
    elapsed = 0;

    while (elapsed < accel_dur_s && st_running.load())
    {
        next += std::chrono::milliseconds(kCtrlPeriodMs);
        od.Set_target_torque(torque_step);
        ec_step(tick++, wheel);

        double t = static_cast<double>(elapsed);
        double w = static_cast<double>(od.Get_Omega());
        sum_t += t;
        sum_o += w;
        sum_to += t * w;
        sum_tt += t * t;
        n_samples++;

        if ((n_samples % 200) == 0)
        {
            std::cout << "  [t=" << std::fixed << std::setprecision(3) << elapsed
                      << "s  omega=" << std::setprecision(3) << od.Get_Omega()
                      << " rad/s]\r" << std::flush;
        }

        elapsed += dt;
        if (!bus_voltage_safe(wheel)) break;
        std::this_thread::sleep_until(next);
    }
    std::cout << "\n";

    // 停止
    od.Set_target_torque(0.0f);
    od.Set_Motor_Mode(ODRIVE_CTRL_VOLTAGE);

    // 最小二乘拟合：ω = α*t + ω₀
    float alpha = 0.0f;
    double N = static_cast<double>(n_samples);
    double denom = N * sum_tt - sum_t * sum_t;
    if (denom > 1e-12 && n_samples > 20)
        alpha = static_cast<float>((N * sum_to - sum_t * sum_o) / denom);

    float net_torque = torque_step - friction_known;
    float J = 0.0f;
    if (std::fabs(alpha) > 1e-3f)
        J = net_torque / alpha;

    result.inertia_J = J;
    result.inertia_alpha = alpha;
    result.inertia_torque_step = torque_step;
    result.inertia_friction_used = friction_known;
    result.inertia_success = (J > 0.0f && std::isfinite(J));

    std::cout << "[ODRIVE_CALIB][INERTIA] alpha=" << alpha
              << " rad/s²  T_net=" << net_torque
              << " Nm  J=" << J << " kg·m²  (samples=" << n_samples << ")\n";

    return result;
}

}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string ifname = std::getenv("IFNAME") ? std::getenv("IFNAME") : "enp86s0";

    // ---- 解析参数 ----
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--debug") g_debug = true;

    const int wheel = std::atoi(cli_get(argc, argv, "wheel",
                                        std::to_string(env_i("ODRIVE_CALIB_WHEEL", 0)).c_str()));
    if (wheel < 0 || wheel >= STEER_NUM)
    {
        std::cerr << "[ODRIVE_CALIB] invalid wheel " << wheel << " (0~" << (STEER_NUM - 1) << ")\n";
        return -1;
    }

    const std::string test = cli_get(argc, argv, "test",
                                     std::getenv("ODRIVE_CALIB_TEST") ? std::getenv("ODRIVE_CALIB_TEST") : "all");

    const float omega_target = std::strtof(cli_get(argc, argv, "omega",
                               std::to_string(env_f("ODRIVE_CALIB_OMEGA", 5.0f)).c_str()), nullptr);
    const float warmup       = std::strtof(cli_get(argc, argv, "warmup",
                               std::to_string(env_f("ODRIVE_CALIB_WARMUP", 1.5f)).c_str()), nullptr);
    const float measure      = std::strtof(cli_get(argc, argv, "measure",
                               std::to_string(env_f("ODRIVE_CALIB_MEASURE", 2.0f)).c_str()), nullptr);

    const float t_step       = std::strtof(cli_get(argc, argv, "tstep",
                               std::to_string(env_f("ODRIVE_CALIB_TSTEP", 0.005f)).c_str()), nullptr);
    const float dwell        = std::strtof(cli_get(argc, argv, "dwell",
                               std::to_string(env_f("ODRIVE_CALIB_DWELL", 0.10f)).c_str()), nullptr);
    const float thresh       = std::strtof(cli_get(argc, argv, "thresh",
                               std::to_string(env_f("ODRIVE_CALIB_THRESH", 0.3f)).c_str()), nullptr);
    const float t_max        = std::strtof(cli_get(argc, argv, "tmax",
                               std::to_string(env_f("ODRIVE_CALIB_TMAX", 0.5f)).c_str()), nullptr);

    const float inertia_T    = std::strtof(cli_get(argc, argv, "inertia_t",
                               std::to_string(env_f("ODRIVE_CALIB_INERTIA_T", 0.15f)).c_str()), nullptr);
    const float inertia_dur  = std::strtof(cli_get(argc, argv, "inertia_dur",
                               std::to_string(env_f("ODRIVE_CALIB_INERTIA_DUR", 0.5f)).c_str()), nullptr);
    float friction_known     = std::strtof(cli_get(argc, argv, "friction",
                               std::to_string(env_f("ODRIVE_CALIB_FRICTION", 0.0f)).c_str()), nullptr);

    std::cout << "===============================================\n"
              << "  ODrive Wheel Parameter Calibration\n"
              << "  IFNAME : " << ifname << "\n"
              << "  WHEEL  : " << wheel << "  (架空提示: 务必让该轮悬空!)\n"
              << "  TEST   : " << test << "\n"
              << "  Kt     : " << Class_ODrive::Kt << " Nm/A\n"
              << "===============================================\n";

    // ---- EtherCAT / LinkX / Chassis 初始化 ----
    if (!ecat_master_init(&st_master, ifname.c_str()))
    {
        std::cerr << "[ODRIVE_CALIB] ecat_master_init failed\n";
        return -1;
    }
    linkx_init(&st_linkx, 1, &st_master.ctx);
    linkx_hw_wakeup(&st_linkx);
    for (int i = 0; i < kChannelCount; i++)
        linkx_set_can_baudrate(&st_linkx, i, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    if (!ecat_master_bring_online(&st_master))
    {
        std::cerr << "[ODRIVE_CALIB] ecat bring_online failed\n";
        return -1;
    }
    st_chassis.Init(&st_linkx);
    st_chassis.Init_Motor_Params();
    st_chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

    // ---- 等待 ODrive 首帧（最多 3 秒）----
    uint32_t tick = 0;
    auto next_wakeup = std::chrono::steady_clock::now();
    std::cout << "[ODRIVE_CALIB] waiting for ODrive data (3s scan)...\n";
    if (g_debug) std::cout << "[DEBUG] scanning ALL CAN frames on CH1...\n";
    for (uint32_t t = 0; t < 3000 && st_running.load(); ++t)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        ec_step(tick++, wheel);
        std::this_thread::sleep_until(next_wakeup);
    }
    auto &od_tgt = st_chassis.ODrive_Motor_Steer[wheel];
    std::cout << "[ODRIVE_CALIB] after 3s scan: omega=" << od_tgt.Get_Omega()
              << "  Vbus=" << od_tgt.Get_Bus_Voltage()
              << "  connected=" << (int)od_tgt.Is_Connected()
              << "  axis_state=" << od_tgt.Get_Axis_State() << "\n";

    // ---- DM 电机置零力矩（安全）----
    dm_all_neutral();

    // ---- 其他 ODrive 设为 IDLE ----
    for (int i = 0; i < STEER_NUM; ++i)
    {
        if (i != wheel)
            st_chassis.ODrive_Motor_Steer[i].Set_Axis_State(ODRIVE_STATE_IDLE);
    }
    ec_busywait_ms(tick, 100, wheel);

    // ---- 检查总线电压 ----
    st_chassis.ODrive_Motor_Steer[wheel].Request_Bus_Voltage();
    ec_busywait_ms(tick, 50, wheel);
    float vbus = st_chassis.ODrive_Motor_Steer[wheel].Get_Bus_Voltage();
    std::cout << "[ODRIVE_CALIB] Vbus=" << vbus << "V  node_id=0x"
              << std::hex << static_cast<int>(st_chassis.ODrive_Motor_Steer[wheel].Get_node_id())
              << std::dec << "\n";

    // ---- 提示确认 ----
    std::cout << "\n>>> 请确认 wheel " << wheel << " 已架空。3 秒后开始测试...\n";
    ec_busywait_ms(tick, 3000, wheel);

    std::ofstream summary("/home/rc/Ethercat_R1/var_data/odrive_calib_result.txt", std::ios::app);
    auto write_header = [&]() {
        if (summary.is_open())
        {
            std::time_t now = std::time(nullptr);
            summary << "\n===== "
                    << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S")
                    << "  wheel=" << wheel << "  test=" << test
                    << "  Kt=" << Class_ODrive::Kt << " =====\n";
        }
    };
    write_header();

    // ---- 测试 1：动摩擦力 ----
    if ((test == "friction" || test == "all") && st_running.load())
    {
        std::cout << "\n--- [1/3] FRICTION calibration ---\n";
        auto r = run_friction_calibration(wheel, omega_target, warmup, measure, tick);
        if (summary.is_open())
            summary << std::fixed << std::setprecision(4)
                    << "FRICTION: T+=" << r.friction_torque_pos
                    << "  T-=" << r.friction_torque_neg
                    << "  AVG=" << r.friction_torque_avg
                    << "  omega+=" << r.friction_omega_pos
                    << "  omega-=" << r.friction_omega_neg << "\n"
                    << std::defaultfloat;
        if (r.friction_success && (test == "all") && friction_known < 1e-6f)
            friction_known = r.friction_torque_avg;

        // 停止 ODrive
        st_chassis.ODrive_Motor_Steer[wheel].Set_Axis_State(ODRIVE_STATE_IDLE);
        ec_busywait_ms(tick, 800, wheel);
    }

    // ---- 测试 2：静摩擦力 ----
    if ((test == "stiction" || test == "all") && st_running.load())
    {
        std::cout << "\n--- [2/3] STICTION calibration ---\n";
        auto r = run_stiction_calibration(wheel, t_step, dwell, thresh, t_max, tick);
        if (summary.is_open())
            summary << std::fixed << std::setprecision(4)
                    << "STICTION: T=" << r.stiction_torque
                    << "  breakaway omega=" << std::setprecision(3) << r.stiction_omega
                    << "  success=" << r.stiction_success << "\n"
                    << std::defaultfloat;

        st_chassis.ODrive_Motor_Steer[wheel].Set_Axis_State(ODRIVE_STATE_IDLE);
        ec_busywait_ms(tick, 800, wheel);
    }

    // ---- 测试 3：转动惯量 ----
    if ((test == "inertia" || test == "all") && st_running.load())
    {
        std::cout << "\n--- [3/3] INERTIA calibration ---\n";
        std::cout << "  using friction_known=" << friction_known << " Nm\n";
        auto r = run_inertia_calibration(wheel, inertia_T, friction_known, 0.5f, inertia_dur, tick);
        if (summary.is_open())
            summary << std::fixed << std::setprecision(6)
                    << "INERTIA: J=" << r.inertia_J
                    << "  alpha=" << std::setprecision(3) << r.inertia_alpha
                    << "  T_step=" << std::setprecision(4) << r.inertia_torque_step
                    << "  friction_used=" << r.inertia_friction_used
                    << "  success=" << r.inertia_success << "\n"
                    << std::defaultfloat;

        st_chassis.ODrive_Motor_Steer[wheel].Set_Axis_State(ODRIVE_STATE_IDLE);
        ec_busywait_ms(tick, 200, wheel);
    }

    // ---- 收尾 ----
    std::cout << "\n[ODRIVE_CALIB] shutting down...\n";
    odrive_all_idle();
    dm_all_neutral();
    for (int i = 0; i < STEER_NUM; ++i)
        st_chassis.Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
    linkx_send_pdos(&st_linkx);
    ec_busywait_ms(tick, 100, wheel);

    std::cout << "[ODRIVE_CALIB] result appended to var_data/odrive_calib_result.txt\n"
              << "[ODRIVE_CALIB] bye.\n";
    return 0;
}
