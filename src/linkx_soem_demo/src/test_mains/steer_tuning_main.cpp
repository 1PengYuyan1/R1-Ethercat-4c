// =============================================================================
// 舵向调参专用程序 (Steer Tuning Tool) — 含自动扫参 (AUTO_SWEEP) 模式
//
// 用途：
//   独立可执行文件，专为调试达妙(DM)电机 MIT 模式 + 外层位置环 + 力矩前馈，
//   不依赖 ROS2 / 遥控器。直接对一个舵轮发指令、读编码器，目标是找出
//   "不过冲、不抖动、快速响应" 的一组 (MIT_KP, MIT_KD, POS_KP, POS_KD, TORQUE_FF)。
//
//   ★ AUTO_SWEEP 模式（按 'a' 进入或环境变量 TUNE_AUTO=1）：
//     程序会用一个连续运动目标（默认正弦），自动遍历你给的参数网格，
//     每组参数采集若干秒响应数据，算综合分（RMS误差 + 过冲 + 抖动），
//     最后排序并输出 var_data/steer_tuning_results.csv，控制台打印 Top 10。
//
// 硬件 ID（与 crt_chassis.cpp 一致）：
//   DM 电机:  CAN_CH=0  Rx_ID(主机ID) 0x11..0x14  Tx_ID(电机ID) 0x01..0x04
//   ODrive :  CAN_CH=1  (本程序不使用，仅初始化句柄)
//   编码器 :  CAN_CH=2  ID 0x05..0x08  分辨率 4096
//
// 启动：
//   sudo ros2 run linkx_soem_demo steer_tuning [ifname]
//   或：sudo ./install/linkx_soem_demo/lib/linkx_soem_demo/steer_tuning enp86s0
//
// 环境变量（覆盖默认值）：
//   TUNE_WHEEL        默认调参的舵轮索引 (0..3，默认 0)
//   TUNE_MIT_KP       MIT 内环 Kp，默认 0.0    (合理 0~30)
//   TUNE_MIT_KD       MIT 内环 Kd，默认 0.6    (合理 0~3)
//   TUNE_TORQUE_FF    MIT 力矩前馈 Nm，默认 0
//   TUNE_POS_KP       外层位置环 Kp，默认 25
//   TUNE_POS_KD       外层位置环 Kd，默认 0.5
//   TUNE_OMEGA_MAX    舵向侧最大角速度 rad/s，默认 30
//   TUNE_OMEGA_MIN    舵向侧最小角速度死区 rad/s，默认 0.3
//   TUNE_DEG_DEAD     角度死区（deg），默认 0.5
//   TUNE_PROFILE      0=step 1=sine 2=hold 3=ramp 4=auto，默认 1（连续正弦）
//   TUNE_TARGET_DEG   step/sine/ramp 的幅值（deg），默认 30
//   TUNE_PERIOD_MS    波形周期（ms），默认 2000
//   TUNE_HOLD_DEG     hold 模式下的固定目标，默认 0
//   TUNE_AUTO         =1 启动后立即进入 AUTO_SWEEP 模式（与 ALL_WHEELS 互斥）
//   TUNE_GRID_POS_KP  逗号分隔，默认 "10,20,30,40"
//   TUNE_GRID_POS_KD  逗号分隔，默认 "0.2,0.5,0.8"
//   TUNE_GRID_MIT_KD  逗号分隔，默认 "0.3,0.6,1.0"
//   TUNE_GRID_MIT_KP  逗号分隔，默认 "0"
//   TUNE_GRID_FF      逗号分隔，默认 "0"
//   TUNE_AUTO_WARMUP_MS   每组前的稳定时间，默认 1000
//   TUNE_AUTO_MEASURE_MS  每组的测量时间，默认 4000
//
//   ★ 多轮并行模式（一次性测 4 轮、做对比分析）：
//     TUNE_ALL_WHEELS   =1 同时驱动 4 轮，每轮用自己的增益（默认 = production 值）
//     TUNE_W{0..3}_POS_KP / POS_KD / MIT_KP / MIT_KD / FF
//                       覆盖第 i 轮的增益；缺省值与 crt_chassis.cpp:558-621 一致
//     在 ALL_WHEELS 模式下：
//       - 4 轮同时执行同一波形（target_deg），各用各的 PD
//       - CSV 每个采样写 4 行（每轮一行，wheel 列区分）
//       - AUTO_SWEEP 不可用
//       - 键盘 1-8 仅影响 wheel_index 单轮（用 w/x 切换被调对象）
//
// 运行时按键（无需回车）：
//   q       退出
//   space   急停（target=0, profile=hold, motors disable）
//   1 / 2   MIT_KP   -/+
//   3 / 4   MIT_KD   -/+
//   5 / 6   POS_KP   -/+
//   7 / 8   POS_KD   -/+
//   9 / 0   TORQUE_FF -/+
//   - / =   TARGET_DEG -/+ (5 deg)
//   [ / ]   PERIOD_MS  -/+ (100 ms)
//   s/v/h/r profile=step/sine/hold/ramp（手动模式）
//   a       开始 AUTO_SWEEP；A 中止 AUTO_SWEEP
//   w/x     wheel_index 上一/下一
//   l       切换 CSV 日志开关
//   p       打印当前参数行（追加到日志）
//
// 日志：
//   var_data/steer_tuning.csv         — 高频原始数据
//   var_data/steer_tuning_results.csv — 自动扫参的每组结果（按 score 升序）
// =============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
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
constexpr uint32_t kDashboardPeriodMs = 200;
constexpr uint32_t kCsvPeriodMs = 5;
constexpr float REDUCTION = REDUCTION_RATIO;  // 3.5f

ecat_master_t st_master;
linkx_t st_linkx;
Class_Chassis st_chassis;

std::atomic<bool> st_running{true};

void on_signal(int)
{
  st_running.store(false);
  st_master.is_running = false;
}

float read_env_f(const char *name, float fallback)
{
  const char *v = std::getenv(name);
  if (!v || v[0] == '\0') return fallback;
  char *end = nullptr;
  float r = std::strtof(v, &end);
  return (end == v) ? fallback : r;
}

int read_env_i(const char *name, int fallback)
{
  const char *v = std::getenv(name);
  if (!v || v[0] == '\0') return fallback;
  return std::atoi(v);
}

std::vector<float> read_env_f_list(const char *name, const char *fallback)
{
  const char *v = std::getenv(name);
  if (!v || v[0] == '\0') v = fallback;
  std::vector<float> out;
  std::stringstream ss(v);
  std::string tok;
  while (std::getline(ss, tok, ','))
  {
    if (tok.empty()) continue;
    char *end = nullptr;
    float x = std::strtof(tok.c_str(), &end);
    if (end != tok.c_str()) out.push_back(x);
  }
  return out;
}

inline float deg2rad(float d) { return d * static_cast<float>(M_PI) / 180.0f; }
inline float rad2deg(float r) { return r * 180.0f / static_cast<float>(M_PI); }

inline float wrap_pi(float a)
{
  while (a >= static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
  while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
  return a;
}

// 终端切到非阻塞 + 非规范模式（按键不需回车，且不阻塞）
struct StdinRaw
{
  termios old_t {};
  int old_flags = 0;
  bool changed_termios = false;
  bool changed_flags = false;

  StdinRaw()
  {
    if (tcgetattr(STDIN_FILENO, &old_t) == 0)
    {
      termios t = old_t;
      t.c_lflag &= ~(ICANON | ECHO);
      t.c_cc[VMIN] = 0;
      t.c_cc[VTIME] = 0;
      if (tcsetattr(STDIN_FILENO, TCSANOW, &t) == 0) changed_termios = true;
    }
    old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (old_flags >= 0)
    {
      if (fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK) == 0)
        changed_flags = true;
    }
  }

  ~StdinRaw()
  {
    if (changed_termios) tcsetattr(STDIN_FILENO, TCSANOW, &old_t);
    if (changed_flags) fcntl(STDIN_FILENO, F_SETFL, old_flags);
  }
};

bool read_keypress(char &c)
{
  return ::read(STDIN_FILENO, &c, 1) == 1;
}

// 本地 CAN 分发（仅 DM motor + encoder，不处理 ODrive）
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
  // ch == 1（ODrive）忽略
}

// ===================== 自动扫参数据结构 =====================
struct ComboParams
{
  float pos_kp;
  float pos_kd;
  float mit_kp;
  float mit_kd;
  float ff;
};

struct ComboResult
{
  ComboParams p;
  float rms_err_deg;       // 跟踪 RMS 误差
  float peak_err_deg;      // 误差峰值（含过冲）
  float overshoot_deg;     // |actual| 超过 |target| 的最大值
  float jitter_deg_per_s;  // 速度抖动 RMS（高频成分代理）
  float lag_ms;            // 简易跟踪相位滞后估计
  uint32_t saturate_cnt;   // omega 触顶次数（饱和样本数）
  uint32_t n_samples;
  float score;             // 综合得分（越小越好）
};

// 扫参状态机阶段
enum AutoState : uint8_t
{
  AUTO_IDLE = 0,
  AUTO_APPLY = 1,    // 应用参数 + 让目标轮回 0
  AUTO_WARMUP = 2,   // 等待波形上稳定
  AUTO_MEASURE = 3,  // 采集指标
  AUTO_NEXT = 4,     // 储存结果 → 切换下一组
  AUTO_DONE = 5,
};

}  // namespace

// 每轮独立的增益（多轮并行模式 TUNE_ALL_WHEELS=1 下使用）
struct WheelTune
{
  float pos_kp;
  float pos_kd;
  float mit_kp;
  float mit_kd;
  float torque_ff;
};

// 实时可调参数
struct TuneParams
{
  int wheel_index = 0;
  // 多轮并行模式：4 个轮子同时执行同一波形，每轮用自己的增益（默认 = crt_chassis.cpp:558-621 production 值）
  bool all_wheels = false;
  WheelTune w[STEER_NUM] = {
    {30.0f, 0.5f, 0.0f, 0.5f, 0.0f},  // ID0 / wheel 0 (production "good")
    {26.0f, 0.7f, 0.0f, 0.7f, 0.0f},  // ID1 / wheel 1 (production "bad")
    {30.0f, 0.5f, 0.0f, 0.5f, 0.0f},  // ID2 / wheel 2 (production "good")
    {26.0f, 0.7f, 0.0f, 0.7f, 0.0f},  // ID3 / wheel 3 (production "bad")
  };
  // 单轮模式下使用的标量（也是键盘 1-8 调整目标）
  float mit_kp = 0.0f;
  float mit_kd = 0.6f;
  float torque_ff = 0.0f;
  float pos_kp = 25.0f;
  float pos_kd = 0.5f;
  float steer_omega_max = 80.0f;    // 默认 80 rad/s（生产代码同值；物理上限 ~43 rad/s 由电机内部限幅自动截）
  float steer_omega_min = 0.3f;
  float deg_dead = 0.5f;
  float vel_ff_gain = 1.0f;        // 速度前馈系数；TUNE_VEL_FF 控制（0=关闭，1=完全前馈）
  bool  dyn_ff_enable = false;     // ★ 动力学前馈 (J·α+Ts·tanh(ω))；TUNE_DYN_FF=1 启用
                                   //   实测：与 MIT 内部速度环 D 项重复反而扰动，默认关闭
  bool  stiction_ff_enable = false;// ★ 方案 C：仅库仑/静摩擦补偿，TUNE_STICTION_FF=1 启用
                                   //   每轮独立用标定的 steer_static_friction × tanh(ω_des/ε)
                                   //   不含 J·α，避免之前 dyn FF 失败的元凶
  float target_slew_dps = 0.0f;    // ★ Slew rate 限速 (°/s)，0=关闭。step 工况建议 150~300。
                                   //   sine 30°/2s 自然速率 ≈ 94°/s，不受影响
  // 温度保护：超过 temp_limit_c 暂停，回落 temp_limit_c - temp_resume_hyst 后恢复
  float temp_limit_c = 70.0f;      // TUNE_TEMP_LIMIT，默认 70°C
  float temp_resume_hyst = 5.0f;   // TUNE_TEMP_HYST，默认 5°C 回差
  bool  thermal_paused = false;    // 是否因高温暂停中
  float last_max_motor_temp_c = 0.0f;  // 4 轮中最高 MOS/Rotor 温度（°C）
  // 0=step 1=sine 2=hold 3=ramp 4=auto
  int profile = 1;
  float target_deg = 30.0f;
  float period_ms = 2000.0f;
  float hold_deg = 0.0f;

  // ----- 自动扫参 -----
  bool auto_running = false;
  AutoState auto_state = AUTO_IDLE;
  uint32_t auto_state_start_tick = 0;
  uint32_t auto_warmup_ms = 1000;
  uint32_t auto_measure_ms = 4000;
  uint32_t auto_apply_ms = 300;
  size_t auto_idx = 0;
  std::vector<ComboParams> grid;
  std::vector<ComboResult> results;
  // 测量过程的累加器
  double acc_sum_err2 = 0.0;
  double acc_max_abs_err = 0.0;
  double acc_max_overshoot = 0.0;
  double acc_sum_omega_diff2 = 0.0;
  double acc_sum_target_x_actual = 0.0;
  double acc_sum_target2 = 0.0;
  double acc_sum_actual2 = 0.0;
  uint32_t acc_n = 0;
  uint32_t acc_sat_cnt = 0;
  float prev_omega_actual = 0.0f;
  bool prev_omega_init = false;
};

// ----------------- 自动扫参辅助 -----------------
static void build_grid_from_env(TuneParams &T)
{
  auto v_pkp = read_env_f_list("TUNE_GRID_POS_KP", "10,20,30,40");
  auto v_pkd = read_env_f_list("TUNE_GRID_POS_KD", "0.2,0.5,0.8");
  auto v_mkp = read_env_f_list("TUNE_GRID_MIT_KP", "0");
  auto v_mkd = read_env_f_list("TUNE_GRID_MIT_KD", "0.3,0.6,1.0");
  auto v_ff  = read_env_f_list("TUNE_GRID_FF",     "0");
  if (v_pkp.empty()) v_pkp.push_back(25.0f);
  if (v_pkd.empty()) v_pkd.push_back(0.5f);
  if (v_mkp.empty()) v_mkp.push_back(0.0f);
  if (v_mkd.empty()) v_mkd.push_back(0.6f);
  if (v_ff .empty()) v_ff.push_back(0.0f);

  T.grid.clear();
  for (float pkp : v_pkp)
    for (float pkd : v_pkd)
      for (float mkp : v_mkp)
        for (float mkd : v_mkd)
          for (float ff : v_ff)
            T.grid.push_back({pkp, pkd, mkp, mkd, ff});
}

static void reset_combo_accumulators(TuneParams &T)
{
  T.acc_sum_err2 = 0.0;
  T.acc_max_abs_err = 0.0;
  T.acc_max_overshoot = 0.0;
  T.acc_sum_omega_diff2 = 0.0;
  T.acc_sum_target_x_actual = 0.0;
  T.acc_sum_target2 = 0.0;
  T.acc_sum_actual2 = 0.0;
  T.acc_n = 0;
  T.acc_sat_cnt = 0;
  T.prev_omega_init = false;
  T.prev_omega_actual = 0.0f;
}

static ComboResult finalize_combo(const TuneParams &T, const ComboParams &P)
{
  ComboResult r{};
  r.p = P;
  r.n_samples = T.acc_n;
  if (T.acc_n == 0)
  {
    r.score = 1e9f;
    return r;
  }
  const double n = static_cast<double>(T.acc_n);
  r.rms_err_deg = static_cast<float>(std::sqrt(T.acc_sum_err2 / n));
  r.peak_err_deg = static_cast<float>(T.acc_max_abs_err);
  r.overshoot_deg = static_cast<float>(T.acc_max_overshoot);
  r.jitter_deg_per_s = static_cast<float>(std::sqrt(T.acc_sum_omega_diff2 / n));
  r.saturate_cnt = T.acc_sat_cnt;
  // 简易相位延迟代理：1 - 归一化互相关，越大越延迟
  double denom = std::sqrt(T.acc_sum_target2 * T.acc_sum_actual2);
  double corr = (denom > 1e-9) ? (T.acc_sum_target_x_actual / denom) : 0.0;
  if (corr > 1.0) corr = 1.0;
  if (corr < -1.0) corr = -1.0;
  r.lag_ms = static_cast<float>((1.0 - corr) * 100.0);
  // 综合得分（越小越好）：rms 与 overshoot 是首要指标，jitter / lag 辅助
  r.score = r.rms_err_deg + 1.5f * r.overshoot_deg + 0.5f * r.jitter_deg_per_s
            + 0.3f * r.lag_ms;
  return r;
}

static void save_results_csv(const TuneParams &T, const std::string &path)
{
  std::vector<ComboResult> sorted = T.results;
  std::sort(sorted.begin(), sorted.end(),
            [](const ComboResult &a, const ComboResult &b) {
              return a.score < b.score;
            });
  std::ofstream out(path, std::ios::trunc);
  if (!out.is_open())
  {
    std::cerr << "[TUNE][AUTO] cannot open " << path << " for write\n";
    return;
  }
  out << "rank,pos_kp,pos_kd,mit_kp,mit_kd,ff,"
      << "rms_err_deg,peak_err_deg,overshoot_deg,jitter_dps,lag_ms,sat_cnt,n,score\n";
  out << std::fixed << std::setprecision(4);
  int rank = 1;
  for (auto &r : sorted)
  {
    out << rank++ << ","
        << r.p.pos_kp << "," << r.p.pos_kd << ","
        << r.p.mit_kp << "," << r.p.mit_kd << "," << r.p.ff << ","
        << r.rms_err_deg << "," << r.peak_err_deg << "," << r.overshoot_deg
        << "," << r.jitter_deg_per_s << "," << r.lag_ms << ","
        << r.saturate_cnt << "," << r.n_samples << "," << r.score << "\n";
  }
  out.close();
  // 控制台打印 Top 10
  std::cout << "\n========== AUTO SWEEP RESULTS (top 10 by score) ==========\n";
  std::cout << std::fixed << std::setprecision(3);
  std::cout
    << "  # | POS_KP POS_KD MIT_KP MIT_KD   FF | RMSerr PeakErr Overshot Jitter  Lag | Score\n"
    << "----+---------------------------------+----------------------------------+--------\n";
  size_t to_show = std::min<size_t>(10, sorted.size());
  for (size_t i = 0; i < to_show; ++i)
  {
    const auto &r = sorted[i];
    std::cout << std::setw(3) << (i + 1) << " | "
              << std::setw(6) << r.p.pos_kp << " "
              << std::setw(6) << r.p.pos_kd << " "
              << std::setw(6) << r.p.mit_kp << " "
              << std::setw(6) << r.p.mit_kd << " "
              << std::setw(4) << r.p.ff << " | "
              << std::setw(6) << r.rms_err_deg << " "
              << std::setw(7) << r.peak_err_deg << " "
              << std::setw(7) << r.overshoot_deg << " "
              << std::setw(6) << r.jitter_deg_per_s << " "
              << std::setw(5) << r.lag_ms << " | "
              << std::setw(7) << r.score << "\n";
  }
  std::cout << "\nFull results saved to: " << path << "\n\n";
  if (!sorted.empty())
  {
    const auto &b = sorted.front();
    std::cout << "Suggested params:\n"
              << "  POS_KP=" << b.p.pos_kp
              << "  POS_KD=" << b.p.pos_kd
              << "  MIT_KP=" << b.p.mit_kp
              << "  MIT_KD=" << b.p.mit_kd
              << "  TORQUE_FF=" << b.p.ff << "\n\n";
  }
}

int main(int argc, char *argv[])
{
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::string ifname = (argc > 1) ? argv[1] : "enp86s0";

  std::cout << "===============================================\n"
            << "   Steer Tuning Tool (DM MIT + Outer PD + AUTO)\n"
            << "   IFNAME: " << ifname << "\n"
            << "===============================================\n";

  // ---- 1. 读取环境变量初值 ----
  TuneParams T;
  T.wheel_index = read_env_i("TUNE_WHEEL", T.wheel_index);
  if (T.wheel_index < 0 || T.wheel_index >= STEER_NUM) T.wheel_index = 0;
  T.mit_kp = read_env_f("TUNE_MIT_KP", T.mit_kp);
  T.mit_kd = read_env_f("TUNE_MIT_KD", T.mit_kd);
  T.torque_ff = read_env_f("TUNE_TORQUE_FF", T.torque_ff);
  T.pos_kp = read_env_f("TUNE_POS_KP", T.pos_kp);
  T.pos_kd = read_env_f("TUNE_POS_KD", T.pos_kd);
  T.steer_omega_max = read_env_f("TUNE_OMEGA_MAX", T.steer_omega_max);
  T.steer_omega_min = read_env_f("TUNE_OMEGA_MIN", T.steer_omega_min);
  T.deg_dead = read_env_f("TUNE_DEG_DEAD", T.deg_dead);
  T.vel_ff_gain = read_env_f("TUNE_VEL_FF", T.vel_ff_gain);
  T.dyn_ff_enable = (read_env_i("TUNE_DYN_FF", 0) == 1);
  T.stiction_ff_enable = (read_env_i("TUNE_STICTION_FF", 0) == 1);
  T.target_slew_dps = read_env_f("TUNE_TARGET_SLEW_DPS", T.target_slew_dps);
  T.temp_limit_c = read_env_f("TUNE_TEMP_LIMIT", T.temp_limit_c);
  T.temp_resume_hyst = read_env_f("TUNE_TEMP_HYST", T.temp_resume_hyst);
  T.profile = read_env_i("TUNE_PROFILE", T.profile);
  T.target_deg = read_env_f("TUNE_TARGET_DEG", T.target_deg);
  T.period_ms = read_env_f("TUNE_PERIOD_MS", T.period_ms);
  T.hold_deg = read_env_f("TUNE_HOLD_DEG", T.hold_deg);
  T.auto_warmup_ms = read_env_i("TUNE_AUTO_WARMUP_MS", T.auto_warmup_ms);
  T.auto_measure_ms = read_env_i("TUNE_AUTO_MEASURE_MS", T.auto_measure_ms);
  bool auto_at_start = (read_env_i("TUNE_AUTO", 0) == 1);

  // ---- 1b. 多轮并行模式：每轮独立增益 ----
  T.all_wheels = (read_env_i("TUNE_ALL_WHEELS", 0) == 1);
  for (int i = 0; i < STEER_NUM; ++i)
  {
    const std::string p = std::string("TUNE_W") + std::to_string(i) + "_";
    T.w[i].pos_kp    = read_env_f((p + "POS_KP").c_str(),    T.w[i].pos_kp);
    T.w[i].pos_kd    = read_env_f((p + "POS_KD").c_str(),    T.w[i].pos_kd);
    T.w[i].mit_kp    = read_env_f((p + "MIT_KP").c_str(),    T.w[i].mit_kp);
    T.w[i].mit_kd    = read_env_f((p + "MIT_KD").c_str(),    T.w[i].mit_kd);
    T.w[i].torque_ff = read_env_f((p + "FF").c_str(),        T.w[i].torque_ff);
  }
  if (T.all_wheels && auto_at_start)
  {
    std::cerr << "[TUNE][WARN] AUTO_SWEEP not supported with TUNE_ALL_WHEELS=1; ignoring TUNE_AUTO\n";
    auto_at_start = false;
  }
  if (T.all_wheels)
  {
    std::cout << "[TUNE] ALL-WHEELS mode ON; per-wheel gains:\n";
    for (int i = 0; i < STEER_NUM; ++i)
    {
      std::cout << "  W" << i
                << ": pos_kp=" << T.w[i].pos_kp
                << " pos_kd=" << T.w[i].pos_kd
                << " mit_kp=" << T.w[i].mit_kp
                << " mit_kd=" << T.w[i].mit_kd
                << " ff=" << T.w[i].torque_ff << "\n";
    }
    std::cout << "[TUNE] Velocity feedforward gain TUNE_VEL_FF=" << T.vel_ff_gain
              << "  (1=full FF, 0=disable)\n";
    std::cout << "[TUNE] Steer omega cap TUNE_OMEGA_MAX=" << T.steer_omega_max
              << " rad/s\n";
    std::cout << "[TUNE] Thermal protection TUNE_TEMP_LIMIT=" << T.temp_limit_c
              << "°C  hyst=" << T.temp_resume_hyst << "°C\n";
  }

  // ---- 2. EtherCAT / LinkX 初始化 ----
  if (!ecat_master_init(&st_master, ifname.c_str()))
  {
    std::cerr << "[TUNE] ecat_master_init failed\n";
    return -1;
  }

  // dual-LinkX 配置下舵电机/编码器都在 classic LinkX (alias=2 / slave_id=2)，
  // FD LinkX (alias=1) 挂夹爪，这里不用。task.cpp 用 alias-based binding，本测试工具简化为硬编码。
  linkx_init(&st_linkx, 2, &st_master.ctx);
  linkx_hw_wakeup(&st_linkx);
  for (int i = 0; i < kChannelCount; i++)
    linkx_set_can_baudrate(&st_linkx, i, 0, 2, 31, 8, 8, 1, 31, 8, 8);

  if (!ecat_master_bring_online(&st_master))
  {
    std::cerr << "[TUNE] ecat bring_online failed\n";
    return -1;
  }

  // ---- 3. Chassis 初始化 ----
  st_chassis.Init(&st_linkx);
  st_chassis.Init_Motor_Params();
  std::cout << "[TUNE] chassis init done (no remote control)\n";
  st_chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

  // ---- 4. 等待编码器首帧（最多 3 秒）----
  std::cout << "[TUNE] waiting for encoder first frames (timeout 3s)...\n";
  auto next_wakeup = std::chrono::steady_clock::now();
  uint32_t tick = 0;
  while (st_running.load() && st_master.is_running)
  {
    next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);
    can_msg_t msg;
    for (uint8_t ch = 0; ch < kChannelCount; ch++)
      while (linkx_quick_recv(&st_linkx, ch, &msg))
        can_dispatch(ch, msg.id, msg.data);
    linkx_send_pdos(&st_linkx);

    if ((tick % 100) == 0)
    {
      bool all_ok = true;
      for (int i = 0; i < STEER_NUM; ++i)
        if (!st_chassis.Encoder_Steer[i].Has_Valid_Wheel_Posture())
        { all_ok = false; break; }
      if (all_ok) { std::cout << "[TUNE] encoders ready\n"; break; }
    }
    if (tick > 3000)
    {
      std::cerr << "[TUNE][WARN] encoder timeout, continuing anyway\n";
      break;
    }
    tick++;
    std::this_thread::sleep_until(next_wakeup);
  }

  // ---- 5. 使能电机 ----
  std::cout << "[TUNE] enabling DM motors...\n";
  for (int i = 0; i < STEER_NUM; ++i)
    st_chassis.Motor_Steer[i].CAN_Send_Enter();
  linkx_send_pdos(&st_linkx);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // ---- 6. 准备网格（无论是否立刻启动 auto）----
  build_grid_from_env(T);
  std::cout << "[TUNE] grid built: " << T.grid.size() << " combos\n"
            << "  warmup_ms=" << T.auto_warmup_ms
            << " measure_ms=" << T.auto_measure_ms << "\n";

  if (auto_at_start)
  {
    T.profile = 4;
    T.auto_running = true;
    T.auto_state = AUTO_APPLY;
    T.auto_idx = 0;
    T.results.clear();
    reset_combo_accumulators(T);
    T.auto_state_start_tick = tick;
    std::cout << "[TUNE] AUTO_SWEEP started (TUNE_AUTO=1)\n";
  }

  // ---- 7. 进入主循环 ----
  std::cout << "\n[TUNE] === entering tuning mode ===\n"
            << "  q=quit  SPACE=e-stop  a=AUTO_SWEEP start  A=stop\n"
            << "  manual: 1/2 MIT_KP  3/4 MIT_KD  5/6 POS_KP  7/8 POS_KD\n"
            << "          9/0 TF  -/= TGT  [/]PER  s/v/h/r profile  w/x wheel\n\n";

  StdinRaw stdin_raw;

  // 高频原始 CSV
  std::ofstream csv("var_data/steer_tuning.csv", std::ios::trunc);
  if (csv.is_open())
  {
    csv << "t_ms,wheel,target_deg,actual_deg,err_deg,"
        << "steer_omega_cmd,motor_omega_cmd,motor_omega_now,motor_torque_now,"
        << "mit_kp,mit_kd,torque_ff,pos_kp,pos_kd,profile,auto_idx\n";
  }
  else std::cerr << "[TUNE][WARN] cannot open var_data/steer_tuning.csv\n";
  bool log_enabled = true;

  // 外层 PD 状态
  float last_pos_err[STEER_NUM] = {0};
  bool first_pos_err[STEER_NUM] = {true, true, true, true};

  // 起始零点：用当前编码器位置作为本次试验的 0°
  float steer_zero_offset_rad[STEER_NUM] = {0};
  for (int i = 0; i < STEER_NUM; ++i)
    if (st_chassis.Encoder_Steer[i].Has_Valid_Wheel_Posture())
      steer_zero_offset_rad[i] =
        wrap_pi(st_chassis.Encoder_Steer[i].Get_Wheel_Posture_radian_True());

  const auto t0 = std::chrono::steady_clock::now();

  while (st_running.load() && st_master.is_running)
  {
    next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);

    // (a) EtherCAT 接收
    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);
    can_msg_t msg;
    for (uint8_t ch = 0; ch < kChannelCount; ch++)
      while (linkx_quick_recv(&st_linkx, ch, &msg))
        can_dispatch(ch, msg.id, msg.data);

    // (b) 100ms 心跳
    if ((tick % k100msPeriod) == 0)
    {
      for (int i = 0; i < STEER_NUM; ++i)
      {
        st_chassis.Motor_Steer[i].TIM_Alive_PeriodElapsedCallback();
        st_chassis.Encoder_Steer[i].TIM_Alive_PeriodElapsedCallback();
        if (st_chassis.Motor_Steer[i].Get_Status() != Motor_DM_Status_ENABLE)
          st_chassis.Motor_Steer[i].CAN_Send_Enter();
      }

      // ---- 温度监控：4 轮 MOS/Rotor 中取最高，超阈值进入暂停 ----
      float max_temp_c = 0.0f;
      int   hot_wheel = -1;
      const char *hot_src = "";
      for (int i = 0; i < STEER_NUM; ++i)
      {
        // Get_Now_*_Temperature 返回开尔文（℃ + 273.15）
        const float mos_c   = st_chassis.Motor_Steer[i].Get_Now_MOS_Temperature()   - CELSIUS_TO_KELVIN;
        const float rotor_c = st_chassis.Motor_Steer[i].Get_Now_Rotor_Temperature() - CELSIUS_TO_KELVIN;
        // 仅在合理范围内（避免空读 0 或离谱值）
        if (mos_c > -50.0f && mos_c < 200.0f && mos_c > max_temp_c)
        { max_temp_c = mos_c; hot_wheel = i; hot_src = "MOS"; }
        if (rotor_c > -50.0f && rotor_c < 200.0f && rotor_c > max_temp_c)
        { max_temp_c = rotor_c; hot_wheel = i; hot_src = "Rotor"; }
      }
      T.last_max_motor_temp_c = max_temp_c;

      if (!T.thermal_paused && max_temp_c >= T.temp_limit_c)
      {
        T.thermal_paused = true;
        // 暂停 AUTO 扫参，强制 hold 0°
        if (T.auto_running) { T.auto_running = false; T.auto_state = AUTO_IDLE; }
        T.profile = 2;
        T.hold_deg = 0.0f;
        T.target_deg = 0.0f;
        std::cerr << "\n[TUNE][THERMAL] PAUSED: wheel=" << hot_wheel
                  << " " << hot_src << "=" << max_temp_c << "°C >= "
                  << T.temp_limit_c << "°C; resume below "
                  << (T.temp_limit_c - T.temp_resume_hyst) << "°C\n";
      }
      else if (T.thermal_paused && max_temp_c <= (T.temp_limit_c - T.temp_resume_hyst))
      {
        T.thermal_paused = false;
        std::cerr << "\n[TUNE][THERMAL] RESUMED: max="
                  << max_temp_c << "°C; press s/v/h/r to choose profile\n";
      }
    }

    // (c) 时间
    const auto t_now = std::chrono::steady_clock::now();
    const float t_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                         t_now - t0).count() / 1000.0f;

    // (d) AUTO 状态机驱动参数 + 阶段
    if (T.auto_running && T.profile == 4)
    {
      const uint32_t state_age = tick - T.auto_state_start_tick;
      if (T.auto_idx >= T.grid.size() && T.auto_state != AUTO_DONE)
      {
        T.auto_state = AUTO_DONE;
      }

      switch (T.auto_state)
      {
        case AUTO_APPLY:
        {
          if (T.auto_idx < T.grid.size())
          {
            const auto &g = T.grid[T.auto_idx];
            T.pos_kp = g.pos_kp;
            T.pos_kd = g.pos_kd;
            T.mit_kp = g.mit_kp;
            T.mit_kd = g.mit_kd;
            T.torque_ff = g.ff;
          }
          if (state_age >= T.auto_apply_ms)
          {
            reset_combo_accumulators(T);
            T.auto_state = AUTO_WARMUP;
            T.auto_state_start_tick = tick;
          }
          break;
        }
        case AUTO_WARMUP:
        {
          if (state_age >= T.auto_warmup_ms)
          {
            reset_combo_accumulators(T);
            T.auto_state = AUTO_MEASURE;
            T.auto_state_start_tick = tick;
          }
          break;
        }
        case AUTO_MEASURE:
        {
          if (state_age >= T.auto_measure_ms)
          {
            ComboResult r = finalize_combo(T, T.grid[T.auto_idx]);
            T.results.push_back(r);
            T.auto_state = AUTO_NEXT;
            T.auto_state_start_tick = tick;
          }
          break;
        }
        case AUTO_NEXT:
        {
          T.auto_idx++;
          if (T.auto_idx >= T.grid.size())
            T.auto_state = AUTO_DONE;
          else {
            T.auto_state = AUTO_APPLY;
            T.auto_state_start_tick = tick;
          }
          break;
        }
        case AUTO_DONE:
        {
          if (T.auto_running)
          {
            save_results_csv(T, "var_data/steer_tuning_results.csv");
            T.auto_running = false;
            // 进入"扫参完成"状态：手动 hold 0°，让用户读 CSV / 滚屏
            T.profile = 2;
            T.hold_deg = 0.0f;
            T.target_deg = 0.0f;
          }
          break;
        }
        case AUTO_IDLE:
        default:
          break;
      }
    }

    // (e) 计算目标角（连续运动：正弦默认；其他保留）
    float target_steer_deg;
    if (T.profile == 4)
    {
      // AUTO 模式：用连续正弦目标（"目标位置一直变" 的连续运动需求）
      target_steer_deg = T.target_deg *
        std::sin(2.0f * static_cast<float>(M_PI) * t_ms / T.period_ms);
    }
    else
    {
      switch (T.profile)
      {
        case 0:
        {
          float ph = std::fmod(t_ms, T.period_ms);
          target_steer_deg = (ph < 0.5f * T.period_ms) ? T.target_deg : -T.target_deg;
          break;
        }
        case 1:
          target_steer_deg = T.target_deg *
            std::sin(2.0f * static_cast<float>(M_PI) * t_ms / T.period_ms);
          break;
        case 2:
          target_steer_deg = T.hold_deg;
          break;
        case 3:
        {
          float p = std::fmod(t_ms, T.period_ms) / T.period_ms;
          if (p < 0.25f) target_steer_deg = (4.0f * p) * T.target_deg;
          else if (p < 0.75f) target_steer_deg = (2.0f - 4.0f * p) * T.target_deg;
          else target_steer_deg = (4.0f * p - 4.0f) * T.target_deg;
          break;
        }
        default:
          target_steer_deg = T.hold_deg;
          break;
      }
    }
    const int idx = T.wheel_index;

    // (f-k) 每轮 PD 计算 + MIT 下发
    // 多轮并行模式 (TUNE_ALL_WHEELS=1)：4 个轮子同时执行同一波形，每轮用自己的增益
    // 单轮模式：只 wheel_index 走 PD，其它 3 个保持原地
    const float dt = static_cast<float>(kCtrlPeriodMs) / 1000.0f;
    const float deadband_rad = deg2rad(T.deg_dead);

    // ★ Slew rate 限制：把 target 的瞬时跳变（如 step）限制在 max_rate °/s 内
    //   sine 自然速率（30°·2π/period）通常 < 100 °/s，不受限；
    //   step 60° 跳变会被斜坡化，避免 vel FF 注入 60000°/s 命令冲爆
    //   TUNE_TARGET_SLEW_DPS=0 关闭，默认 200 °/s（step 60° 约 0.3s 完成）
    if (T.target_slew_dps > 0.0f)
    {
      static float slewed_target = 0.0f;
      static bool  slew_init = false;
      if (!slew_init) { slewed_target = target_steer_deg; slew_init = true; }
      const float max_step = T.target_slew_dps * dt;
      float dtg = target_steer_deg - slewed_target;
      if (dtg >  max_step) dtg =  max_step;
      if (dtg < -max_step) dtg = -max_step;
      slewed_target += dtg;
      target_steer_deg = slewed_target;
    }

    // ★ 速度前馈：基于目标位置的差分（4 轮共用同一目标，所以只算一次）
    static float last_target_steer_deg = 0.0f;
    static bool  ff_initialized = false;
    if (!ff_initialized) {
      last_target_steer_deg = target_steer_deg;
      ff_initialized = true;
    }
    const float target_omega_ff_rad_s =
        deg2rad(target_steer_deg - last_target_steer_deg) / dt;
    last_target_steer_deg = target_steer_deg;

    // 选中轮的最终状态（供 dashboard / AUTO_SWEEP 使用）
    float now_steer_rad = 0.0f;
    float err = 0.0f;
    float steer_omega = 0.0f;
    float motor_omega_cmd = 0.0f;
    bool saturated = false;

    for (int i = 0; i < STEER_NUM; ++i)
    {
      WheelTune g;
      if (T.all_wheels)
      {
        g = T.w[i];
      }
      else if (i == idx)
      {
        g = WheelTune{T.pos_kp, T.pos_kd, T.mit_kp, T.mit_kd, T.torque_ff};
      }
      else
      {
        // 单轮模式下，非选中轮保持原地（弱阻尼）
        st_chassis.Motor_Steer[i].Set_Control_Torque_P_D_MIT(0.0f, 0.0f, 0.5f);
        st_chassis.Motor_Steer[i].Set_Control_Parameter_MIT(0.0f, 0.0f);
        continue;
      }

      // 当前舵向角
      float wi_now_rad;
      if (st_chassis.Encoder_Steer[i].Has_Valid_Wheel_Posture())
        wi_now_rad = st_chassis.Encoder_Steer[i].Get_Wheel_Posture_radian_True();
      else
        wi_now_rad = st_chassis.Motor_Steer[i].Get_Now_Radian() / REDUCTION;
      wi_now_rad = wrap_pi(wi_now_rad);

      const float wi_target_rad =
        wrap_pi(deg2rad(target_steer_deg) + steer_zero_offset_rad[i]);
      float wi_err = wrap_pi(wi_target_rad - wi_now_rad);

      // 外层 PD
      if (first_pos_err[i])
      {
        last_pos_err[i] = wi_err;
        first_pos_err[i] = false;
      }
      const float wi_derr = (wi_err - last_pos_err[i]) / dt;
      last_pos_err[i] = wi_err;

      float wi_omega = 0.0f;
      bool wi_sat = false;
      // ★ 前馈始终激活（即使在死区内也要跟住 target 速度）
      wi_omega = T.vel_ff_gain * target_omega_ff_rad_s;
      if (std::fabs(wi_err) > deadband_rad)
      {
        wi_omega += g.pos_kp * wi_err + g.pos_kd * wi_derr;
      }
      if (wi_omega > T.steer_omega_max) { wi_omega = T.steer_omega_max; wi_sat = true; }
      if (wi_omega < -T.steer_omega_max) { wi_omega = -T.steer_omega_max; wi_sat = true; }
      if (std::fabs(wi_omega) > 1e-6f && std::fabs(wi_omega) < T.steer_omega_min)
        wi_omega = (wi_omega >= 0.0f ? 1.0f : -1.0f) * T.steer_omega_min;
      const float wi_motor_omega_cmd = wi_omega * REDUCTION;

      // 电机绝对位置目标（让 MIT_KP > 0 时位置项有效）
      const float wi_now_motor_rad = st_chassis.Motor_Steer[i].Get_Now_Radian();
      const float wi_target_motor_rad = wi_now_motor_rad + wi_err * REDUCTION;

      // ★ 动力学前馈（电机轴侧）：T_ff = J*α_target + B*ω_target + Ts*tanh(ω_target/1.0)
      //   注意：必须用 ★ 目标 ★ 量（来自 target_omega_ff_rad_s 即位置目标的纯差分），
      //   而不是 PD 输出 wi_omega（含反馈，差分后会爆冲为 α）。
      static float last_target_omega_motor[STEER_NUM] = {0};
      static bool  dyn_ff_init[STEER_NUM] = {false, false, false, false};
      const float target_omega_motor = target_omega_ff_rad_s * REDUCTION;
      // 第一帧不算 α（避免 (X-0)/dt 瞬态冲爆）
      float alpha_target_motor = 0.0f;
      if (dyn_ff_init[i])
        alpha_target_motor = (target_omega_motor - last_target_omega_motor[i]) / dt;
      else
        dyn_ff_init[i] = true;
      // 安全限幅（命令突变时仍可能很大）
      if (alpha_target_motor >  500.0f) alpha_target_motor =  500.0f;
      if (alpha_target_motor < -500.0f) alpha_target_motor = -500.0f;
      last_target_omega_motor[i] = target_omega_motor;

      const auto &wp = st_chassis.steer_wheel_params_[i];
      const float dyn_ff = (T.dyn_ff_enable
        ? (wp.steer_inertia * alpha_target_motor
         + wp.steer_damping  * target_omega_motor
         + wp.steer_static_friction * std::tanh(target_omega_motor / 1.0f))
        : 0.0f);
      // ★ 方案 C：仅 stiction 摩擦补偿（电机轴侧），无 J·α 项
      //   每轮用各自标定的 steer_static_friction（W0=0.04 / W1=0.05 / W2=0.08 / W3=0.11 Nm）
      //   tanh(ω/0.5) 比 dyn_ff 用的 tanh(ω/1.0) 更陡 → 小速度也能尽快到位
      const float stict_ff = (T.stiction_ff_enable
        ? wp.steer_static_friction * std::tanh(target_omega_motor / 0.5f)
        : 0.0f);
      const float total_torque_ff = g.torque_ff + dyn_ff + stict_ff;

      auto &wi_mot = st_chassis.Motor_Steer[i];
      wi_mot.Set_Control_Torque_P_D_MIT(total_torque_ff, g.mit_kp, g.mit_kd);
      wi_mot.Set_Control_Parameter_MIT(wi_target_motor_rad, wi_motor_omega_cmd);

      // 高频 CSV：每个 active 轮子写一行（多轮模式下每个采样 4 行）
      if (log_enabled && (tick % kCsvPeriodMs) == 0 && csv.is_open())
      {
        csv << t_ms << "," << i
            << "," << target_steer_deg
            << "," << rad2deg(wi_now_rad - steer_zero_offset_rad[i])
            << "," << rad2deg(wi_err)
            << "," << wi_omega
            << "," << wi_motor_omega_cmd
            << "," << wi_mot.Get_Now_Omega()
            << "," << wi_mot.Get_Now_Torque()
            << "," << g.mit_kp
            << "," << g.mit_kd
            << "," << g.torque_ff
            << "," << g.pos_kp
            << "," << g.pos_kd
            << "," << T.profile
            << "," << static_cast<int>(T.auto_running ? T.auto_idx : 0)
            << "\n";
      }

      // 选中轮的状态保存到外层（供 dashboard / AUTO_SWEEP 使用）
      if (i == idx)
      {
        now_steer_rad = wi_now_rad;
        err = wi_err;
        steer_omega = wi_omega;
        motor_omega_cmd = wi_motor_omega_cmd;
        saturated = wi_sat;
      }
    }

    // 触发 MIT 帧发送
    for (int i = 0; i < STEER_NUM; ++i)
      st_chassis.Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
    linkx_send_pdos(&st_linkx);

    // 选中轮 motor 句柄（供 AUTO_SWEEP / dashboard 读 omega/torque）
    auto &mot = st_chassis.Motor_Steer[idx];

    // (l) 测量阶段累加指标
    if (T.auto_running && T.profile == 4 && T.auto_state == AUTO_MEASURE)
    {
      const float err_deg = rad2deg(err);
      const float target_abs = std::fabs(target_steer_deg);
      const float actual_deg = rad2deg(now_steer_rad - steer_zero_offset_rad[idx]);
      const float over =
        (target_abs > 0.5f && std::fabs(actual_deg) > target_abs)
          ? (std::fabs(actual_deg) - target_abs) : 0.0f;
      T.acc_sum_err2 += static_cast<double>(err_deg) * err_deg;
      T.acc_max_abs_err = std::max(T.acc_max_abs_err, static_cast<double>(std::fabs(err_deg)));
      T.acc_max_overshoot = std::max(T.acc_max_overshoot, static_cast<double>(over));
      T.acc_sum_target_x_actual += target_steer_deg * actual_deg;
      T.acc_sum_target2 += target_steer_deg * target_steer_deg;
      T.acc_sum_actual2 += actual_deg * actual_deg;
      const float actual_omega_steer = mot.Get_Now_Omega() / REDUCTION;
      if (T.prev_omega_init)
      {
        const float dom = actual_omega_steer - T.prev_omega_actual;
        T.acc_sum_omega_diff2 += static_cast<double>(dom) * dom;
      }
      T.prev_omega_actual = actual_omega_steer;
      T.prev_omega_init = true;
      if (saturated) T.acc_sat_cnt++;
      T.acc_n++;
    }

    // (m) 仪表盘
    if ((tick % kDashboardPeriodMs) == 0)
    {
      const float motor_omega_now = mot.Get_Now_Omega();
      const float motor_torque_now = mot.Get_Now_Torque();
      const char *prof_name =
        (T.profile == 0 ? "step" :
         T.profile == 1 ? "sine" :
         T.profile == 2 ? "hold" :
         T.profile == 3 ? "ramp" : "AUTO");

      std::cout << "\033[2J\033[H";
      std::cout << std::fixed << std::setprecision(3);
      std::cout << "[STEER-TUNING]  t=" << t_ms << " ms"
                << "  mode=" << (T.all_wheels ? "ALL-WHEELS" : "SINGLE")
                << "  log=" << (log_enabled ? "ON" : "OFF")
                << "  Tmax=" << T.last_max_motor_temp_c << "°C"
                << (T.thermal_paused ? "  [THERMAL-PAUSED]" : "")
                << "\n"
                << "  wheel=" << T.wheel_index << "  profile=" << prof_name << "\n"
                << "  target_deg=" << target_steer_deg
                << "  actual_deg=" << rad2deg(now_steer_rad - steer_zero_offset_rad[idx])
                << "  err_deg=" << rad2deg(err) << "\n"
                << "  steer_omega_cmd=" << steer_omega
                << "  motor_omega_cmd=" << motor_omega_cmd
                << "  motor_omega_now=" << motor_omega_now
                << "  motor_torque_now=" << motor_torque_now << "\n";

      if (T.all_wheels)
      {
        // 多轮模式：列出 4 个轮子的实测状态
        std::cout << "  ---- 4-WHEEL STATE ----\n";
        std::cout << "    W | pos_kp pos_kd mit_kp mit_kd  ff |  actual_deg     err_deg  m_omega   m_torq\n"
                  << "  ----+-------------------------------+-------------------------------------------\n";
        for (int i = 0; i < STEER_NUM; ++i)
        {
          float wi_rad = st_chassis.Encoder_Steer[i].Has_Valid_Wheel_Posture()
                       ? wrap_pi(st_chassis.Encoder_Steer[i].Get_Wheel_Posture_radian_True())
                       : wrap_pi(st_chassis.Motor_Steer[i].Get_Now_Radian() / REDUCTION);
          float wi_act_deg = rad2deg(wi_rad - steer_zero_offset_rad[i]);
          float wi_err_deg = target_steer_deg - wi_act_deg;
          std::cout << "    " << i << " | "
                    << std::setw(6) << T.w[i].pos_kp << " "
                    << std::setw(6) << T.w[i].pos_kd << " "
                    << std::setw(6) << T.w[i].mit_kp << " "
                    << std::setw(6) << T.w[i].mit_kd << " "
                    << std::setw(4) << T.w[i].torque_ff << " | "
                    << std::setw(11) << wi_act_deg << "  "
                    << std::setw(9) << wi_err_deg << "  "
                    << std::setw(7) << st_chassis.Motor_Steer[i].Get_Now_Omega() << "  "
                    << std::setw(7) << st_chassis.Motor_Steer[i].Get_Now_Torque() << "\n";
        }
      }
      else
      {
        std::cout << "  ---- params (live, single-wheel) ----\n"
                  << "  MIT_KP=" << T.mit_kp << "  MIT_KD=" << T.mit_kd
                  << "  TORQUE_FF=" << T.torque_ff << "\n"
                  << "  POS_KP=" << T.pos_kp << "  POS_KD=" << T.pos_kd << "\n";
      }
      std::cout << "  TARGET_DEG=" << T.target_deg
                << "  PERIOD_MS=" << T.period_ms
                << "  HOLD_DEG=" << T.hold_deg << "\n";

      if (T.auto_running)
      {
        const char *st_name =
          (T.auto_state == AUTO_APPLY   ? "APPLY  " :
           T.auto_state == AUTO_WARMUP  ? "WARMUP " :
           T.auto_state == AUTO_MEASURE ? "MEASURE" :
           T.auto_state == AUTO_NEXT    ? "NEXT   " :
           T.auto_state == AUTO_DONE    ? "DONE   " : "IDLE   ");
        std::cout << "  ---- AUTO_SWEEP ----\n"
                  << "  state=" << st_name
                  << "  combo=" << (T.auto_idx + 1) << "/" << T.grid.size()
                  << "  done=" << T.results.size() << "\n";
        if (!T.results.empty())
        {
          auto best_it = std::min_element(T.results.begin(), T.results.end(),
            [](const ComboResult &a, const ComboResult &b){ return a.score < b.score; });
          const auto &b = *best_it;
          std::cout << "  BEST so far: POS_KP=" << b.p.pos_kp
                    << " POS_KD=" << b.p.pos_kd
                    << " MIT_KP=" << b.p.mit_kp
                    << " MIT_KD=" << b.p.mit_kd
                    << " FF=" << b.p.ff
                    << " score=" << b.score << "\n";
        }
      }
      else if (!T.results.empty() && T.results.size() == T.grid.size())
      {
        // 扫参已完成，常驻显示 Top 3 摘要
        std::vector<ComboResult> sorted = T.results;
        std::sort(sorted.begin(), sorted.end(),
                  [](const ComboResult &a, const ComboResult &b){
                    return a.score < b.score;
                  });
        std::cout << "  ---- AUTO_SWEEP COMPLETE ----\n"
                  << "  total combos: " << T.grid.size()
                  << "   results saved: var_data/steer_tuning_results.csv\n"
                  << "  Top 3 (by score, lower=better):\n";
        size_t to_show = std::min<size_t>(3, sorted.size());
        for (size_t i = 0; i < to_show; ++i)
        {
          const auto &r = sorted[i];
          std::cout << "    #" << (i + 1) << " "
                    << "POS_KP=" << r.p.pos_kp
                    << " POS_KD=" << r.p.pos_kd
                    << " MIT_KP=" << r.p.mit_kp
                    << " MIT_KD=" << r.p.mit_kd
                    << " FF=" << r.p.ff
                    << " | rms=" << r.rms_err_deg
                    << " over=" << r.overshoot_deg
                    << " jit=" << r.jitter_deg_per_s
                    << " score=" << r.score << "\n";
        }
      }
      std::cout << "\n  Keys: q=quit  SPACE=e-stop  a=AUTO_SWEEP  A=stop\n"
                << "        manual: 1/2 MIT_KP  3/4 MIT_KD  5/6 POS_KP  7/8 POS_KD\n"
                << "                9/0 TF  -/= TGT  [/]PER  s/v/h/r profile  w/x wheel\n";
      std::cout.flush();
    }

    // (n) CSV 写入已移入上面的"每轮 PD"循环（每轮各写一行；多轮模式下 4 行/采样）

    // (o) 键盘处理
    char k;
    while (read_keypress(k))
    {
      switch (k)
      {
        case 'q':
        case 'Q':
          st_running.store(false);
          st_master.is_running = false;
          break;
        case ' ':
          for (int i = 0; i < STEER_NUM; ++i)
          {
            st_chassis.Motor_Steer[i].Set_Control_Status(Motor_DM_Status_DISABLE);
            st_chassis.Motor_Steer[i].CAN_Send_Exit();
          }
          T.profile = 2;
          T.hold_deg = 0.0f;
          T.target_deg = 0.0f;
          T.auto_running = false;
          T.auto_state = AUTO_IDLE;
          break;
        case 'a':  // 启动 AUTO
          if (T.all_wheels)
          {
            std::cout << "\n[TUNE] AUTO_SWEEP not supported in TUNE_ALL_WHEELS mode\n";
            break;
          }
          if (!T.auto_running && !T.grid.empty())
          {
            T.profile = 4;
            T.auto_running = true;
            T.auto_state = AUTO_APPLY;
            T.auto_idx = 0;
            T.results.clear();
            reset_combo_accumulators(T);
            T.auto_state_start_tick = tick;
            std::cout << "\n[TUNE] AUTO_SWEEP started: " << T.grid.size() << " combos\n";
          }
          break;
        case 'A':  // 中止 AUTO
          if (T.auto_running)
          {
            T.auto_running = false;
            T.auto_state = AUTO_IDLE;
            std::cout << "\n[TUNE] AUTO_SWEEP aborted by user\n";
            if (!T.results.empty())
              save_results_csv(T, "var_data/steer_tuning_results.csv");
            T.profile = 2;
            T.hold_deg = 0.0f;
          }
          break;
        case '1': if (!T.auto_running) T.mit_kp = std::max(0.0f, T.mit_kp - 0.5f); break;
        case '2': if (!T.auto_running) T.mit_kp = std::min(500.0f, T.mit_kp + 0.5f); break;
        case '3': if (!T.auto_running) T.mit_kd = std::max(0.0f, T.mit_kd - 0.05f); break;
        case '4': if (!T.auto_running) T.mit_kd = std::min(5.0f, T.mit_kd + 0.05f); break;
        case '5': if (!T.auto_running) T.pos_kp = std::max(0.0f, T.pos_kp - 1.0f); break;
        case '6': if (!T.auto_running) T.pos_kp = std::min(500.0f, T.pos_kp + 1.0f); break;
        case '7': if (!T.auto_running) T.pos_kd = std::max(0.0f, T.pos_kd - 0.05f); break;
        case '8': if (!T.auto_running) T.pos_kd = std::min(20.0f, T.pos_kd + 0.05f); break;
        case '9': if (!T.auto_running) T.torque_ff = std::max(-5.0f, T.torque_ff - 0.1f); break;
        case '0': if (!T.auto_running) T.torque_ff = std::min(5.0f, T.torque_ff + 0.1f); break;
        case '-': T.target_deg = std::max(1.0f, T.target_deg - 5.0f); break;
        case '=': T.target_deg = std::min(180.0f, T.target_deg + 5.0f); break;
        case '[': T.period_ms = std::max(100.0f, T.period_ms - 100.0f); break;
        case ']': T.period_ms = std::min(20000.0f, T.period_ms + 100.0f); break;
        case 's': if (!T.auto_running) T.profile = 0; break;
        case 'v': if (!T.auto_running) T.profile = 1; break;
        case 'h': if (!T.auto_running) { T.profile = 2; T.hold_deg = 0.0f; } break;
        case 'r': if (!T.auto_running) T.profile = 3; break;
        case 'w':
          T.wheel_index = (T.wheel_index + 1) % STEER_NUM;
          for (int i = 0; i < STEER_NUM; ++i) first_pos_err[i] = true;
          break;
        case 'x':
          T.wheel_index = (T.wheel_index + STEER_NUM - 1) % STEER_NUM;
          for (int i = 0; i < STEER_NUM; ++i) first_pos_err[i] = true;
          break;
        case 'l': log_enabled = !log_enabled; break;
        case 'p':
          if (csv.is_open())
            csv << "# t=" << t_ms << " wheel=" << T.wheel_index
                << " mit_kp=" << T.mit_kp << " mit_kd=" << T.mit_kd
                << " ff=" << T.torque_ff
                << " pos_kp=" << T.pos_kp << " pos_kd=" << T.pos_kd << "\n";
          break;
        default:
          break;
      }
    }

    tick++;
    std::this_thread::sleep_until(next_wakeup);
  }

  // 退出：失能电机
  std::cout << "\n[TUNE] disabling all DM motors...\n";
  for (int i = 0; i < STEER_NUM; ++i)
  {
    st_chassis.Motor_Steer[i].Set_Control_Status(Motor_DM_Status_DISABLE);
    st_chassis.Motor_Steer[i].CAN_Send_Exit();
  }
  linkx_send_pdos(&st_linkx);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (csv.is_open()) csv.close();

  // 如果还有 AUTO 中途采到的数据，也写一次结果
  if (!T.results.empty() && !T.auto_running)
  {
    // 已经在 AUTO_DONE 内写过；这里只在异常退出再写一遍兜底
    save_results_csv(T, "var_data/steer_tuning_results.csv");
  }

  std::cout << "[TUNE] CSV saved to var_data/steer_tuning.csv\n"
            << "[TUNE] bye.\n";
  return 0;
}
