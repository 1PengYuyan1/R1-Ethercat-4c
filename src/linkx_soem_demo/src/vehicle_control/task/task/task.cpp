/**
 * @file task.cpp
 * @brief 机器人主控制循环（task 层）。
 *
 *  职责（仅做调度，业务全部托管给 Class_Robot / Class_Chassis）：
 *    - 初始化 EtherCAT 主站与 LinkX (4×CAN) 适配器；
 *    - 1 ms 节拍：CAN 收 → Robot 周期任务 → CAN 发；
 *    - 周期诊断：CAN 统计、LIVE 仪表盘、舵向多圈累计持久化；
 *    - 启动一次性流程：舵向零点抓取、上电断电位移异常检查；
 *    - 退出时强制把累计脉冲落盘并关闭 ROS2 桥。
 */

#include "task.h"

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "robot.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include "math.h"

// ============================================================================
//   全局对象 —— 与 main.cpp 共享（main.cpp 中通过 extern 引用 master）
// ============================================================================

Class_Robot   robot;          ///< 机器人逻辑入口
ecat_master_t master;         ///< EtherCAT 主站（信号处理读 is_running）
linkx_t       linkx_dev;      ///< LinkX 4 通道 CAN 适配器（slave_id=1，经典 1Mbps）
linkx_t       linkx_dev_fd;   ///< LinkX 4 通道 CAN-FD 适配器（slave_id=2，ch0 经典 1M / ch1-3 FD）

// ============================================================================
//                             编译期可调参数
// ============================================================================
namespace {

constexpr int      kChannelCount                  = 4;        ///< CAN 通道数
constexpr uint32_t kControlLoopPeriodMs           = 1;        ///< 主循环周期 1 ms (1 kHz)
constexpr uint32_t kTaskPeriod2Ms                 = 2;        ///< 2 ms 任务节拍
constexpr uint32_t kTaskPeriod100Ms               = 100;      ///< 100 ms 任务节拍
constexpr uint32_t kCanStatPrintPeriodMs          = 5000;     ///< CAN 统计打印周期
constexpr uint32_t kLiveDashboardPeriodMs         = 200;      ///< LIVE 仪表盘刷新周期
constexpr uint32_t kSteerUnwrappedSavePeriodMs    = 200;      ///< 舵向多圈累计落盘周期
constexpr uint32_t kCalibrateRequiredFreshFrames  = 3;        ///< 抓零所需的新鲜帧数

constexpr bool kEnableCanStatPrint        = false;  // [DIAG] temporarily off to isolate stdout backpressure
constexpr bool kEnableLiveDashboard       = false;  // [DIAG] temporarily off to isolate stdout backpressure
constexpr bool kEnableSteerUnwrappedSave  = true;

constexpr const char *kDefaultVarDataFile = "var_data/live_variables.log";
constexpr const char *kDefaultVelocityLogFile = "var_data/odrive_velocity.csv";
constexpr uint32_t kVelocityLogPeriodMs = 10;   ///< 速度记录周期 10 ms (100 Hz)
constexpr float    kWheelRadius         = 0.018f;

std::ofstream g_var_data_stream;        ///< 仪表盘镜像日志（懒打开）
bool          g_var_data_stream_inited = false;

std::ofstream g_velocity_log_stream;    ///< ODrive 速度 CSV 日志
bool          g_velocity_log_inited = false;

/** @brief 抓零状态机的运行时数据（仅启动期使用一次）。 */
struct Steer_Zero_Capture_State
{
    bool     pending      = false;                       ///< 是否需要抓零
    bool     baseline_set = false;                       ///< 是否已固定基线
    uint32_t baseline_rx[kChannelCount] = {0};           ///< 基线时各编码器 rx_count
};

} // namespace

// ============================================================================
//                                工具函数
// ============================================================================

/**
 * @brief 把任意角度规范到 [-180°, 180°)。
 */
static inline float Wrap_Deg_Signed(float deg)
{
    float d = std::fmod(deg, 360.0f);
    if (d >= 180.0f)
        d -= 360.0f;
    else if (d < -180.0f)
        d += 360.0f;
    return d;
}

/**
 * @brief 懒打开变量日志文件流；优先取环境变量 VAR_DATA_FILE 指定的路径。
 *        失败仅 stderr 警告，不影响其他逻辑。
 */
static void Ensure_Var_Data_Stream()
{
    if (g_var_data_stream_inited)
        return;

    const char *path = std::getenv("VAR_DATA_FILE");
    if (path == nullptr || path[0] == '\0')
        path = kDefaultVarDataFile;

    g_var_data_stream.open(path, std::ios::app);
    g_var_data_stream_inited = true;
    if (!g_var_data_stream.is_open())
        std::cerr << "[WARN] Failed to open variable data file: " << path << std::endl;
}

// ============================================================================
//                          诊断打印 —— CAN 统计
// ============================================================================

/**
 * @brief 打印 4 个 CAN 通道的累计 TX/RX/丢帧/丢包率，以及全局合计。
 *        每 kCanStatPrintPeriodMs 调用一次。
 */
static void Print_CAN_Stats(linkx_t *linkx)
{
    std::cout << "\n[CAN-TOTAL]\n";

    uint64_t sum_tx_req_f = 0, sum_tx_req_b = 0;
    uint64_t sum_tx_sent_f = 0, sum_tx_sent_b = 0;
    uint64_t sum_tx_drop_f = 0;
    uint64_t sum_rx_f = 0,      sum_rx_b = 0;

    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        const auto &s = linkx->can_stats[ch];
        const uint16_t q_fill  = linkx->tx_queues[ch].size;
        const uint64_t loss_f  = (s.tx_frames > s.rx_frames) ? (s.tx_frames - s.rx_frames) : 0;
        const double   loss_rt = (s.tx_frames > 0) ? (100.0 * (double)loss_f / (double)s.tx_frames) : 0.0;

        std::cout << "  CH" << ch
                  << " TX_REQ=" << s.tx_enqueued_frames << " frames (" << s.tx_enqueued_bytes << " B)"
                  << " TX_SENT=" << s.tx_frames << " frames (" << s.tx_bytes << " B)"
                  << " TX_DROP=" << s.tx_dropped_frames
                  << " Q=" << q_fill
                  << " RX_TOTAL=" << s.rx_frames << " frames (" << s.rx_bytes << " B)"
                  << " TOTAL_LOSS=" << loss_f
                  << " TOTAL_LOSS_RATE=" << loss_rt << "%"
                  << std::endl;

        sum_tx_req_f  += s.tx_enqueued_frames;
        sum_tx_req_b  += s.tx_enqueued_bytes;
        sum_tx_sent_f += s.tx_frames;
        sum_tx_sent_b += s.tx_bytes;
        sum_tx_drop_f += s.tx_dropped_frames;
        sum_rx_f      += s.rx_frames;
        sum_rx_b      += s.rx_bytes;
    }

    const uint64_t sum_loss_f  = (sum_tx_sent_f > sum_rx_f) ? (sum_tx_sent_f - sum_rx_f) : 0;
    const double   sum_loss_rt = (sum_tx_sent_f > 0) ? (100.0 * (double)sum_loss_f / (double)sum_tx_sent_f) : 0.0;
    std::cout << "  ALL TX_REQ=" << sum_tx_req_f << " frames (" << sum_tx_req_b << " B)"
              << " TX_SENT=" << sum_tx_sent_f << " frames (" << sum_tx_sent_b << " B)"
              << " TX_DROP=" << sum_tx_drop_f
              << " RX_TOTAL=" << sum_rx_f << " frames (" << sum_rx_b << " B)"
              << " TOTAL_LOSS=" << sum_loss_f
              << " TOTAL_LOSS_RATE=" << sum_loss_rt << "%"
              << std::endl;
}

// ============================================================================
//                       诊断打印 —— LIVE 仪表盘
// ============================================================================

/**
 * @brief 清屏并打印仪表盘头，统一两路输出的浮点格式。
 */
static void Render_Dashboard_Header()
{
    std::cout << "\033[2J\033[H";       // 清屏并回到左上角
    std::cout << "[LIVE-DASHBOARD] (refresh " << kLiveDashboardPeriodMs << " ms)\n";
    std::cout << std::fixed << std::setprecision(3);
    if (g_var_data_stream.is_open())
        g_var_data_stream << std::fixed << std::setprecision(3);
}

/**
 * @brief 打印遥控/底盘目标量/底盘当前态（仅终端，方便排障）。
 */
static void Render_Dashboard_Remote()
{
    std::cout << "\n[REMOTE]\n"
              << "  chassis_ctrl=" << static_cast<int>(robot.Chassis.Get_Chassis_Control_Type())
              << " key=0x" << std::hex << robot.Get_Debug_Remote_Key_Code() << std::dec
              << " recent="    << static_cast<int>(robot.Get_Debug_Remote_Is_Recent())
              << " enabled="   << static_cast<int>(robot.Get_Debug_Remote_Is_Enabled())
              << " recv_vx="   << robot.Get_Debug_Remote_Vx()
              << " recv_vy="   << robot.Get_Debug_Remote_Vy()
              << " recv_omega="<< robot.Get_Debug_Remote_Omega()
              << " target_vx=" << robot.Chassis.Get_Target_Velocity_X()
              << " target_vy=" << robot.Chassis.Get_Target_Velocity_Y()
              << " target_omega=" << robot.Chassis.Get_Target_Omega()
              << " now_vx="    << robot.Chassis.Get_Now_Velocity_X()
              << " now_vy="    << robot.Chassis.Get_Now_Velocity_Y()
              << " now_omega=" << robot.Chassis.Get_Now_Omega()
              << "\n";
}

/**
 * @brief 打印 4 个 DM 舵向电机状态（终端 + 文件，文件略掉 rx/status）。
 */
static void Render_Dashboard_DM()
{
    std::cout << "\n[DM]\n";
    if (g_var_data_stream.is_open())
        g_var_data_stream << "\n[DM]\n";

    for (int i = 0; i < kChannelCount; ++i)
    {
        auto &dm = robot.Chassis.Motor_Steer[i];
        const float steer_rad = Math_Modulus_Normalization(dm.Get_Now_Radian() / REDUCTION_RATIO, 2.0f * PI);
        const float steer_deg = steer_rad * 180.0f / 3.14159265358979323846f;

        std::cout << "  M" << i
                  << " rx=0x"      << std::hex << dm.DM_CAN_Rx_ID
                  << " tx=0x"      << dm.DM_CAN_Tx_ID << std::dec
                  << " motor_rad=" << dm.Get_Now_Radian()
                  << " steer_rad=" << steer_rad
                  << " steer_deg=" << steer_deg
                  << " omega="     << dm.Get_Now_Omega()
                  << " torque="    << dm.Get_Now_Torque()
                  << " status="    << static_cast<int>(dm.Get_Now_Control_Status())
                  << "\n";

        if (g_var_data_stream.is_open())
        {
            g_var_data_stream << "  M" << i
                              << " tx=0x"      << std::hex << dm.DM_CAN_Tx_ID << std::dec
                              << " motor_rad=" << dm.Get_Now_Radian()
                              << " steer_rad=" << steer_rad
                              << " steer_deg=" << steer_deg
                              << " omega="     << dm.Get_Now_Omega()
                              << " torque="    << dm.Get_Now_Torque()
                              << " \n";
        }
    }
}

/**
 * @brief 打印 4 个舵向编码器状态，含掉电恢复关键字段（终端 + 文件，文件略掉 status）。
 */
static void Render_Dashboard_Encoders()
{
    std::cout << "\n[ENC]\n";
    if (g_var_data_stream.is_open())
        g_var_data_stream << "\n[ENC]\n";

    for (int i = 0; i < kChannelCount; ++i)
    {
        auto &enc = robot.Chassis.Encoder_Steer[i];
        const float wheel_rad_true        = enc.Get_Wheel_Posture_radian_True();
        const float wheel_deg_true        = enc.Get_Wheel_Angle_True();
        const float wheel_deg_true_signed = Wrap_Deg_Signed(wheel_deg_true);
        const int   restore_ok = enc.Is_Position_Restore_Ok()       ? 1 : 0;
        const int   degraded   = enc.Is_Degraded_Mode()             ? 1 : 0;
        const int   anchor_ok  = enc.Is_Logical_Zero_Anchor_Valid() ? 1 : 0;
        const int64_t total_pulses = enc.Get_Total_Unwrapped_Pulses();
        const int64_t anchor       = enc.Get_Logical_Zero_Anchor();

        std::cout << "  E" << i
                  << " id=0x"             << std::hex << static_cast<int>(enc.Get_Can_ID()) << std::dec
                  << " raw_abs="          << enc.Get_EncoderValue()
                  << " raw_true_cal="     << enc.Get_Raw_True_Cal()
                  << " wheel_deg_true="   << wheel_deg_true
                  << " deg_true_signed="  << wheel_deg_true_signed
                  << " wheel_rad_true="   << wheel_rad_true
                  << " zero_offset_true=" << enc.Get_Zero_Offset()
                  << " omega_rpm="        << enc.Get_AngularVelocity()
                  << " status="           << static_cast<int>(enc.Get_Status())
                  << " restore_ok="       << restore_ok
                  << " degraded="         << degraded
                  << " anchor_ok="        << anchor_ok
                  << " L="                << total_pulses
                  << " L0="               << anchor
                  << "\n";

        if (g_var_data_stream.is_open())
        {
            g_var_data_stream << "  E" << i
                              << " id=0x"             << std::hex << static_cast<int>(enc.Get_Can_ID()) << std::dec
                              << " raw_abs="          << enc.Get_EncoderValue()
                              << " raw_true_cal="     << enc.Get_Raw_True_Cal()
                              << " wheel_deg_true="   << wheel_deg_true
                              << " deg_true_signed="  << wheel_deg_true_signed
                              << " wheel_rad_true="   << wheel_rad_true
                              << " zero_offset_true=" << enc.Get_Zero_Offset()
                              << " omega_rpm="        << enc.Get_AngularVelocity()
                              << " restore_ok="       << restore_ok
                              << " degraded="         << degraded
                              << " anchor_ok="        << anchor_ok
                              << " L="                << total_pulses
                              << " L0="               << anchor
                              << " \n";
        }
    }
}

/**
 * @brief 打印 4 个 ODrive 驱动轮状态（终端 + 文件，文件略掉 connected）。
 */
static void Render_Dashboard_ODrives()
{
    std::cout << "\n[ODRIVE]\n";
    if (g_var_data_stream.is_open())
        g_var_data_stream << "\n[ODRIVE]\n";

    for (int i = 0; i < kChannelCount; ++i)
    {
        auto &od = robot.Chassis.ODrive_Motor_Steer[i];

        const float omega_i = od.Get_Omega();
        const float vel_i   = omega_i * kWheelRadius;

        std::cout << "  O" << i
                  << " node=0x"     << std::hex << static_cast<int>(od.Get_node_id()) << std::dec
                  << " pos="        << od.Get_Position()
                  << " omega="      << omega_i
                  << " vel_m/s="    << vel_i
                  << " busV="       << od.Get_Bus_Voltage()
                  << " axis_state=" << static_cast<int>(od.Get_Axis_State())
                  << " axis_err=0x" << std::hex << static_cast<uint32_t>(od.Get_Axis_Error()) << std::dec
                  << " connected="  << static_cast<int>(od.Is_Connected())
                  << "\n";

        if (g_var_data_stream.is_open())
        {
            g_var_data_stream << "  O" << i
                              << " node=0x"     << std::hex << static_cast<int>(od.Get_node_id()) << std::dec
                              << " pos="        << od.Get_Position()
                              << " omega="      << omega_i
                              << " vel_m/s="    << vel_i
                              << " busV="       << od.Get_Bus_Voltage()
                              << " axis_state=" << static_cast<int>(od.Get_Axis_State())
                              << " axis_err=0x" << std::hex << static_cast<uint32_t>(od.Get_Axis_Error()) << std::dec
                              << " \n";
        }
    }
}

/**
 * @brief 将整车遥测写入 CSV 日志（用于 ODrive 摩擦/惯量参数辨识）。
 *
 *  每行字段（27 列）：
 *    tick_ms,
 *    cx_tgt, cy_tgt, com_tgt,                        ← 底盘目标 vx/vy/ω（标记事件起止）
 *    cx_now, cy_now, com_now,                        ← 底盘实测 vx/vy/ω（来自 Self_Resolution）
 *    O[i]_omega, O[i]_omega_tgt, O[i]_iq, O[i]_iq_set, O[i]_pos, O[i]_state   (i=0..3)
 *
 *  单位：vx/vy = m/s, omega = rad/s, iq = A, pos = rad, state = int.
 *  力矩可在后处理算：T_motor = Kt * iq_meas，Kt = 0.0827 Nm/A (Makerbase ODrive mini)。
 *
 *  辨识模型：T_motor = J·α + B·ω + Tc·sign(ω)  (+ Ts·sign(α) for breakaway)
 *  其中 α = dω/dt 在后处理由 omega 数值差分得到。
 */
static void Log_ODrive_Velocity(uint32_t tick)
{
    if (!g_velocity_log_inited)
    {
        const char *path = std::getenv("VELOCITY_LOG_FILE");
        if (path == nullptr || path[0] == '\0')
            path = kDefaultVelocityLogFile;

        g_velocity_log_stream.open(path, std::ios::app);
        g_velocity_log_inited = true;

        if (!g_velocity_log_stream.is_open())
        {
            std::cerr << "[WARN] Failed to open velocity log: " << path << std::endl;
            return;
        }

        g_velocity_log_stream << std::fixed << std::setprecision(4);
        g_velocity_log_stream << "tick_ms"
                              << ",cx_tgt,cy_tgt,com_tgt,cx_now,cy_now,com_now";
        for (int i = 0; i < kChannelCount; ++i)
        {
            g_velocity_log_stream << ",O" << i << "_omega"
                                  << ",O" << i << "_omega_tgt"
                                  << ",O" << i << "_iq"
                                  << ",O" << i << "_iq_set"
                                  << ",O" << i << "_pos"
                                  << ",O" << i << "_state";
        }
        g_velocity_log_stream << "\n";
    }

    if (!g_velocity_log_stream.is_open())
        return;

    g_velocity_log_stream << tick
                          << "," << robot.Chassis.Get_Target_Velocity_X()
                          << "," << robot.Chassis.Get_Target_Velocity_Y()
                          << "," << robot.Chassis.Get_Target_Omega()
                          << "," << robot.Chassis.Get_Now_Velocity_X()
                          << "," << robot.Chassis.Get_Now_Velocity_Y()
                          << "," << robot.Chassis.Get_Now_Omega();
    for (int i = 0; i < kChannelCount; ++i)
    {
        auto &od = robot.Chassis.ODrive_Motor_Steer[i];
        g_velocity_log_stream << "," << od.Get_Omega()
                              << "," << od.Get_Target_Omega()
                              << "," << od.Get_IQ_Measured()
                              << "," << od.Get_IQ_Setpoint()
                              << "," << od.Get_Position()
                              << "," << static_cast<int>(od.Get_Axis_State());
    }
    g_velocity_log_stream << "\n";

    if ((tick % 100) == 0)
        g_velocity_log_stream.flush();
}

/**
 * @brief 打印 OPS-9 状态（终端 + 文件）：连接状态 / 位姿 / 角速度 / 帧统计。
 *        停车排障时主要看 status 是否 ENABLE、rx_frames 是否在持续增长。
 */
static void Render_Dashboard_OPS()
{
    auto &ops = robot.Chassis.OPS;
    const char *status_str = (ops.Get_Status() == OPS_Status_ENABLE) ? "ENABLE " : "DISABLE";
    const auto data = ops.Get_Data();

    std::cout << "\n[OPS]\n"
              << "  status="   << status_str
              << " yaw_deg="   << data.yaw_deg
              << " pitch_deg=" << data.pitch_deg
              << " roll_deg="  << data.roll_deg
              << " pos_x_mm="  << data.pos_x_mm
              << " pos_y_mm="  << data.pos_y_mm
              << " yaw_rate_dps=" << data.yaw_rate_dps
              << " rx_frames=" << ops.Get_Rx_Frame_Count()
              << " resync="    << ops.Get_Resync_Count()
              << " tail_err="  << ops.Get_Tail_Mismatch_Count()
              << "\n";

    if (g_var_data_stream.is_open())
    {
        g_var_data_stream << "\n[OPS]\n"
                          << "  status="   << status_str
                          << " yaw_deg="   << data.yaw_deg
                          << " pitch_deg=" << data.pitch_deg
                          << " roll_deg="  << data.roll_deg
                          << " pos_x_mm="  << data.pos_x_mm
                          << " pos_y_mm="  << data.pos_y_mm
                          << " yaw_rate_dps=" << data.yaw_rate_dps
                          << " rx_frames=" << ops.Get_Rx_Frame_Count()
                          << " resync="    << ops.Get_Resync_Count()
                          << " tail_err="  << ops.Get_Tail_Mismatch_Count()
                          << " \n";
    }
}

/**
 * @brief 渲染整张 LIVE 仪表盘并刷出两路流。
 */
static void Print_Live_Dashboard()
{
    Ensure_Var_Data_Stream();
    Render_Dashboard_Header();
    Render_Dashboard_Remote();
    Render_Dashboard_DM();
    Render_Dashboard_Encoders();
    Render_Dashboard_ODrives();
    Render_Dashboard_OPS();

    std::cout.flush();
    if (g_var_data_stream.is_open())
        g_var_data_stream.flush();
}

// ============================================================================
//                                初始化
// ============================================================================

/**
 * @brief 完成 EtherCAT 主站 + LinkX 适配器初始化，并把网络带入 OP 状态。
 *
 *  slave_id 绑定策略:
 *    优先按 Station Alias 绑（alias=1 -> FD, alias=2 -> classic）;
 *    若 alias 未生效，按当前实测物理拓扑回退:
 *        PC -> slave1(FD,挂夹爪) -> slave2(经典,挂底盘 DM/ODrive/Encoder/OPS)
 *        即 fd_slave_id=1, classic_slave_id=2
 *
 * @return true 成功；false 表示主站启动失败，调用方应直接返回。
 */
static bool Init_Ethercat_And_Linkx(const char *ifname)
{
    constexpr uint16_t kAliasFd      = 1;   ///< FD 那片(挂夹爪)
    constexpr uint16_t kAliasClassic = 2;   ///< 经典那片(挂底盘电机/编码器)

    if (!ecat_master_init(&master, ifname))
        return false;

    if (master.ctx.slavecount < 2)
    {
        std::cerr << "[TASK] FATAL: expected >=2 EtherCAT slaves (LinkX classic + LinkX FD), found "
                  << master.ctx.slavecount
                  << ". Check downstream LinkX cabling (old.OUT -> new.IN)." << std::endl;
        return false;
    }

    // 按 alias 绑 slave_id
    int classic_slave_id = 0;
    int fd_slave_id      = 0;
    for (int i = 1; i <= master.ctx.slavecount; ++i)
    {
        uint16_t alias = master.ctx.slavelist[i].aliasadr;
        if      (alias == kAliasFd)      fd_slave_id      = i;
        else if (alias == kAliasClassic) classic_slave_id = i;
    }

    if (fd_slave_id > 0 && classic_slave_id > 0)
    {
        std::cout << "[ECAT] alias-based binding: classic=slave " << classic_slave_id
                  << " (alias=" << kAliasClassic << "), fd=slave " << fd_slave_id
                  << " (alias=" << kAliasFd << ")" << std::endl;
    }
    else
    {
        std::cerr << "[ECAT][WARN] Station Alias not properly set "
                  << "(fd_slave_id=" << fd_slave_id
                  << " classic_slave_id=" << classic_slave_id
                  << "). Falling back to fixed topology: classic=2, fd=1." << std::endl;
        classic_slave_id = 2;
        fd_slave_id      = 1;
    }

    // 经典 CAN 1Mbps（DM/ODrive/Encoder/OPS 都跑这里）
    linkx_init(&linkx_dev, (uint32_t)classic_slave_id, &master.ctx);
    linkx_hw_wakeup(&linkx_dev);
    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_set_can_baudrate(&linkx_dev, ch, 0, 2, 31, 8, 8, 1, 31, 8, 8);

    // 第二片 LinkX:ch0 + ch1 经典 CAN 1M(ch0 挂夹爪 DM,ch1 透传 0x20 帧);ch2-3 保留 CAN-FD(仲裁 1M / 数据 5M)
    linkx_init(&linkx_dev_fd, (uint32_t)fd_slave_id, &master.ctx);
    linkx_hw_wakeup(&linkx_dev_fd);
    linkx_set_can_baudrate(&linkx_dev_fd, 0, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    linkx_set_can_baudrate(&linkx_dev_fd, 1, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    for (int ch = 2; ch < kChannelCount; ++ch)
        linkx_set_can_baudrate(&linkx_dev_fd, ch, 1, 2, 31, 8, 8, 1, 6, 1, 1);

    return ecat_master_bring_online(&master);
}

/**
 * @brief 启动 Robot 业务层（含 ROS2 桥接），不阻塞返回。
 */
static void Init_Robot_Logic()
{
    robot.Init(&linkx_dev, &linkx_dev_fd);
    robot.Start_ROS2_Remote_Bridge();
    std::cout << "[TASK] Robot Logic Initialized." << std::endl;
}

// ============================================================================
//                       一次性流程：舵向零点抓取
// ============================================================================

/**
 * @brief 解析 CAPTURE_STEER_ZERO / CAPTURE_STEER_ZERO_FORCE 环境变量并初始化抓零状态。
 *        当零点文件已存在且未强制覆盖时，安全跳过抓零流程。
 */
static void Init_Steer_Zero_Capture_If_Requested(Steer_Zero_Capture_State &state)
{
    const char *env = std::getenv("CAPTURE_STEER_ZERO");
    if (env == nullptr || env[0] != '1')
        return;

    const std::string zero_path = Class_Chassis::Default_Steer_Zero_Offsets_Path();
    const bool zero_file_exists = static_cast<bool>(std::ifstream(zero_path));
    const char *force_env = std::getenv("CAPTURE_STEER_ZERO_FORCE");
    const bool force_capture = (force_env != nullptr && force_env[0] == '1');

    if (zero_file_exists && !force_capture)
    {
        std::cerr << "[TASK][SAFE-GUARD] CAPTURE_STEER_ZERO=1 ignored because zero file already exists: '"
                  << zero_path << "'\n"
                  << "  This prevents accidental overwrite of calibrated steer zero.\n"
                  << "  If you really want to recalibrate now, set CAPTURE_STEER_ZERO_FORCE=1 and restart."
                  << std::endl;
        return;
    }

    state.pending = true;
    std::cout << "[TASK] CAPTURE_STEER_ZERO=1: will save steer zero offsets "
                 "after each encoder delivers >= "
              << kCalibrateRequiredFreshFrames
              << " new frames since startup."
              << (force_capture ? " (force overwrite enabled)" : "")
              << std::endl;
}

/**
 * @brief 推进零点抓取状态机：
 *           1) 等所有编码器拿到首个有效姿态后，固定基线 rx_count；
 *           2) 等每个编码器都新收到 N 帧后做 zero capture，
 *              并立即把当前 L 落入 unwrapped 文件，避免下次启动 anchor 与 L 不一致。
 *           成功一次即关闭 pending，不再重试。
 */
static void Step_Steer_Zero_Capture(Steer_Zero_Capture_State &state)
{
    if (!state.pending)
        return;

    // 1) 等所有编码器至少收到过一帧，再设基线
    if (!state.baseline_set)
    {
        for (int i = 0; i < kChannelCount; ++i)
            if (!robot.Chassis.Encoder_Steer[i].Has_Valid_Wheel_Posture())
                return;

        for (int i = 0; i < kChannelCount; ++i)
            state.baseline_rx[i] = robot.Chassis.Encoder_Steer[i].Get_Rx_Count();
        state.baseline_set = true;

        std::cout << "[TASK] capture baseline set, waiting for "
                  << kCalibrateRequiredFreshFrames
                  << " more frames per encoder..." << std::endl;
    }

    // 2) 基线之后，要求每个 rx_count 至少递增 N 才算「新鲜」
    for (int i = 0; i < kChannelCount; ++i)
    {
        const uint32_t now_rx = robot.Chassis.Encoder_Steer[i].Get_Rx_Count();
        if (now_rx < state.baseline_rx[i] + kCalibrateRequiredFreshFrames)
            return;
    }

    if (robot.Chassis.Capture_And_Save_Steer_Zero_Offsets(
            Class_Chassis::Default_Steer_Zero_Offsets_Path()))
    {
        std::cout << "[TASK] steer zero calibration captured. "
                     "Restart without CAPTURE_STEER_ZERO to use it." << std::endl;
        robot.Chassis.Force_Save_Steer_Unwrapped_Pulses(
            Class_Chassis::Default_Steer_Unwrapped_Pulses_Path(),
            "after_zero_capture");
    }
    state.pending = false;
}

// ============================================================================
//                  一次性流程：上电断电位移异常检查
// ============================================================================

/**
 * @brief 等待所有舵向编码器完成断电恢复后，一次性扫描是否有断电位移超阈值；
 *        若发现，锁定底盘并禁用底盘控制。done=true 后不再调用。
 */
static void Step_Poweroff_Encoder_Anomaly_Check(bool &done)
{
    if (done)
        return;

    for (int i = 0; i < kChannelCount; ++i)
        if (!robot.Chassis.Encoder_Steer[i].Is_Restore_Valid())
            return;

    bool any_abnormal = false;
    for (int i = 0; i < kChannelCount; ++i)
    {
        if (robot.Chassis.Encoder_Steer[i].Is_Degraded_Mode())
        {
            any_abnormal = true;
            std::cerr << "[TASK][!! 异常 !!] 编码器 id=0x" << std::hex
                      << static_cast<int>(robot.Chassis.Encoder_Steer[i].Get_Can_ID())
                      << std::dec << " 断电位移超阈值，舵向已锁定" << std::endl;
        }
    }
    if (any_abnormal)
    {
        robot.Chassis.Set_Steer_Abnormal_Lock();
        robot.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        std::cerr << "\n"
                  << "================================================================\n"
                  << "[TASK][!! 舵向编码器断电位移异常 !!]\n"
                  << "  至少一个编码器断电期间位移超过10圈，系统已锁定舵向。\n"
                  << "  请执行重新设置零点（Homing）后再运行：\n"
                  << "    export CAPTURE_STEER_ZERO=1\n"
                  << "================================================================\n"
                  << std::endl;
    }
    done = true;
}

// ============================================================================
//                            主循环每帧子步骤
// ============================================================================

/**
 * @brief 抽干两片 LinkX 共 8 个 CAN 通道的接收队列，逐帧分发给 Robot。
 *        module_id=0 -> 经典 LinkX (slave_id=1)；module_id=1 -> CAN-FD LinkX (slave_id=2)
 */
static void Pump_CAN_Receive()
{
    can_msg_t msg;
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
        while (linkx_quick_recv(&linkx_dev, ch, &msg))
            robot.CAN_Rx_Callback(0, ch, msg.id, msg.data, msg.dlen);
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
        while (linkx_quick_recv(&linkx_dev_fd, ch, &msg))
            robot.CAN_Rx_Callback(1, ch, msg.id, msg.data, msg.dlen);
}

/**
 * @brief 按 1 ms / 2 ms / 100 ms 节拍触发 Robot 周期任务。
 */
static void Dispatch_Robot_Tick(uint32_t tick)
{
    robot.TIM_1ms_Calculate_Callback();
    if ((tick % kTaskPeriod2Ms)   == 0)
        robot.TIM_2ms_Calculate_PeriodElapsedCallback();
    if ((tick % kTaskPeriod100Ms) == 0)
        robot.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 收到 Ctrl+C 等关停信号后，把所有电机失能再退出。
 *
 *  对象（共 10 个执行器）：
 *    - 4× DM 舵向电机   (linkx_dev   ch0, ID 0x01-0x04)  → CAN_Send_Exit (FF..FD)
 *    - 4× ODrive 驱动轮 (linkx_dev   ch1, node 0x10/18/20/28) → Emergency_Stop (轴 -> IDLE)
 *    - 2× 夹爪 DM       (linkx_dev_fd ch0, ID 0x01/0x02) → CAN_Send_Exit
 *
 *  每个 tick 重新入队一次失能帧；linkx_send_pdos 每 ch 只 pop 1 帧，
 *  所以共跑 kDisableSendCycles ≈ 50ms，确保多 ID 全部出栈到总线，
 *  且 Makerbase ODrive 也能拿到多次重发（memory: 反复重发 Emergency_Stop）。
 *  期间继续 ecat_master_sync + linkx_recv_pdos，保持 EtherCAT 链路活动。
 */
static void Disable_All_Devices()
{
    constexpr int kDisableSendCycles = 50;   // 50 × 1ms ≈ 50ms

    std::cout << "\n[TASK] Disabling all devices (Ctrl+C safety)..." << std::endl;

    for (int cycle = 0; cycle < kDisableSendCycles; ++cycle)
    {
        // 保持 EtherCAT/PDO 活动，顺带把回包消化掉避免 RX 队列堵塞
        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev);
        linkx_recv_pdos(&linkx_dev_fd);
        Pump_CAN_Receive();

        // 4 个 DM 舵向电机失能
        for (int i = 0; i < kChannelCount; ++i)
            robot.Chassis.Motor_Steer[i].CAN_Send_Exit();

        // 4 个 ODrive 驱动轮 -> IDLE
        for (int i = 0; i < kChannelCount; ++i)
            robot.Chassis.ODrive_Motor_Steer[i].Emergency_Stop();

        // 2 个夹爪 DM 电机失能
        robot.Clamp.Motor_Pitch_Large.CAN_Send_Exit();
        robot.Clamp.Motor_Pitch_Small.CAN_Send_Exit();

        // 把入队的失能帧推到总线（每 ch 出 1 帧）
        linkx_send_pdos(&linkx_dev);
        linkx_send_pdos(&linkx_dev_fd);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "[TASK] All devices disabled." << std::endl;
}

/**
 * @brief CAN 统计 / LIVE 仪表盘 / 多圈累计持久化的周期触发。
 *        每个动作各自带开关与节拍，互不影响。
 */
static void Dispatch_Periodic_Diagnostics(uint32_t tick)
{
    if (kEnableCanStatPrint && (tick % kCanStatPrintPeriodMs) == 0 && tick != 0)
    {
        std::cout << "\n[CAN-STATS] === LinkX classic (slave_id=1) ===";
        Print_CAN_Stats(&linkx_dev);
        std::cout << "\n[CAN-STATS] === LinkX FD      (slave_id=2) ===";
        Print_CAN_Stats(&linkx_dev_fd);
    }

    if ((tick % kVelocityLogPeriodMs) == 0)
        Log_ODrive_Velocity(tick);

    if (kEnableLiveDashboard && (tick % kLiveDashboardPeriodMs) == 0)
        Print_Live_Dashboard();

    // [OPS-PROBE] 1Hz 状态探针：dashboard 关闭时也能看到 OPS 是否在收帧
    if ((tick % 1000U) == 0U)
    {
        auto &ops = robot.Chassis.OPS;
        std::cout << "[OPS] status="
                  << (ops.Get_Status() == OPS_Status_ENABLE ? "ENABLE " : "DISABLE")
                  << " rx_frames=" << ops.Get_Rx_Frame_Count()
                  << " resync="    << ops.Get_Resync_Count()
                  << " yaw="       << ops.Get_Yaw_Deg()
                  << " x="         << ops.Get_Pos_X_Mm()
                  << " y="         << ops.Get_Pos_Y_Mm()
                  << std::endl;
    }

    if (kEnableSteerUnwrappedSave && (tick % kSteerUnwrappedSavePeriodMs) == 0)
        robot.Chassis.Save_Steer_Unwrapped_Pulses(
            Class_Chassis::Default_Steer_Unwrapped_Pulses_Path());
}

// ============================================================================
//                                  主入口
// ============================================================================

/**
 * @brief 机器人主控制阻塞循环：
 *           初始化 → 1 kHz 主循环（CAN 收 → Robot 节拍 → CAN 发 → 诊断/校准）
 *           → 收到关闭信号后强制持久化 → 关闭 ROS 桥。
 *
 * @param ifname EtherCAT 网卡名（如 "enp86s0"）。
 */
void Robot_Control_Loop(const char *ifname)
{
    if (!Init_Ethercat_And_Linkx(ifname))
        return;
    Init_Robot_Logic();

    Steer_Zero_Capture_State zero_capture;
    Init_Steer_Zero_Capture_If_Requested(zero_capture);

    bool poweroff_check_done = false;
    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;

    while (master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kControlLoopPeriodMs);

        // ---- 进站：EtherCAT 同步 + CAN 接收分发 ----
        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev);
        linkx_recv_pdos(&linkx_dev_fd);
        Pump_CAN_Receive();

        // ---- 计算：Robot 周期任务 ----
        Dispatch_Robot_Tick(tick);

        // ---- 出站：提交 CAN 发送 ----
        linkx_send_pdos(&linkx_dev);
        linkx_send_pdos(&linkx_dev_fd);

        // ---- 旁路：诊断 / 持久化 / 一次性校验 ----
        Dispatch_Periodic_Diagnostics(tick);
        Step_Poweroff_Encoder_Anomaly_Check(poweroff_check_done);
        Step_Steer_Zero_Capture(zero_capture);

        ++tick;
        std::this_thread::sleep_until(next_wakeup);
    }

    // 退出钩子：先把所有电机失能（安全），再落 L、关 ROS 桥
    Disable_All_Devices();
    robot.Chassis.Force_Save_Steer_Unwrapped_Pulses(
        Class_Chassis::Default_Steer_Unwrapped_Pulses_Path(),
        "before_exit");
    robot.Stop_ROS2_Remote_Bridge();
}
