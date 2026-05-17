/**
 * @file  ops_test_main.cpp
 * @brief OPS-9 接收解码 + 命令应答冒烟测试
 *
 *  默认模式：仅观察接收速率与解码值（10s 后退出）
 *  --zero  ：t=3s 发 ACT0，对比 zero 前后的 x/y/yaw
 *  --yaw N ：t=3s 发 ACTJ + N(deg)，对比 yaw 前后变化
 *  --x   N ：t=3s 发 ACTX + N(mm)
 *  --y   N ：t=3s 发 ACTY + N(mm)
 *
 *  期望硬件链路：
 *      OPS  ──RS232 115200 8N1──►  泥人转换器 (MODE0 透传, CAN ID = 0x01)
 *           ──CAN 1Mbps──►          EtherCAT-4C 通道 3
 *
 *  用法：
 *      sudo ./build/linkx_soem_demo/ops_test enp86s0
 *      sudo ./build/linkx_soem_demo/ops_test enp86s0 --zero
 *      sudo ./build/linkx_soem_demo/ops_test enp86s0 --yaw 90
 */

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "dvc_ops.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"

namespace
{
volatile std::sig_atomic_t g_running = 1;

// SIGINT/SIGTERM 处理器：清掉运行标志让主循环正常退出
void signal_handler(int)
{
    g_running = 0;
}

enum CmdMode
{
    CMD_NONE = 0,
    CMD_ZERO,
    CMD_UPDATE_YAW,
    CMD_UPDATE_X,
    CMD_UPDATE_Y,
};

struct CliArgs
{
    const char *ifname = "enp86s0";
    CmdMode    cmd     = CMD_NONE;
    float      cmd_arg = 0.0f;
    int        run_seconds = 10;  ///< 没命令时跑 10s；有命令时跑 8s（基线 3s + 观察 5s）
};

// 解析命令行：识别 --zero / --yaw N / --x N / --y N，其余非 '-' 开头参数当作网卡名
// 带命令时把运行时长缩到 8s（基线 3s + 观察 5s），无命令时默认 10s
CliArgs parse_args(int argc, char *argv[])
{
    CliArgs a;
    for (int i = 1; i < argc; ++i)
    {
        const std::string s = argv[i];
        if      (s == "--zero")                           { a.cmd = CMD_ZERO; }
        else if (s == "--yaw" && i + 1 < argc)            { a.cmd = CMD_UPDATE_YAW; a.cmd_arg = std::strtof(argv[++i], nullptr); }
        else if (s == "--x"   && i + 1 < argc)            { a.cmd = CMD_UPDATE_X;   a.cmd_arg = std::strtof(argv[++i], nullptr); }
        else if (s == "--y"   && i + 1 < argc)            { a.cmd = CMD_UPDATE_Y;   a.cmd_arg = std::strtof(argv[++i], nullptr); }
        else if (s.size() > 0 && s[0] != '-')             { a.ifname = argv[i]; }
    }
    if (a.cmd != CMD_NONE) a.run_seconds = 8;
    return a;
}

// CmdMode 枚举转可读字符串，仅用于日志打印
const char *cmd_name(CmdMode c)
{
    switch (c)
    {
    case CMD_ZERO:        return "ACT0  (zero)";
    case CMD_UPDATE_YAW:  return "ACTJ  (update yaw)";
    case CMD_UPDATE_X:    return "ACTX  (update x)";
    case CMD_UPDATE_Y:    return "ACTY  (update y)";
    default:              return "(none)";
    }
}
} // namespace

// OPS-9 接收解码 + 命令应答冒烟测试入口
// 流程：初始化 EtherCAT/LinkX → 4 通道 1Mbps CAN → 启动 1ms 主循环抽取 ch3 帧并喂给 OPS
//      → t=3s 时按 --cmd 发 ACT0/ACTJ/ACTX/ACTY，1.5s 后采样并打印 baseline/after/Δ
//      → 1Hz 打印解码速率与 yaw/x/y 实时值；到时长上限或 SIGINT 退出
int main(int argc, char *argv[])
{
    const CliArgs args = parse_args(argc, argv);

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "[OPS-TEST] ifname=" << args.ifname
              << "  cmd=" << cmd_name(args.cmd)
              << "  arg=" << args.cmd_arg
              << "  duration=" << args.run_seconds << "s" << std::endl;
    std::cout << "[OPS-TEST] expecting OPS-9 on channel 3, CAN_ID=0x01, 1Mbps classic CAN" << std::endl;

    /* ---- EtherCAT + LinkX 初始化 ---- */
    ecat_master_t master {};
    if (!ecat_master_init(&master, args.ifname))
    {
        std::cerr << "[OPS-TEST] ecat_master_init failed (need sudo & correct ifname)." << std::endl;
        return 1;
    }

    linkx_t linkx_dev {};
    linkx_init(&linkx_dev, 1, &master.ctx);
    linkx_hw_wakeup(&linkx_dev);

    for (int ch = 0; ch < 4; ++ch)
        linkx_set_can_baudrate(&linkx_dev, ch, 0, 2, 31, 8, 8, 1, 31, 8, 8);

    if (!ecat_master_bring_online(&master))
    {
        std::cerr << "[OPS-TEST] ecat_master_bring_online failed." << std::endl;
        return 2;
    }

    /* ---- OPS 实例 ---- */
    Class_OPS ops;
    ops.Init(&linkx_dev, /*can_channel=*/3, /*can_id=*/0x01);

    /* ---- 主循环 ---- */
    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;
    uint32_t ch3_raw_can_frames = 0;
    uint32_t prev_decoded_frames = 0;

    bool cmd_sent = false;
    Struct_OPS_Data baseline = {};   ///< 命令发送前 200ms 的最后一帧
    Struct_OPS_Data after    = {};   ///< 命令发送后采到的稳定帧

    /* 命令发出时间（ms）：基线运行 3s 后发送 */
    const uint32_t kCmdAtMs = 3000;

    while (g_running && master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(1);

        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev);

        /* 抽干通道 3 接收队列 */
        can_msg_t msg;
        while (linkx_quick_recv(&linkx_dev, /*ch=*/3, &msg))
        {
            ++ch3_raw_can_frames;
            ops.CAN_RxCpltCallback(msg.data, msg.dlen);
        }

        /* 100ms 节拍：刷新存活检测 */
        if ((tick % 100U) == 0U)
            ops.TIM_Alive_PeriodElapsedCallback();

        /* 命令发送时机：t=kCmdAtMs，且发送前抓一次 baseline */
        if (!cmd_sent && args.cmd != CMD_NONE && tick == kCmdAtMs)
        {
            baseline = ops.Get_Data();
            std::cout << "\n[OPS-TEST] === BASELINE (just before cmd) ===" << std::endl;
            std::cout << std::fixed << std::setprecision(3)
                      << "  yaw=" << baseline.yaw_deg
                      << "  x="   << baseline.pos_x_mm
                      << "  y="   << baseline.pos_y_mm << std::endl;

            std::cout << "[OPS-TEST] === SEND CMD: " << cmd_name(args.cmd) << " arg=" << args.cmd_arg << " ===" << std::endl;
            switch (args.cmd)
            {
            case CMD_ZERO:        ops.Send_Zero(); break;
            case CMD_UPDATE_YAW:  ops.Send_Update_Yaw(args.cmd_arg); break;
            case CMD_UPDATE_X:    ops.Send_Update_X(args.cmd_arg);   break;
            case CMD_UPDATE_Y:    ops.Send_Update_Y(args.cmd_arg);   break;
            default: break;
            }
            cmd_sent = true;
        }

        /* 命令发送 1.5s 后稳定采样一次 after */
        if (cmd_sent && tick == kCmdAtMs + 1500)
        {
            after = ops.Get_Data();
            std::cout << "\n[OPS-TEST] === AFTER CMD (1.5s later) ===" << std::endl;
            std::cout << std::fixed << std::setprecision(3)
                      << "  yaw=" << after.yaw_deg
                      << "  x="   << after.pos_x_mm
                      << "  y="   << after.pos_y_mm << std::endl;

            std::cout << "[OPS-TEST] === DELTA ===" << std::endl;
            std::cout << "  yaw: " << baseline.yaw_deg << "  →  " << after.yaw_deg
                      << "  (Δ=" << (after.yaw_deg - baseline.yaw_deg) << ")\n"
                      << "  x:   " << baseline.pos_x_mm << "  →  " << after.pos_x_mm
                      << "  (Δ=" << (after.pos_x_mm - baseline.pos_x_mm) << ")\n"
                      << "  y:   " << baseline.pos_y_mm << "  →  " << after.pos_y_mm
                      << "  (Δ=" << (after.pos_y_mm - baseline.pos_y_mm) << ")"
                      << std::endl;
        }

        /* 1Hz 打印 */
        if (tick != 0 && (tick % 1000U) == 0U)
        {
            const uint32_t decoded = ops.Get_Rx_Frame_Count();
            const uint32_t per_sec = decoded - prev_decoded_frames;
            prev_decoded_frames    = decoded;

            const auto data = ops.Get_Data();
            std::cout << "[OPS] t=" << std::setw(2) << (tick / 1000U) << "s"
                      << " status=" << (ops.Get_Status() == OPS_Status_ENABLE ? "ENABLE " : "DISABLE")
                      << " decoded=" << decoded << " (" << per_sec << " Hz)"
                      << " resync=" << ops.Get_Resync_Count()
                      << " tail_err=" << ops.Get_Tail_Mismatch_Count()
                      << "  yaw=" << std::fixed << std::setprecision(2) << std::setw(7) << data.yaw_deg
                      << "  x=" << std::setw(8) << data.pos_x_mm
                      << "  y=" << std::setw(8) << data.pos_y_mm
                      << std::endl;
        }

        linkx_send_pdos(&linkx_dev);

        ++tick;
        if (static_cast<int>(tick / 1000U) >= args.run_seconds)
            g_running = 0;

        std::this_thread::sleep_until(next_wakeup);
    }

    std::cout << "\n[OPS-TEST] === SHUTDOWN ===" << std::endl;
    std::cout << "[OPS-TEST] decoded=" << ops.Get_Rx_Frame_Count()
              << " resync=" << ops.Get_Resync_Count()
              << " tail_err=" << ops.Get_Tail_Mismatch_Count() << std::endl;
    return 0;
}
