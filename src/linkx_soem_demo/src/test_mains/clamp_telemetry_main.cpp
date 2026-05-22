// =============================================================================
// clamp_telemetry_main.cpp
//
// 夹爪 (Pitch_Large + Pitch_Small DM-J4310) 只读遥测程序.
//
// 设计原则:
//   - 不调 Class_Clamp (避免它走 MIT 控制帧路径).
//   - 直接拿两个 Class_Motor_DM_Normal, 唯一向 CAN 写入的就是 Enter 帧.
//     用户已明确选择 "只发 Enter 帧, 不发控制帧": 反馈帧由 DM 收到 Enter 后回一帧
//     触发, 频率 = Enter 发送频率 (本程序 50Hz / 20ms).
//   - 读 Get_Now_Radian / Get_Now_Omega, UDP 广播到 127.0.0.1:<port>
//     (默认 5005) 给 Python 端实时动画.
//
// 用法:
//   sudo ./clamp_telemetry enp86s0           # 默认 UDP 5005, 50Hz
//   sudo ./clamp_telemetry enp86s0 --port 5006 --rate 100
//
// UDP 包格式 (28B, little-endian, 与 tools/clamp_arm_visualizer.py 对齐):
//   uint32  magic  = 0x434C4D50 ('CLMP')
//   uint32  seq                                // 单调累加
//   float   t_sec                              // 程序启动后秒数
//   float   theta_large_rad                    // Pitch_Large Get_Now_Radian
//   float   theta_small_rad                    // Pitch_Small Get_Now_Radian
//   float   omega_large_rad_s
//   float   omega_small_rad_s
// =============================================================================

#include "dvc_motor_dm.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>


namespace {

constexpr int      kChannelCount   = 4;
constexpr uint16_t kAliasFd        = 1;     // FD LinkX (挂夹爪)
constexpr uint16_t kAliasClassic   = 2;
constexpr uint8_t  kClampChannel   = 0;     // FD-LinkX ch0
constexpr uint32_t kMagic          = 0x434C4D50U;  // 'CLMP'

ecat_master_t master;
linkx_t       linkx_dev_classic;
linkx_t       linkx_dev_fd;
Class_Motor_DM_Normal motor_large;
Class_Motor_DM_Normal motor_small;
std::atomic<bool> g_running{true};

void Signal_Handler(int /*sig*/) {
    g_running.store(false);
    master.is_running = false;
}

// 解析 alias, 退回到固定拓扑 (与 task.cpp Init_Ethercat_And_Linkx 一致)
bool Bring_Up_Ecat_And_Linkx(const char *ifname) {
    if (!ecat_master_init(&master, ifname)) {
        std::cerr << "[FATAL] ecat_master_init failed on " << ifname << std::endl;
        return false;
    }
    if (master.ctx.slavecount < 2) {
        std::cerr << "[FATAL] expected >=2 EtherCAT slaves, found "
                  << master.ctx.slavecount << std::endl;
        return false;
    }

    int classic_slave_id = 0;
    int fd_slave_id      = 0;
    for (int i = 1; i <= master.ctx.slavecount; ++i) {
        uint16_t alias = master.ctx.slavelist[i].aliasadr;
        if      (alias == kAliasFd)      fd_slave_id      = i;
        else if (alias == kAliasClassic) classic_slave_id = i;
    }
    if (fd_slave_id == 0 || classic_slave_id == 0) {
        std::cerr << "[WARN] Station Alias 未设, 退回固定拓扑 fd=1 classic=2" << std::endl;
        fd_slave_id      = 1;
        classic_slave_id = 2;
    }
    std::cout << "[ECAT] classic=slave " << classic_slave_id
              << "  fd=slave " << fd_slave_id << std::endl;

    // 两片都要 init: 单 init 一片在某些底层会判 PDO 排布异常
    linkx_init(&linkx_dev_classic, (uint32_t)classic_slave_id, &master.ctx);
    linkx_hw_wakeup(&linkx_dev_classic);
    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_set_can_baudrate(&linkx_dev_classic, ch, 0, 2, 31, 8, 8, 1, 31, 8, 8);

    linkx_init(&linkx_dev_fd, (uint32_t)fd_slave_id, &master.ctx);
    linkx_hw_wakeup(&linkx_dev_fd);
    linkx_set_can_baudrate(&linkx_dev_fd, 0, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    linkx_set_can_baudrate(&linkx_dev_fd, 1, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    for (int ch = 2; ch < kChannelCount; ++ch)
        linkx_set_can_baudrate(&linkx_dev_fd, ch, 1, 2, 31, 8, 8, 1, 6, 1, 1);

    if (!ecat_master_bring_online(&master)) {
        std::cerr << "[FATAL] ecat_master_bring_online failed" << std::endl;
        return false;
    }
    std::cout << "[ECAT] online" << std::endl;
    return true;
}

#pragma pack(push, 1)
struct UdpPacket {
    uint32_t magic;
    uint32_t seq;
    float    t_sec;
    float    theta_large;
    float    theta_small;
    float    omega_large;
    float    omega_small;
};
#pragma pack(pop)
static_assert(sizeof(UdpPacket) == 28, "UDP packet must be 28 bytes");

int Open_Udp_Socket() {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "[FATAL] socket() failed: " << std::strerror(errno) << std::endl;
        return -1;
    }
    return fd;
}

}  // namespace


int main(int argc, char *argv[]) {
    std::string ifname    = "enp86s0";
    std::string dst_ip    = "127.0.0.1";
    uint16_t    dst_port  = 5005;
    int         rate_hz   = 50;

    // 极简 CLI: <iface> [--port N] [--rate N] [--ip A.B.C.D]
    if (argc > 1) ifname = argv[1];
    for (int i = 2; i + 1 < argc; i += 2) {
        std::string k = argv[i];
        std::string v = argv[i + 1];
        if      (k == "--port") dst_port = static_cast<uint16_t>(std::stoi(v));
        else if (k == "--rate") rate_hz  = std::stoi(v);
        else if (k == "--ip")   dst_ip   = v;
    }
    if (rate_hz <= 0 || rate_hz > 500) {
        std::cerr << "[FATAL] --rate 必须在 [1, 500]" << std::endl;
        return -1;
    }
    const uint32_t enter_period_ticks = static_cast<uint32_t>(1000 / rate_hz);

    std::signal(SIGINT,  Signal_Handler);
    std::signal(SIGTERM, Signal_Handler);

    std::cout << "================================================\n"
              << "  Clamp Telemetry (read-only, Enter-frame only)\n"
              << "  iface=" << ifname
              << "  udp=" << dst_ip << ":" << dst_port
              << "  rate=" << rate_hz << "Hz\n"
              << "================================================" << std::endl;

    if (!Bring_Up_Ecat_And_Linkx(ifname.c_str())) return -1;

    // 两个 DM 电机 (与 Class_Clamp::Init 完全一致的参数, 但 *不* 用 Class_Clamp)
    motor_large.Init(&linkx_dev_fd, kClampChannel, /*Rx*/0x11, /*Tx*/0x01,
                     Motor_DM_Control_Method_NORMAL_MIT,
                     12.5f, 20.0f, 15.0f, 10.261194f);
    motor_small.Init(&linkx_dev_fd, kClampChannel, /*Rx*/0x12, /*Tx*/0x02,
                     Motor_DM_Control_Method_NORMAL_MIT,
                     12.5f, 20.0f, 15.0f, 10.261194f);

    int sock = Open_Udp_Socket();
    if (sock < 0) return -1;
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(dst_port);
    if (::inet_pton(AF_INET, dst_ip.c_str(), &dst.sin_addr) != 1) {
        std::cerr << "[FATAL] invalid ip: " << dst_ip << std::endl;
        ::close(sock);
        return -1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    uint32_t   seq = 0;
    uint32_t   tick = 0;
    auto       next_wakeup = std::chrono::steady_clock::now();

    while (g_running.load() && master.is_running) {
        next_wakeup += std::chrono::milliseconds(1);
        std::this_thread::sleep_until(next_wakeup);

        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev_classic);
        linkx_recv_pdos(&linkx_dev_fd);

        // 收 CAN 反馈 (只从 FD ch0 拿夹爪的)
        can_msg_t msg;
        while (linkx_quick_recv(&linkx_dev_fd, kClampChannel, &msg)) {
            // DM 反馈帧 ID = Motor_ID + 0x10, 0x11 / 0x12
            // CAN_RxCpltCallback 内部会 ID 自检, 给两路都喂一次最稳
            motor_large.CAN_RxCpltCallback(msg.data);
            motor_small.CAN_RxCpltCallback(msg.data);
        }

        // 每 enter_period_ticks 发一次 Enter + 一次 UDP 遥测
        // —— 唯一的 TX 路径, 无任何 MIT/控制参数写入
        if ((tick % enter_period_ticks) == 0) {
            motor_large.CAN_Send_Enter();
            motor_small.CAN_Send_Enter();

            UdpPacket pkt{};
            pkt.magic       = kMagic;
            pkt.seq         = seq++;
            pkt.t_sec       = std::chrono::duration<float>(
                                  std::chrono::steady_clock::now() - t0).count();
            pkt.theta_large = motor_large.Get_Now_Radian();
            pkt.theta_small = motor_small.Get_Now_Radian();
            pkt.omega_large = motor_large.Get_Now_Omega();
            pkt.omega_small = motor_small.Get_Now_Omega();
            ::sendto(sock, &pkt, sizeof(pkt), 0,
                     reinterpret_cast<sockaddr *>(&dst), sizeof(dst));

            // 每秒打一行 stdout 方便人眼对照
            if ((seq % static_cast<uint32_t>(rate_hz)) == 0) {
                std::printf("[T+%6.2fs] seq=%u  tL=%+7.3f rad  tS=%+7.3f rad  "
                            "wL=%+6.2f  wS=%+6.2f\n",
                            pkt.t_sec, pkt.seq, pkt.theta_large, pkt.theta_small,
                            pkt.omega_large, pkt.omega_small);
                std::fflush(stdout);
            }
        }

        linkx_send_pdos(&linkx_dev_classic);
        linkx_send_pdos(&linkx_dev_fd);
        ++tick;
    }

    std::cout << "[CLAMP-TELE] shutting down..." << std::endl;
    // 离开前对两路 motor 发一次 Exit, 让它们退出 motor mode
    motor_large.CAN_Send_Exit();
    motor_small.CAN_Send_Exit();
    linkx_send_pdos(&linkx_dev_fd);

    ::close(sock);
    return 0;
}
