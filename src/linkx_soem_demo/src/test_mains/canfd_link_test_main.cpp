// canfd_link_test:验证总线挂第二片 LinkX 后,能扫到 slavecount==2,
// 第二片 4 通道按 CAN-FD 配置成功,并能持续发出 64B BRS 帧。
//
// 用法:
//   sudo ./build/linkx_soem_demo/canfd_link_test [ifname]
//
// 物理前提:把新 LinkX 串在老 LinkX 下游(老 OUT → 新 IN),否则 slave_id 会反转。
//   slave_id=1 → linkx_classic (老,4 路经典 1Mbps)
//   slave_id=2 → linkx_fd      (新,4 路 CAN-FD,仲裁 1M / 数据 5M)
//
// 发包频率:classic 100Hz × 8B、FD 50Hz × 64B,统计 1Hz 打印。
// 没接 CAN 对端时 rx 计数为 0 是正常的——只看 tx_frames 与 baudrate wkc 即可判断链路是否通。

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>

#include "ecat_manager.h"
#include "linkx4c_handler.h"

namespace
{
volatile std::sig_atomic_t g_running = 1;

void signal_handler(int) { g_running = 0; }

constexpr uint32_t kClassicBaseId = 0x130U;  // 老模块 ch0..3 → 0x130..0x133
constexpr uint32_t kFDBaseId      = 0x140U;  // 新模块 ch0..3 → 0x140..0x143
constexpr uint8_t  kFDPayloadLen  = 64;
constexpr uint8_t  kClassicPayloadLen = 8;
constexpr uint32_t kClassicSendPeriodTicks = 10;   // 100 Hz @ 1ms
constexpr uint32_t kFDSendPeriodTicks      = 20;   // 50 Hz @ 1ms
constexpr uint32_t kStatsPeriodTicks       = 1000; // 1 Hz

void print_stats_row(const char *tag, const linkx_t &dev)
{
    std::cout << "  " << tag
              << " tx["
              << dev.can_stats[0].tx_frames << "/" << dev.can_stats[1].tx_frames << "/"
              << dev.can_stats[2].tx_frames << "/" << dev.can_stats[3].tx_frames << "]"
              << " rx["
              << dev.can_stats[0].rx_frames << "/" << dev.can_stats[1].rx_frames << "/"
              << dev.can_stats[2].rx_frames << "/" << dev.can_stats[3].rx_frames << "]"
              << " drop["
              << dev.can_stats[0].tx_dropped_frames << "/" << dev.can_stats[1].tx_dropped_frames << "/"
              << dev.can_stats[2].tx_dropped_frames << "/" << dev.can_stats[3].tx_dropped_frames << "]"
              << std::endl;
}
} // namespace

int main(int argc, char *argv[])
{
    const char *ifname = (argc > 1) ? argv[1] : "enp86s0";

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "[CANFD-TEST] ifname=" << ifname << std::endl;

    ecat_master_t master {};
    if (!ecat_master_init(&master, ifname))
    {
        std::cerr << "[CANFD-TEST] ecat_master_init failed." << std::endl;
        return 1;
    }

    if (master.ctx.slavecount < 2)
    {
        std::cerr << "[CANFD-TEST] FATAL: expected >=2 EtherCAT slaves, found "
                  << master.ctx.slavecount << "."
                  << " Check downstream LinkX cabling (old.OUT -> new.IN)." << std::endl;
        return 3;
    }
    std::cout << "[CANFD-TEST] slavecount=" << master.ctx.slavecount
              << " (slave_id=1 -> classic, slave_id=2 -> fd)" << std::endl;

    linkx_t linkx_classic {};
    linkx_t linkx_fd {};
    linkx_init(&linkx_classic, 1, &master.ctx);
    linkx_init(&linkx_fd,      2, &master.ctx);

    linkx_hw_wakeup(&linkx_classic);
    linkx_hw_wakeup(&linkx_fd);

    int classic_ok = 0;
    int fd_ok = 0;
    for (int ch = 0; ch < 4; ++ch)
    {
        // classic:仲裁 1Mbps,数据段无效(fd_en=0)
        if (linkx_set_can_baudrate(&linkx_classic, ch, 0, 2, 31, 8, 8, 1, 31, 8, 8))
            ++classic_ok;
        // fd:仲裁 1Mbps + 数据 5Mbps(待示波器实测;时序 d_pre=1/d_seg1=6/d_seg2=1/d_sjw=1)
        if (linkx_set_can_baudrate(&linkx_fd, ch, 1, 2, 31, 8, 8, 1, 6, 1, 1))
            ++fd_ok;
    }
    std::cout << "[CANFD-TEST] baudrate config: classic=" << classic_ok
              << "/4  fd=" << fd_ok << "/4" << std::endl;
    if (fd_ok != 4)
        std::cerr << "[CANFD-TEST] WARN: FD module baudrate not all OK; check SDO 0x8002." << std::endl;

    if (!ecat_master_bring_online(&master))
    {
        std::cerr << "[CANFD-TEST] ecat_master_bring_online failed." << std::endl;
        return 2;
    }

    std::cout << "[CANFD-TEST] Online."
              << "  CLASSIC[ch0..3]->0x" << std::hex << kClassicBaseId << ".." << (kClassicBaseId + 3)
              << " (8B)"
              << "  FD[ch0..3]->0x"      << kFDBaseId      << ".." << (kFDBaseId + 3)
              << std::dec << " (64B BRS)" << std::endl;

    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;
    uint8_t classic_payload[kClassicPayloadLen];
    uint8_t fd_payload[kFDPayloadLen];

    while (g_running && master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(1);

        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_classic);
        linkx_recv_pdos(&linkx_fd);

        if ((tick % kClassicSendPeriodTicks) == 0U)
        {
            for (uint8_t ch = 0; ch < 4; ++ch)
            {
                classic_payload[0] = ch;
                classic_payload[1] = static_cast<uint8_t>(tick & 0xFFU);
                classic_payload[2] = static_cast<uint8_t>((tick >> 8U) & 0xFFU);
                classic_payload[3] = 0xC1;
                classic_payload[4] = 0xA5;
                classic_payload[5] = 0x5A;
                classic_payload[6] = 0x11;
                classic_payload[7] = 0x22;
                linkx_quick_can_send(&linkx_classic, ch, kClassicBaseId + ch, classic_payload);
            }
        }

        if ((tick % kFDSendPeriodTicks) == 0U)
        {
            for (uint8_t ch = 0; ch < 4; ++ch)
            {
                std::memset(fd_payload, 0, sizeof(fd_payload));
                fd_payload[0]  = ch;
                fd_payload[1]  = static_cast<uint8_t>(tick & 0xFFU);
                fd_payload[2]  = static_cast<uint8_t>((tick >> 8U) & 0xFFU);
                fd_payload[3]  = 0xFD;
                fd_payload[4]  = 0xCA;
                fd_payload[63] = 0xEE; // 末位标识,便于嗅探器/示波器对位
                linkx_send_can(&linkx_fd, ch, kFDBaseId + ch,
                               /*canfd=*/true, /*brs=*/true, /*ext=*/false, /*rtr=*/false,
                               /*dlen=*/kFDPayloadLen,
                               reinterpret_cast<uint32_t *>(fd_payload));
            }
        }

        linkx_send_pdos(&linkx_classic);
        linkx_send_pdos(&linkx_fd);

        if ((tick % kStatsPeriodTicks) == 0U)
        {
            std::cout << "[CANFD-TEST] tick=" << tick << " --- stats ---" << std::endl;
            print_stats_row("CLASSIC[1]", linkx_classic);
            print_stats_row("FD     [2]", linkx_fd);

            for (uint8_t ch = 0; ch < 4; ++ch)
            {
                can_tx_pdo_t *pdo = linkx_recv_can(&linkx_fd, ch);
                if (pdo != nullptr && pdo->params.dlen > 0U)
                {
                    std::cout << "    FD ch" << static_cast<int>(ch)
                              << " last_rx: id=0x" << std::hex << pdo->can_id << std::dec
                              << " canfd=" << static_cast<int>(pdo->params.canfd)
                              << " brs="   << static_cast<int>(pdo->params.brs)
                              << " dlen="  << static_cast<int>(pdo->params.dlen)
                              << std::endl;
                }
            }
        }

        ++tick;
        std::this_thread::sleep_until(next_wakeup);
    }

    master.is_running = false;
    std::cout << "[CANFD-TEST] Exit." << std::endl;
    return 0;
}
