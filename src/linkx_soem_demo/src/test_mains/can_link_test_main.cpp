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

void signal_handler(int)
{
    g_running = 0;
}
}

int main(int argc, char *argv[])
{
    const char *ifname = (argc > 1) ? argv[1] : "enp86s0";

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "[CAN-TEST] ifname=" << ifname << std::endl;

    ecat_master_t master {};
    if (!ecat_master_init(&master, ifname))
    {
        std::cerr << "[CAN-TEST] ecat_master_init failed." << std::endl;
        return 1;
    }

    linkx_t linkx_dev {};
    linkx_init(&linkx_dev, 1, &master.ctx);

    linkx_hw_wakeup(&linkx_dev);
    for (int ch = 0; ch < 4; ++ch)
    {
        // 经典 CAN 1M，与主程序保持一致
        linkx_set_can_baudrate(&linkx_dev, ch, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    }

    if (!ecat_master_bring_online(&master))
    {
        std::cerr << "[CAN-TEST] ecat_master_bring_online failed." << std::endl;
        return 2;
    }

    std::cout << "[CAN-TEST] Sending on all channels..." << std::endl;
    std::cout << "[CAN-TEST] CH0->0x120 CH1->0x121 CH2->0x122 CH3->0x123" << std::endl;

    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;

    while (g_running && master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(1);

        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev);

        // 100Hz 发包，降低总线和终端打印压力
        if ((tick % 10U) == 0U)
        {
            for (uint8_t ch = 0; ch < 4; ++ch)
            {
                uint8_t data[8];
                data[0] = ch;
                data[1] = static_cast<uint8_t>(tick & 0xFFU);
                data[2] = static_cast<uint8_t>((tick >> 8U) & 0xFFU);
                data[3] = 0xA5;
                data[4] = 0x5A;
                data[5] = 0x11;
                data[6] = 0x22;
                data[7] = 0x33;

                const uint32_t can_id = 0x120U + ch;
                linkx_quick_can_send(&linkx_dev, ch, can_id, data);
            }
        }

        linkx_send_pdos(&linkx_dev);

        if ((tick % 1000U) == 0U)
        {
            std::cout << "[CAN-TEST] tick=" << tick
                      << " tx_frames: ch0=" << linkx_dev.can_stats[0].tx_frames
                      << " ch1=" << linkx_dev.can_stats[1].tx_frames
                      << " ch2=" << linkx_dev.can_stats[2].tx_frames
                      << " ch3=" << linkx_dev.can_stats[3].tx_frames
                      << std::endl;
        }

        ++tick;
        std::this_thread::sleep_until(next_wakeup);
    }

    master.is_running = false;
    std::cout << "[CAN-TEST] Exit." << std::endl;
    return 0;
}

