// enc3_set_period_main.cpp
//
// 用慢节奏（USB 上位机风格）单独给 enc3 设自动回传时间。
//
// 假设 enc3 已在 1M / id=0x08，状态正常。
// 试图把 enc3 的 auto_send_time 设成 5000us (200Hz)。
//
// 跟 encoder_ack_test PHASE C 的区别：
//   - 单独操作 enc3，不动其他 enc
//   - 每步等 2 秒（让 EEPROM 充分写入 + 总线安静）
//   - 重试 10 次
//   - 每次重试前后单独打印诊断
//
// 用法：sudo IFNAME=enp86s0 ./enc3_set_period

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "dvc_encoder.h"

namespace
{
constexpr int kChannelCount = 4;
constexpr uint32_t kCtrlPeriodMs = 1;
constexpr uint8_t kEncoderChannel = 2;
constexpr uint8_t kTargetId = 0x08;
constexpr uint16_t kTargetAutoSendUs = 5000;  // 5ms = 200Hz
constexpr double kTargetHz = 200.0;
constexpr double kHzTolerance = 50.0;

ecat_master_t st_master;
linkx_t       st_linkx;
std::atomic<bool> st_running{true};
Class_Encoder_BRT st_enc;

void on_signal(int) { st_running.store(false); st_master.is_running = false; }

void can_dispatch(uint8_t ch, uint32_t can_id, uint8_t *data)
{
    if (ch != kEncoderChannel) return;
    if ((can_id & 0x7FFU) == kTargetId)
        st_enc.CAN_RxCpltCallback(data);
}

bool ec_step()
{
    if (!st_running.load() || !st_master.is_running) return false;
    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);
    can_msg_t msg;
    for (uint8_t ch = 0; ch < kChannelCount; ch++)
        while (linkx_quick_recv(&st_linkx, ch, &msg))
            can_dispatch(ch, msg.id, msg.data);
    linkx_send_pdos(&st_linkx);
    return true;
}

void busywait_ms(uint32_t ms)
{
    auto next = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < ms; ++i) {
        next += std::chrono::milliseconds(kCtrlPeriodMs);
        if (!ec_step()) return;
        std::this_thread::sleep_until(next);
    }
}

double measure_hz(uint32_t window_ms)
{
    const uint32_t before = st_enc.Get_Rx_Count();
    busywait_ms(window_ms);
    const uint32_t delta = st_enc.Get_Rx_Count() - before;
    return static_cast<double>(delta) * 1000.0 / window_ms;
}

}  // namespace

int main()
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string ifname = std::getenv("IFNAME") ? std::getenv("IFNAME") : "enp86s0";

    std::cout << "================================================================\n"
              << "  ENC3 SET-PERIOD TOOL  (slow-paced retry, USB-tool style)\n"
              << "  Target: enc3 (id=0x08) auto_send_time = 5000us (200Hz)\n"
              << "  IFNAME: " << ifname << "\n"
              << "================================================================\n";

    if (!ecat_master_init(&st_master, ifname.c_str())) return 1;
    linkx_init(&st_linkx, 1, &st_master.ctx);
    linkx_hw_wakeup(&st_linkx);
    for (int i = 0; i < kChannelCount; i++)
        linkx_set_can_baudrate(&st_linkx, i, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    if (!ecat_master_bring_online(&st_master)) return 1;

    st_enc.Init(&st_linkx, 1, kTargetId, 4096);

    // 先让总线稳定 + 测 baseline
    busywait_ms(500);
    const double hz_baseline = measure_hz(1000);
    std::cout << "\n  baseline rx_freq = " << hz_baseline << " Hz\n";
    if (std::abs(hz_baseline - kTargetHz) < kHzTolerance) {
        std::cout << "  ✅ already at target, nothing to do\n";
        return 0;
    }

    // 慢节奏重试 10 次
    constexpr int kMaxRetries = 10;
    int succeed_at = -1;
    for (int retry = 1; retry <= kMaxRetries; ++retry)
    {
        std::cout << "\n--- retry " << retry << "/" << kMaxRetries << " ---\n";

        std::cout << "  [1] SetMode(QUERY) — quiet enc3 first...\n";
        st_enc.CAN_Send_SetMode(BRT_MODE_QUERY);
        ec_step();
        busywait_ms(2000);  // 等 2 秒，让 enc3 完全停下来 + EEPROM 写入

        std::cout << "  [2] SetAutoSendTime(" << kTargetAutoSendUs << "us)...\n";
        st_enc.CAN_Send_SetAutoSendTime(kTargetAutoSendUs);
        ec_step();
        busywait_ms(2000);  // 等 2 秒，让 EEPROM 写入

        std::cout << "  [3] SetMode(AUTO_RETURN_ANGLE)...\n";
        st_enc.CAN_Send_SetMode(BRT_MODE_AUTO_RETURN_ANGLE);
        ec_step();
        busywait_ms(1000);

        const double hz = measure_hz(1000);
        std::cout << "  [verify] rx_freq = " << hz << " Hz";
        if (std::abs(hz - kTargetHz) < kHzTolerance) {
            std::cout << "  ✅ within tolerance, set TOOK EFFECT!\n";
            succeed_at = retry;
            break;
        }
        std::cout << "  ❌ still off target\n";
    }

    std::cout << "\n================================================================\n";
    if (succeed_at > 0)
        std::cout << "  ✅ SUCCESS at retry #" << succeed_at
                  << " — enc3 now at 200Hz\n";
    else
        std::cout << "  ❌ FAILED after " << kMaxRetries << " slow-paced retries\n"
                  << "     LinkX bridge cannot deliver SetAutoSendTime to this enc3.\n"
                  << "     Use the USB tool to set 'auto-send-time' = 5000 directly.\n";
    std::cout << "================================================================\n";
    return succeed_at > 0 ? 0 : 1;
}
