// enc3_recover_main.cpp
//
// enc3 出厂恢复后的两阶段重配工具。
//
// 前置条件：用户已通过"黄线接地 2 分钟"恢复 enc3 出厂设置:
//   波特率 500K，CAN ID 0x01，默认模式
//
// 目标：把 enc3 配回 1M / 0x08 / AUTO_RETURN_ANGLE / 5ms (200Hz)
//
// 必须分两阶段，因为 SOEM SDO 写入只在 PRE-OP 状态有效，
// 进 OP 后不能改 ch=2 的波特率：
//
//   阶段 1：ecat init + ch=2 配 500K + 进 OP → 跟 enc3 (500K, 0x01) 通信
//           发 SetBaudrate(1M)，让 enc3 立即切到 1M
//           ecx_close 关掉 ecat
//
//   阶段 2：ecat init + ch=2 配 1M + 进 OP → 跟 enc3 (1M, 0x01) 通信
//           发 SetEncoderID(0x08) + SetAutoSendTime(5000) + SetMode(0xAA)
//           验证 enc3 (1M, 0x08) 在 200Hz 上自动上报
//
// 用法（必须 sudo）：
//   sudo IFNAME=enp86s0 ./enc3_recover

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <thread>

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "soem.h"
#include "dvc_encoder.h"

namespace
{
constexpr int kChannelCount = 4;
constexpr uint32_t kCtrlPeriodMs = 1;
constexpr uint8_t kEncoderChannel = 2;
constexpr uint8_t kDefaultId = 0x01;
constexpr uint8_t kTargetId = 0x08;
constexpr uint16_t kTargetAutoSendUs = 5000;

ecat_master_t st_master;
linkx_t       st_linkx;
std::atomic<bool> st_running{true};

Class_Encoder_BRT st_enc;
uint8_t st_current_target_id = kDefaultId;
std::map<uint32_t, uint32_t> st_rx_histogram;

void on_signal(int)
{
    st_running.store(false);
    st_master.is_running = false;
}

void can_dispatch(uint8_t ch, uint32_t can_id, uint8_t *data)
{
    if (ch != kEncoderChannel) return;
    const uint32_t id_std = can_id & 0x7FFU;
    st_rx_histogram[id_std]++;
    if (id_std == st_current_target_id)
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
    for (uint32_t i = 0; i < ms; ++i)
    {
        next += std::chrono::milliseconds(kCtrlPeriodMs);
        if (!ec_step()) return;
        std::this_thread::sleep_until(next);
    }
}

// 1Mbps: prescaler=2, seg1=31, seg2=8, sjw=8 → 40 TQ × 2 = 80 / 80MHz = 1Mbps
// 500K:  prescaler=4, seg1=31, seg2=8, sjw=8 → 40 TQ × 4 = 160 / 80MHz = 500K
bool config_ch2_baudrate(uint8_t prescaler)
{
    return linkx_set_can_baudrate(&st_linkx, kEncoderChannel, 0,
                                  prescaler, 31, 8, 8,
                                  1, 31, 8, 8);
}

// 启动一个 ecat 主站会话，ch=2 用指定 prescaler，其他通道全部 1M
bool ecat_session_start(const std::string &ifname, uint8_t ch2_prescaler)
{
    if (!ecat_master_init(&st_master, ifname.c_str())) {
        std::cerr << "  ❌ ecat_master_init failed\n";
        return false;
    }
    linkx_init(&st_linkx, 1, &st_master.ctx);
    linkx_hw_wakeup(&st_linkx);
    // ch=0/1/3 都用 1M
    for (int i = 0; i < kChannelCount; i++) {
        if (i == kEncoderChannel) continue;
        linkx_set_can_baudrate(&st_linkx, i, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    }
    // ch=2 用指定 prescaler
    if (!config_ch2_baudrate(ch2_prescaler)) {
        std::cerr << "  ❌ failed to set ch=2 baudrate (prescaler=" << (int)ch2_prescaler << ")\n";
        return false;
    }
    if (!ecat_master_bring_online(&st_master)) {
        std::cerr << "  ❌ ecat bring_online failed\n";
        return false;
    }
    return true;
}

void ecat_session_stop()
{
    ecx_close(&st_master.ctx);
    st_master.is_running = false;
}

}  // namespace

int main()
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string ifname = std::getenv("IFNAME") ? std::getenv("IFNAME") : "enp86s0";

    std::cout << "================================================================\n"
              << "  ENC3 RECOVERY TOOL  (after factory-reset)\n"
              << "  500K/0x01 → 1M/0x08, AUTO_RETURN_ANGLE @ 5ms (200Hz)\n"
              << "  IFNAME: " << ifname << "\n"
              << "================================================================\n";

    // ═════════════════════════════════════════════════════════════
    // STAGE 1: ch=2 用 500K，跟 enc3 (500K, 0x01) 通信，让它切 1M
    // ═════════════════════════════════════════════════════════════
    std::cout << "\n══ STAGE 1: 500K mode — switch enc3 to 1M ══\n";
    if (!ecat_session_start(ifname, /*ch2_prescaler=500K*/ 4)) return 1;

    st_enc.Init(&st_linkx, 1, kDefaultId, 4096);
    st_current_target_id = kDefaultId;

    // 出厂默认 enc3 可能是 QUERY 模式（不主动上报），主动发 0x01 探测它
    std::cout << "  Probing enc3 at 500K with active queries (10x)...\n";
    st_rx_histogram.clear();
    for (int i = 0; i < 10; ++i)
    {
        st_enc.CAN_Send_ReadEncoderValue();
        ec_step();
        busywait_ms(100);
    }
    std::cout << "  ch=2 RX histogram during probe:\n";
    for (auto &kv : st_rx_histogram)
        std::cout << "    id=0x" << std::hex << kv.first << std::dec
                  << " count=" << kv.second << "\n";
    bool enc3_at_500k = (st_rx_histogram.count(kDefaultId) > 0 &&
                         st_rx_histogram[kDefaultId] > 0);
    if (enc3_at_500k)
        std::cout << "  ✅ enc3 confirmed at 500K / id=0x01\n";
    else
        std::cout << "  ⚠️ no id=0x01 response at 500K — enc3 may already be on 1M, "
                     "or factory-reset incomplete. Will try SetBaudrate anyway.\n";

    std::cout << "  Sending SetBaudrate(1M = 0x01) to enc3 (id=0x01)...\n";
    st_enc.CAN_Send_SetBaudrate(BRT_BAUDRATE_1M);
    ec_step();
    busywait_ms(500);  // 让 enc3 完成 EEPROM 写入 + 切换波特率

    ecat_session_stop();
    std::cout << "  ecat session closed; enc3 should now be on 1M\n";

    // ═════════════════════════════════════════════════════════════
    // STAGE 2: ch=2 用 1M，完成后续配置
    // ═════════════════════════════════════════════════════════════
    std::cout << "\n══ STAGE 2: 1M mode — finalize id/mode/freq ══\n";
    busywait_ms(500);
    if (!ecat_session_start(ifname, /*ch2_prescaler=1M*/ 2)) return 1;

    st_enc.Init(&st_linkx, 1, kDefaultId, 4096);
    st_current_target_id = kDefaultId;

    // 主动 probe enc3 at 1M / id=0x01
    std::cout << "  Probing enc3 at 1M / id=0x01 with active queries (10x)...\n";
    st_rx_histogram.clear();
    for (int i = 0; i < 10; ++i)
    {
        st_enc.CAN_Send_ReadEncoderValue();
        ec_step();
        busywait_ms(100);
    }
    std::cout << "  ch=2 RX histogram during probe:\n";
    bool enc3_at_1m = false;
    for (auto &kv : st_rx_histogram) {
        std::cout << "    id=0x" << std::hex << kv.first << std::dec
                  << " count=" << kv.second << "\n";
        if (kv.first == kDefaultId && kv.second > 0) enc3_at_1m = true;
    }
    if (!enc3_at_1m) {
        std::cerr << "  ❌ no id=0x01 response on 1M — STAGE 1 SetBaudrate may have failed\n";
        std::cerr << "     check enc3 hardware (LED, power, CAN wiring) and rerun\n";
        return 1;
    }
    std::cout << "  ✅ enc3 on 1M / id=0x01 confirmed\n";

    // ─────────────────────────────────────────────────────────────
    // SetEncoderID(0x08)
    std::cout << "\n  Sending SetEncoderID(0x08) to enc3 (id=0x01)...\n";
    st_enc.CAN_Send_SetEncoderID(kTargetId);
    ec_step();
    busywait_ms(500);

    // 切路由：现在 enc3 应该是 ID=0x08
    st_enc.Init(&st_linkx, 1, kTargetId, 4096);
    st_current_target_id = kTargetId;

    std::cout << "  Listening for id=0x08 traffic (0.5s)...\n";
    st_rx_histogram.clear();
    busywait_ms(500);
    bool enc3_on_new_id = false;
    for (auto &kv : st_rx_histogram)
        if (kv.first == kTargetId && kv.second > 0) enc3_on_new_id = true;
    if (!enc3_on_new_id) {
        std::cerr << "  ❌ id=0x08 not seen — SetEncoderID may have failed\n";
        return 1;
    }
    std::cout << "  ✅ enc3 ID changed to 0x08\n";

    // ─────────────────────────────────────────────────────────────
    // SetMode(QUERY) → SetAutoSendTime(5000) → SetMode(AUTO_RETURN_ANGLE)
    std::cout << "\n  Configuring enc3 (id=0x08): QUERY → 5ms period → AUTO_RETURN_ANGLE\n";

    st_enc.CAN_Send_SetMode(BRT_MODE_QUERY);
    ec_step();
    busywait_ms(200);

    st_enc.CAN_Send_SetAutoSendTime(kTargetAutoSendUs);
    ec_step();
    busywait_ms(200);

    st_enc.CAN_Send_SetMode(BRT_MODE_AUTO_RETURN_ANGLE);
    ec_step();
    busywait_ms(500);

    // ─────────────────────────────────────────────────────────────
    // 验证频率
    std::cout << "\n  Verifying enc3 (id=0x08) rx frequency over 1s...\n";
    const uint32_t before = st_enc.Get_Rx_Count();
    busywait_ms(1000);
    const uint32_t delta = st_enc.Get_Rx_Count() - before;
    const double hz = static_cast<double>(delta);
    const bool ok = std::abs(hz - 200.0) < 50.0;

    std::cout << "\n================================================================\n";
    if (ok)
        std::cout << "  ✅ ENC3 RECOVERY SUCCESSFUL\n"
                  << "     1M / id=0x08 / AUTO_RETURN_ANGLE / " << hz << " Hz\n"
                  << "     You can now restart the main program.\n";
    else
        std::cout << "  ⚠️ ENC3 reconfigured but freq " << hz << " Hz off target 200Hz\n"
                  << "     enc3 is alive on 1M/0x08 but rx_freq mismatched.\n";
    std::cout << "================================================================\n";

    ecat_session_stop();
    return ok ? 0 : 1;
}
