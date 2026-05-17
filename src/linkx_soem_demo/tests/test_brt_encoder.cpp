// Dry-run unit test for BRT encoder CAN send byte order + ACK parsing.
//
// 验证目标（与手册对照）：
//   1) 0x05/0x0B/0x0D 多字节 set 指令按手册 V2.5 §6.4.3 使用 little-endian（低字节在前）
//   2) 0x02~0x0F set 指令的 ACK [0x04][id][cmd][status] 被正确解析
//   3) 0x01 读响应按 little-endian（回归）
//   4) ID 不匹配的帧被丢弃
//
// 通过 stub linkx_quick_can_send 捕获字节流来断言，不需要硬件 / sudo。
//
// Build target: encoder_byteorder_test （见 CMakeLists.txt）
// 运行：./encoder_byteorder_test  → 退出码 0 = 全部通过

#include "dvc_encoder.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

struct CapturedFrame
{
    uint8_t ch;
    uint32_t id;
    uint8_t data[8];
};

static std::vector<CapturedFrame> g_tx;

extern "C" void linkx_quick_can_send(linkx_t *, uint8_t ch, uint32_t id, uint8_t *data)
{
    CapturedFrame f{};
    f.ch = ch;
    f.id = id;
    std::memcpy(f.data, data, 8);
    g_tx.push_back(f);
}

extern "C" void linkx_quick_FDcan_send(linkx_t *, uint8_t, uint32_t, uint8_t *) {}
extern "C" void linkx_quick_can_send_rtr(linkx_t *, uint8_t, uint32_t, uint8_t *) {}

#define EXPECT_EQ(a, b, label) do {                                                               \
    if ((a) != (b)) {                                                                             \
        std::fprintf(stderr, "FAIL %s: got 0x%X expected 0x%X\n", label, (unsigned)(a), (unsigned)(b)); \
        ++fails;                                                                                  \
    } else {                                                                                      \
        std::fprintf(stdout, "  ok  %s = 0x%X\n", label, (unsigned)(a));                          \
    }                                                                                             \
} while (0)

int main()
{
    int fails = 0;
    Class_Encoder_BRT enc;
    enc.Init(nullptr, /*slave_index=*/2, /*encoder_id=*/0x05, /*single_turn_res=*/4096);

    // T1: 0x05 Set Auto Send Time, 1000us (0x03E8) → little-endian [0xE8][0x03]
    // 依据：手册 V2.5 §6.4.3 / §6.5.2 明确"多字节低字节在前"。
    // 手册第 13 页 0x05 示例：[0x05][0x01][0x05][0xE8][0x03]。
    g_tx.clear();
    enc.CAN_Send_SetAutoSendTime(1000);
    std::printf("[T1] 0x05 Set Auto Send Time = 1000us (0x03E8) -> ");
    for (int i = 0; i < 8; ++i) std::printf("%02X ", g_tx[0].data[i]); std::printf("\n");
    EXPECT_EQ(g_tx[0].data[0], 0x05, "T1 length=payload+3");
    EXPECT_EQ(g_tx[0].data[1], 0x05, "T1 encoder id");
    EXPECT_EQ(g_tx[0].data[2], BRT_CMD_SET_AUTO_SEND_TIME, "T1 cmd 0x05");
    EXPECT_EQ(g_tx[0].data[3], 0xE8, "T1 payload[0] low byte (little-endian)");
    EXPECT_EQ(g_tx[0].data[4], 0x03, "T1 payload[1] high byte");

    // T2: 0x0B Set Velocity Sample Time, 1000ms → little-endian [0xE8][0x03]
    g_tx.clear();
    enc.CAN_Send_SetVelocitySampleTime(1000);
    std::printf("[T2] 0x0B Set Vel Sample Time = 1000ms (0x03E8) -> ");
    for (int i = 0; i < 8; ++i) std::printf("%02X ", g_tx[0].data[i]); std::printf("\n");
    EXPECT_EQ(g_tx[0].data[2], BRT_CMD_SET_VELOCITY_SAMP_TIME, "T2 cmd 0x0B");
    EXPECT_EQ(g_tx[0].data[3], 0xE8, "T2 payload low byte");
    EXPECT_EQ(g_tx[0].data[4], 0x03, "T2 payload high byte");

    // T3: 0x0D Set Current Value 0x00012345 → little-endian [0x45][0x23][0x01][0x00]
    g_tx.clear();
    enc.CAN_Send_SetCurrentValue(0x00012345);
    std::printf("[T3] 0x0D Set Current Value = 0x00012345 -> ");
    for (int i = 0; i < 8; ++i) std::printf("%02X ", g_tx[0].data[i]); std::printf("\n");
    EXPECT_EQ(g_tx[0].data[0], 0x07, "T3 length=4+3=7");
    EXPECT_EQ(g_tx[0].data[2], BRT_CMD_SET_CURRENT_VALUE, "T3 cmd 0x0D");
    EXPECT_EQ(g_tx[0].data[3], 0x45, "T3 LSB first");
    EXPECT_EQ(g_tx[0].data[4], 0x23, "T3 byte1");
    EXPECT_EQ(g_tx[0].data[5], 0x01, "T3 byte2");
    EXPECT_EQ(g_tx[0].data[6], 0x00, "T3 MSB last");

    // T4: 0x06 Set Zero ACK 成功
    {
        uint8_t resp[8] = {0x04, 0x05, BRT_CMD_SET_ZERO, 0x00, 0, 0, 0, 0};
        enc.CAN_RxCpltCallback(resp);
    }
    std::printf("[T4] 0x06 Set Zero ACK success: zero_ok=%d last_status=0x%X\n",
                enc.Is_Zero_Ack_Ok(), enc.Get_Last_Ack_Status());
    EXPECT_EQ(enc.Is_Zero_Ack_Ok(), true, "T4 Is_Zero_Ack_Ok==true");
    EXPECT_EQ(enc.Get_Last_Ack_Cmd(), BRT_CMD_SET_ZERO, "T4 last_ack_cmd=0x06");
    EXPECT_EQ(enc.Get_Last_Ack_Status(), 0x00, "T4 status=0x00");

    // T5: 0x04 Set Mode ACK 失败码 0x55
    {
        uint8_t resp[8] = {0x04, 0x05, BRT_CMD_SET_MODE, 0x55, 0, 0, 0, 0};
        enc.CAN_RxCpltCallback(resp);
    }
    std::printf("[T5] 0x04 Set Mode ACK fail: mode_ok=%d status=0x%X\n",
                enc.Is_Mode_Ack_Ok(), enc.Get_Last_Ack_Status());
    EXPECT_EQ(enc.Is_Mode_Ack_Ok(), false, "T5 Is_Mode_Ack_Ok==false");
    EXPECT_EQ(enc.Get_Last_Ack_Status(), 0x55, "T5 status=0x55");

    // T6: 0x0C Set Midpoint ACK 成功
    {
        uint8_t resp[8] = {0x04, 0x05, BRT_CMD_SET_MIDPOINT, 0x00, 0, 0, 0, 0};
        enc.CAN_RxCpltCallback(resp);
    }
    std::printf("[T6] 0x0C Set Midpoint ACK success: mid_ok=%d\n", enc.Is_Midpoint_Ack_Ok());
    EXPECT_EQ(enc.Is_Midpoint_Ack_Ok(), true, "T6 Is_Midpoint_Ack_Ok==true");

    // T7: 0x01 read response 仍按小端解析（回归）
    {
        uint8_t resp[8] = {0x07, 0x05, BRT_CMD_READ_ENCODER_VALUE, 0x45, 0x23, 0x01, 0x00, 0};
        enc.CAN_RxCpltCallback(resp);
    }
    std::printf("[T7] 0x01 read response 74565 -> Get_EncoderValue()=%u\n", enc.Get_EncoderValue());
    EXPECT_EQ(enc.Get_EncoderValue(), 74565u, "T7 encoder_value=74565");

    // T8: ID 不匹配的帧被丢弃
    {
        uint8_t resp[8] = {0x04, 0x99, BRT_CMD_SET_ZERO, 0x77, 0, 0, 0, 0};
        enc.CAN_RxCpltCallback(resp);
    }
    std::printf("[T8] wrong-id frame ignored: last_status=0x%X (expect not 0x77)\n",
                enc.Get_Last_Ack_Status());
    if (enc.Get_Last_Ack_Status() == 0x77) {
        std::fprintf(stderr, "FAIL T8: id-mismatch frame leaked into ack state\n");
        ++fails;
    } else {
        std::fprintf(stdout, "  ok  T8 id-mismatch dropped\n");
    }

    if (fails == 0) {
        std::printf("\nALL TESTS PASSED\n");
        return 0;
    }
    std::printf("\n%d FAILURES\n", fails);
    return 1;
}
