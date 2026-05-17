
#include "linkx4c_handler.h"

#include "stdio.h"
#include "string.h"

#include "soem/osal.h"    // 提供基础操作系统抽象类型
#include "soem/ec_type.h" // 提供 uint16, uint8, ecx_contextt 等类型定义
#include "soem.h"         // 提供 SOEM 核心功能
#include "soem/ec_coe.h"  // 提供 SDO 读写函数 (ecx_SDOwrite)

#include "unistd.h"

// linkx_rx_snapshot_t 已在 linkx.h 中定义并按实例隔离（linkx_t::rx_dedup），
// 避免函数静态变量在多个 linkx_t 实例之间串扰。

static uint16_t linkx_pack_params(const can_pdo_param_t *params)
{
    uint16_t raw = 0;
    raw |= (uint16_t)(params->ext & 0x01u);
    raw |= (uint16_t)((params->rtr & 0x01u) << 1);
    raw |= (uint16_t)((params->canfd & 0x01u) << 2);
    raw |= (uint16_t)((params->brs & 0x01u) << 3);
    raw |= (uint16_t)(params->dlen << 8);
    return raw;
}

static bool linkx_snapshot_equals_pdo(const linkx_rx_snapshot_t *snap, const can_tx_pdo_t *pdo, uint8_t dlen)
{
    if (!snap->valid)
        return false;

    if (snap->can_id != pdo->can_id)
        return false;

    if (snap->params_raw != linkx_pack_params(&pdo->params))
        return false;

    if (snap->timestamp != pdo->timestamp)
        return false;

    if (snap->dlen != dlen)
        return false;

    if (dlen == 0)
        return true;

    return (memcmp(snap->data, pdo->data_u32, dlen) == 0);
}

static void linkx_snapshot_update(linkx_rx_snapshot_t *snap, const can_tx_pdo_t *pdo, uint8_t dlen)
{
    snap->valid = true;
    snap->can_id = pdo->can_id;
    snap->params_raw = linkx_pack_params(&pdo->params);
    snap->timestamp = pdo->timestamp;
    snap->dlen = dlen;

    if (dlen > 0)
        memcpy(snap->data, pdo->data_u32, dlen);
}

// 封装 SDO 唤醒逻辑，屏蔽 0x8001 寄存器细节
//   每个 CAN 通道最多重试 kMaxRetries 次；任意一次成功即认为该通道 OK。
//   返回值：仅当所有通道全部成功才返回 true（任何一个通道挂了 = 总线不可用）。
bool linkx_hw_wakeup(linkx_t *linkx)
{
    static const int kMaxRetries = 3;
    bool all_ok = true;

    printf("[LinkX] Waking up CAN PHYs via SDO...\n");

    for (uint8_t ch = 0; ch < LINKX_CAN_CHANNEL_NUM; ch++)
    {
        bool ok = false;
        int  attempts = 0;
        for (; attempts < kMaxRetries; ++attempts)
        {
            ok = linkx_switch_can_channel(linkx, ch, true);
            if (ok) break;
        }
        if (ok && attempts > 0)
            printf("[LinkX] CAN Channel %d: WAKEUP SUCCESS (after %d retries)\n", ch, attempts);
        else
            printf("[LinkX] CAN Channel %d: WAKEUP %s\n", ch, ok ? "SUCCESS" : "FAILED");
        if (!ok) all_ok = false;
    }
    printf("[LinkX] All hardware initialization commands sent.\n");
    return all_ok;
}

// 8 字节固定长度发送（FDcan / 经典 CAN / 经典 CAN+RTR 三个公共 API 的共同实现）
// 参数顺序：linkx, channel, id, canfd, brs, ext, rtr, dlen, data
static inline void linkx_send_8b(linkx_t *linkx, uint8_t ch, uint32_t id,
                                  bool canfd, bool brs, bool rtr, uint8_t *data)
{
    linkx_send_can(linkx, ch, id, canfd, brs, false, rtr, 8, (uint32_t *)data);
}

// 封装发送：FDCAN 自动处理指针强转和 linkx 参数顺序
void linkx_quick_FDcan_send(linkx_t *linkx, uint8_t ch, uint32_t id, uint8_t *data)
{
    linkx_send_8b(linkx, ch, id, true, true, false, data);
}

// 封装发送：经典 CAN 自动处理指针强转和 linkx 参数顺序
void linkx_quick_can_send(linkx_t *linkx, uint8_t ch, uint32_t id, uint8_t *data)
{
    linkx_send_8b(linkx, ch, id, false, false, false, data);
}

// 封装发送（RTR 帧）：经典 CAN + RTR 位置位
void linkx_quick_can_send_rtr(linkx_t *linkx, uint8_t ch, uint32_t id, uint8_t *data)
{
    linkx_send_8b(linkx, ch, id, false, false, true, data);
}

// 封装接收：内置硬件时间戳去重逻辑
bool linkx_quick_recv(linkx_t *linkx, uint8_t ch, can_msg_t *out_msg)
{
    if (!linkx || !out_msg || ch >= LINKX_CAN_CHANNEL_NUM)
        return false;

    can_tx_pdo_t *rx_pdo = linkx_recv_can(linkx, ch);
    if (rx_pdo == NULL)
        return false;

    uint8_t pdo_dlen = rx_pdo->params.dlen;
    if (pdo_dlen > LINKX_CAN_MAX_DATA_BYTES)
        pdo_dlen = LINKX_CAN_MAX_DATA_BYTES;

    if (linkx_snapshot_equals_pdo(&linkx->rx_dedup[ch], rx_pdo, pdo_dlen))
        return false;

    const uint8_t out_dlen = (pdo_dlen > sizeof(out_msg->data))
                             ? (uint8_t)sizeof(out_msg->data) : pdo_dlen;

    out_msg->id = rx_pdo->can_id;
    out_msg->timestamp = rx_pdo->timestamp;
    out_msg->dlen = out_dlen;

    memset(out_msg->data, 0, sizeof(out_msg->data));
    if (out_dlen > 0)
        memcpy(out_msg->data, rx_pdo->data_u32, out_dlen);

    linkx_snapshot_update(&linkx->rx_dedup[ch], rx_pdo, pdo_dlen);
    linkx->can_stats[ch].rx_frames++;
    linkx->can_stats[ch].rx_bytes += pdo_dlen;
    return true;
}

// 0x8002 配置项的单字节 SDO 写入封装
static int linkx_sdo_write_byte(linkx_t *linkx, uint16_t index, uint8_t sub, uint8_t val)
{
    return ecx_SDOwrite(linkx->master, linkx->slave_id, index, sub,
                        FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
}

// 配置指定 CAN 通道的波特率 (仲裁段和数据段)
bool linkx_set_can_baudrate(linkx_t *linkx, uint8_t ch, uint8_t fd_en,
                          uint8_t n_pre, uint8_t n_seg1, uint8_t n_seg2, uint8_t n_sjw,
                          uint8_t d_pre, uint8_t d_seg1, uint8_t d_seg2, uint8_t d_sjw)
{
    if (ch >= LINKX_CAN_CHANNEL_NUM) return false;

    const uint16_t index = 0x8002;  // 统一的时序配置索引
    int wkc = 0;

    printf("[LinkX] Configuring CAN %d Timings...\n", ch);

    wkc += linkx_sdo_write_byte(linkx, index, 0x01, ch);      // 通道号
    wkc += linkx_sdo_write_byte(linkx, index, 0x02, fd_en);   // 使能 CANFD
    // 仲裁段 (Nominal) 经典 CAN 波特率参数
    wkc += linkx_sdo_write_byte(linkx, index, 0x03, n_pre);
    wkc += linkx_sdo_write_byte(linkx, index, 0x04, n_seg1);
    wkc += linkx_sdo_write_byte(linkx, index, 0x05, n_seg2);
    wkc += linkx_sdo_write_byte(linkx, index, 0x06, n_sjw);
    // 数据段 (Data) CAN-FD 高速波特率参数
    wkc += linkx_sdo_write_byte(linkx, index, 0x07, d_pre);
    wkc += linkx_sdo_write_byte(linkx, index, 0x08, d_seg1);
    wkc += linkx_sdo_write_byte(linkx, index, 0x09, d_seg2);
    wkc += linkx_sdo_write_byte(linkx, index, 0x0A, d_sjw);
    wkc += linkx_sdo_write_byte(linkx, index, 0x0B, 1);       // 配置生效触发

    const bool ok = (wkc == 11);
    printf(ok ? "[LinkX] CAN Channel %d Timing configured successfully!\n"
              : "[LinkX] CAN Channel %d Timing config FAILED! (Success Count = %d/11)\n",
           ch, wkc);
    return ok;
}
