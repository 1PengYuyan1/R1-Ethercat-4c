#ifndef LINKX_HANDLER_H
#define LINKX_HANDLER_H

#include "linkx.h"
#include <stdint.h>

// CAN 消息结构体，简化接收数据的处理
typedef struct
{
    uint32_t id;
    uint8_t dlen;
    uint8_t data[8];
    uint64_t timestamp;
} can_msg_t;

#ifdef __cplusplus
extern "C"
{
#endif

    // 封装 SDO 唤醒逻辑，屏蔽 0x8001 寄存器细节
    bool linkx_hw_wakeup(linkx_t *linkx);
    // 封装发送：自动处理指针强转和 linkx 参数顺序
    void linkx_quick_FDcan_send(linkx_t *linkx, uint8_t ch, uint32_t id, uint8_t *data);
    // 封装发送：自动处理指针强转和 linkx 参数顺序
    void linkx_quick_can_send(linkx_t *linkx, uint8_t ch, uint32_t id, uint8_t *data);
    // 封装发送（RTR 帧）：用于 ODrive 等设备的数据请求
    void linkx_quick_can_send_rtr(linkx_t *linkx, uint8_t ch, uint32_t id, uint8_t *data);
    // 封装接收：内置硬件时间戳去重逻辑
    bool linkx_quick_recv(linkx_t *linkx, uint8_t ch, can_msg_t *out_msg);
    // 配置 CAN 波特率
    bool linkx_set_can_baudrate(linkx_t *linkx, uint8_t ch, uint8_t fd_en,
                                uint8_t n_pre, uint8_t n_seg1, uint8_t n_seg2, uint8_t n_sjw,
                                uint8_t d_pre, uint8_t d_seg1, uint8_t d_seg2, uint8_t d_sjw);

#ifdef __cplusplus
}
#endif

#endif