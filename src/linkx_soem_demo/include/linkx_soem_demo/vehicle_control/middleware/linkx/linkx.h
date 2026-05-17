//
// Created by 93094 on 2026/3/12.
//

#ifndef LINKX_SOEM_DEMO_LINKX_H
#define LINKX_SOEM_DEMO_LINKX_H

#include <stdint.h>
#include <stdbool.h>

#define LINKX_CAN_CHANNEL_NUM 4
#define LINKX_CAN_MAX_DATA_BYTES 64
#define LINKX_TX_QUEUE_DEPTH 64

#pragma pack(push, 1)

// CAN 波特率设置结构体
typedef struct
{
    uint8_t channel;
    uint8_t enable_canfd;
    uint8_t nominal_prescaler;
    uint8_t nominal_seg1;
    uint8_t nominal_seg2;
    uint8_t nominal_sjw;
    uint8_t data_prescaler;
    uint8_t data_seg1;
    uint8_t data_seg2;
    uint8_t data_sjw;

} can_baudrate_setting_t;

// CAN PDO 参数结构体
typedef struct
{
    uint16_t ext : 1;
    uint16_t rtr : 1;
    uint16_t canfd : 1;
    uint16_t brs : 1;
    uint16_t reserved : 4;
    uint16_t dlen : 8;
} can_pdo_param_t;

// CAN 接收 PDO 结构体
typedef struct
{
    uint32_t can_id;
    can_pdo_param_t params;
    uint32_t data_u32[16];
} can_rx_pdo_t;

// CAN 发送 PDO 结构体，带时间戳
typedef struct
{
    uint32_t can_id;
    can_pdo_param_t params;
    uint64_t timestamp;
    uint32_t data_u32[16];

} can_tx_pdo_t;

#pragma pack(pop)

// LinkX 设备结构体
typedef struct
{
    uint64_t tx_frames;
    uint64_t tx_bytes;
    uint64_t tx_enqueued_frames;
    uint64_t tx_enqueued_bytes;
    uint64_t tx_dropped_frames;
    uint64_t rx_frames;
    uint64_t rx_bytes;
} linkx_can_stats_t;

typedef struct
{
    uint32_t can_id;
    can_pdo_param_t params;
    uint8_t data[LINKX_CAN_MAX_DATA_BYTES];
} linkx_tx_frame_t;

typedef struct
{
    linkx_tx_frame_t frames[LINKX_TX_QUEUE_DEPTH];
    uint16_t head;
    uint16_t tail;
    uint16_t size;
} linkx_tx_queue_t;

// 接收去重快照（每通道一份），用于 linkx_quick_recv 比对硬件时间戳/参数变化
typedef struct
{
    bool valid;
    uint32_t can_id;
    uint16_t params_raw;
    uint64_t timestamp;
    uint8_t dlen;
    uint8_t data[LINKX_CAN_MAX_DATA_BYTES];
} linkx_rx_snapshot_t;

typedef struct
{
    uint32_t slave_id; // 从站 ID

    can_baudrate_setting_t channel_baudrates[LINKX_CAN_CHANNEL_NUM];

    can_tx_pdo_t tx_pdos[LINKX_CAN_CHANNEL_NUM]; // 发送 PDO 缓存
    can_rx_pdo_t rx_pdos[LINKX_CAN_CHANNEL_NUM]; // 接收 PDO 缓存
    linkx_tx_queue_t tx_queues[LINKX_CAN_CHANNEL_NUM]; // 发送队列（防单槽覆盖）
    linkx_can_stats_t can_stats[LINKX_CAN_CHANNEL_NUM]; // CAN 收发统计
    linkx_rx_snapshot_t rx_dedup[LINKX_CAN_CHANNEL_NUM]; // 接收去重快照（按实例隔离）

    struct ecx_context *master; // 主站上下文
    struct ec_slave *slave;     // 从站信息

} linkx_t;

#ifdef __cplusplus
extern "C"
{
#endif

    void linkx_init(linkx_t *linkx, uint32_t slave_id, struct ecx_context *master);
    bool linkx_start(linkx_t *linkx);
    bool linkx_stop(linkx_t *linkx);
    bool linkx_read_baudrate(linkx_t *linkx, uint8_t channel);
    bool linkx_write_baudrate(linkx_t *linkx, uint8_t channel);
    bool linkx_switch_can_channel(linkx_t *linkx, uint8_t channel, bool enable);
    void linkx_recv_pdos(linkx_t *linkx);
    void linkx_send_pdos(linkx_t *linkx);
    bool linkx_send_can(linkx_t *linkx, uint8_t channel, uint32_t canid, bool canfd, bool brs, bool ext, bool rtr, uint8_t dlen, uint32_t *data);
    can_tx_pdo_t *linkx_recv_can(linkx_t *linkx, uint8_t channel);
    char *linkx_get_error_string(linkx_t *linkx);

#ifdef __cplusplus
}
#endif

#endif // LINKX_SOEM_DEMO_LINKX_H
