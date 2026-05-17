//
// Created by 93094 on 2026/3/12.
//

#include "linkx.h"
#include "soem.h"
#include <string.h>

static bool linkx_tx_queue_match_key(const linkx_tx_frame_t *a, const linkx_tx_frame_t *b)
{
    return (a->can_id == b->can_id) &&
           (a->params.ext == b->params.ext) &&
           (a->params.rtr == b->params.rtr) &&
           (a->params.canfd == b->params.canfd);
}

static bool linkx_tx_queue_push_or_update(linkx_tx_queue_t *queue,
                                          const linkx_tx_frame_t *frame,
                                          bool *dropped_oldest)
{
    if (dropped_oldest)
        *dropped_oldest = false;

    for (uint16_t i = 0; i < queue->size; ++i)
    {
        uint16_t idx = (uint16_t)((queue->head + i) % LINKX_TX_QUEUE_DEPTH);
        if (linkx_tx_queue_match_key(&queue->frames[idx], frame))
        {
            queue->frames[idx] = *frame;
            return true;
        }
    }

    if (queue->size >= LINKX_TX_QUEUE_DEPTH)
    {
        queue->head = (uint16_t)((queue->head + 1U) % LINKX_TX_QUEUE_DEPTH);
        queue->size--;
        if (dropped_oldest)
            *dropped_oldest = true;
    }

    queue->frames[queue->tail] = *frame;
    queue->tail = (uint16_t)((queue->tail + 1U) % LINKX_TX_QUEUE_DEPTH);
    queue->size++;
    return true;
}

static bool linkx_tx_queue_pop(linkx_tx_queue_t *queue, linkx_tx_frame_t *frame)
{
    if (queue->size == 0)
        return false;

    *frame = queue->frames[queue->head];
    queue->head = (uint16_t)((queue->head + 1U) % LINKX_TX_QUEUE_DEPTH);
    queue->size--;
    return true;
}

static void linkx_fill_rx_pdo(can_rx_pdo_t *pdo, const linkx_tx_frame_t *frame)
{
    uint8_t payload_len = frame->params.dlen;
    if (payload_len > LINKX_CAN_MAX_DATA_BYTES)
        payload_len = LINKX_CAN_MAX_DATA_BYTES;

    memset(pdo, 0, sizeof(*pdo));
    pdo->can_id = frame->can_id;
    pdo->params = frame->params;
    if (payload_len > 0)
        memcpy(pdo->data_u32, frame->data, payload_len);
}

// 初始化 LinkX 设备实例，关联 EtherCAT 主站上下文和从站信息
void linkx_init(linkx_t *linkx, uint32_t slave_id, struct ecx_context *master)
{
    linkx->slave_id = slave_id;
    linkx->master = master;
    linkx->slave = &master->slavelist[slave_id];
    memset(linkx->channel_baudrates, 0, sizeof(linkx->channel_baudrates));
    memset(linkx->tx_pdos, 0, sizeof(linkx->tx_pdos));
    memset(linkx->rx_pdos, 0, sizeof(linkx->rx_pdos));
    memset(linkx->tx_queues, 0, sizeof(linkx->tx_queues));
    memset(linkx->can_stats, 0, sizeof(linkx->can_stats));
    memset(linkx->rx_dedup, 0, sizeof(linkx->rx_dedup));
}

// 封装状态切换逻辑，屏蔽 EtherCAT 状态机细节
static bool linkx_set_state(linkx_t *linkx, ec_state request_state)
{
    if (!linkx->master)
        return false;

    linkx->slave->state = request_state;
    ecx_writestate(linkx->master, linkx->slave_id);

    ecx_readstate(linkx->master);

    if (linkx->slave->state == request_state)
        return true;
    return false;
}

// 控制 EtherCAT 从站进入操作模式，底层通过 SDO 写 0x8001 寄存器实现
bool linkx_start(linkx_t *linkx)
{
    return linkx_set_state(linkx, EC_STATE_OPERATIONAL);
}

// 控制 EtherCAT 从站进入安全操作模式，底层通过 SDO 写 0x8001 寄存器实现
bool linkx_stop(linkx_t *linkx)
{
    return linkx_set_state(linkx, EC_STATE_SAFE_OP);
}

// 从 EtherCAT 从站读取 CAN 波特率配置，底层通过 SDO 读 0x8003 寄存器实现
bool linkx_read_baudrate(linkx_t *linkx, uint8_t channel)
{
    if (channel >= LINKX_CAN_CHANNEL_NUM)
        return false;

    /* switch to safe op mode */
    if (!linkx_set_state(linkx, EC_STATE_SAFE_OP))
        return false;

    /* write channel & config_read */
    int config_read = 1;
    ecx_SDOwrite(linkx->master, linkx->slave_id, 0x8003, 1, false, 1, &channel, 300);
    ecx_SDOwrite(linkx->master, linkx->slave_id, 0x8003, 11, false, 1, &config_read, 300);

    /* sdo read index 0x8003 ob */
    int read_size = 0;
    uint8_t subitem_cnt = 0;
    uint8_t read_bytes[sizeof(can_baudrate_setting_t)] = {0};

    /* get subitem count */
    ecx_SDOread(linkx->master, linkx->slave_id, 0x8003, 0, false, &read_size, &subitem_cnt, 300);

    if (read_size == 0 || subitem_cnt == 0)
        return false;

    for (int i = 1; i < subitem_cnt - 1; i++)
    {
        ecx_SDOread(linkx->master, linkx->slave_id, 0x8003, i, false, &read_size, read_bytes + i - 1, 300);
        if (read_size == 0)
            return false;
    }

    memcpy(&linkx->channel_baudrates[channel], read_bytes, sizeof(can_baudrate_setting_t));
    return true;
}


// 封装发送：FDCAN自动处理指针强转和 linkx 参数顺序
bool linkx_write_baudrate(linkx_t *linkx, uint8_t channel)
{
    if (channel >= LINKX_CAN_CHANNEL_NUM)
        return false;

    /* switch to safe op mode */
    if (!linkx_set_state(linkx, EC_STATE_SAFE_OP))
        return false;

    /* sdo read index 0x8003 ob */
    int read_size = 0;
    uint8_t subitem_cnt = 0;
    uint8_t read_bytes[sizeof(can_baudrate_setting_t)] = {0};

    /* get subitem count */
    ecx_SDOread(linkx->master, linkx->slave_id, 0x8002, 0, false, &read_size, &subitem_cnt, 300);
    if (read_size == 0 || subitem_cnt == 0)
        return false;

    for (int i = 0; i < subitem_cnt; i++)
    {
        ecx_SDOwrite(linkx->master, linkx->slave_id, 0x8002, i + 1, false, 1, read_bytes + i, 300);
    }

    return true;
}

// 控制 CAN 通道使能，底层通过 SDO 写 0x8001 寄存器实现
bool linkx_switch_can_channel(linkx_t *linkx, uint8_t channel, bool enable)
{
    if (channel >= LINKX_CAN_CHANNEL_NUM)
        return false;
    int wkc = ecx_SDOwrite(linkx->master, linkx->slave_id, 0x8001, channel + 1, false, 1, &enable, EC_TIMEOUTRXM);
    return (wkc > 0);
}

// 从 EtherCAT 从站输入缓冲区读取接收 PDO 数据，更新 linkx->tx_pdos 供外部读取
void linkx_recv_pdos(linkx_t *linkx)
{
    if (!linkx || !linkx->slave)
        return;

    /* receive pdos from slave  pData=master inputs*/
    for (int i = 0; i < LINKX_CAN_CHANNEL_NUM; i++)
    {
        // memcpy(&linkx->rx_pdos[i], linkx->slave->inputs + i * sizeof(can_rx_pdo_t), sizeof(can_rx_pdo_t));
        memcpy(&linkx->tx_pdos[i], linkx->slave->inputs + i * sizeof(can_tx_pdo_t), sizeof(can_tx_pdo_t));
    }
}

// 将主站输出数据（发送 PDO）写回从站的输出缓冲区，供 EtherCAT 主站周期性发送
void linkx_send_pdos(linkx_t *linkx)
{
    if (!linkx || !linkx->slave)
        return;

    /* transmit pdos to slave pData=master outputs*/
    for (int i = 0; i < LINKX_CAN_CHANNEL_NUM; i++)
    {
        linkx_tx_frame_t frame = {0};
        if (linkx_tx_queue_pop(&linkx->tx_queues[i], &frame))
        {
            linkx_fill_rx_pdo(&linkx->rx_pdos[i], &frame);
            linkx->can_stats[i].tx_frames++;
            linkx->can_stats[i].tx_bytes += frame.params.dlen;
        }

        // memcpy(linkx->slave->outputs + i * sizeof(can_tx_pdo_t), &linkx->tx_pdos[i], sizeof(can_tx_pdo_t));
        memcpy(linkx->slave->outputs + i * sizeof(can_rx_pdo_t), &linkx->rx_pdos[i], sizeof(can_rx_pdo_t));
    }
}

// 发送 CAN 数据，参数详见 linkx_send_can 定义
bool linkx_send_can(linkx_t *linkx, uint8_t channel, uint32_t canid, bool canfd, bool brs, bool ext, bool rtr, uint8_t dlen, uint32_t *data)
{
    if (!linkx || channel >= LINKX_CAN_CHANNEL_NUM)
        return false;

    if (dlen > LINKX_CAN_MAX_DATA_BYTES)
        dlen = LINKX_CAN_MAX_DATA_BYTES;

    if (dlen > 0 && data == NULL)
        return false;

    linkx_tx_frame_t frame = {0};
    frame.can_id = canid;
    frame.params.canfd = canfd;
    frame.params.brs = brs;
    frame.params.ext = ext;
    frame.params.rtr = rtr;
    frame.params.dlen = dlen;

    if (dlen > 0)
        memcpy(frame.data, (const uint8_t *)data, dlen);

    bool dropped_oldest = false;
    linkx_tx_queue_push_or_update(&linkx->tx_queues[channel], &frame, &dropped_oldest);
    if (dropped_oldest)
        linkx->can_stats[channel].tx_dropped_frames++;

    linkx->can_stats[channel].tx_enqueued_frames++;
    linkx->can_stats[channel].tx_enqueued_bytes += dlen;
    return true;
}

// 获取指向接收 PDO 的指针，供外部读取
can_tx_pdo_t *linkx_recv_can(linkx_t *linkx, uint8_t channel)
{
    if (channel >= LINKX_CAN_CHANNEL_NUM)
        return NULL;
    return &linkx->tx_pdos[channel];
}

// 获取 LinkX 错误字符串
char *linkx_get_error_string(linkx_t *linkx)
{
    if (!linkx->slave)
        return "slave is null";
    return ec_ALstatuscode2string(linkx->slave->ALstatuscode);
}
