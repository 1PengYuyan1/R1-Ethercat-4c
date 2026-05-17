#ifndef ECAT_MANAGER_H
#define ECAT_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "soem.h"
#include <stdbool.h>

// EtherCAT 主站管理器，封装上下文、I/O 映射和运行状态
typedef struct {
    ecx_contextt ctx;
    uint8_t iomap[4096];
    int slave_count;
    volatile bool is_running;
} ecat_master_t;



// 初始化网卡并扫描从站
bool ecat_master_init(ecat_master_t *master, const char *ifname);
// 核心同步：发送并接收过程数据
void ecat_master_sync(ecat_master_t *master);
// 安全切换到 OP 状态（含看门狗预热逻辑）
bool ecat_master_bring_online(ecat_master_t *master);

#ifdef __cplusplus
}
#endif

#endif