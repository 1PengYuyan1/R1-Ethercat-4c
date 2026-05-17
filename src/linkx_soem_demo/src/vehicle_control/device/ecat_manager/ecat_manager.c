#include "ecat_manager.h"
#include <stdio.h>
#include "soem/soem.h"

// 单帧 PDO 泵浦：sync 与 bring_online 的共同基础动作
static inline void ecat_pump_pdo(ecat_master_t *master)
{
    ecx_send_processdata(&master->ctx);
    ecx_receive_processdata(&master->ctx, EC_TIMEOUTRET);
}

static void ecat_dump_slave_states(ecat_master_t *master)
{
    ecx_readstate(&master->ctx);
    for (int i = 1; i <= master->ctx.slavecount; i++)
    {
        uint16_t al = master->ctx.slavelist[i].ALstatuscode;
        printf("[ECAT] Slave %d state=0x%02x AL=0x%04X (%s)\n",
               i,
               master->ctx.slavelist[i].state,
               al,
               ec_ALstatuscode2string(al));
    }
}

// 初始化网卡并扫描从站
bool ecat_master_init(ecat_master_t *master, const char *ifname)
{
    printf("[ECAT] Initializing adapter %s...\n", ifname);
    if (!ecx_init(&master->ctx, ifname))
    {
        printf("[ERROR] Failed to init adapter %s\n", ifname);
        return false;
    }

    if (ecx_config_init(&master->ctx) <= 0)
    {
        printf("[ERROR] No slaves found on the bus!\n");
        return false;
    }

    printf("[ECAT] %d slaves found and configured.\n", master->ctx.slavecount);

    // 运行时按物理位置分配 Station Alias（LinkX EEPROM 不持久化，每次启动设一次）
    // slave[1] -> alias=1 (FD/夹爪), slave[2] -> alias=2 (经典/底盘)
    for (int i = 1; i <= master->ctx.slavecount; i++)
    {
        master->ctx.slavelist[i].aliasadr = (uint16_t)i;
        printf("[ECAT]   slave[%d] name='%s' vendor=0x%08X product=0x%08X alias=%u\n",
               i,
               master->ctx.slavelist[i].name,
               master->ctx.slavelist[i].eep_man,
               master->ctx.slavelist[i].eep_id,
               master->ctx.slavelist[i].aliasadr);
    }

    // 1. 映射 IO 数据 (告诉系统报文有多长)
    ecx_config_map_group(&master->ctx, master->iomap, 0);

    // DC 核心功能 ：对表与计算延迟
    // 必须放在 IO 映射之后，状态机检查之前！
    ecx_configdc(&master->ctx);

    // // DC 核心功能 ：激活从站的 Sync0 硬件心跳
    // // 假设你的控制周期是 1ms (1,000,000 纳秒)
    // // 偏移量 (Shift) 通常设置为周期的一半 (500,000 纳秒)，确保报文到达后，再触发动作
    // uint32_t cycle_time_ns = 1000000;
    // uint32_t shift_time_ns = cycle_time_ns / 2;
    //
    // for (int i = 1; i <= master->ctx.slavecount; i++) {
    //     // 激活第 i 个从站的 Sync0 中断
    //     ecx_dcsync0(&master->ctx, i, TRUE, cycle_time_ns, shift_time_ns);
    // }
    // printf("[ECAT] DC and Sync0 configured for %d slaves.\n", master->ctx.slavecount);

    // 检查从站状态
    int state = ecx_statecheck(&master->ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 2);
    if (state != EC_STATE_SAFE_OP)
    {
        printf("[WARN] Not all slaves reached SAFE_OP (state=0x%x)\n", state);
        ecat_dump_slave_states(master);
    }

    master->slave_count = master->ctx.slavecount;
    master->is_running = true;
    return true;
}

// 核心同步：发送并接收过程数据
void ecat_master_sync(ecat_master_t *master)
{
    ecat_pump_pdo(master);
}

// 安全切换到 OP 状态（含看门狗预热逻辑）
bool ecat_master_bring_online(ecat_master_t *master)
{
    printf("[ECAT] Requesting OP state for all slaves...\n");

    // 按 SOEM 推荐流程：先交换若干帧过程数据，避免 SAFE_OP->OP 时输出无效导致拒绝切换
    for (int i = 0; i < 5; i++)
    {
        ecat_pump_pdo(master);
        osal_usleep(1000);
    }

    master->ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&master->ctx, 0);

    int chk = 40;
    do
    {
        ecat_pump_pdo(master);
        osal_usleep(10000);
        ecx_statecheck(&master->ctx, 0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (master->ctx.slavelist[0].state != EC_STATE_OPERATIONAL));

    if (master->ctx.slavelist[0].state == EC_STATE_OPERATIONAL)
    {
        printf("[SUCCESS] All slaves reached OP state. System is ONLINE.\n");
        return true;
    }

    printf("[ERROR] Failed to reach OP state. Current state: 0x%x\n",
           master->ctx.slavelist[0].state);
    ecat_dump_slave_states(master);
    return false;
}
