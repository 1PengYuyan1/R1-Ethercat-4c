/**
 * @file linkx_set_alias_main.cpp
 * @brief 一次性给两片 LinkX 写 EtherCAT Station Alias，从此 slave_id 不再依赖物理串接顺序。
 *
 *  约定：FD 那片(挂夹爪)            alias = 1
 *        经典那片(挂底盘 DM/ODrive/Encoder)  alias = 2
 *
 *  用法（必须 sudo / setcap）：
 *    show:                   sudo ./linkx_set_alias enp86s0 show
 *    给指定 slave 写 alias:   sudo ./linkx_set_alias enp86s0 set <slave_idx> <alias>
 *    一次性写当前接线:       sudo ./linkx_set_alias enp86s0 auto <slave1_alias> <slave2_alias>
 *
 *  ⚠ 写完必须给 LinkX 断电再上电（拔 XT30 等 1-2 秒再插），否则 aliasadr 不生效。
 */

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string>
#include "soem.h"
#include "ec_main.h"
#include "ec_options.h"

// EtherCAT EEPROM word address for "Configured Station Alias" (per ETG.1000.6 spec)
static constexpr uint16_t kEepromWordAlias = 0x0004;

// 大块 SOEM 上下文 + iomap 放堆（栈太小放不下 iomap）
struct AliasCtx {
    ecx_contextt ctx;
    uint8_t      iomap[4096];
};

static void usage(const char *p)
{
    fprintf(stderr,
        "Usage:\n"
        "  %s <ifname> show\n"
        "  %s <ifname> set <slave_idx> <alias>\n"
        "  %s <ifname> auto <slave1_alias> <slave2_alias>\n"
        "\n"
        "Convention: FD module -> alias=1, classic module -> alias=2\n"
        "After set, POWER-CYCLE the LinkX modules for alias to take effect.\n",
        p, p, p);
}

static bool ec_init_and_scan(ecx_contextt *ctx, const char *ifname)
{
    printf("[ALIAS] Initializing adapter %s...\n", ifname);
    if (!ecx_init(ctx, ifname))
    {
        fprintf(stderr, "[ALIAS][ERROR] ecx_init failed on %s\n", ifname);
        return false;
    }
    if (ecx_config_init(ctx) <= 0)
    {
        fprintf(stderr, "[ALIAS][ERROR] No slaves found.\n");
        return false;
    }
    printf("[ALIAS] %d slaves found.\n", ctx->slavecount);
    return true;
}

static void show_aliases(ecx_contextt *ctx)
{
    for (int i = 1; i <= ctx->slavecount; ++i)
    {
        // slavelist[].aliasadr 是启动 cache，再读一次 EEPROM 拿真实最新值
        uint32_t eep = ecx_readeeprom(ctx, (uint16_t)i, kEepromWordAlias, EC_TIMEOUTEEP);
        uint16_t alias_eep = (uint16_t)(eep & 0xFFFF);
        printf("[ALIAS] slave[%d] name='%s' aliasadr_cache=0x%04X eeprom_alias=0x%04X\n",
               i, ctx->slavelist[i].name,
               ctx->slavelist[i].aliasadr,
               alias_eep);
    }
}

static bool write_alias(ecx_contextt *ctx, int slave_idx, uint16_t alias)
{
    if (slave_idx < 1 || slave_idx > ctx->slavecount)
    {
        fprintf(stderr, "[ALIAS][ERROR] slave_idx %d out of range [1, %d]\n",
                slave_idx, ctx->slavecount);
        return false;
    }
    printf("[ALIAS] writing slave[%d] alias = 0x%04X (%u) ...\n",
           slave_idx, alias, alias);
    int wkc = ecx_writeeeprom(ctx, (uint16_t)slave_idx,
                              kEepromWordAlias, alias, EC_TIMEOUTEEP);
    if (wkc <= 0)
    {
        fprintf(stderr, "[ALIAS][ERROR] ecx_writeeeprom wkc=%d (failed)\n", wkc);
        return false;
    }
    uint32_t eep = ecx_readeeprom(ctx, (uint16_t)slave_idx,
                                  kEepromWordAlias, EC_TIMEOUTEEP);
    uint16_t got = (uint16_t)(eep & 0xFFFF);
    if (got != alias)
    {
        fprintf(stderr, "[ALIAS][ERROR] readback mismatch: wrote 0x%04X got 0x%04X\n",
                alias, got);
        return false;
    }
    printf("[ALIAS] slave[%d] EEPROM verified = 0x%04X.\n", slave_idx, got);
    return true;
}

int main(int argc, char *argv[])
{
    if (argc < 3) { usage(argv[0]); return 1; }

    const char *ifname = argv[1];
    const std::string op = argv[2];

    AliasCtx ac;
    std::memset(&ac, 0, sizeof(ac));
    ecx_contextt *ctx = &ac.ctx;

    if (!ec_init_and_scan(ctx, ifname))
        return 2;

    int rc = 0;
    if (op == "show")
    {
        show_aliases(ctx);
    }
    else if (op == "set" && argc >= 5)
    {
        int slave_idx = std::atoi(argv[3]);
        long alias_int = std::atol(argv[4]);
        if (alias_int < 0 || alias_int > 0xFFFF)
        {
            fprintf(stderr, "[ALIAS][ERROR] alias %ld out of uint16 range\n", alias_int);
            rc = 1;
        }
        else
        {
            if (!write_alias(ctx, slave_idx, (uint16_t)alias_int)) rc = 3;
            printf("\n[ALIAS] Final state:\n");
            show_aliases(ctx);
        }
    }
    else if (op == "auto" && argc >= 5)
    {
        if (ctx->slavecount != 2)
        {
            fprintf(stderr, "[ALIAS][ERROR] auto mode expects exactly 2 slaves, got %d\n",
                    ctx->slavecount);
            rc = 4;
        }
        else
        {
            int a1 = std::atoi(argv[3]);
            int a2 = std::atoi(argv[4]);
            if (write_alias(ctx, 1, (uint16_t)a1) &&
                write_alias(ctx, 2, (uint16_t)a2))
            {
                printf("\n[ALIAS] Final state:\n");
                show_aliases(ctx);
            }
            else
            {
                rc = 3;
            }
        }
    }
    else
    {
        usage(argv[0]);
        rc = 1;
    }

    if (rc == 0 && (op == "set" || op == "auto"))
    {
        // 尝试通过 CoE 0x1010 Store Parameters 让固件把 EEPROM 保存到 Flash
        printf("\n[ALIAS] Attempting CoE Store Parameters (0x1010) ...\n");
        ecx_config_map_group(ctx, ac.iomap, 0);
        ecx_statecheck(ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

        uint32_t save_sig = 0x65766173; // "save" ASCII little-endian
        for (int i = 1; i <= ctx->slavecount; ++i)
        {
            int sz = sizeof(save_sig);
            int wkc = ecx_SDOwrite(ctx, (uint16_t)i, 0x1010, 0x01,
                                   FALSE, sz, &save_sig, EC_TIMEOUTRXM);
            printf("[ALIAS] slave[%d] SDO 0x1010:01 store wkc=%d %s\n",
                   i, wkc, (wkc > 0) ? "OK" : "(not supported or failed)");
        }

        printf("\n⚠ POWER-CYCLE the LinkX modules now (拔 XT30 1-2 秒再插)。\n");
        printf("   Aliases are loaded from EEPROM at slave power-on, no soft-reload.\n");
    }

    ecx_close(ctx);
    return rc;
}
