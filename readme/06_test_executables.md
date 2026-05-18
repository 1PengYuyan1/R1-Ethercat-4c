# 06 · 测试与诊断可执行程序

`linkx_soem_demo` 包除主可执行 `linkx_soem_demo` / `remote_node_cpp` 外，还编出多个独立 main，按用途分类。

## 1. 总览

| 可执行 | 源文件 | 用途 | 是否需要硬件 |
| --- | --- | --- | --- |
| `linkx_soem_demo` | `vehicle_control/main.cpp` | 主程序入口（EtherCAT 1ms 主循环） | ✅ |
| `remote_node_cpp` | `remote/remote_node.cpp` | 手柄 → ROS2 话题 | 仅手柄 |
| `can_link_test` | `test_mains/can_link_test_main.cpp` | EtherCAT-4C 4 路 CAN 发包能力验证 | ✅ |
| `canfd_link_test` | `test_mains/canfd_link_test_main.cpp` | 双 LinkX (CAN-FD + classic) alias 绑定与 FD 发包验证 | ✅ |
| `ops_test` | `test_mains/ops_test_main.cpp` | OPS-9 接收解码冒烟测试 | ✅ |
| `steer_tuning` | `test_mains/steer_tuning_main.cpp` | 舵向 PD / 力矩前馈调参（独立运行） | ✅ |
| `motor_calib` | `test_mains/motor_calib_main.cpp` | DM 舵向电机参数标定 | ✅（轮胎离地） |
| `odrive_calib` | `test_mains/odrive_calib_main.cpp` | ODrive 驱动轮参数标定 | ✅（轮胎离地） |
| `odrive_protocol_test` | `test_mains/odrive_protocol_test_main.cpp` | CAN Simple 协议函数级冒烟 | ✅ |
| `encoder_ack_test` | `test_mains/encoder_ack_test_main.cpp` | 4 路 BRT 编码器 ACK 验证 | ✅ |
| `enc3_recover` | `test_mains/enc3_recover_main.cpp` | 单独抢救 `enc3`（卡 ~19/30Hz 时执行复位序列） | ✅ |
| `enc3_set_period` | `test_mains/enc3_set_period_main.cpp` | 单独给 `enc3` 改自发周期 | ✅ |
| `linkx_set_alias` | `test_mains/linkx_set_alias_main.cpp` | 给 LinkX 写 alias（EEPROM）—— **不持久**，每次上电要重写 | ✅ |
| `encoder_byteorder_test` | `tests/test_brt_encoder.cpp` | BRT 字节序 + ACK 解析（dry-run，**无需硬件**） | ❌ |
| `robot_test` | `test_mains/robot_test_main.cpp` | `Class_Robot` 上机回归测试，限速短转 | ✅ |

所有需要硬件的可执行都通过 EtherCAT，必须 `sudo` 或可执行文件已 `setcap`。

## 2. CAN 链路冒烟：`can_link_test`

仅验证 EtherCAT-4C 的 4 路 CAN 发包能力，不依赖电机/编码器逻辑。

```bash
sudo ./install/linkx_soem_demo/lib/linkx_soem_demo/can_link_test enp86s0
```

## 3. OPS-9 解码：`ops_test`

通道 3 收 CAN，喂给 `Class_OPS` 解码后 1 Hz 打印。

```bash
sudo IFNAME=enp86s0 \
    ./build/linkx_soem_demo/ops_test [enp86s0]
```

## 4. 舵向调参：`steer_tuning`

MIT Kp/Kd + 外层位置环 + 力矩前馈，**不依赖 ROS2 / 遥控器**，独立运行。
推荐通过封装脚本调用：

```bash
sudo ./tools/run_steer_step_test.sh
sudo ./tools/run_step_kd_sweep.sh
sudo ./tools/run_remaining_tuning.sh
```

输出 CSV 进 `var_data/`，对应绘图脚本见 [05_calibration.md §4.2](05_calibration.md)。

## 5. 电机标定：`motor_calib` / `odrive_calib`

详见 [05_calibration.md](05_calibration.md)。

## 6. ODrive 协议级冒烟：`odrive_protocol_test`

逐函数下发 CAN 帧并验证回包，定位"哪个 CAN Simple 命令出问题"。

```bash
sudo IFNAME=enp86s0 \
    ./install/linkx_soem_demo/lib/linkx_soem_demo/odrive_protocol_test --wheel 0
```

## 7. BRT 编码器 ACK 验证：`encoder_ack_test`

复用 motor_calib 的 EtherCAT 启动流程，对每个编码器逐一发**非破坏性** set 指令，等待 ACK 并断言 `status == 0x00`。

主要用途：

- 大端字节序改动是否被硬件接受
- `Data_Process` 对 `0x02..0x0F` ACK 路由是否在真实数据上工作

```bash
sudo IFNAME=enp86s0 ./install/linkx_soem_demo/lib/linkx_soem_demo/encoder_ack_test
```

## 8. BRT 字节序单元测试：`encoder_byteorder_test`

通过 stub `linkx_quick_can_send` 验证 set 指令字节序符合手册示例 + `0x02..0x0F` set ACK 路由。
**无需硬件**，干跑可过。

```bash
./build/linkx_soem_demo/encoder_byteorder_test
```

## 9. Robot 上机回归：`robot_test`

顺序自动跑全套测试，限速短转，自动停。

```bash
sudo ./build/linkx_soem_demo/robot_test enp86s0
```

注意：会真实驱动车体（限速），现场需保证安全。

## 10. CAN-FD 双从站验证：`canfd_link_test`

2026-05-11 集成双 LinkX 后用的冒烟工具。`task.cpp` 用 alias-based binding 区分：
`slave[1] = FD`（alias=1）、`slave[2] = classic`（alias=2）。本测试只验证 FD 从站能
正确发包并回 ACK。

```bash
sudo ./install/linkx_soem_demo/lib/linkx_soem_demo/canfd_link_test enp86s0
```

## 11. enc3 抢救：`enc3_recover` / `enc3_set_period`

某些硬件上 `enc3` 自发频率会被锁死在 ~19 或 ~30 Hz（BRT 编码器固件警告锁定，
不是软件 bug）。这两个工具是单独操作 `enc3` 用的复位 / 设置周期 fallback。
日常不需要跑；只有当 `[ENC] enc3 freq=...` 长期偏低时才用。

```bash
sudo ./install/linkx_soem_demo/lib/linkx_soem_demo/enc3_recover
sudo ./install/linkx_soem_demo/lib/linkx_soem_demo/enc3_set_period
```

## 12. LinkX alias 写入：`linkx_set_alias`

给 LinkX 从站写 EEPROM alias，用于 alias-based binding。

```bash
sudo ./install/linkx_soem_demo/lib/linkx_soem_demo/linkx_set_alias <ifname> <slave_pos> <alias>
```

> ⚠️ 实测 2026-05-11 起的 LinkX 固件 **alias 不持久**：写完上电就丢。
> 当前规避方法是每次上电后跑一次这个工具补写。后续如固件更新需要复测。
