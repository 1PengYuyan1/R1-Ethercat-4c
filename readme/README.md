# Ethercat_R1

> 基于 EtherCAT + SOEM 的四舵轮全向底盘控制系统（ROS 2 Humble）

本仓库实现：手柄遥控 → ROS 2 节点 → EtherCAT 主站 → EtherCAT-4C 网关 → 4 路 CAN 总线 → 4×ODrive 驱动轮 + 4×DM 舵向电机 + 4×BRT 多圈绝对编码器 + OPS-9 里程计。

主进程 1 ms 周期闭环，关键特性：

- **舵向编码器虚拟无限位移 + 掉电恢复**：断电后无需重新找零即可恢复角度参考
- **舵向 DM 内置 MIT 位置环**：mit_kp=50 / kd=1.5 + 软件 slew 500°/s + 滞回，60°~90° step 800ms settled、±0.1° 静差
- **drive_mode 双档 profile**：`MANUAL`（slew 500°/s、LPF α=0.3，遥控用）/ `SEMI_AUTO`（slew 200°/s、α=0.15，路径跟随用）由 auto_pilot 自动切换
- **Auto_Pilot 拐角软化**：`V = seg_speed × cos²(Δθ/2)` + smoothstep 进出窗口，方向 lerp 与速度共用同一权重，避免拐角 steer 跟不上
- **4 轮速度标定**：编译期 `kWheelSpeedCalib[4]` 在 `Set_target_omega` 边界一次乘进，消除 4 轮稳态 ~1.5% 偏差导致的前进 yaw 漂
- **F710 trigger 速度缩放**：RT 加速 / LT 减速，`scale = 0.65 + (rt-lt)×0.35`，钳位 `[0.30, 1.00]`

---

## 文档导航

| 文件 | 内容 |
| --- | --- |
| [01_structure.md](01_structure.md) | 工作区与代码目录结构、各模块职责 |
| [02_quickstart.md](02_quickstart.md) | 最小可执行步骤（编译 + 启动） |
| [03_run_guide.md](03_run_guide.md) | 完整运行指南、启动参数、注意事项 |
| [04_steer_encoder.md](04_steer_encoder.md) | 舵向编码器虚拟无限位移 + 掉电恢复 |
| [05_calibration.md](05_calibration.md) | 电机/底盘标定流程 |
| [06_test_executables.md](06_test_executables.md) | 各类测试可执行程序 |
| [07_tools.md](07_tools.md) | 调试脚本与波形/曲线绘制 |
| [08_troubleshooting.md](08_troubleshooting.md) | 常见故障排查 |

---

## 工程一览

```text
Ethercat_R1/                       工作空间根（colcon workspace）
├── src/
│   ├── linkx_soem_demo/           车辆主控（C++）：EtherCAT 主站 + 控制算法 + ROS2 节点
│   │   └── src/vehicle_control/
│   │       ├── chariot/
│   │       │   ├── chassis/       4 舵轮运动学 + 编码器恢复 + 舵向校准（已抽离）
│   │       │   ├── auto_pilot/    路径跟随 + 拐角软化 + drive_mode 切换
│   │       │   └── clamp/         夹爪
│   │       ├── device/            Motor / Encoder / OPS / ecat_manager / linkx4c_handler
│   │       ├── interaction/       robot.cpp：子系统聚合 + ROS2 桥
│   │       └── task/              task.cpp：1ms/2ms/100ms 周期调度
│   └── linkx_bringup/             启动编排：joy / remote / vehicle_control 串起来
├── tools/                         调试脚本（标定、扫参、波形打印）
├── instructions/                  历史实施记录（标定卡死修复、编码器恢复步骤）
├── var_data/                      运行时持久化数据 + 曲线 CSV/PNG + 分析脚本
├── readme/                        本说明文档目录
├── start_full_system.sh           一键启动脚本（推荐入口）
├── build/ install/ log/           colcon 自动生成
└── .claude/                       Claude Code 工作目录（settings/memory）
```

## 系统组成

### 软件栈

- **OS**：Ubuntu + ROS 2 Humble
- **EtherCAT 主站**：SOEM（包内自带 `middleware/soem`），原始以太网，需要 `cap_net_raw` 或 `sudo`
- **网关**：EtherCAT-4C，4 路 CAN（0/1/2/3）
- **CAN 协议**：
  - ODrive：CAN Simple（0..3 节点 ID）
  - DM 电机：MIT 力矩控制
  - BRT 编码器：自定义 0x01..0x0F 指令集
  - OPS-9：通道 3 接收

### 节点拓扑（`linkx_bringup/full_system.launch.py`）

```
joy_node  →  /joy
              ↓
remote_node_cpp  →  /cmd_vel, /robot_buttons
  ├─ XInput: RT/LT trigger 双键合成 scale ∈ [0.30, 1.00]
  └─ max_speed 单点权威源（默认 declare_parameter，launch/脚本不再覆盖）
                          ↓
                  vehicle_control（linkx_soem_demo）
                  ├─ 1ms EtherCAT 周期主循环
                  ├─ 底盘解算（4 舵轮）+ drive_mode profile 切换
                  ├─ Auto_Pilot 路径跟随（cos²(Δθ/2) 拐角软化）
                  ├─ ODrive / DM / Encoder 驱动
                  └─ 持久化（var_data/*.txt）
```

## 快速开始（最简）

```bash
cd /home/<你>/code/Ethercat_R1
./start_full_system.sh --sudo --ifname enp86s0
```

详见 [02_quickstart.md](02_quickstart.md)。

## 关键约束（先看再用）

1. **EtherCAT 网卡必须独立专用**，不要混跑普通网络业务。
2. **首次跑前需要标定**：舵向找零（`CAPTURE_STEER_ZERO=1`）+ DM 电机参数（`tools/run_motor_calib.sh`）+ ODrive 参数 + 4 轮速度标定 `kWheelSpeedCalib`（详见 [05_calibration.md](05_calibration.md)）。
3. **断电恢复依赖 `var_data/steer_zero_offsets.txt` + `var_data/steer_unwrapped_pulses.txt`**，删除将丢失角度基准。
4. **权限模型**：要么 `--sudo`，要么 `setcap 'cap_net_raw,cap_net_admin+ep' ...`。`/etc/sudoers.d/linkx-setcap` 已配 NOPASSWD，启动脚本会自动给真实 `build/` 二进制 `setcap`。
5. **`var_data/*.txt` 文件 owner**：跑过 `sudo` 后会归 root，下次非 sudo 跑会报 `[CHASSIS][PERSIST-FAIL]`。切模式前 `chown` 一次。
6. **速度默认值只剩一处**：`src/linkx_soem_demo/src/remote/remote_node.cpp` 的 `declare_parameter("max_speed", ...)`，launch 与启动脚本不再传 max_speed。改默认速度必须改 C++ 代码 + rebuild。
7. **手动模式无航向纠偏**：cmd_omega 直接透传；稳态 yaw 漂靠 4 轮速度标定在底盘端吸收。半自动 / Auto_Pilot 路径才接 OPS。
