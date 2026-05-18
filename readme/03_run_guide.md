# 03 · 完整运行指南

## 1. 推荐入口：`start_full_system.sh`

```bash
cd /home/<你>/code/Ethercat_R1
./start_full_system.sh --sudo --ifname enp86s0
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--ifname <name>` | `enp86s0` | EtherCAT 网卡名 |
| `--auto-ifname` | 关 | 自动选第一块 `en*`/`eth*` 有线网卡 |
| `--sudo` | 关 | 给车体主控前缀 `sudo -E env ...` 透传 ROS 环境 |

> 2026-05-18 起脚本不再接受 `--max-speed`。最大线速度由 `remote_node.cpp::declare_parameter`
> 单点决定；改默认值动 C++，运行时改用 `ros2 param set /remote_node max_speed <v>`。

脚本流程：

1. 重置 `var_data/live_variables.log`
2. 检查 `/opt/ros/humble/setup.bash`
3. 解析 ifname；状态非 `UP` 时尝试 `ip link set <if> up`
4. `source /opt/ros/humble/setup.bash`
5. `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
6. `source install/setup.bash`
7. `exec ros2 launch linkx_bringup full_system.launch.py ifname:=... vehicle_prefix:=... ros_nodes_prefix:=...`

非 `--sudo` 模式下，脚本会用 `getcap` 检查可执行文件权限，发现缺 `cap_net_raw/cap_net_admin` 会提示。

## 2. 包内脚本：`src/linkx_soem_demo/launch/run_link.sh`

兼容旧习惯。位于包内便于单独打包；行为接近根脚本。

```bash
bash src/linkx_soem_demo/launch/run_link.sh --sudo --ifname enp86s0
```

额外参数：

- `--no-vehicle`：仅启动遥控链路，不跑车体主控

注意：包内脚本默认网卡是 `enp3s0`（与根脚本不同），多数情况要显式带 `--ifname`。
2026-05-18 起 `--max-speed` 已废，详见上节注。

## 3. 直接 launch（最透明）

```bash
ros2 launch linkx_bringup full_system.launch.py \
    ifname:=enp86s0 \
    start_vehicle_control:=true \
    vehicle_prefix:="sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH ..." \
    ros_nodes_prefix:=""
```

可选参数（见 `linkx_bringup/launch/full_system.launch.py`）：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `ifname` | `enp86s0` | 网卡名 |
| `start_vehicle_control` | `true` | 是否启动车体 |
| `vehicle_prefix` | `""` | 车体进程前缀 |
| `ros_nodes_prefix` | `""` | 全部 ROS 节点前缀 |

> ⚠️ `max_speed` 不再是 launch 参数，唯一权威源在 `remote_node.cpp::declare_parameter`。
> 运行时改用 `ros2 param set /remote_node max_speed <v>`。

## 4. 权限模型

二选一：

**方案 A（推荐）：每次用 `--sudo`**

```bash
./start_full_system.sh --sudo --ifname enp86s0
```

**方案 B：给可执行文件加 capability（一次性）**

```bash
sudo setcap 'cap_net_raw,cap_net_admin+ep' \
    install/linkx_soem_demo/lib/linkx_soem_demo/linkx_soem_demo
```

⚠️ 每次 `colcon build` 重装可执行文件后 capability 会**丢失**，需重新 `setcap`。

## 5. 启动后的进程节点

| 节点 | 包 / 可执行 | 角色 |
| --- | --- | --- |
| `joy_node` | `joy` | 手柄事件 → `/joy` |
| `remote_node` | `linkx_soem_demo/remote_node_cpp` | `/joy` → `/cmd_vel` + `/robot_buttons` |
| `vehicle_control` | `linkx_soem_demo/linkx_soem_demo` | EtherCAT 主站 + 1ms 控制循环 |

## 6. 编码器零点 / 累计值的两种启动场景

### 场景 1：日常运行（已有零点 + 累计值）

```bash
unset CAPTURE_STEER_ZERO
unset CAPTURE_STEER_ZERO_FORCE
./start_full_system.sh --sudo --ifname enp86s0
```

### 场景 2：重新设置编码器零点

```bash
rm -f var_data/steer_unwrapped_pulses.txt   # 清掉旧累计值
export CAPTURE_STEER_ZERO=1
export CAPTURE_STEER_ZERO_FORCE=1
./start_full_system.sh --sudo --ifname enp86s0
```

看到日志 `[TASK] steer zero calibration captured` 后停止，再正常运行。

详见 [04_steer_encoder.md](04_steer_encoder.md)。

## 7. 实时观察反馈波形

仓库已自带 `tools/plot_feedback_wave.py`：

**A. 直连方式**

```bash
./start_full_system.sh --sudo --ifname enp86s0 2>&1 | \
python3 tools/plot_feedback_wave.py \
  --series now_vx --series now_vy --series now_omega \
  --height 18 --history 600 --refresh-hz 10
```

观察某个编码器：

```bash
... 2>&1 | python3 tools/plot_feedback_wave.py \
  --series E0.wheel_rad --series E0.omega_rpm
```

**B. 二终端方式**

```bash
# 终端 1：
./start_full_system.sh --sudo --ifname enp86s0 2>&1 | tee /tmp/r1_live.log

# 终端 2：
tail -f /tmp/r1_live.log | python3 tools/plot_feedback_wave.py --series now_vx
```

常用参数：

| 参数 | 用途 |
| --- | --- |
| `--series <key>` | 要绘制的字段，可重复 |
| `--list-keys` | 打印当前所有字段名（先用这个找） |
| `--csv <file>` | 同时保存采样数据到 CSV |
| `--y-min / --y-max` | 固定纵轴范围 |
| `--height` / `--history` / `--refresh-hz` | ASCII 高度 / 历史长度 / 刷新率 |

## 8. 关键注意事项

- **网卡专用**：EtherCAT 网卡建议独立有线网卡，不要混跑普通业务。
- **网卡链路**：必须 `UP` + `LOWER_UP`（有载波）。
- **网卡名错** → EtherCAT 主站初始化失败。
- **build/install/log** 是自动产物，异常时可清理后重编。
- **优雅退出**：`Ctrl+C` 会触发信号处理，主循环跳出前会落盘累计值。

## 9. 控制模式（2026-05-18）

### 9.1 drive_mode 双档 profile

底盘 `Class_Chassis::drive_mode_` 在两档间切换，影响舵向 slew 和 LPF：

| profile | slew | LPF α | τ ≈ dt/α | 用途 |
| --- | --- | --- | --- | --- |
| `Drive_Mode_MANUAL` | 500 °/s | 0.30 | 6.7 ms | 手柄遥控（默认） |
| `Drive_Mode_SEMI_AUTO` | 200 °/s | 0.15 | 13 ms | 路径跟随 / Auto_Pilot |

切档由 `Class_Auto_Pilot::Start/Stop` 自动完成（`auto_pilot.cpp:152 / 189`）。手动模式
下任何外部直接 `Set_Drive_Mode` 都会再被 auto_pilot 接管时改回。

> 不要 fork 底盘代码做 manual / semi_auto 双实例，profile 切换是在同一底盘对象上完成的。

环境变量覆盖（开发期调参，正式发布前回默认）：

```bash
STEER_SLEW_RATE_DEG_S=500    # MANUAL slew
STEER_LPF_ALPHA=0.30         # MANUAL LPF α
STEER_SLEW_RATE_DEG_S_SEMI=200
STEER_LPF_ALPHA_SEMI=0.15
```

### 9.2 Auto_Pilot 拐角软化

路径点之间过渡用 cos²(Δθ/2) + smoothstep 重写（`auto_pilot.cpp:283 起`）：

- `V_corner = seg_speed × cos²(Δθ/2)`：直线全速、90° 半速、180° U-turn 停下转
- 窗口 `L = clamp(L0 × Δθ/(π/2), Lmin, Lmax)`，`L0 = 220 mm`、`Lmin = 40 mm`
- 进出曲线 smoothstep `3u² − 2u³`：边界 V′ = 0，无加速度阶跃
- 方向 lerp 与速度共用同一 smoothstep 权重，避免方向先到位但速度还在拐角速→ steer 跟不上

旧版的 blend + corner-slow 死锁分支已删，不要再加。

### 9.3 手动模式航向纠偏已删

`robot.cpp:347` 手动分支只有 cmd_omega 透传，**不接 OPS 纠偏**。三轮迭代后用户反馈
「关纠偏比开纠偏好」。稳态 yaw 漂靠 `kWheelSpeedCalib[4]` 4 轮速度标定在底盘端
吸收（见 [05_calibration.md](05_calibration.md) §5）。

半自动 / Auto_Pilot 路径才走 OPS 横向 PID + 航向 PID。

### 9.4 F710 trigger 速度缩放（XInput）

`Class_LogF710::Resolve_Speed_Scale` (`dvc_logF710.cpp:124`)：

```text
lt = (1 - axes[2]) * 0.5    # LT 压下量 ∈ [0, 1]
rt = (1 - axes[5]) * 0.5    # RT 压下量 ∈ [0, 1]
scale = 0.65 + (rt - lt) * 0.35
scale = clamp(scale, 0.30, 1.00)
```

- 不踩两键 → 0.65× 基线
- 只踩 RT → 1.0× max_speed
- 只踩 LT → 0.30× max_speed
- 两键同时按 → 抵消，仍 0.65×

DInput / 旧驱动下 `Resolve_Speed_Scale` 返回 1.0 透传，不缩放（避免误伤）。
ABXY 按键在 XInput 下重映射为 `0=A, 1=B, 2=X, 3=Y`。
