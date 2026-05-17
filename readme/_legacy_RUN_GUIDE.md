# Ethercat_R1 运行指南（含注意事项）

本文档说明如何在本仓库运行整套系统，以及 EtherCAT 场景下的关键注意事项。

## 1. 运行前准备

- 操作系统：Ubuntu（已按 `ROS 2 Humble` 路径组织）
- ROS 2：需存在 `/opt/ros/humble/setup.bash`
- 工作区根目录：`/home/pzx/code/R1/Ethercat_R1`
- 关键依赖：
  - `colcon`（构建）
  - ROS 包：`rclcpp`、`geometry_msgs`、`std_msgs`、`sensor_msgs`
  - 启动依赖：`joy`（手柄节点）
  - 系统库：`libpcap`（EtherCAT 原始以太网收发）

建议先检查：

```bash
ls /opt/ros/humble/setup.bash
ip -br link
```

## 2. 构建

在工作区根目录执行：

```bash
cd /home/pzx/code/R1/Ethercat_R1
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 3. 推荐启动方式（根目录一键脚本）

```bash
cd /home/用户名/项目名 && ./start_full_system.sh --sudo --ifname enp86s0（Ethercat网卡名）
```

常用参数：

- `--ifname <网卡名>`：指定 EtherCAT 网卡（默认 `enp86s0`）
- `--auto-ifname`：自动选择第一块有线网卡（`en*`/`eth*`）
- `--sudo`：用 sudo 启动车辆主控进程（最稳妥）

脚本会自动完成：

1. 检查 ROS2 环境
2. 检查网卡是否存在/是否 UP
3. `colcon build`
4. `source install/setup.bash`
5. `ros2 launch linkx_bringup full_system.launch.py ...`

## 4. 可选启动方式（包内脚本）

也可使用：

```bash
cd /home/pzx/code/R1/Ethercat_R1
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/linkx_soem_demo/launch/run_link.sh --sudo --ifname enp86s0
```

常用参数：

- `--max-speed 1.5`：设置遥控最大速度
- `--no-vehicle`：不启动车体主控（仅测遥控/话题）
- `--gimbal`：启用云台桥接

## 5. 直接 launch（不经脚本）

```bash
cd /home/pzx/code/R1/Ethercat_R1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch linkx_bringup full_system.launch.py ifname:=enp86s0
```

可选参数：

- `ifname:=enp86s0`
- `max_speed:=1.5`
- `start_vehicle_control:=true|false`
- `start_gimbal_bridge:=true|false`
- `vehicle_prefix:="sudo -E env ..."`（需要 sudo 启动车体主控时）

## 6. 进程与节点说明

整套启动默认包含：

- `joy/joy_node`：读取手柄
- `linkx_soem_demo/remote_node_cpp`：遥控映射
- `linkx_soem_demo/stm32_node_cpp`：底盘桥接
- `linkx_soem_demo/linkx_soem_demo`：车辆主控（EtherCAT 主循环）

主控程序会读取命令行第一个参数作为网卡名（例如 `enp86s0`），并进入 1ms 周期控制循环。

## 7. 注意事项（重点）

- EtherCAT 网卡必须选对：`ifname` 错误会导致主站初始化失败。
- 网卡链路要正常：建议网卡 `UP` 且有载波（`LOWER_UP`）。
- 权限必须满足其一：
  - 使用 `--sudo` 启动车体主控；或
  - 给可执行文件加 capability：

```bash
sudo setcap 'cap_net_raw,cap_net_admin+ep' /home/pzx/code/R1/Ethercat_R1/install/linkx_soem_demo/lib/linkx_soem_demo/linkx_soem_demo
```

- 建议使用独立有线网卡跑 EtherCAT，避免和普通网络业务混用。
- `build/ install/ log/` 是自动生成目录，异常时可清理后重编。
- 中断退出时（`Ctrl+C`），主程序会触发信号处理并尝试优雅停止。

## 8. 常见故障排查

1. 报错 `ROS 2 Humble not found`
- 检查 `/opt/ros/humble/setup.bash` 是否存在。

2. 报错 `Interface not found`
- 执行 `ip -br link`，确认网卡名后通过 `--ifname` 指定。

3. 报错权限不足（raw Ethernet）
- 改用 `--sudo`，或执行 `setcap` 后重试。

4. 启动后 EtherCAT 仍不上线
- 检查网线、从站供电、网卡状态（是否 `LOWER_UP`）、网卡是否被其他程序占用。

5. 只想先测遥控链路
- 用 `--no-vehicle`，先确认 ROS 话题链路正常，再接入 EtherCAT 主控。

## 9. 最小可执行命令（推荐）

```bash
cd /home/pzx/code/R1/Ethercat_R1
./start_full_system.sh --sudo --ifname enp86s0
```

## 9.1 编码器零点与累计值运行流程（两种场景）

场景 1：已经记录过编码器总值，正常运行

```bash
unset CAPTURE_STEER_ZERO
unset CAPTURE_STEER_ZERO_FORCE
./start_full_system.sh --sudo --ifname enp86s0
```

场景 2：重新设置了编码器零点，希望总值清零并重建基线

```bash
rm -f var_data/steer_unwrapped_pulses.txt
export CAPTURE_STEER_ZERO=1
export CAPTURE_STEER_ZERO_FORCE=1
./start_full_system.sh --sudo --ifname enp86s0
```

看到日志 `steer zero calibration captured` 后停止程序，再正常运行：

```bash
unset CAPTURE_STEER_ZERO
unset CAPTURE_STEER_ZERO_FORCE
./start_full_system.sh --sudo --ifname enp86s0
```

## 10. 打印反馈波形（终端实时）

仓库已提供脚本：`tools/plot_feedback_wave.py`，可从控制台日志中提取 `key=value` 字段并实时画 ASCII 波形。

方式 A（推荐，实时直连）：

```bash
cd /home/pzx/code/R1/Ethercat_R1
./start_full_system.sh --sudo --ifname enp86s0 2>&1 | \
python3 tools/plot_feedback_wave.py \
  --series now_vx --series now_vy --series now_omega \
  --height 18 --history 600 --refresh-hz 10
```

示例：看某个编码器反馈（如 `E0.wheel_rad`）：

```bash
cd /home/pzx/code/R1/Ethercat_R1
./start_full_system.sh --sudo --ifname enp86s0 2>&1 | \
python3 tools/plot_feedback_wave.py \
  --series E0.wheel_rad --series E0.omega_rpm \
  --height 18 --history 600 --refresh-hz 10
```

方式 B（二终端）：

终端 1：先把系统输出保存到日志

```bash
cd /home/pzx/code/R1/Ethercat_R1
./start_full_system.sh --sudo --ifname enp86s0 2>&1 | tee /tmp/r1_live.log
```

终端 2：对日志实时画波形

```bash
tail -f /tmp/r1_live.log | python3 tools/plot_feedback_wave.py --series now_vx --series now_vy
```

常用参数：

- `--series <键名>`：要画的反馈字段，可重复写多个。
- `--list-keys`：打印当前发现到的所有字段名，便于先找“某个反馈”叫什么。
- `--csv <文件>`：同时保存采样数据（CSV）。
- `--y-min / --y-max`：固定纵轴范围，便于不同时间段对比。

## 11. 虚拟无限位移（舵向编码器掉电恢复 + 长期连续累计）

舵向编码器物理上是 50 圈（204800 脉冲）有界绝对值，齿比 3.5:1（输出轴一圈 = 编码器 14336 脉冲）。
本仓库已实现"虚拟无限位移"方案：把它抽象成软件层面的无限累计长轴 L，控制语义只关心
`phase = (L - logical_zero_anchor) mod 14336`，与物理 50 圈边界完全解耦。

### 11.1 核心变量定义

| 名称 | 实现字段 | 物理含义 |
| --- | --- | --- |
| L（逻辑长轴） | `Class_Encoder_BRT::total_unwrapped_pulses` | 软件解卷后的全生命周期累计脉冲（int64） |
| P_now | `data.encoder_value % ENC_TRUE_MAX_PULSES` | 编码器最新一帧的物理 raw（0..204799） |
| P_last | `Class_Encoder_BRT::last_raw_abs` | 上一帧的物理 raw |
| L0（角度参考点） | `Class_Encoder_BRT::logical_zero_anchor` | 找零时记录的 L 值 |
| 舵角相位 | `phase = (L - L0) mod 14336` | 舵向输出轴角度（与物理跨边界无关） |

### 11.2 跨边界缝合公式

每收到一帧 raw 增量更新（见 `Update_Unwrapped_Total`）：

```
delta = P_now - P_last
if delta < -102400 : delta += 204800   # 反向跨边界
if delta >  102400 : delta -= 204800   # 正向跨边界
L += delta
```

掉电恢复（见 `Init_Restore_Position`，首帧 raw 触发）：

```
expected_raw = saved_pulses mod 204800
diff = current_raw_abs - expected_raw   # 同样做 ±102400 折叠
L = saved_pulses + diff
```

### 11.3 启动恢复路径与降级判据

启动时 `Class_Chassis::Init` 会打印 `[CHASSIS][STARTUP-STATE]` 摘要，按以下三种路径判定：

| 路径 | zero 文件 | unwrapped 文件 | 表现 | 处理 |
| --- | --- | --- | --- | --- |
| A | 有效 | 有效 | 完全恢复，L 自动 diff 缝合 | 直接运行 |
| B | 有效 | 缺失 | 角度参考点 OK，但 L 从 raw 起算 | 系统标 degraded，可继续运行 |
| C-PARTIAL | 缺失 | 有效 | 角度参考点丢失，机械零不归 0 | 必须 `CAPTURE_STEER_ZERO=1` 重找零 |
| C | 缺失 | 缺失 | 完全冷启动 | 必须 `CAPTURE_STEER_ZERO=1` 重找零 |

降级判据（`Init_Restore_Position` 内）：若 `|diff| > 7 圈 × 4096 = 28672` 脉冲，
则 `degraded_mode=true`、`position_restore_ok=false`，并打印 `[ENC] RESTORE DEGRADED`。
这通常意味着断电后被人为大幅手推；现场应判停 + 重找零。

### 11.4 重新找零流程

1. 把舵向手动/电控转到目标"零位"（如四轮全部正对前方）。
2. 若使用 BRT 编码器中点功能，先在该位置 `CAN_Send_SetMidpoint`。
3. 设环境变量后启动一次：
   ```bash
   export CAPTURE_STEER_ZERO=1
   ./start_full_system.sh --sudo --ifname enp86s0
   ```
4. 看到 `[TASK] steer zero calibration captured` 表示已写入：
   - `var_data/steer_zero_offsets.txt`（raw 零点 + logical_zero_anchor）
   - `var_data/steer_unwrapped_pulses.txt`（强制立即落盘，与 anchor 同步）
5. 关掉 `CAPTURE_STEER_ZERO`，正常启动；从此 `wheel_deg_true` 在物理零位读 0。

### 11.5 最小排障命令

查看零点文件：
```bash
cat var_data/steer_zero_offsets.txt
# 每行：<can_id> <raw_zero_offset> <logical_zero_anchor>
```

查看累计值文件（含 magic / seq / crc）：
```bash
cat var_data/steer_unwrapped_pulses.txt
# 每行：<can_id> <0x454e4350> <total_pulses> <seq> <crc32>
```

运行中查看四轮恢复状态（[LIVE-DASHBOARD] → [ENC] 区域已打印）：
```bash
# 关键字段：
#   restore_ok=1  → 上电恢复成功且未降级
#   degraded=1    → 降级，建议手动确认/重找零
#   anchor_ok=1   → 逻辑零位锚点已加载（机械零位 wheel_deg=0）
#   L=...         → 当前逻辑长轴累计值
#   L0=...        → 找零时记录的锚点
```

不变量自检：跨边界后必须满足 `L mod 204800 == raw_true_cal + zero_offset_true`（mod 204800）。

### 11.6 何时会触发"立即落盘"

主循环每 200 ms 调一次 `Save_Steer_Unwrapped_Pulses`（含 200 ms 限频 + 无变化跳过）。
此外以下事件会调用 `Force_Save_Steer_Unwrapped_Pulses`（绕过限频）：

- `CAPTURE_STEER_ZERO=1` 找零成功（防止 anchor 与 L 不同步）。
- 主循环退出前（Ctrl+C / SIGINT 触发）。

写盘失败不会中断控制主循环：会打印 `[CHASSIS][PERSIST-FAIL]` 告警，下次 200 ms 周期会重试。
