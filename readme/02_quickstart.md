# 02 · 快速开始

## 1. 环境检查（一次性）

```bash
ls /opt/ros/humble/setup.bash    # 必须存在
ip -br link                       # 看到你的 EtherCAT 网卡（典型：enp86s0）
```

依赖：

- ROS 2 Humble
- `colcon`、`rclcpp`、`geometry_msgs`、`std_msgs`、`sensor_msgs`、`joy`
- 系统库：`libpcap`（EtherCAT 原始以太网）

## 2. 编译

```bash
cd /home/<你>/code/Ethercat_R1
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 3. 启动（推荐：根目录脚本）

```bash
cd /home/<你>/code/Ethercat_R1
./start_full_system.sh --sudo --ifname enp86s0
```

脚本会自动：

1. 检查 ROS 2 环境
2. 检查/拉起网卡
3. `colcon build` + `source install/setup.bash`
4. `ros2 launch linkx_bringup full_system.launch.py ...`

启动后默认开启的节点：

- `joy/joy_node` —— 读取手柄
- `linkx_soem_demo/remote_node_cpp` —— 手柄解算 → `/cmd_vel`
- `linkx_soem_demo/linkx_soem_demo` —— 车体主控（EtherCAT 1ms 循环）

## 4. 常用启动参数

| 参数 | 用途 |
| --- | --- |
| `--ifname <name>` | 指定 EtherCAT 网卡 |
| `--auto-ifname` | 自动选第一块有线网卡（`en*`/`eth*`） |
| `--sudo` | 给车体主控加 `sudo -E env ...` 前缀 |
| `--max-speed 1.5` | 遥控最大线速度 |

## 5. 三种启动方式选择

| 场景 | 推荐 |
| --- | --- |
| 日常运行（最常用） | 根目录 `./start_full_system.sh --sudo --ifname enp86s0` |
| 包内脚本（旧习惯兼容） | `bash src/linkx_soem_demo/launch/run_link.sh --sudo --ifname enp86s0` |
| 直接 launch | `ros2 launch linkx_bringup full_system.launch.py ifname:=enp86s0 vehicle_prefix:="sudo -E env ..."` |

## 6. 仅测遥控链路（不上车）

```bash
./start_full_system.sh --sudo --ifname enp86s0 \
  # 或者直接：
ros2 launch linkx_bringup full_system.launch.py \
  ifname:=enp86s0 start_vehicle_control:=false
```

或用包内脚本：

```bash
bash src/linkx_soem_demo/launch/run_link.sh --no-vehicle
```

可单独订阅检查链路：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /robot_buttons
```

## 7. 优雅退出

`Ctrl+C` 触发 `SIGINT`：

- `main()` 信号处理函数置 `master.is_running=false`
- 主循环跳出
- 退出前 `Force_Save_Steer_Unwrapped_Pulses` 强制落盘累计值
- `EtherCAT` 主站关闭
