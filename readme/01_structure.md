# 01 · 工程目录与代码结构

## 1. 工作区根目录

```text
Ethercat_R1/
├── src/                     ROS 2 源码包
│   ├── linkx_soem_demo/     车辆主控（C++）+ ROS 2 遥控节点
│   └── linkx_bringup/       launch / 启动编排包
├── build/                   colcon 中间产物（自动生成）
├── install/                 colcon 安装产物（自动生成）
├── log/                     colcon 构建/运行日志
├── tools/                   调试脚本（标定、扫参、绘图）
├── instructions/            过往实施记录（卡死修复、编码器恢复）
├── var_data/                运行时持久化文件 + 曲线 CSV/PNG + Python 分析脚本
├── readme/                  本说明文档目录
└── start_full_system.sh     一键启动入口
```

## 2. `linkx_soem_demo` 包内结构

```text
linkx_soem_demo/
├── CMakeLists.txt           显式列源码避免误链多 main
├── package.xml
├── include/linkx_soem_demo/
│   ├── remote/              手柄外设抽象（Logitech F710，XInput/DInput 自适应）
│   └── vehicle_control/
│       ├── middleware/      SOEM / linkx 协议封装 / PID 算法 / 数学工具
│       ├── device/          硬件驱动层（编码器/电机/EtherCAT/OPS）
│       ├── chariot/
│       │   ├── chassis/     底盘运动学 + 编码器持久化 + 舵向校准（已抽离）
│       │   ├── auto_pilot/  路径跟随 + 拐角软化 + drive_mode 切换
│       │   └── clamp/       夹爪控制
│       ├── interaction/     机器人交互（指令组织 + ROS2 遥控桥）
│       └── task/            任务调度（1ms / 2ms / 100ms 周期）
├── src/
│   ├── remote/              ROS 2 遥控节点 + Logitech F710 解码
│   ├── test_mains/          独立测试 main（见 06 文档）
│   └── vehicle_control/     与 include/ 镜像的实现层
├── tests/                   纯软件单元测试（如 BRT 字节序）
├── launch/run_link.sh       包内启动脚本（兼容旧用法）
└── msg/ srv/ action/ urdf/  当前为空，预留
```

### 2.1 五层架构

按编号反映依赖方向：低层不依赖高层。

| 层 | 角色 | 关键文件 |
| --- | --- | --- |
| middleware | 协议栈 / 通用算法 | `soem/*.c`、`linkx/linkx.c`、`Algorithm/alg_pid.cpp`、`math/math.h` |
| device | 硬件抽象 | `Motor/dvc_odrive.{h,cpp}`、`Motor/dvc_motor_dm.{h,cpp}`、`Motor/dvc_encoder.{h,cpp}`、`OPS/dvc_ops.{h,cpp}`、`ecat_manager/`、`linkx4c_handler/`、`rt_timing/` |
| chariot/chassis | 底盘 + 舵向 | `chassis/crt_chassis.{h,cpp}` —— 4 舵轮运动学、drive_mode profile、`kWheelSpeedCalib`、输出端 slew。`chassis/steer_calibration.{h,cpp}` —— MIT 位置环 + slew + ω_des=0 + 滞回的找零序列。`chassis/encoder_persistence.{h,cpp}` —— `steer_zero_offsets.txt` / `steer_unwrapped_pulses.txt` 读写 + CRC32 |
| chariot/auto_pilot | 路径跟随 | `auto_pilot/dvc_auto_pilot.{h,cpp}` —— 拐角软化 `V = seg_speed × cos²(Δθ/2)`、smoothstep 进出窗口、方向 lerp、自动切 `Drive_Mode_SEMI_AUTO` |
| chariot/clamp | 夹爪 | `clamp/crt_clamp.{h,cpp}` |
| interaction | 上位整合 | `robot/robot.{h,cpp}` —— 子系统聚合 + 周期回调 + ROS2 遥控桥（手动模式 cmd_omega 直接透传，无航向纠偏） |
| task | 任务调度 | `task/task.{h,cpp}` —— `Robot_Control_Loop()` 主循环入口、双 LinkX (CAN-FD/classic) alias 绑定 |

### 2.2 主程序入口

- `vehicle_control/main.cpp`：`linkx_soem_demo` 可执行，命令行第一个参数为网卡名（默认 `enp86s0`），调用 `Robot_Control_Loop()` 阻塞运行直到 `SIGINT/SIGTERM`。
- `remote/remote_node.cpp`：`remote_node_cpp` 可执行，`/joy` → `/cmd_vel + /robot_buttons`。

## 3. `linkx_bringup` 包

```text
linkx_bringup/
├── launch/
│   ├── full_system.launch.py    主启动文件：joy + remote + vehicle_control
│   └── teleop.launch.py         仅遥控链路（不带车体）
├── config/
│   ├── teleop.params.yaml       预留 ROS 2 参数文件
│   └── fastrtps_profiles.xml    Fast DDS 配置（避免默认 XML 报错）
└── package.xml
```

`full_system.launch.py` 接受参数：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `ifname` | `enp86s0` | EtherCAT 网卡 |
| `start_vehicle_control` | `true` | 是否启动车体主控（false 时只跑遥控链路） |
| `vehicle_prefix` | `""` | 主控前缀（用于 `sudo -E env ...`） |
| `ros_nodes_prefix` | `""` | 全部 ROS 节点前缀 |

> ⚠️ 2026-05-18 起 launch 不再声明 `max_speed` 参数。遥控最大线速度的唯一权威源是
> `src/linkx_soem_demo/src/remote/remote_node.cpp` 的 `declare_parameter("max_speed", ...)`。
> 想改默认速度直接动 C++ 代码 + rebuild；运行时仍可 `ros2 param set /remote_node max_speed <v>` 临时改。

## 4. `tools/` 调试脚本

| 脚本 | 用途 |
| --- | --- |
| `plot_feedback_wave.py` | 实时把控制台 `key=value` 输出绘成 ASCII 波形 |
| `run_motor_calib.sh` | 单轮 DM 电机参数标定（动摩擦/静摩擦/转动惯量） |
| `run_steer_step_test.sh` | 舵向阶跃响应测试 |
| `run_step_kd_sweep.sh` | 舵向 Kd 扫参 |
| `run_remaining_tuning.sh` | 批量执行剩余调参用例 |
| `analyze_steer_trace.py` | 解析 `steer_trace.cpp` 落盘的舵向跟踪 trace |

## 5. `var_data/` 持久化与分析

运行期会读写：

| 文件 | 含义 |
| --- | --- |
| `steer_zero_offsets.txt` | 舵向编码器零点 raw + 逻辑零位锚点（找零产物） |
| `steer_unwrapped_pulses.txt` | 4 路舵向编码器累计长轴值 + CRC32 校验 |
| `motor_calib_result.txt` | DM 电机标定结果（Tc/Ts/J） |
| `odrive_calib_result.txt` | ODrive 驱动轮标定结果 |
| `live_variables.log` | 启动时清空，运行时记录关键变量 |

`var_data/*.py` 一系列绘图脚本对应不同测试 CSV，例如 `plot_4wheels_step.py` 画 4 轮阶跃响应、`plot_4wheels_sine.py` 画正弦跟踪。

## 6. `instructions/` 历史记录

| 文件 | 内容 |
| --- | --- |
| `chassis_calibration_fix_steps.md` | 底盘舵向"偶发卡死"问题的修复步骤记录 |
| `ENCODER_UNWRAP_POWERLOSS_RECOVERY_STEPS.md` | 舵向编码器掉电恢复方案的实施步骤 |

这两份是过程性记录，**不是当前操作手册**——日常使用看 `readme/` 即可。
