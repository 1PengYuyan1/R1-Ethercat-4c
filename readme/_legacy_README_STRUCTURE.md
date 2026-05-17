# Ethercat_R1 ROS2 工作区结构说明

本项目以仓库根目录 `Ethercat_R1/` 作为 ROS2 工作空间根目录。

## 标准结构

```text
Ethercat_R1/
├── src/                # 所有源码包
│   ├── linkx_soem_demo # 车辆控制与 EtherCAT 主逻辑（C++）
│   ├── linkx_bringup   # launch 启动编排包
│   └── third_party/    # 第三方包（可选）
├── build/              # colcon 自动生成的中间文件
├── install/            # colcon 自动生成的安装产物
└── log/                # colcon 自动生成的构建日志
```

## 包内结构（以 `linkx_soem_demo` 为例）

```text
linkx_soem_demo/
├── CMakeLists.txt
├── package.xml
├── include/linkx_soem_demo/
├── src/
├── launch/             # 启动脚本与 launch 文件
├── config/
├── urdf/
└── msg/srv/action/
```

## `linkx_soem_demo/src` 代码职责说明

```text
linkx_soem_demo/src/
├── vehicle_control/                         # 车体控制主流程（EtherCAT 主站 + 控制算法）
│   ├── main.cpp                             # 主程序入口（linkx_soem_demo 可执行）
│   ├── can_link_test_main.cpp               # CAN 链路独立测试入口（can_link_test 可执行）
│   ├── task/task/task.cpp                   # 控制任务调度与周期执行
│   ├── interaction/robot/robot.cpp          # 机器人交互层（状态/指令组织）
│   ├── chariot/chassis/crt_chassis.cpp      # 底盘运动学与底盘控制
│   ├── device/                              # 硬件设备驱动层
│   │   ├── Motor/dvc_encoder.cpp            # 编码器驱动与数据收发
│   │   ├── Motor/dvc_motor_dm.cpp           # DM 电机控制逻辑
│   │   ├── Motor/dvc_odrive.cpp             # ODrive 电机控制逻辑
│   │   ├── ecat_manager/ecat_manager.c      # EtherCAT 周期收发与管理
│   │   ├── linkx4c_handler/linkx4c_handler.c# LinkX4C 设备协议处理
│   │   └── rt_timing/rt_timing.c            # 实时周期定时工具
│   └── middleware/                          # 中间件与基础算法
│       ├── Algorithm/alg_pid.cpp            # PID 算法实现
│       ├── linkx/linkx.c                    # LinkX 基础通信封装
│       └── soem/*.c                         # SOEM 协议栈源码（EtherCAT 主站底层）
└── remote/                                  # 遥控与 ROS2 侧节点
    ├── ros2/remote_node.cpp                 # 遥控输入节点（remote_node_cpp）
    ├── ros2/joystick_mapper.cpp             # 手柄输入映射与归一化
    ├── ros2/stm32_node.cpp                  # STM32 通信节点（stm32_node_cpp）
    └── device/Remote/dvc_logF710.cpp        # Logitech F710 设备读写封装
```

## 你现在应该怎么用

1. 进入工作区根目录：`cd /home/pzx/code/R1/Ethercat_R1`
2. 构建：`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
3. source：`source install/setup.bash`
4. 启动（示例）：`ros2 launch linkx_bringup full_system.launch.py`

## 一键启动脚本

推荐使用根目录脚本：

```bash
cd /home/pzx/code/R1/Ethercat_R1
./start_full_system.sh --sudo --ifname enp86s0
```

常用参数：

- `--ifname <name>`：手动指定 EtherCAT 网卡（如 `enp86s0`）
- `--auto-ifname`：自动选择第一块有线网卡（`en*` / `eth*`）
- `--sudo`：用 `sudo` 前缀启动车体主控进程（EtherCAT 常用）

## 说明

- `run_link.sh` 仍可从 `linkx_soem_demo` 包根目录执行（为了兼容旧习惯），实际脚本已放到 `launch/run_link.sh`。
- `build/install/log` 都是自动生成目录，通常不需要手改。
- 仓库根目录不再保留 `SOEM_1` 软链接；请统一使用 `src/linkx_soem_demo` 作为源码路径。
