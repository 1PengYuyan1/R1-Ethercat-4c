cmake_minimum_required(VERSION 3.10)
project(linkx_soem_demo C CXX)

# 设置 C 标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# SOEM 主程序源文件（显式列表，避免 src 目录新增文件时误链接 main）
set(SOEM_SRC_FILES
    src/vehicle_control/main.cpp
    src/vehicle_control/task/task/task.cpp
    src/vehicle_control/interaction/robot/robot.cpp
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/chariot/chassis/encoder_persistence.cpp
    src/vehicle_control/chariot/chassis/steer_trace.cpp
    src/vehicle_control/chariot/chassis/steer_calibration.cpp
    src/vehicle_control/chariot/clamp/crt_clamp.cpp
    src/vehicle_control/chariot/auto_pilot/dvc_auto_pilot.cpp
    src/vehicle_control/chariot/manual_yaw_hold/dvc_manual_yaw_hold.cpp
    src/vehicle_control/device/Motor/dvc_odrive.cpp
    src/vehicle_control/device/Motor/dvc_encoder.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/OPS/dvc_ops.cpp
    src/remote/dvc_logF710.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)

# 生成可执行程序
add_executable(${PROJECT_NAME} ${SOEM_SRC_FILES})

# add_executable(linkx_soem_demo 
#     src/main.cpp   
#     src/task/task/task.cpp
# )

# 指定头文件搜索路径
# (推荐使用 target_include_directories，比 include_directories 作用域更清晰安全)
target_include_directories(${PROJECT_NAME} PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/remote
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/middleware/Algorithm
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
    include/linkx_soem_demo/vehicle_control/device/OPS
    include/linkx_soem_demo/vehicle_control/chariot/chassis
    include/linkx_soem_demo/vehicle_control/chariot/clamp
    include/linkx_soem_demo/vehicle_control/chariot/auto_pilot
    include/linkx_soem_demo/vehicle_control/chariot/manual_yaw_hold
    include/linkx_soem_demo/vehicle_control/interaction/robot
    include/linkx_soem_demo/vehicle_control/task/task
)

# 4. 链接 Linux 系统库
# pcap   : 用于 SOEM 底层的网卡数据收发
# pthread: 用于多线程支持
# rt     : 用于 clock_gettime, clock_nanosleep 等高精度实时时钟函数
target_link_libraries(${PROJECT_NAME}
    pcap 
    pthread 
    rt
    m
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    std_msgs
)

# ROS2 遥控节点（手柄 -> /cmd_vel + /robot_buttons）
add_executable(remote_node_cpp
    src/remote/remote_node.cpp
    src/remote/dvc_logF710.cpp
)
target_include_directories(remote_node_cpp PRIVATE
    include
)
ament_target_dependencies(remote_node_cpp
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
)

# 独立 CAN 链路测试程序：仅验证 EtherCAT-4C 的 4 路 CAN 发包能力
set(CAN_LINK_TEST_SRC_FILES
    src/test_mains/can_link_test_main.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(can_link_test ${CAN_LINK_TEST_SRC_FILES})
target_include_directories(can_link_test PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
)
target_link_libraries(can_link_test
    pcap
    pthread
    rt
    m
)

# 双 LinkX (CAN-FD) 链路测试程序：验证总线挂第二片 LinkX 后 slavecount==2、
# 第二片 4 通道按 CAN-FD (1M/5M) 配置成功、并能持续发出 64B BRS 帧
set(CANFD_LINK_TEST_SRC_FILES
    src/test_mains/canfd_link_test_main.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(canfd_link_test ${CANFD_LINK_TEST_SRC_FILES})
target_include_directories(canfd_link_test PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
)
target_link_libraries(canfd_link_test
    pcap
    pthread
    rt
    m
)

# OPS-9 接收解码冒烟测试：通道 3 收 CAN，喂给 Class_OPS 解码后 1Hz 打印
# 用法： sudo IFNAME=enp86s0 ./build/linkx_soem_demo/ops_test [enp86s0]
set(OPS_TEST_SRC_FILES
    src/test_mains/ops_test_main.cpp
    src/vehicle_control/device/OPS/dvc_ops.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(ops_test ${OPS_TEST_SRC_FILES})
target_include_directories(ops_test PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/OPS
)
target_link_libraries(ops_test
    pcap
    pthread
    rt
    m
)

# 舵向调参专用程序：MIT Kp/Kd + 外层位置环 + 力矩前馈
# 不依赖 ROS2 / 遥控器，独立可运行
set(STEER_TUNING_SRC_FILES
    src/test_mains/steer_tuning_main.cpp
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/chariot/chassis/encoder_persistence.cpp
    src/vehicle_control/chariot/chassis/steer_trace.cpp
    src/vehicle_control/chariot/chassis/steer_calibration.cpp
    src/vehicle_control/chariot/auto_pilot/dvc_auto_pilot.cpp
    src/vehicle_control/device/Motor/dvc_odrive.cpp
    src/vehicle_control/device/Motor/dvc_encoder.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/OPS/dvc_ops.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(steer_tuning ${STEER_TUNING_SRC_FILES})
target_include_directories(steer_tuning PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/middleware/Algorithm
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
    include/linkx_soem_demo/vehicle_control/device/OPS
    include/linkx_soem_demo/vehicle_control/chariot/chassis
    include/linkx_soem_demo/vehicle_control/chariot/auto_pilot
)
target_link_libraries(steer_tuning
    pcap
    pthread
    rt
    m
)

# 单轮电机参数标定程序：动摩擦力 / 静摩擦力 / 转动惯量
# 复用 chassis 初始化（编码器/电机使能等），调用 motor 的 Begin_*_Calibration() API
set(MOTOR_CALIB_SRC_FILES
    src/test_mains/motor_calib_main.cpp
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/chariot/chassis/encoder_persistence.cpp
    src/vehicle_control/chariot/chassis/steer_trace.cpp
    src/vehicle_control/chariot/chassis/steer_calibration.cpp
    src/vehicle_control/chariot/auto_pilot/dvc_auto_pilot.cpp
    src/vehicle_control/device/Motor/dvc_odrive.cpp
    src/vehicle_control/device/Motor/dvc_encoder.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/OPS/dvc_ops.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(motor_calib ${MOTOR_CALIB_SRC_FILES})
target_include_directories(motor_calib PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/middleware/Algorithm
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
    include/linkx_soem_demo/vehicle_control/device/OPS
    include/linkx_soem_demo/vehicle_control/chariot/chassis
    include/linkx_soem_demo/vehicle_control/chariot/auto_pilot
)
target_link_libraries(motor_calib
    pcap
    pthread
    rt
    m
)

# 4 舵向电机并行 摩擦力测试 (静摩擦 + 动摩擦)
# 一次性同时测 4 个 DM 舵向电机的 Ts/Tc, 单轮 ≤10s, 轮间 settle 后再测
set(STEER_FRIC4_SRC_FILES
    src/test_mains/steer_friction_4wheel_main.cpp
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/chariot/chassis/encoder_persistence.cpp
    src/vehicle_control/chariot/chassis/steer_trace.cpp
    src/vehicle_control/chariot/chassis/steer_calibration.cpp
    src/vehicle_control/chariot/auto_pilot/dvc_auto_pilot.cpp
    src/vehicle_control/device/Motor/dvc_odrive.cpp
    src/vehicle_control/device/Motor/dvc_encoder.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/OPS/dvc_ops.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(steer_friction_4wheel ${STEER_FRIC4_SRC_FILES})
target_include_directories(steer_friction_4wheel PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/middleware/Algorithm
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
    include/linkx_soem_demo/vehicle_control/device/OPS
    include/linkx_soem_demo/vehicle_control/chariot/chassis
    include/linkx_soem_demo/vehicle_control/chariot/auto_pilot
)
target_link_libraries(steer_friction_4wheel
    pcap
    pthread
    rt
    m
)

# ODrive 驱动轮参数标定程序：摩擦力 / 静摩擦力 / 转动惯量
# 通过 CAN 控制 ODrive 速度/力矩模式，读取编码器和 IQ 反馈
set(ODRIVE_CALIB_SRC_FILES
    src/test_mains/odrive_calib_main.cpp
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/chariot/chassis/encoder_persistence.cpp
    src/vehicle_control/chariot/chassis/steer_trace.cpp
    src/vehicle_control/chariot/chassis/steer_calibration.cpp
    src/vehicle_control/chariot/auto_pilot/dvc_auto_pilot.cpp
    src/vehicle_control/device/Motor/dvc_odrive.cpp
    src/vehicle_control/device/Motor/dvc_encoder.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/OPS/dvc_ops.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(odrive_calib ${ODRIVE_CALIB_SRC_FILES})
target_include_directories(odrive_calib PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/middleware/Algorithm
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
    include/linkx_soem_demo/vehicle_control/device/OPS
    include/linkx_soem_demo/vehicle_control/chariot/chassis
    include/linkx_soem_demo/vehicle_control/chariot/auto_pilot
)
target_link_libraries(odrive_calib
    pcap
    pthread
    rt
    m
)

# ODrive CAN Simple 协议函数级冒烟测试：逐函数下发 CAN 帧并验证回包
# 用法： sudo IFNAME=enp86s0 ./odrive_protocol_test --wheel 0
set(ODRIVE_PROTOCOL_TEST_SRC_FILES
    src/test_mains/odrive_protocol_test_main.cpp
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/chariot/chassis/encoder_persistence.cpp
    src/vehicle_control/chariot/chassis/steer_trace.cpp
    src/vehicle_control/chariot/chassis/steer_calibration.cpp
    src/vehicle_control/chariot/auto_pilot/dvc_auto_pilot.cpp
    src/vehicle_control/device/Motor/dvc_odrive.cpp
    src/vehicle_control/device/Motor/dvc_encoder.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/OPS/dvc_ops.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(odrive_protocol_test ${ODRIVE_PROTOCOL_TEST_SRC_FILES})
target_include_directories(odrive_protocol_test PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/middleware/Algorithm
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
    include/linkx_soem_demo/vehicle_control/device/OPS
    include/linkx_soem_demo/vehicle_control/chariot/chassis
    include/linkx_soem_demo/vehicle_control/chariot/auto_pilot
)
target_link_libraries(odrive_protocol_test
    pcap
    pthread
    rt
    m
)

# ===== 单元测试：BRT 编码器字节序 + ACK 解析（dry-run，无需硬件）=====
# 通过 stub linkx_quick_can_send 验证 set 指令字节序符合手册示例，
# 以及 0x02~0x0F set 指令的 ACK 帧被 Data_Process 正确路由。
# 运行： ./build/linkx_soem_demo/encoder_byteorder_test
add_executable(encoder_byteorder_test
    tests/test_brt_encoder.cpp
    src/vehicle_control/device/Motor/dvc_encoder.cpp
)
target_include_directories(encoder_byteorder_test PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/Motor
)

# ===== 硬件 ACK 验证：4 路 BRT 编码器实物冒烟测试（需 sudo + EtherCAT 网卡）=====
# 复用 motor_calib 的 EtherCAT 启动流程，对每个编码器逐一发非破坏性 set 指令，
# 等待 ACK 并断言 status==0x00。用于：
#   * 大端字节序改动是否被硬件接受
#   * Data_Process 对 0x02~0x0F ACK 路由是否在真实数据上工作
# 运行：sudo IFNAME=enp86s0 ./encoder_ack_test
set(ENCODER_ACK_TEST_SRC_FILES
    src/test_mains/encoder_ack_test_main.cpp
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/chariot/chassis/encoder_persistence.cpp
    src/vehicle_control/chariot/chassis/steer_trace.cpp
    src/vehicle_control/chariot/chassis/steer_calibration.cpp
    src/vehicle_control/chariot/auto_pilot/dvc_auto_pilot.cpp
    src/vehicle_control/device/Motor/dvc_odrive.cpp
    src/vehicle_control/device/Motor/dvc_encoder.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/OPS/dvc_ops.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(encoder_ack_test ${ENCODER_ACK_TEST_SRC_FILES})
target_include_directories(encoder_ack_test PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/middleware/Algorithm
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
    include/linkx_soem_demo/vehicle_control/device/OPS
    include/linkx_soem_demo/vehicle_control/chariot/chassis
    include/linkx_soem_demo/vehicle_control/chariot/auto_pilot
)
target_link_libraries(encoder_ack_test
    pcap
    pthread
    rt
    m
)

# ===== ENC3 出厂恢复后的重配工具 =====
# 用户用"黄线接地 2 分钟"恢复 enc3 出厂设置后，跑此工具把 enc3 配回:
#   500K/0x01/默认模式 → 1M/0x08/AUTO_RETURN_ANGLE/200Hz
# 用法：sudo IFNAME=enp86s0 ./enc3_recover
add_executable(enc3_recover
    src/test_mains/enc3_recover_main.cpp
    src/vehicle_control/device/Motor/dvc_encoder.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
target_include_directories(enc3_recover PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
)
target_link_libraries(enc3_recover
    pcap
    pthread
    rt
    m
)

# ===== ENC3 设置周期工具（慢节奏单独操作 enc3）=====
# 假设 enc3 已在 1M/0x08，慢节奏（每步 2s）重试 10 次设 SetAutoSendTime(5000us)
# 用法：sudo IFNAME=enp86s0 ./enc3_set_period
add_executable(enc3_set_period
    src/test_mains/enc3_set_period_main.cpp
    src/vehicle_control/device/Motor/dvc_encoder.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
target_include_directories(enc3_set_period PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
)
target_link_libraries(enc3_set_period
    pcap
    pthread
    rt
    m
)

# 一次性写 LinkX EtherCAT Station Alias，从此 slave_id 不再依赖物理串接顺序。
# 用法：sudo ./linkx_set_alias enp86s0 show / set <idx> <alias> / auto <a1> <a2>
add_executable(linkx_set_alias
    src/test_mains/linkx_set_alias_main.cpp
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
target_include_directories(linkx_set_alias PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
)
target_link_libraries(linkx_set_alias
    pcap
    pthread
    rt
    m
)

# 夹爪 (Pitch_Large + Pitch_Small) 只读遥测: 仅周期发 DM Enter 帧, 把 Get_Now_Radian
# 通过 UDP 广播给 Python 端动画 (tools/clamp_arm_visualizer.py --udp-port 5005)
# 用法: sudo ./build/linkx_soem_demo/clamp_telemetry enp86s0 [--port 5005] [--rate 50]
set(CLAMP_TELEMETRY_SRC_FILES
    src/test_mains/clamp_telemetry_main.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(clamp_telemetry ${CLAMP_TELEMETRY_SRC_FILES})
target_include_directories(clamp_telemetry PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
)
target_link_libraries(clamp_telemetry
    pcap
    pthread
    rt
    m
)

# 夹爪位置实时监视器: 只读, 终端用 ANSI 控制码刷两行显示 Pitch_Large/Small
# 位置 + 速度 + 力矩 + 温度 + alive 状态.
# 用法: sudo ./build/linkx_soem_demo/clamp_position_monitor enp86s0 [--rate 20]
set(CLAMP_POSITION_MONITOR_SRC_FILES
    src/test_mains/clamp_position_monitor_main.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(clamp_position_monitor ${CLAMP_POSITION_MONITOR_SRC_FILES})
target_include_directories(clamp_position_monitor PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
)
target_link_libraries(clamp_position_monitor
    pcap
    pthread
    rt
    m
)

# Class_Robot 真实上机回归测试程序：顺序自动跑全套测试，限速短转，自动停。
# 用法： sudo ./build/linkx_soem_demo/robot_test enp86s0
set(ROBOT_TEST_SRC_FILES
    src/test_mains/robot_test_main.cpp
    src/vehicle_control/interaction/robot/robot.cpp
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/chariot/chassis/encoder_persistence.cpp
    src/vehicle_control/chariot/chassis/steer_trace.cpp
    src/vehicle_control/chariot/chassis/steer_calibration.cpp
    src/vehicle_control/chariot/clamp/crt_clamp.cpp
    src/vehicle_control/chariot/auto_pilot/dvc_auto_pilot.cpp
    src/vehicle_control/chariot/manual_yaw_hold/dvc_manual_yaw_hold.cpp
    src/vehicle_control/device/Motor/dvc_odrive.cpp
    src/vehicle_control/device/Motor/dvc_encoder.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/OPS/dvc_ops.cpp
    src/remote/dvc_logF710.cpp
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)
add_executable(robot_test ${ROBOT_TEST_SRC_FILES})
target_include_directories(robot_test PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/remote
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/middleware
    include/linkx_soem_demo/vehicle_control/device
    include/linkx_soem_demo/vehicle_control/middleware/soem
    include/linkx_soem_demo/vehicle_control/middleware/linkx
    include/linkx_soem_demo/vehicle_control/middleware/npcap
    include/linkx_soem_demo/vehicle_control/middleware/math
    include/linkx_soem_demo/vehicle_control/middleware/Algorithm
    include/linkx_soem_demo/vehicle_control/device/ecat_manager
    include/linkx_soem_demo/vehicle_control/device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/device/rt_timing
    include/linkx_soem_demo/vehicle_control/device/Motor
    include/linkx_soem_demo/vehicle_control/device/OPS
    include/linkx_soem_demo/vehicle_control/chariot/chassis
    include/linkx_soem_demo/vehicle_control/chariot/clamp
    include/linkx_soem_demo/vehicle_control/chariot/auto_pilot
    include/linkx_soem_demo/vehicle_control/chariot/manual_yaw_hold
    include/linkx_soem_demo/vehicle_control/interaction/robot
)
target_link_libraries(robot_test
    pcap
    pthread
    rt
    m
)
ament_target_dependencies(robot_test
    rclcpp
    geometry_msgs
    std_msgs
)

install(TARGETS
    ${PROJECT_NAME}
    remote_node_cpp
    can_link_test
    canfd_link_test
    steer_tuning
    motor_calib
    steer_friction_4wheel
    odrive_calib
    odrive_protocol_test
    robot_test
    clamp_telemetry
    clamp_position_monitor
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    launch/run_link.sh
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
    launch
    config
    urdf
    msg
    srv
    action
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
