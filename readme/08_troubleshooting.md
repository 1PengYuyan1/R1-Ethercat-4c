# 08 · 故障排查

## 1. ROS 2 / 编译

### 1.1 `ROS 2 Humble not found`
检查：
```bash
ls /opt/ros/humble/setup.bash
```
不存在则需要先安装 ROS 2 Humble。

### 1.2 `colcon: command not found`
```bash
sudo apt install python3-colcon-common-extensions
```

### 1.3 编译错误："找不到头文件"
- 检查包结构是否完整：`src/linkx_soem_demo/CMakeLists.txt` 是否存在
- 清理重编：`rm -rf build/ install/ log/ && colcon build ...`

## 2. EtherCAT / 网卡

### 2.1 `Interface not found: enp86s0`
```bash
ip -br link              # 列出实际网卡
./start_full_system.sh --auto-ifname --sudo
# 或显式指定：
./start_full_system.sh --sudo --ifname <实际名字>
```

### 2.2 网卡状态非 UP
```bash
sudo ip link set enp86s0 up
```

### 2.3 No carrier（`LOWER_UP` 缺失）
- 检查网线是否插好
- 检查从站（EtherCAT-4C）供电
- 检查网卡是否被其他程序占用

### 2.4 EtherCAT 主站初始化失败
顺序排查：
1. 网卡名是否正确（`ip -br link`）
2. 网卡是否 `UP` + `LOWER_UP`
3. 是否有原始套接字权限（用 `--sudo` 或 `setcap`）
4. 网卡是否独占（不要混跑普通业务）

### 2.5 权限不足
```bash
# 方案 A：用 sudo
./start_full_system.sh --sudo --ifname enp86s0

# 方案 B：给可执行文件加 capability（每次重编后重做）
sudo setcap 'cap_net_raw,cap_net_admin+ep' \
    install/linkx_soem_demo/lib/linkx_soem_demo/linkx_soem_demo
```

## 3. CAN / 设备层

### 3.1 ODrive 不切状态
- Makerbase 版本可能要**反复重发** `SET_ClosedLoop` / `Emergency_Stop`
- `Reboot` 后必须等 **≥ 15 s** 才能重连
- 检查心跳帧（命令 `0x001`）能否收到

### 3.2 BRT 编码器 set 指令无效
- 检查发送通道：set 必须走 quiet TX 路径
- 注意 LinkX TX 队列对**同 ID** 帧采用「同 ID 更新」，旧帧会被新帧覆盖
- 用 `encoder_byteorder_test`（无需硬件）先验证字节序
- 用 `encoder_ack_test`（需硬件）验证真实 ACK

### 3.3 编码器频率异常
- 确认配置：`ch=2 / 1Mbps / 4 enc / 5ms 周期 → 200 Hz`
- 已知现象：某硬件上 `enc3` 稳定在 ~19 Hz（硬件特性，不是软件 bug）

## 4. 控制 / 标定

### 4.1 舵向校准卡死
当前主线已修，关键检查点（来自 `instructions/chassis_calibration_fix_steps.md`）：
- `WAIT_STABLE` 不要把 `wheel_rad == 0.0f` 当作"未就绪"
- `EXECUTING` 阶段必须有超时
- DM MIT 打包必须支持负值
- 校准目标角度必须做"可发送范围一致化"

### 4.2 `RESTORE DEGRADED`
- `|diff| > 28672` 脉冲 → 系统判定降级
- 通常是断电后被人为大幅手推
- **必须重找零**：
  ```bash
  rm -f var_data/steer_unwrapped_pulses.txt
  export CAPTURE_STEER_ZERO=1
  export CAPTURE_STEER_ZERO_FORCE=1
  ./start_full_system.sh --sudo --ifname enp86s0
  ```

### 4.3 `wheel_deg_true` 物理零位不是 0
- 检查 `var_data/steer_zero_offsets.txt` 是否存在
- 文件丢失 → 走路径 C / C-PARTIAL，需重新找零（见 [04_steer_encoder.md](04_steer_encoder.md)）

### 4.4 `[CHASSIS][PERSIST-FAIL]`
- 写盘失败：检查 `var_data/` 目录权限和磁盘空间
- 不会中断控制循环，下次 200ms 周期会重试

## 5. 遥控链路

### 5.1 手柄无响应
```bash
ros2 topic echo /joy             # 看是否有事件
ls /dev/input/js*                # 看驱动是否识别
```

### 5.2 `/cmd_vel` 没数据
- 看 `remote_node_cpp` 是否启动
- `--no-vehicle` 模式下也应有 `/cmd_vel`
- 手柄死区参数：launch 默认 `deadzone=0.05`

### 5.3 只想测遥控不上车
```bash
./start_full_system.sh --sudo --ifname enp86s0 \
  ros2 launch linkx_bringup full_system.launch.py \
    ifname:=enp86s0 start_vehicle_control:=false
# 或
bash src/linkx_soem_demo/launch/run_link.sh --no-vehicle
```

## 6. 常用日志关键字

| 日志关键字 | 含义 |
| --- | --- |
| `[CHASSIS][STARTUP-STATE]` | 启动恢复路径摘要（A/B/C/C-PARTIAL） |
| `[CHASSIS][PERSIST-FAIL]` | 累计值落盘失败告警 |
| `[ENC] RESTORE DEGRADED` | 编码器恢复降级（diff 超阈值） |
| `[TASK] steer zero calibration captured` | 舵向找零成功 |
| `[LIVE-DASHBOARD]` / `[ENC]` | 周期性运行状态打印 |
| `[FATAL] Exception` | `main.cpp` 顶层异常捕获 |

## 7. 速查：「这事儿用哪个工具」

| 我想做什么 | 用什么 |
| --- | --- |
| 跑日常 | `./start_full_system.sh --sudo --ifname enp86s0` |
| 仅测遥控 | `--no-vehicle` 或 `start_vehicle_control:=false` |
| 看实时波形 | `tools/plot_feedback_wave.py` |
| ODrive 标定 | `odrive_calib --wheel <N> --test all` |
| DM 电机标定 | `tools/run_motor_calib.sh <N>` |
| 舵向找零 | `CAPTURE_STEER_ZERO=1 ./start_full_system.sh ...` |
| 舵向 PD 调参 | `tools/run_steer_step_test.sh` |
| 验证 CAN 链路 | `can_link_test` |
| 验证 ODrive 协议 | `odrive_protocol_test` |
| 验证 BRT 编码器 | `encoder_byteorder_test`（dry）/ `encoder_ack_test`（实物） |
| 上机回归（限速短转） | `robot_test` |
