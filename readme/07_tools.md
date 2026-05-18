# 07 · 工具与脚本

## 1. 启动脚本

| 路径 | 用途 |
| --- | --- |
| `start_full_system.sh` | 工程根目录推荐入口（colcon build + launch） |
| `src/linkx_soem_demo/launch/run_link.sh` | 包内启动脚本（兼容旧用法） |
| `src/linkx_soem_demo/run_link.sh` | 包根的转发壳，最终调 launch/run_link.sh |

## 2. `tools/` 标定与调参脚本

### 2.1 `tools/run_motor_calib.sh`

DM 舵向电机标定（动摩擦 / 静摩擦 / 转动惯量）。

```bash
sudo ./tools/run_motor_calib.sh <wheel> [test] [extra args...]

sudo ./tools/run_motor_calib.sh 0                         # 全套
sudo ./tools/run_motor_calib.sh 0 friction                # 单项
sudo ./tools/run_motor_calib.sh 0 inertia --inertia_t 0.5 --friction 0.05
```

⚠️ 必须把对应轮架空再跑。

### 2.2 `tools/run_steer_step_test.sh`

舵向阶跃响应测试，单组用例。

### 2.3 `tools/run_step_kd_sweep.sh`

舵向 Kd 扫参（多组 Kd 自动跑，CSV 落到 `var_data/`）。

### 2.4 `tools/run_remaining_tuning.sh`

批量执行剩余调参用例（接续之前的扫参，或一次性跑完全套）。

### 2.5 `tools/plot_feedback_wave.py`

实时 ASCII 波形绘制（详见 [03_run_guide.md §7](03_run_guide.md)）。

```bash
./start_full_system.sh --sudo --ifname enp86s0 2>&1 | \
python3 tools/plot_feedback_wave.py \
  --series now_vx --series now_vy --series now_omega
```

| 参数 | 用途 |
| --- | --- |
| `--series <key>` | 要绘制的字段（重复多次可叠多条） |
| `--list-keys` | 打印当前所有字段名（便于先找） |
| `--csv <file>` | 同时保存采样数据为 CSV |
| `--y-min / --y-max` | 固定纵轴 |
| `--height / --history / --refresh-hz` | 高度 / 历史长度 / 刷新率 |

### 2.6 `tools/analyze_steer_trace.py`

解析 `steer_trace.cpp`（`chariot/chassis/steer_trace.cpp`）落盘的舵向跟踪 trace
文件，画出 setpoint / measured / err 波形。trace 文件路径在 `Class_Chassis` 内部决定，
日常调参时建议跟 `steer_tuning` 配合使用。

## 3. `var_data/` 数据分析脚本

### 3.1 综合

| 脚本 | 用途 |
| --- | --- |
| `analyze_all.py` | 一键综合分析所有结果 |
| `compare_sine.py` | 正弦用例两两对比 |

### 3.2 单轮调参

| 脚本 | 用途 |
| --- | --- |
| `plot_steer_tuning.py` | 单轮原始波形 |
| `plot_steer_tuning_results.py` | 单轮综合统计 |
| `plot_motor_rad.py` | 电机端角度 |
| `plot_wheel_rad.py` | 输出端（轮端）角度 |

### 3.3 4 轮统计

| 脚本 | 用途 |
| --- | --- |
| `plot_4wheels_step.py` | 4 轮阶跃响应对比 |
| `plot_4wheels_step_stats.py` | 阶跃统计指标 |
| `plot_4wheels_settle_zoom.py` | 整定时间放大 |
| `plot_4wheels_event_zoom.py` | 单事件放大 |
| `plot_4wheels_drift.py` | 漂移分析 |
| `plot_4wheels_sine.py` | 正弦跟踪 |
| `plot_4wheels_worst_step.py` | 最差阶跃用例 |

## 4. 实用命令片段

### 4.1 监听 ROS 2 话题

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /robot_buttons
ros2 topic list
```

### 4.2 看网卡状态

```bash
ip -br link
ip link show enp86s0
```

### 4.3 给可执行文件加 capability（避免 `--sudo`）

```bash
sudo setcap 'cap_net_raw,cap_net_admin+ep' \
    install/linkx_soem_demo/lib/linkx_soem_demo/linkx_soem_demo
```

`colcon build` 重装后 capability 会丢失，需要重新 `setcap`。

### 4.4 清理重编（异常时）

```bash
rm -rf build/ install/ log/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 4.5 查看持久化文件

```bash
cat var_data/steer_zero_offsets.txt
cat var_data/steer_unwrapped_pulses.txt
cat var_data/motor_calib_result.txt
cat var_data/odrive_calib_result.txt
```

### 4.6 修复 `var_data/*.txt` root 占用

跑过 `sudo` 后 `var_data/*.txt` 归 root，下次非 sudo 跑会刷
`[CHASSIS][PERSIST-FAIL]`。切模式前 `chown` 一次：

```bash
sudo chown -R $USER:$USER var_data/
```

## 5. 环境变量速查

| 变量 | 默认 | 用途 |
| --- | --- | --- |
| `CAPTURE_STEER_ZERO` | 未设 | 1 时启动后捕捉舵向零点 |
| `CAPTURE_STEER_ZERO_FORCE` | 未设 | 1 时强制覆盖已有零点 |
| `STEER_SLEW_RATE_DEG_S` | 500 | MANUAL profile slew |
| `STEER_LPF_ALPHA` | 0.30 | MANUAL profile LPF α |
| `STEER_SLEW_RATE_DEG_S_SEMI` | 200 | SEMI_AUTO profile slew |
| `STEER_LPF_ALPHA_SEMI` | 0.15 | SEMI_AUTO profile LPF α |
| `WHEEL_OUTPUT_ACCEL` | 5.0 | 驱动轮加速 rad/s² |
| `WHEEL_OUTPUT_DECEL` | 100.0 | 驱动轮减速 rad/s² |
| `IFNAME` | `enp86s0` | 测试可执行的网卡名 |
