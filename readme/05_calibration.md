# 05 · 标定指南

工程涉及三类标定，必须按顺序进行：

1. **ODrive 驱动轮参数**：摩擦力 / 静摩擦 / 转动惯量（CAN Simple 下发）
2. **DM 舵向电机参数**：Tc / Ts / J（MIT 力矩控制）
3. **舵向编码器零点**：参见 [04_steer_encoder.md](04_steer_encoder.md)

## 1. ODrive 驱动轮标定

### 1.1 用途
通过 CAN 控制 ODrive 速度/力矩模式，读取编码器和 IQ 反馈，输出 `J / B / Tc / Ts`。

### 1.2 运行

```bash
sudo IFNAME=enp86s0 \
    ./install/linkx_soem_demo/lib/linkx_soem_demo/odrive_calib --wheel 0 --test all
```

参数：

| 参数 | 用途 |
| --- | --- |
| `--wheel <0..3>` | 标定的轮号 |
| `--test all\|friction\|stiction\|inertia` | 单项或全部 |
| `--inertia_t / --friction` | 复用其他测试得到的中间值 |

### 1.3 状态机切换注意事项

- Makerbase 版本 ODrive 可能要**反复重发** `SET_ClosedLoop` / `Emergency_Stop` 才能稳定切换（已知问题）
- ODrive `Reboot` 后必须等 **≥ 15 s** 才能重连，否则会失败

### 1.4 输出
`var_data/odrive_calib_result.txt`，例：

```text
===== 2026-05-07 03:52:31  wheel=0  test=all  Kt=0.0827 =====
FRICTION: T+=0.0000  T-=0.0000  AVG=0.0000  ...
STICTION: T=0.5000  breakaway omega=0.000  success=0
INERTIA: J=0.000000  alpha=0.000  T_step=0.1500  success=0
```

> 注：当前记录的 4 轮全部标定成功（Tc/Ts/J），wheel 0 已修复。详情看 `var_data/odrive_calib_result.txt`。

## 2. DM 舵向电机标定

### 2.1 用途
单轮电机参数标定（动摩擦力 / 静摩擦力 / 转动惯量）。复用 chassis 初始化（编码器/电机使能等），调用 motor 的 `Begin_*_Calibration()` API。

⚠️ **必须把对应车轮架空**（轮胎离地）后再标定。

### 2.2 运行

封装脚本：

```bash
sudo IFNAME=enp86s0 ./tools/run_motor_calib.sh <wheel> [test] [extra args...]

# 例子：
sudo ./tools/run_motor_calib.sh 0                           # 全部测试
sudo ./tools/run_motor_calib.sh 0 friction                  # 只测动摩擦
sudo ./tools/run_motor_calib.sh 0 stiction                  # 只测静摩擦
sudo ./tools/run_motor_calib.sh 0 inertia --inertia_t 0.5 --friction 0.05
sudo ./tools/run_motor_calib.sh 1 friction --omega 3.0 --measure 3.0
```

直跑可执行：

```bash
sudo ./install/linkx_soem_demo/lib/linkx_soem_demo/motor_calib \
    --wheel 0 --test all
```

### 2.3 输出
`var_data/motor_calib_result.txt`，例：

```text
===== 2026-05-06 00:59:39  wheel=0  test=all =====
FRICTION: T+=0.0403  T-=0.0408  AVG=0.0405  omega+=1.8730  omega-=-2.0215
STICTION: T=0.0400  breakaway omega=0.3297  success=1
INERTIA: J=0.005638  alpha=81.495  T_step=0.5000  friction_used=0.0405  success=1
```

## 3. 舵向编码器零点

完整流程见 [04_steer_encoder.md](04_steer_encoder.md)。

最小步骤：

```bash
rm -f var_data/steer_unwrapped_pulses.txt
export CAPTURE_STEER_ZERO=1
export CAPTURE_STEER_ZERO_FORCE=1
./start_full_system.sh --sudo --ifname enp86s0
```

看到 `[TASK] steer zero calibration captured` 后停止，再 `unset` 环境变量正常启动。

## 4. 舵向 PD / 力矩前馈调参

### 4.1 阶跃响应

```bash
sudo ./tools/run_steer_step_test.sh        # 单组阶跃
sudo ./tools/run_step_kd_sweep.sh          # Kd 扫参
sudo ./tools/run_remaining_tuning.sh       # 批量剩余用例
```

底层用 `steer_tuning` 可执行（`src/test_mains/steer_tuning_main.cpp`）：MIT Kp/Kd + 外层位置环 + 力矩前馈，独立于 ROS2 / 遥控器。

### 4.2 数据分析

`var_data/` 内已存大量调参 CSV / PNG，对应脚本：

| Python 脚本 | 用途 |
| --- | --- |
| `plot_steer_tuning.py` | 单轮原始波形 |
| `plot_steer_tuning_results.py` | 单轮综合统计 |
| `plot_4wheels_step.py` | 4 轮阶跃响应对比 |
| `plot_4wheels_sine.py` | 4 轮正弦跟踪 |
| `plot_4wheels_settle_zoom.py` | 整定时间放大 |
| `plot_4wheels_event_zoom.py` | 单事件放大 |
| `plot_4wheels_drift.py` | 漂移分析 |
| `plot_4wheels_step_stats.py` | 阶跃统计指标 |
| `plot_4wheels_worst_step.py` | 最差阶跃用例 |
| `plot_motor_rad.py` / `plot_wheel_rad.py` | 电机端 / 轮端角度 |
| `analyze_all.py` | 一键综合分析 |
| `compare_sine.py` | 正弦用例两两对比 |

## 5. 历史卡死问题（仅作背景了解）

`instructions/chassis_calibration_fix_steps.md` 记录了过去舵向校准"偶发卡死"问题的修复要点，主要风险点：

- `WAIT_STABLE` 不能把 `wheel_rad == 0.0f` 当作"无效数据"
- `EXECUTING` 阶段必须有超时
- 校准目标角度必须做"可发送范围一致化"
- DM MIT 打包时 `Math_Float_To_Int` 的 `y_min` 必须支持负值
- 保存零点命令的发送通道要正确

这些已在主线代码修复，文档保留作为历史索引。
