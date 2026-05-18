# 05 · 标定指南

工程涉及四类标定，必须按顺序进行：

1. **ODrive 驱动轮参数**：摩擦力 / 静摩擦 / 转动惯量（CAN Simple 下发）
2. **DM 舵向电机参数**：Tc / Ts / J（MIT 力矩控制）
3. **舵向编码器零点**：参见 [04_steer_encoder.md](04_steer_encoder.md)
4. **4 轮稳态速度**：补偿 ODrive 4 轮稳态速度 ~1.5% 偏差，编译期常量 `kWheelSpeedCalib`

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

### 4.1 当前默认（2026-05-09 / 05-17 起）

舵向已改用 **DM 内置 MIT 位置环**：`mit_kp = 50`、`kd = 1.5`、`MAX = 12`，
软件 PD 退化为纯 FF。性能指标（60–90° step）：

- 800 ms 内 settled
- 稳态误差 ±0.1°
- 零振荡

软件层在 MIT 之外再串：

- **slew limiter**：MANUAL 500 °/s / SEMI_AUTO 200 °/s（见 [03_run_guide.md §9.1](03_run_guide.md)）
- **LPF**：MANUAL α=0.3 / SEMI_AUTO α=0.15（`τ = dt/α`）

> ⚠️ 同一 slew 数值下，不同 LPF 比例 τ 决定过冲：5000×0.1 (τ=20ms) 比
> 1667×0.3 (τ=6.7ms) 过冲低 6×。改前先想清楚 τ。

### 4.2 阶跃响应

```bash
sudo ./tools/run_steer_step_test.sh        # 单组阶跃
sudo ./tools/run_step_kd_sweep.sh          # Kd 扫参
sudo ./tools/run_remaining_tuning.sh       # 批量剩余用例
```

底层用 `steer_tuning` 可执行（`src/test_mains/steer_tuning_main.cpp`）：
MIT Kp/Kd + 外层位置环 + 力矩前馈，独立于 ROS2 / 遥控器。

### 4.3 数据分析

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

## 5. 4 轮稳态速度标定

### 5.1 现象

等命令下，ODrive 4 轮实测速度有 1~2% 偏差（实测 `[2.807, 2.872, 2.865, 2.859] rad/s`
对同一 cmd），前进/后退累计成净 yaw 漂；遥控时表现为「直行往左/往右偏」。

### 5.2 步骤

1. 跑遥控直行采样，收集 `manual_correction_*.csv`（车体 `chassis` 内置会写到 `var_data/`）。
2. 取稳态段（约 2000+ 样本），算 4 轮平均 `avg`。
3. 算 `k_i = avg / s_i`，让标定后 4 轮输出趋同。
4. 把数字填进 `crt_chassis.cpp:963` 的 `kWheelSpeedCalib[STEER_NUM]`：

```cpp
static constexpr float kWheelSpeedCalib[STEER_NUM] = {
  1.0157f,  // wheel 0
  0.9927f,  // wheel 1
  0.9951f,  // wheel 2
  0.9972f,  // wheel 3
};
```

5. `colcon build` 后重测，残差应 < 0.5%。

> ⚠️ 这个标定是 **编译期常量**，不靠环境变量。换轮 / 换电机后必须重测。
> 标定值乘进发生在 `Set_target_omega` 边界一次，不会进入控制循环常量乘法。

## 6. 输出端 slew limiter（2026-05-17）

ODrive `vel_ramp_rate` 实测不可信（配置 10，实测 28 rad/s²），所以在 chassis
端再串一层软 slew。环境变量调（不动代码也可改）：

| 变量 | 默认 | 含义 |
| --- | --- | --- |
| `WHEEL_OUTPUT_ACCEL` | `5.0` | 加速 rad/s²（启动柔和度） |
| `WHEEL_OUTPUT_DECEL` | `100.0` | 减速 rad/s²（用户主观最优） |

**已知现象**（不是 bug，根因在 ODrive PI，等 USB 调）：
- 停车 cmd→0 时反向位移 7~8 mm，跟 cmd 减速形态无关
- `DECEL` 在 5~30 区间最糟（vel_integrator 反向过冲），要么 5（柔），要么 100（接近阶跃）

`ACCEL = 5 / DECEL = 100` 是 2026-05-17 用户主观最优默认。

## 7. 舵向校准抽离（2026-05-13）

`Class_Chassis::Run_Steer_Calibration` 已抽到独立文件 `steer_calibration.{h,cpp}`，
方案 F：MIT 位置环 + slew + ω_des=0 + done 滞回，600 ms 收敛无过冲。

> ⚠️ 标定段**不要改 MIT 位置环参数**。之前两轮失败经验：
> 真根因是 `ω_des` 在加速、不是 `Kp` 大；F 方案 `ω_des = 0` 验证可行后才稳定。
> 如果重新调，先固定 `ω_des = 0` 再动 Kp。

调用入口仍是 `Class_Chassis::Begin_Steer_Calibration()`，下面直接转 `steer_calibration.cpp`。

## 8. 历史卡死问题（仅作背景了解）

`instructions/chassis_calibration_fix_steps.md` 记录了过去舵向校准"偶发卡死"问题的修复要点，主要风险点：

- `WAIT_STABLE` 不能把 `wheel_rad == 0.0f` 当作"无效数据"
- `EXECUTING` 阶段必须有超时
- 校准目标角度必须做"可发送范围一致化"
- DM MIT 打包时 `Math_Float_To_Int` 的 `y_min` 必须支持负值
- 保存零点命令的发送通道要正确

这些已在主线代码修复，文档保留作为历史索引。
