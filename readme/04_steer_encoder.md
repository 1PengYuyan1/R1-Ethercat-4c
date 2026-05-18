# 04 · 舵向编码器：虚拟无限位移 + 掉电恢复

舵向编码器物理上是 **50 圈（204800 脉冲）有界** 的多圈绝对值，齿比 **3.5 : 1**（输出轴 1 圈 = 编码器 14336 脉冲）。

仓库实现「虚拟无限位移」方案：把它抽象为软件层的无限累计长轴 `L`，控制语义只关心
`phase = (L − logical_zero_anchor) mod 14336`，与物理 50 圈边界完全解耦。

## 1. 核心变量

| 名称 | 字段 | 含义 |
| --- | --- | --- |
| `L`（逻辑长轴） | `Class_Encoder_BRT::total_unwrapped_pulses` | 解卷后的全生命周期累计脉冲（int64） |
| `P_now` | `data.encoder_value % ENC_TRUE_MAX_PULSES` | 当前帧物理 raw（0..204799） |
| `P_last` | `Class_Encoder_BRT::last_raw_abs` | 上一帧的物理 raw |
| `L0`（角度参考点） | `Class_Encoder_BRT::logical_zero_anchor` | 找零时记录的 `L` |
| 舵角相位 | `phase = (L − L0) mod 14336` | 输出轴角度（与跨边界无关） |

## 2. 跨边界缝合公式（`Update_Unwrapped_Total`）

```text
delta = P_now - P_last
if delta < -102400 : delta += 204800   # 反向跨边界
if delta >  102400 : delta -= 204800   # 正向跨边界
L += delta
```

## 3. 掉电恢复（`Init_Restore_Position`，首帧 raw 触发）

```text
expected_raw = saved_pulses mod 204800
diff = current_raw_abs - expected_raw   # 同样做 ±102400 折叠
L = saved_pulses + diff
```

## 4. 启动恢复路径

`Class_Chassis::Init` 启动时会打印 `[CHASSIS][STARTUP-STATE]`，按下表判定：

| 路径 | zero 文件 | unwrapped 文件 | 表现 | 处理 |
| --- | --- | --- | --- | --- |
| A | 有效 | 有效 | 完全恢复，`L` 自动 diff 缝合 | 直接运行 |
| B | 有效 | 缺失 | 角度参考 OK，`L` 从 raw 起算 | 系统标 `degraded`，可继续运行 |
| C-PARTIAL | 缺失 | 有效 | 参考点丢失，机械零不归 0 | 必须 `CAPTURE_STEER_ZERO=1` 重找零 |
| C | 缺失 | 缺失 | 完全冷启动 | 必须 `CAPTURE_STEER_ZERO=1` 重找零 |

降级判据：若 `|diff| > 7 圈 × 4096 = 28672` 脉冲，
则 `degraded_mode=true`、`position_restore_ok=false`，并打印 `[ENC] RESTORE DEGRADED`。
通常意味着断电后被人为大幅手推 → 现场判停 + 重找零。

## 5. 重新找零流程

1. 把舵向手动/电控转到目标"零位"（如四轮全部正对前方）。
2. 若使用 BRT 编码器中点功能，先在该位置 `CAN_Send_SetMidpoint`。
3. 设环境变量后启动一次：

```bash
rm -f var_data/steer_unwrapped_pulses.txt
export CAPTURE_STEER_ZERO=1
export CAPTURE_STEER_ZERO_FORCE=1
./start_full_system.sh --sudo --ifname enp86s0
```

4. 看到 `[TASK] steer zero calibration captured` 表示已写入：

   - `var_data/steer_zero_offsets.txt`：raw 零点 + `logical_zero_anchor`
   - `var_data/steer_unwrapped_pulses.txt`：强制立即落盘，与 anchor 同步

5. 关掉环境变量，正常启动；从此 `wheel_deg_true` 在物理零位读 0。

```bash
unset CAPTURE_STEER_ZERO
unset CAPTURE_STEER_ZERO_FORCE
./start_full_system.sh --sudo --ifname enp86s0
```

## 6. 持久化文件格式

### 6.1 `var_data/steer_zero_offsets.txt`

```text
# 每行：<can_id> <raw_zero_offset> <logical_zero_anchor>
0x05  12345  -1234567
```

- `raw_zero_offset` ∈ [0, 204800)：raw 50 圈域内的零点 raw
- `logical_zero_anchor`：校准瞬间的 `total_unwrapped_pulses`（int64）
- 角度公式：`phase = (L − logical_zero_anchor) mod 14336`

### 6.2 `var_data/steer_unwrapped_pulses.txt`

```text
# 每行：<can_id> <magic_hex> <total_pulses> <raw_abs> <seq> <crc32_hex>
0x05 0x454e4350 -1476567 161833 1087 0xe4c504b6
```

| 字段 | 含义 |
| --- | --- |
| `magic` | `0x454E4350`（`'ENCP'`） |
| `total_pulses` | 软件累计长轴值（int64） |
| `raw_abs` | 当前帧 raw（用于诊断） |
| `seq` | 写盘序号，单调递增 |
| `crc32` | CRC32-IEEE，多项式 `0xEDB88320`，初值 `0xFFFFFFFF`，末值 `^0xFFFFFFFF` |

## 7. 排障命令

```bash
cat var_data/steer_zero_offsets.txt
cat var_data/steer_unwrapped_pulses.txt
```

运行中观察四轮恢复状态（`[LIVE-DASHBOARD]` → `[ENC]` 区域）：

```text
restore_ok=1   上电恢复成功且未降级
degraded=1     降级，建议手动确认/重找零
anchor_ok=1    逻辑零位锚点已加载
L=...          当前逻辑长轴累计值
L0=...         找零时记录的锚点
```

不变量自检：跨边界后必须满足

```text
L mod 204800 == raw_true_cal + zero_offset_true   (mod 204800)
```

## 8. 立即落盘的触发时机

主循环每 200 ms 调一次 `Save_Steer_Unwrapped_Pulses`（含 200ms 限频 + 无变化跳过）。
以下事件会调用 `Force_Save_Steer_Unwrapped_Pulses`（绕过限频）：

- `CAPTURE_STEER_ZERO=1` 找零成功（防止 anchor 与 L 不同步）
- 主循环退出前（Ctrl+C / SIGINT 触发）

写盘失败不会中断控制：会打印 `[CHASSIS][PERSIST-FAIL]` 告警，下次 200ms 周期自动重试。

## 9. BRT 编码器现场配置参考

> 来自实测记录（仅供参考，具体硬件可能不同）

- 通道：`ch=2`
- 波特率：`1 Mbps`
- 数量：`4` 路
- 自发周期：`5 ms`（理论 200 Hz）
- 已知问题：`enc3` 在某些硬件上稳定在 ~19 Hz（非软件问题）

## 10. 同 ID TX 队列陷阱

LinkX TX 队列对同一 CAN ID 的多帧采用「**同 ID 更新**」策略（旧的覆盖新的）。
对编码器 `set_*` 指令必须走 **quiet TX 路径**，避免被周期性 `read` 帧覆盖。
代码里集中实现于 `dvc_encoder.{h,cpp}`，不要绕过。

## 11. DM 舵向 ±PMAX 单圈相位 + wrap fix

舵向 DM 电机（DM2325 / 2026-05-17 起的固件）只用 **±PMAX 单圈相位**反馈：
`Get_Now_Radian()` 返回值始终钳在 `[-PMAX, +PMAX]`，跨过去后会从对面端跳回。

因此 `Class_Chassis::Steer_To_Motor_Position`（`crt_chassis.cpp:471`）做的事是：

1. 把 BRT 长轴 `phase`（无限累计）换算成期望电机位置 `motor_target`；
2. **anchor 折回**到 `[-PMAX, +PMAX]` 与固件单圈相位对齐；
3. 跨边界时只 clamp 不 wrap，否则电机会走长路震荡。

> ⚠️ 不要写 `motor_target = wrap(motor_target, ±PMAX)`：
> 之前曾因此让单方向转 ~1 圈卡死。

翻轮决策（`crt_chassis.cpp:634-640`）相关：
`overshoot = max(0, |motor_after_action| − 0.85×PMAX)`，离 ±PMAX 远时退化为
原 `|delta| > π/2` 翻态判据；接近边界时 overshoot 把翻轮 cost 拉高，避免
靠近 ±PMAX 的轮被单独翻 180° → yaw 漂。多轮决策已改为聚合（sum cost +
group hysteresis），不再各轮独立 cost。
