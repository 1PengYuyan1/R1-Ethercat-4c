# ENCODER Unwrap Powerloss Recovery Steps

适用范围：`Ethercat_R1` 舵向编码器虚拟无限位移（`int64_t total_unwrapped_pulses`）方案。

## 1. 持久化文件格式

文件：`var_data/steer_unwrapped_pulses.txt`

每行格式：

`<can_id> <magic_hex> <total_pulses> <seq> <crc32_hex>`

示例：

```text
# format: <can_id> <magic_hex> <total_pulses> <seq> <crc32_hex>
0x05 0x454e4350 28307 148 0xbd230d62
0x06 0x454e4350 17171 148 0xb92414a0
0x07 0x454e4350 7432 148 0x400e3ae1
0x08 0x454e4350 -696 148 0xeb67b245
```

字段说明：

- `can_id`：编码器 CAN ID。
- `magic_hex`：魔数，固定 `0x454E4350`（`'ENCP'`）。
- `total_pulses`：软件累计长轴值（`int64_t`）。
- `seq`：写盘序号（`uint32_t`），单调递增。
- `crc32_hex`：CRC32 校验值。

## 2. CRC32 算法

实现对齐：`crt_chassis.cpp::crc32_ieee`。

- 多项式：`0xEDB88320`（reflected）。
- 初值：`0xFFFFFFFF`。
- 末值异或：`^ 0xFFFFFFFF`。

CRC payload 字节序列：

- `can_id`（4 字节，小端）
- `total_pulses`（8 字节，小端）
- `seq`（4 字节，小端）

即：`can_id(LE4) || total_pulses(LE8) || seq(LE4)`。

## 3. 写盘策略

接口：`Class_Chassis::Save_Steer_Unwrapped_Pulses` 与 `Force_Save_Steer_Unwrapped_Pulses`。

- 常规路径：上层约每 200ms 调用一次 `Save`。
- 节流：`Save` 内部按 `min_interval_ms`（默认 200ms）限频。
- 变化检测：仅当有效编码器中存在 `total_pulses` 变化时才写盘。
- 强制路径：`Force_Save` 绕过节流，用于：
  - 零点捕获后立即落盘
  - 程序退出前落盘

## 4. 启动加载与恢复流程

1. `Load_Steer_Unwrapped_Pulses` 读取文件并校验 `magic + crc`。
2. 对每个编码器执行 `Set_Pending_Restore(saved_total)`。
3. 首帧到达后 `Update_Unwrapped_Total(raw_true)` 自动分流：
   - 若有 pending：调用 `Init_Restore_Position(saved, current_raw)`。
   - 若无 pending：冷启动，用当前 raw 初始化累计轴。
4. 后续每帧走差分缝合增量更新。

## 5. 降级判定

在 `Init_Restore_Position` 中：

- `diff = current_raw - expected_raw` 经环面最近邻缝合到 `[-102400, 102400]`。
- 若 `|diff| > 28672`（7 圈 = `7 * 4096`）则：
  - `degraded_mode = true`
  - `position_restore_ok = false`
  - 输出告警，建议现场重新校准。

## 6. 启动路径与现场处置

- PATH A：`zero_offsets` 与 `unwrapped` 都有效。
  - 行为：直接恢复；应看到 `restore_ok=1 degraded=0 anchor_ok=1`。
- PATH B：仅 `zero_offsets` 有效。
  - 行为：冷启动累计轴；标记降级。建议尽快执行一次校准。
- PATH C：`zero_offsets` 缺失或无效。
  - 行为：必须带 `CAPTURE_STEER_ZERO=1` 重新校准。

## 7. 核心不变量

- 解卷不变量：`L mod 204800 == raw_true`。
- 角度计算：`phase = (L - L0) mod 14336`。
- 角度输出：`wheel_angle = phase / 14336 * 360`。

其中 `L0` 为 `logical_zero_anchor`，在校准捕获瞬间写入零点文件并长期保持。
