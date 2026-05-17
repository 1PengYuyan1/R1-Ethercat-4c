# 底盘舵向校准“偶发卡死”详细修改步骤

## 1. 现象与目标

### 现象
- 有时能校准成功，有时卡在校准流程。
- 卡死时可见 DM 电机处于使能状态，且 `Kp/Kd` 已写入，但舵向不转。

### 本次修改目标
- 校准流程必须“可结束”：成功则完成，失败则超时退出并报错，不能无限卡死。
- 修复导致“看起来使能但不转”的关键控制路径问题。
- 提高可观测性，能快速定位是“编码器没数据”还是“电机没执行指令”。

---

## 2. 已定位的关键风险点（基于当前代码）

1. `WAIT_STABLE` 阶段把 `wheel_rad == 0.0f` 当作“无效数据”
- 文件：`src/linkx_soem_demo/src/vehicle_control/chariot/chassis/crt_chassis.cpp`
- 位置：约 `229-239` 行
- 问题：`0 rad` 本身是合法姿态，这个判断会把“合法零位”误判为未就绪，导致状态机无法进入下一步。

2. `EXECUTING` 阶段无超时
- 文件：同上
- 位置：约 `248-275` 行
- 问题：只要某个轮子始终达不到误差阈值，流程会无限停留在 `CALIB_STATE_EXECUTING`。

3. 校准目标角度没有做“可发送范围一致化”
- 文件：同上
- 位置：约 `200-213` 行（目标计算）
- 问题：目标基于 `Get_Now_Radian()` 累积值计算，后续发送阶段又会被限幅，可能出现“目标和实际发出的命令不一致”。

4. DM MIT 打包映射可疑（负角/负速/负扭矩可能被截断）
- 文件：`src/linkx_soem_demo/src/vehicle_control/device/Motor/dvc_motor_dm.cpp`
- 位置：约 `253-255` 行
- 问题：`Math_Float_To_Int` 调用使用 `y_min=0`，而实际控制量存在正负，可能导致一部分命令被压成中点/边界，表现为“有参数但不转”。

5. 保存零点命令发送错误
- 文件：同上
- 位置：约 `128-130` 行
- 问题：`CAN_Send_Save_Zero()` 当前发送的是 `Tx_Data`，不是 `DM_Motor_CAN_Message_Save_Zero`，会导致“看似执行保存零点，实际没发对命令”。

---

## 3. 详细修改步骤

### 步骤 1：先修复保存零点命令（低风险高收益）

修改 `Class_Motor_DM_Normal::CAN_Send_Save_Zero()`：
- 现状：
  - `linkx_quick_can_send(..., Tx_Data);`
- 改为：
  - `linkx_quick_can_send(..., DM_Motor_CAN_Message_Save_Zero);`

目的：确保校准完成后的零点保存是有效命令，不再引入“下一次上电行为随机”的问题。

---

### 步骤 2：给编码器增加“数据有效”判据，不再用 `wheel_rad == 0`

#### 2.1 在编码器类增加有效标志
文件：`src/linkx_soem_demo/include/linkx_soem_demo/vehicle_control/device/Motor/dvc_encoder.h`
- 在 `Struct_BRT_Encoder_Data` 增加：
  - `uint32_t rx_count;`
  - `uint8_t wheel_posture_valid;`
- 增加接口：
  - `inline bool Has_Valid_Wheel_Posture();`
  - `inline uint32_t Get_Rx_Count();`

#### 2.2 在接收流程里置位
文件：`src/linkx_soem_demo/src/vehicle_control/device/Motor/dvc_encoder.cpp`
- 在 `CAN_RxCpltCallback()` 中：`rx_count++`
- 在 `Data_Process()` 解析 `BRT_CMD_READ_ENCODER_VALUE` 成功后：
  - `wheel_posture_valid = 1;`

#### 2.3 校准等待阶段改判据
文件：`src/linkx_soem_demo/src/vehicle_control/chariot/chassis/crt_chassis.cpp`
- `WAIT_STABLE` 中把：
  - `Get_Wheel_Posture_radian() == 0.0f`
- 改为：
  - `Encoder_Steer[i].Get_Status() == BRT_STATUS_ENABLE`
  - 且 `Encoder_Steer[i].Has_Valid_Wheel_Posture()`

目的：彻底消除“合法 0 rad 被误判导致不进入下一步”的不确定性。

---

### 步骤 3：为校准状态机补全“阶段超时 + 失败退出”

文件：`src/linkx_soem_demo/include/linkx_soem_demo/vehicle_control/chariot/chassis/crt_chassis.h`
- 新增状态：
  - `CALIB_STATE_FAIL`
- 新增字段：
  - `uint32_t calib_exec_tick = 0;`
  - `uint8_t calib_fail_reason = 0;`（例如 1=编码器未就绪, 2=执行超时）

文件：`src/linkx_soem_demo/src/vehicle_control/chariot/chassis/crt_chassis.cpp`
- 在 `WAIT_STABLE` 增加总等待超时（例如 4s）。超时后进入 `CALIB_STATE_FAIL`。
- 在 `EXECUTING` 每周期 `calib_exec_tick++`，到达超时（例如 6s）后：
  - 置 `calib_fail_reason`
  - 进入 `CALIB_STATE_FAIL`
- 在 `FAIL` 分支中：
  - 给 DM 发安全保持/退出策略（例如 `omega=0` + 当前位置保持），
  - 打印失败原因，
  - 返回失败标志，不允许无限循环。

目的：不管成功失败，流程都能闭环退出。

---

### 步骤 4：校准目标计算改为“相对当前最短路径 + 可发范围一致化”

文件：`src/linkx_soem_demo/src/vehicle_control/chariot/chassis/crt_chassis.cpp`

将 `Steer_Calibration_Start()` 的目标计算改为：
1. `enc_rad` 归一化到 `[-pi, pi]` 的误差 `error_rad = normalize(0 - enc_rad)`。
2. 目标电机角 = 当前电机角 + `error_rad * REDUCTION_RATIO`。
3. 再做一次与发送逻辑一致的归一化/限制（与 `Execute_Steer_State()` 的 `motor_pos_cmd` 同规则）。

建议：把“电机角命令归一化”提取成公共函数（例如 `Normalize_Motor_Pos_Cmd()`），避免校准和运行态两套不同规则。

目的：避免“内部目标是 A，实际发出变成 B”导致转不动或来回抖。

---

### 步骤 5：修正 DM MIT 量化映射（重点）

文件：`src/linkx_soem_demo/src/vehicle_control/device/Motor/dvc_motor_dm.cpp`
- 当前（可疑）写法：
  - `Math_Float_To_Int(Control_Radian, 0, Radian_Max, 0x7fff, 65535)`
  - `Math_Float_To_Int(Control_Omega, 0, Omega_Max, 0x7ff, 4095)`
  - `Math_Float_To_Int(Control_Torque, 0, Torque_Max, 0x7ff, 4095)`

建议改成“对称区间映射”并与反馈解码一致：
- 位置：`[-Radian_Max, +Radian_Max] -> [0, 65535]`
- 速度：`[-Omega_Max, +Omega_Max] -> [0, 4095]`
- 扭矩：`[-Torque_Max, +Torque_Max] -> [0, 4095]`

同时同步检查 `Data_Process()` 的 `Math_Int_To_Float(...)` 反解范围，保证发/收使用同一对称区间。

目的：杜绝负方向命令被截断成“看起来发了但电机不动”。

---

### 步骤 6：校准执行阶段补充“最小运动监测”

文件：`src/linkx_soem_demo/src/vehicle_control/chariot/chassis/crt_chassis.cpp`
- 在 `EXECUTING` 中增加每轮监测：
  - 若 N 个周期（例如 200ms）内 `|enc_delta| < very_small` 且 `|wheel_err|` 未下降，判定该轮“无运动进展”。
- 对无进展轮打印：目标角、当前角、命令角、Kp/Kd、电机状态。
- 可选：对该轮执行一次恢复动作（`CAN_Send_Enter()` 或降低 `Kp` 后重试 1 次）。

目的：当硬件处于“已使能但不跟随”时，能快速识别并退出，而不是死等。

---

### 步骤 7：增强可观测性（必须做）

建议新增校准调试输出（建议 50~100ms 打印一次，避免刷屏）：
- `calib_step`
- `wait_tick / exec_tick`
- 每轮：`enc_rad`、`wheel_err`、`target_motor_pos`、`now_motor_pos`、`dm_status`
- 失败时打印 `calib_fail_reason`

这样你能一眼看出是：
- 编码器没回包，还是
- 命令量化异常，还是
- 电机已使能但误差不收敛。

---

## 4. 推荐修改顺序（实际执行顺序）

1. 修 `CAN_Send_Save_Zero()`。
2. 上“编码器有效判据”并替换 `wheel_rad==0` 判断。
3. 给状态机加 `FAIL` 和超时。
4. 做“校准目标一致化”。
5. 修 MIT 正负映射与反解一致性。
6. 加最小运动监测与日志。
7. 最后再调校准阈值（`0.03rad`、超时秒数、Kp/Kd）。

---

## 5. 回归验证步骤（每次改完都跑）

1. 冷启动后直接校准，重复 20 次，统计成功率（目标 100%）。
2. 将某轮机械对到接近 `0 rad` 再校准，确认不会卡在 `WAIT_STABLE`。
3. 人为断开一个编码器回包，确认在超时后进入 `FAIL`，不会无限卡死。
4. 人为限制某轮无法转动，确认 `EXECUTING` 超时退出并打印失败原因。
5. 校准成功后重启上电，检查零点是否保持一致（验证 `Save_Zero` 修复）。

---

## 6. 建议验收标准

- 校准流程不再出现无限等待。
- 任何失败都能在日志中定位到具体轮和具体原因。
- 舵向存在负方向目标时仍可稳定执行。
- 连续多次上电校准结果一致，无“这次能校下次不能”的随机性。
