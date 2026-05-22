//
// Created by pzx on 2025/12/20.
//
#include "crt_chassis.h"
#include <math.h>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>

namespace {
inline uint64_t chassis_now_ms()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

// === Init_Motor_Params 用的 env 覆盖辅助 (从函数体提到 ns 顶层；函数体不变) ===
inline float parse_env_f32(const char *name, float fallback)
{
  const char *v = std::getenv(name);
  if (!v || v[0] == '\0')
    return fallback;
  char *end = nullptr;
  const float parsed = std::strtof(v, &end);
  return (end == v) ? fallback : parsed;
}

inline void load_wheel_env(int idx_0based, SteerWheelParams &p)
{
  const int id = idx_0based + 1;
  const std::string prefix = "STEER_ID" + std::to_string(id) + "_";
  p.steer_pos_kp = parse_env_f32((prefix + "POS_KP").c_str(), p.steer_pos_kp);
  p.steer_pos_kd = parse_env_f32((prefix + "POS_KD").c_str(), p.steer_pos_kd);
  p.mit_kp = parse_env_f32((prefix + "MIT_KP").c_str(), p.mit_kp);
  p.mit_kd = parse_env_f32((prefix + "MIT_KD").c_str(), p.mit_kd);
  p.steer_omega_deadzone = parse_env_f32((prefix + "OMEGA_DZ").c_str(), p.steer_omega_deadzone);
  p.wheel_omega_deadzone = parse_env_f32((prefix + "WHEEL_OMEGA_DZ").c_str(), p.wheel_omega_deadzone);
  p.wheel_feedforward = parse_env_f32((prefix + "WHEEL_FF").c_str(), p.wheel_feedforward);
  p.wheel_direction = parse_env_f32((prefix + "WHEEL_DIR").c_str(), p.wheel_direction);
  p.flip_speed_threshold = parse_env_f32((prefix + "FLIP_SPEED_TH").c_str(), p.flip_speed_threshold);
  p.flip_drive_scale = parse_env_f32((prefix + "FLIP_DRIVE_SCALE").c_str(), p.flip_drive_scale);
  p.steer_inertia = parse_env_f32((prefix + "INERTIA").c_str(), p.steer_inertia);
  p.steer_damping = parse_env_f32((prefix + "DAMPING").c_str(), p.steer_damping);
  p.steer_static_friction = parse_env_f32((prefix + "STATIC_FRICTION").c_str(), p.steer_static_friction);
  // 驱动轮（ODrive）动力学参数
  p.wheel_inertia = parse_env_f32((prefix + "WHEEL_J").c_str(), p.wheel_inertia);
  p.wheel_damping = parse_env_f32((prefix + "WHEEL_B").c_str(), p.wheel_damping);
  p.wheel_friction = parse_env_f32((prefix + "WHEEL_TC").c_str(), p.wheel_friction);
  p.wheel_stiction = parse_env_f32((prefix + "WHEEL_TS").c_str(), p.wheel_stiction);
  p.wheel_alpha_limit = parse_env_f32((prefix + "WHEEL_ALPHA_LIM").c_str(), p.wheel_alpha_limit);
  p.wheel_torque_limit = parse_env_f32((prefix + "WHEEL_TORQUE_LIM").c_str(), p.wheel_torque_limit);
}
}  // namespace

/**
 * @brief 初始化硬件
 */
void Class_Chassis::Init(linkx_t *__LinkX_Handler)
{
  LinkX_Handler = __LinkX_Handler;

  // 三组 4 元素显式表：保留显式数值比"聪明"循环 +0x08 更不易错。
  // 特别注意 ODrive can_id 0x10/0x18/0x20/0x28 步长是 8（节点间隔 4×2），不是顺序值。
  static const struct { uint8_t can_id; uint8_t dm_id; } kSteerMotors[STEER_NUM] = {
    {0x11, 0x01}, {0x12, 0x02}, {0x13, 0x03}, {0x14, 0x04},
  };
  static const uint8_t kODriveCanIds[STEER_NUM] = {0x10, 0x18, 0x20, 0x28};
  static const uint8_t kEncoderCanIds[STEER_NUM] = {0x05, 0x06, 0x07, 0x08};

  for (int i = 0; i < STEER_NUM; ++i)
    Motor_Steer[i].Init(LinkX_Handler, 0, kSteerMotors[i].can_id, kSteerMotors[i].dm_id,
                        Motor_DM_Control_Method_NORMAL_MIT, 21.99f, 150.0f, 5.0f, 15.0f);

  for (int i = 0; i < STEER_NUM; ++i)
    ODrive_Motor_Steer[i].Init(LinkX_Handler, 1, kODriveCanIds[i]);

  for (int i = 0; i < STEER_NUM; ++i)
    Encoder_Steer[i].Init(LinkX_Handler, 2, kEncoderCanIds[i], 4096);

  // OPS-9 全方位平面定位（RS232 → 泥人转换器 → EtherCAT-4C 通道 3）
  // 转换器配置：UART 115200 8N1（与 OPS 一致）；CAN 1Mbps 经典模式；CAN ID = 0x01
  // 透传模式：转换器把 OPS 28B 串口帧拆为 4 个 CAN 帧（DLC=8/8/8/4）共用同一 ID
  OPS.Init(LinkX_Handler, /*can_channel=*/3, /*can_id=*/0x01);

  // 加载持久化的舵向编码器零点偏移（让设了中点的物理位置 → wheel_rad = 0）
  const bool zero_loaded = Load_Steer_Zero_Offsets(Default_Steer_Zero_Offsets_Path());

  // 加载多圈累计脉冲（断电恢复）；首帧 raw 到达后 Update_Unwrapped_Total 自动 diff 缝合
  const bool unwrapped_loaded = Load_Steer_Unwrapped_Pulses(Default_Steer_Unwrapped_Pulses_Path());

  // === 启动状态机摘要：四种路径用一行 KV 形式打印 ===
  //   zero=OK/MISS unwrapped=OK/MISS → 推断
  //   PATH A  zero+unwrapped 都 OK   → 期望恢复，首帧验证
  //   PATH B  zero OK + unwrapped MISS → 允许冷启动，降级到下次 CAPTURE_STEER_ZERO
  //   PATH C-PARTIAL  zero MISS + unwrapped OK → anchor 丢失，wheel_rad 在机械零位不为 0
  //   PATH C  全部 MISS              → 冷启动，所有编码器降级
  const char *path_msg;
  if (zero_loaded && unwrapped_loaded)        path_msg = "PATH A: restore expected (verify on first frame)";
  else if (zero_loaded && !unwrapped_loaded)  path_msg = "PATH B: degraded mode (run CAPTURE_STEER_ZERO=1 to bootstrap)";
  else if (!zero_loaded && unwrapped_loaded)  path_msg = "PATH C-PARTIAL: angle anchor lost (run CAPTURE_STEER_ZERO=1)";
  else                                        path_msg = "PATH C: cold-start (run CAPTURE_STEER_ZERO=1 to bootstrap)";
  std::cout << "[CHASSIS] startup: zero=" << (zero_loaded ? "OK" : "MISS")
            << " unwrapped=" << (unwrapped_loaded ? "OK" : "MISS")
            << " → " << path_msg << std::endl;
}

const char *Class_Chassis::Default_Steer_Zero_Offsets_Path()
{
  const char *env = std::getenv("STEER_ZERO_OFFSETS_FILE");
  if (env && env[0] != '\0')
    return env;
  return "var_data/steer_zero_offsets.txt";
}

// === 零点偏移持久化 (thin wrapper —— 实现见 encoder_persistence.cpp) ===

bool Class_Chassis::Load_Steer_Zero_Offsets(const std::string &path)
{
  return encoder_persistence::LoadZeroOffsets(Encoder_Steer, path);
}

bool Class_Chassis::Capture_And_Save_Steer_Zero_Offsets(const std::string &path)
{
  if (!encoder_persistence::SaveZeroOffsets(Encoder_Steer, path))
    return false;
  // Homing (零点捕获) 完成后：清除异常锁定，允许正常运行
  if (steer_abnormal_locked_)
    Clear_Steer_Abnormal_Lock();
  return true;
}

const char *Class_Chassis::Default_Steer_Unwrapped_Pulses_Path()
{
  const char *env = std::getenv("STEER_UNWRAPPED_PULSES_FILE");
  if (env && env[0] != '\0')
    return env;
  return "var_data/steer_unwrapped_pulses.txt";
}

// === 多圈累计值持久化 (thin wrapper —— 实现见 encoder_persistence.cpp) ===

bool Class_Chassis::Load_Steer_Unwrapped_Pulses(const std::string &path)
{
  return encoder_persistence::LoadUnwrappedPulses(Encoder_Steer, persist_state_, path);
}

bool Class_Chassis::Save_Steer_Unwrapped_Pulses(const std::string &path,
                                                uint32_t min_interval_ms)
{
  return encoder_persistence::SaveUnwrappedPulses(
    Encoder_Steer, persist_state_, path, min_interval_ms);
}

bool Class_Chassis::Force_Save_Steer_Unwrapped_Pulses(const std::string &path,
                                                      const char *reason)
{
  return encoder_persistence::ForceSaveUnwrappedPulses(
    Encoder_Steer, persist_state_, path, reason);
}

// =================== 舵向编码器上层工具函数 ===================

float Class_Chassis::SteerAngleFromTotal(int64_t total_pulses)
{
  return static_cast<float>(total_pulses) * 360.0f / STEER_PULSES_PER_TURN_F;
}

float Class_Chassis::SteerRadianFromTotal(int64_t total_pulses)
{
  return static_cast<float>(total_pulses) * 2.0f * PI / STEER_PULSES_PER_TURN_F;
}

int64_t Class_Chassis::RestoreTotalWithRaw(int64_t saved_total, int32_t saved_raw, int32_t current_raw)
{
  int32_t delta = current_raw - saved_raw;
  if (delta > ENC_HALF_PULSES_I32)
    delta -= ENC_MAX_PULSES_I32;
  else if (delta < -ENC_HALF_PULSES_I32)
    delta += ENC_MAX_PULSES_I32;
  return saved_total + static_cast<int64_t>(delta);
}

bool Class_Chassis::CheckPowerOffDelta(int32_t delta)
{
  const int32_t abs_delta = (delta < 0) ? -delta : delta;
  return abs_delta > ENC_ABNORMAL_THRESHOLD;
}

void Class_Chassis::Set_Steer_Abnormal_Lock()
{
  steer_abnormal_locked_ = true;
  std::cerr << "[CHASSIS] 舵向异常锁定已激活：禁止运动，等待 Homing" << std::endl;
}

void Class_Chassis::Clear_Steer_Abnormal_Lock()
{
  steer_abnormal_locked_ = false;
  std::cout << "[CHASSIS] 舵向异常锁定已解除（Homing 完成）" << std::endl;
}

/**
 * @brief 初始化参数
 * @note  4 轮共享 15 个调参字段（kSteerDefaults），各自的 J/T_friction (motor-axis) 与
 *        J/Tc/Ts (ODrive) 5 个标定值在 kPerWheelCalib 表里按轮覆盖。
 */
void Class_Chassis::Init_Motor_Params()
{
  // ★ 2026-05 sweep + 2026-05-17 增量调参：mit_kp=50/kd=2.5 + slew=5000×α=0.1 (eff 500°/s, τ=20ms)
  //   sweep 阶段 (5月初): mit_kd=1.5, sine RMS 0.45° / peak 1.70° (vs 旧配置 1.93°/2.93°)
  //   2026-05-17: mit_kd 1.5→2.5 抑制 4Hz hold 振; slew/alpha 改 5000/0.1 让 τ=20ms,
  //               快反向瞬态过冲从 ~30° 降到 <5° (eff slew 同 500°/s 不变)
  static const SteerWheelParams kSteerDefaults = {
    .steer_pos_kp = 15.0f,
    .steer_pos_kd = 0.4f,
    .mit_kp = 50.0f,
    .mit_kd = 2.5f,
    .steer_omega_deadzone = 1.2f / REDUCTION_RATIO,
    .wheel_omega_deadzone = 0.05f,
    .wheel_feedforward = 0.5f,
    .wheel_direction = 1.0f,
    .flip_speed_threshold = 2.0f,
    .flip_drive_scale = 0.4f,
    .steer_inertia = 0.0f,           // overridden per-wheel
    .steer_damping = 0.0f,
    .steer_static_friction = 0.0f,   // overridden per-wheel
    .wheel_inertia = 0.0f,           // overridden per-wheel
    .wheel_damping = 0.0f,
    .wheel_friction = 0.0f,          // overridden per-wheel
    .wheel_stiction = 0.0f,          // overridden per-wheel
    .wheel_alpha_limit = 100.0f,
    .wheel_torque_limit = 1.0f,
  };

  // 每轮独立标定值（按索引 0~3 对应 ID1~ID4）：
  //   motor-axis : 2026-05-06 calib 静摩擦补偿（原注释里有 T_stiction 但代码不用）
  //   ODrive     : 2026-05-07 悬空 calib 给 J/Tc；Ts 用 2026-05-10 在地辨识值
  //                （仅替换 Ts 测试启动顿挫；R² 见每行末注释）
  static const struct {
    float steer_J;             // motor-axis J [kg·m²]    -> steer_inertia
    float steer_T_friction;    // motor-axis T_friction [Nm] -> steer_static_friction
    float wheel_J;             // ODrive J [kg·m²]         -> wheel_inertia
    float wheel_Tc;            // ODrive Tc [Nm] 库仑动摩擦  -> wheel_friction
    float wheel_Ts;            // ODrive Ts [Nm] 在地辨识静摩擦 -> wheel_stiction
  } kPerWheelCalib[STEER_NUM] = {
    /* W0 */ {0.00564f, 0.0405f, 0.010f, 0.078f, 2.310f},  // odrive R²=0.752
    /* W1 */ {0.00519f, 0.0428f, 0.006f, 0.048f, 2.398f},  // odrive R²=0.611
    /* W2 */ {0.00499f, 0.0488f, 0.010f, 0.069f, 2.448f},  // odrive R²=0.735
    /* W3 */ {0.00503f, 0.0493f, 0.008f, 0.090f, 2.463f},  // odrive R²=0.705
  };

  for (int i = 0; i < STEER_NUM; ++i)
  {
    steer_wheel_params_[i] = kSteerDefaults;
    steer_wheel_params_[i].steer_inertia         = kPerWheelCalib[i].steer_J;
    steer_wheel_params_[i].steer_static_friction = kPerWheelCalib[i].steer_T_friction;
    steer_wheel_params_[i].wheel_inertia         = kPerWheelCalib[i].wheel_J;
    steer_wheel_params_[i].wheel_friction        = kPerWheelCalib[i].wheel_Tc;
    steer_wheel_params_[i].wheel_stiction        = kPerWheelCalib[i].wheel_Ts;
  }

  // 2026-05-23 过冲不对称修正:w0/w1 静摩擦比 w2/w3 低 ~20%,同 mit_kd 下严重欠阻尼
  // (实测 90° step 过冲 w0/w1 25-37% vs w2/w3 -13~+5%, 振铃 2-4Hz)
  // 抬高 w0/w1 的 mit_kd 补回阻尼差; w2/w3 也抬到 3.5 让 stick-slip snap 后刹得住
  steer_wheel_params_[0].mit_kd = 3.5f;
  steer_wheel_params_[1].mit_kd = 3.5f;
  steer_wheel_params_[2].mit_kd = 3.5f;
  steer_wheel_params_[3].mit_kd = 3.5f;

  // 可选：运行时用环境变量覆盖单轮参数（无需重编译）
  // 命名示例：STEER_ID2_POS_KP / STEER_ID2_POS_KD / STEER_ID2_MIT_KP / STEER_ID2_MIT_KD
  // load_wheel_env / parse_env_f32 定义在 file-scope 匿名 ns。
  for (int i = 0; i < STEER_NUM; ++i)
    load_wheel_env(i, steer_wheel_params_[i]);

  for (int i = 0; i < STEER_NUM; ++i)
  {
    std::cout << "[CHASSIS] steer param ID" << (i + 1)
              << " pos_kp=" << steer_wheel_params_[i].steer_pos_kp
              << " pos_kd=" << steer_wheel_params_[i].steer_pos_kd
              << " mit_kp=" << steer_wheel_params_[i].mit_kp
              << " mit_kd=" << steer_wheel_params_[i].mit_kd
              << " omega_dz=" << steer_wheel_params_[i].steer_omega_deadzone
              << " wheel_omega_dz=" << steer_wheel_params_[i].wheel_omega_deadzone
              << " wheel_ff=" << steer_wheel_params_[i].wheel_feedforward
              << " wheel_dir=" << steer_wheel_params_[i].wheel_direction
              << " J=" << steer_wheel_params_[i].steer_inertia
              << " B=" << steer_wheel_params_[i].steer_damping
              << " Ts=" << steer_wheel_params_[i].steer_static_friction
              << std::endl;
  }

  // 兼容旧字段（已不参与运行阶段控制，保留以免影响其它路径）
  Steer_Inertia = 1.000f;
  Steer_Damping = 1.00f;
  Steer_Static_Friction = 1.00f;

  // 校准模块依赖 steer_wheel_params_,在这里绑定(Init_Motor_Params 之后)
  steer_calib_.Bind(Motor_Steer, Encoder_Steer, steer_wheel_params_, REDUCTION_RATIO);
}

/**
 * @brief 100ms 定时器：检测电机心跳
 */
void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
  // OPS 存活判活：100ms 窗口（OPS 200Hz 出帧，余量充足）
  // 不调这行，OPS.Get_Status() 永远 DISABLE，robot.cpp 手动纠偏分支进不去
  OPS.TIM_Alive_PeriodElapsedCallback();

  for (int i = 0; i < STEER_NUM; i++)
  {
    Motor_Steer[i].TIM_Alive_PeriodElapsedCallback();
    Encoder_Steer[i].TIM_Alive_PeriodElapsedCallback();
    ODrive_Motor_Steer[i].TIM_Alive_CheckCallback();
  }

  if (Chassis_Control_Type != Chassis_Control_Type_ENABLE ||
    !steer_calib_.Is_Complete())
    return;

  for (int i = 0; i < STEER_NUM; i++)
  {
    if (Motor_Steer[i].Get_Status() != Motor_DM_Status_ENABLE)
      Motor_Steer[i].CAN_Send_Enter();

    uint32_t current_error = ODrive_Motor_Steer[i].Get_Axis_Error();
    if (current_error != AXIS_ERROR_NONE)
    {
      ODrive_Motor_Steer[i].Clear_Errors();
      continue;
    }

    if (ODrive_Motor_Steer[i].Get_Axis_State() != ODRIVE_STATE_CLOSED_LOOP_CONTROL)
    {
      // 切到 VEL_RAMP 模式：让 ODrive 内部对速度命令做斜坡，避免阶跃造成 iq 撞限幅
      // vel_ramp_rate 由 ODrive 固件配置（CAN 协议无对应命令，需 USB odrivetool 调）
      ODrive_Motor_Steer[i].Set_Control_Mode(ODRIVE_CTRL_VELOCITY, ODRIVE_INPUT_VEL_RAMP);
      ODrive_Motor_Steer[i].SET_ClosedLoop();
    }
  }
}

/**
 * @brief 2ms 定时器：姿态/速度解算
 */
void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback()
{
  Self_Resolution();
  
    //     for (int i = 0; i < STEER_NUM; i++)
    // Encoder_Steer[i].TIM_Query_PeriodElapsedCallback();
}

/**
 * @brief 2ms 定时器：底盘控制主循环
 */
void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback()
{
  // 舵向异常锁定：断电位移超阈值，禁止一切舵向运动，直到 Homing 完成
  if (steer_abnormal_locked_)
  {
    for (int i = 0; i < STEER_NUM; i++)
    {
      Motor_Steer[i].Set_Control_Status(Motor_DM_Status_DISABLE);
      Motor_Steer[i].Set_Control_Parameter_MIT(0.0f, 0.0f, 0.0f);
      if (Motor_Steer[i].Get_Now_Control_Status() != Motor_DM_Status_DISABLE)
        Motor_Steer[i].CAN_Send_Exit();
      if (ODrive_Motor_Steer[i].Get_Axis_State() != ODRIVE_STATE_IDLE)
      {
        ODrive_Motor_Steer[i].Emergency_Stop();
        ODrive_Motor_Steer[i].Set_Velocity(0.0f);
      }
    }
    return;
  }

  if (!steer_calib_.Is_Complete())
  {
    for (int i = 0; i < STEER_NUM; i++)
    {
      if (Motor_Steer[i].Get_Status() != Motor_DM_Status_ENABLE)
        Motor_Steer[i].CAN_Send_Enter();
    }
    uint8_t result = steer_calib_.Tick();
    if (result == 1)
    {
      // 校准刚完成:接管 chassis 端状态(原 Calib_Done 末段做的)
      for (int i = 0; i < STEER_NUM; i++) steer_ref_filtered[i] = 0.0f;
    }
    for (int i = 0; i < STEER_NUM; i++)
      Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
    return;
  }

  switch (Chassis_Control_Type)
  {
  case Chassis_Control_Type_DISABLE:
    // 停止所有电机，清除目标
    for (int i = 0; i < STEER_NUM; i++)
    {
      Motor_Steer[i].Set_Control_Status(Motor_DM_Status_DISABLE);
      Motor_Steer[i].Set_Control_Parameter_MIT(0.0f, 0.0f, 0.0f);
      if (Motor_Steer[i].Get_Now_Control_Status() != Motor_DM_Status_DISABLE)
        Motor_Steer[i].CAN_Send_Exit();

      if (ODrive_Motor_Steer[i].Get_Axis_State() != ODRIVE_STATE_IDLE)
      {
        ODrive_Motor_Steer[i].Emergency_Stop();
        ODrive_Motor_Steer[i].Set_Velocity(0.0f);
        ODrive_Motor_Steer[i].Set_Torque(0.0f);
      }

      Target_Wheel_Omega[i] = 0.0f;
      Target_Wheel_torque[i] = 0.0f;
      wheel_omega_cmd_prev_[i] = 0.0f;  // 紧急停车清 slew 状态，避免下次启动从旧值 ramp
    }
    Target_Velocity_X = 0.0f;
    Target_Velocity_Y = 0.0f;
    Target_Omega = 0.0f;
    return;

  case Chassis_Control_Type_ENABLE:

    Kinematics_Inverse_Resolution();
    Output_To_Motor();
    break;
  }
}

void Class_Chassis::Steer_Calibration_Init()
{
  steer_calib_.Init();
}

/**
 * @brief 获取当前舵向角度（虚拟无限位移方案：唯一观测源 = 编码器逻辑相位域）
 *
 * 设计原则（与 VIRTUAL_UNLIMITED_DISPLACEMENT_PLAN.md 对齐）：
 *   - 所有舵角闭环误差计算（Self_Resolution / Compute_Wheel_Raw_Targets /
 *     Update_Flip_Decision / Shape_Steer_Targets / Update_Steer_State /
 *     Execute_Steer_State）统一从此函数读取舵角，
 *     单一真实语义。
 *   - 数据来自 Encoder_Steer[].Get_Wheel_Posture_radian_True()（逻辑相位域，
 *     由 dvc_encoder.cpp 用 phase = (L - logical_zero_anchor) mod 14336 计算）。
 *   - 电机端 Get_Now_Radian() 仅用于"位置环换算"（Steer_To_Motor_Position
 *     与校准阶段位置目标），不再作为舵角状态观测主源。
 *   - 输出归一化到 [0, 2π)，与底盘内部坐标系保持一致。
 *
 * 编码器 invalid 时（启动早期未收到首帧）回退到电机轴反算，避免控制环白屏；
 * 但这种情况只在校准前发生，校准之后编码器一定有效。
 */
float Class_Chassis::Get_Now_Steer_Radian(int index)
{
  if (index < 0 || index >= STEER_NUM)
    return 0.0f;

  // 主路径：从编码器逻辑相位域读取（虚拟无限位移核心入口）
  if (Encoder_Steer[index].Has_Valid_Wheel_Posture())
  {
    float enc_rad = Encoder_Steer[index].Get_Wheel_Posture_radian_True();
    return Math_Modulus_Normalization(enc_rad, 2.0f * PI);
  }

  // 回退路径：编码器尚未收到首帧时用电机轴反算（仅启动早期短暂使用）
  float real_wheel_rad = Motor_Steer[index].Get_Now_Radian() / REDUCTION_RATIO;
  return Math_Modulus_Normalization(real_wheel_rad, 2.0f * PI);
}

/**
 * @brief 把舵向目标角转为电机 MIT 位置指令
 *
 * DM 固件用线性 PD（2026-05-14 trace 实测，不是 mod 最短弧），所以 host 端
 * wrap target 会让电机走长路震荡。这里只 clamp 不 wrap：跨边界时停在 ±PMAX
 * 不弹到对侧；Update_Flip_Decision 的 cost-flip 会预防电机累加到边界。
 */
float Class_Chassis::Steer_To_Motor_Position(float target_steer, int index)
{
  float error = target_steer - Get_Now_Steer_Radian(index);
  if (error >  PI) error -= 2.0f * PI;
  if (error < -PI) error += 2.0f * PI;

  const float pmax   = Motor_Steer[index].Get_Radian_Max();
  const float period = 2.0f * pmax;

  // anchor 折回 [-PMAX, +PMAX] 与固件单圈相位对齐
  float anchor = fmodf(Motor_Steer[index].Get_Now_Radian() + pmax, period);
  if (anchor < 0.0f) anchor += period;
  anchor -= pmax;

  float motor_target = anchor + error * REDUCTION_RATIO;
  if (motor_target > pmax)       { motor_target =  pmax; steer_boundary_hits_[index]++; }
  else if (motor_target < -pmax) { motor_target = -pmax; steer_boundary_hits_[index]++; }
  return motor_target;
}

/**
 * @brief 整车速度正解算
 * @note  每周期先清零，再从各轮积分
 */
void Class_Chassis::Self_Resolution()
{
  Now_Velocity_X = 0.0f;
  Now_Velocity_Y = 0.0f;
  Now_Omega = 0.0f;

  for (int i = 0; i < STEER_NUM; i++)
  {
    float omega = ODrive_Motor_Steer[i].Get_Omega();
    // 统一观测源：通过 Get_Now_Steer_Radian 走逻辑相位域（≡ encoder ctrl）
    float enc_rad = Get_Now_Steer_Radian(i);

    Now_Velocity_X += omega * Wheel_Radius * cosf(enc_rad) / 4.0f;
    Now_Velocity_Y += omega * Wheel_Radius * sinf(enc_rad) / 4.0f;
    Now_Omega += omega * Wheel_Radius *
      sinf(enc_rad - Wheel_Azimuth[i]) /
      (Wheel_To_Core_Distance[i] * 4.0f);
  }
}

/**
 * @brief 运动学逆解算：由目标速度计算各轮舵向目标角和轮速目标
 */
/**
 * @brief 运动学逆解算：由目标速度计算各轮舵向目标角和轮速目标
 *
 * 链路（每 2ms 调用一次）：
 *   1. 输入限幅 + 全局死区
 *   2. Compute_Wheel_Raw_Targets  → 写 Target_Steer_Rad_Cmd（未翻轮）+ omega_raw
 *   3. Update_Flip_Decision        → 只更新 steer_flipped[]，决策用「未 slew」的真目标
 *   4. 主函数合成最终 cmd          → 翻轮的轮速取反 + 目标角 ±π
 *   5. Shape_Steer_Targets         → slew + LPF + FF velocity，写 Target_Steer_Rad / Omage
 *
 * 关键不变量：
 *   - Target_Steer_Rad_Cmd[i] = 翻轮后但未 slew 的真目标，Execute_Steer_State 用它算 cos 投影
 *   - Target_Steer_Rad[i]     = slew/LPF 后送 MIT 位置环的舵向目标
 *   - flip 期间 |Cmd - current| ≈ π → proj = 0 → 轮速归零，避免"先朝反方向冲一下"
 */
void Class_Chassis::Kinematics_Inverse_Resolution()
{
  Math_Constrain(&Target_Omega, -MAX_CHASSIS_OMEGA, MAX_CHASSIS_OMEGA);

  // 全局死区：整体输入太小直接停车，但仍要走整形以更新 Target_Steer_Rad/Omage
  // 2026-05-21: 0.02 → 0.001。原 0.02 m/s 在摇杆 max_speed=0.1 下要 20% 位移才过门槛,
  // 实测 cmd 5/10/15 mm/s 全被截 (Δ=0), 20 mm/s 一下冲 300mm — 用户感受到的"推大位移才动"。
  const float input_mod = sqrtf(Target_Velocity_X * Target_Velocity_X +
                                Target_Velocity_Y * Target_Velocity_Y +
                                Target_Omega * Target_Omega);
  if (input_mod < 0.001f)
  {
    for (int i = 0; i < STEER_NUM; i++)
    {
      Target_Wheel_Omega[i] = 0.0f;
      Target_Steer_Rad_Cmd[i] = Get_Now_Steer_Radian(i);  // 锁住，避免 slew 漂移
      steer_flipped[i] = 0;
    }
    Shape_Steer_Targets();
    return;
  }

  // 平移速度限幅
  const float chassis_speed = sqrtf(Target_Velocity_X * Target_Velocity_X +
                                    Target_Velocity_Y * Target_Velocity_Y);
  if (chassis_speed > MAX_CHASSIS_SPEED)
  {
    const float scale = MAX_CHASSIS_SPEED / chassis_speed;
    Target_Velocity_X *= scale;
    Target_Velocity_Y *= scale;
  }

  // 1) 逆解 atan2 + 轮速归一化（写 Target_Steer_Rad_Cmd，omega_raw 局部）
  float omega_raw[STEER_NUM];
  Compute_Wheel_Raw_Targets(omega_raw);

  // 2) 翻轮决策（只更新 steer_flipped[]，不直接改 Cmd / omega）
  Update_Flip_Decision(omega_raw);

  // 3) 合成最终命令：翻轮 → Cmd += π，轮速取反
  for (int i = 0; i < STEER_NUM; i++)
  {
    if (steer_flipped[i])
    {
      Target_Steer_Rad_Cmd[i] = Math_Modulus_Normalization(
        Target_Steer_Rad_Cmd[i] + PI, 2.0f * PI);
      Target_Wheel_Omega[i] = -omega_raw[i];
    }
    else
    {
      Target_Wheel_Omega[i] = omega_raw[i];
    }
  }

  // 4) 目标整形：slew + LPF + FF velocity（写 Target_Steer_Rad / Target_Steer_Omage）
  Shape_Steer_Targets();
}

/**
 * @brief 逆解每轮的 raw 目标角（atan2）和 raw 轮速，写入 Target_Steer_Rad_Cmd 和 omega_raw
 *
 * - 该轮平移分量过小时 omega_raw = 0，Cmd 保持当前舵向（避免乱跳）
 * - 完成后做一次全局轮速归一化
 */
void Class_Chassis::Compute_Wheel_Raw_Targets(float omega_raw[STEER_NUM])
{
  float max_omega = 0.0f;
  for (int i = 0; i < STEER_NUM; i++)
  {
    float sin_phi = sinf(Wheel_Azimuth[i]);
    float cos_phi = cosf(Wheel_Azimuth[i]);
    if (fabsf(sin_phi) < 0.001f) sin_phi = 0.001f;
    if (fabsf(cos_phi) < 0.001f) cos_phi = 0.001f;

    const float vx = Target_Velocity_X -
                     Target_Omega * Wheel_To_Core_Distance[i] * sin_phi;
    const float vy = Target_Velocity_Y +
                     Target_Omega * Wheel_To_Core_Distance[i] * cos_phi;
    const float v_mod = sqrtf(vx * vx + vy * vy);

    if (v_mod < 0.0005f)
    {
      omega_raw[i] = 0.0f;
      Target_Steer_Rad_Cmd[i] = Get_Now_Steer_Radian(i);
      continue;
    }

    Target_Steer_Rad_Cmd[i] = Math_Modulus_Normalization(atan2f(vy, vx), 2.0f * PI);
    omega_raw[i] = v_mod / Wheel_Radius;
    if (omega_raw[i] > max_omega) max_omega = omega_raw[i];
  }

  const float MAX_WHEEL_OMEGA = MAX_CHASSIS_SPEED / Wheel_Radius;
  if (max_omega > MAX_WHEEL_OMEGA && max_omega > 1e-4f)
  {
    const float scale = MAX_WHEEL_OMEGA / max_omega;
    for (int i = 0; i < STEER_NUM; i++) omega_raw[i] *= scale;
  }
}

/**
 * @brief 翻轮决策（cost-function）：cost = |delta| + λ × overshoot²
 *
 * overshoot = max(0, |motor_after_action| - 0.85×PMAX)。离边界远时 overshoot=0
 * 退化为 |delta| 较小者胜 ≈ 原 |delta|>π/2 阈值；接近 ±PMAX 时 penalty 把翻轮
 * 代价拉低，自动把电机推回中心。rotation_dominant 时禁"新翻"，wheel ω 低于死
 * 区时强制解锁翻（避免低速 sign 翻飞）。
 *
 * 全局一致性 (2026-05-16): 纯平移时(|Target_Omega|<0.05 且有平移),4 轮的 kinematics
 * 目标角相同 → 翻态必须统一。原各轮独立决策会因单轮电机靠近 ±PMAX 而单独翻 180°,
 * 形成 3 轮正向 + 1 轮反向的 3:1 力不平衡,前进段 yaw 1.3° 累计漂移。改为聚合 cost:
 * sum(cost_no) vs sum(cost_flip),滞回放大到 N×kHysteresis 防 flap。rotation_dominant
 * 段保持原各轮独立(各轮 kinematics 目标角本就不同)。
 */
void Class_Chassis::Update_Flip_Decision(const float omega_raw[STEER_NUM])
{
  const float trans_speed = sqrtf(Target_Velocity_X * Target_Velocity_X +
                                  Target_Velocity_Y * Target_Velocity_Y);
  float max_radius = 0.0f;
  for (int i = 0; i < STEER_NUM; i++)
    if (Wheel_To_Core_Distance[i] > max_radius) max_radius = Wheel_To_Core_Distance[i];
  const bool rotation_dominant = (fabsf(Target_Omega) > 0.1f) &&
                                 (fabsf(Target_Omega) * max_radius > trans_speed);

  constexpr float kDangerFrac = 0.85f, kPenaltyW = 3.0f, kHysteresis = 0.15f;
  constexpr float kPureTransOmegaThr = 0.05f;  // rad/s
  const bool pure_translation = (fabsf(Target_Omega) < kPureTransOmegaThr) &&
                                (trans_speed > 0.01f);

  float cost_no_arr[STEER_NUM]   = {0.0f};
  float cost_flip_arr[STEER_NUM] = {0.0f};
  bool  idle[STEER_NUM]          = {false};

  // 第一遍: 算每个轮的 cost_no / cost_flip(纯平移和独立路径共用)
  for (int i = 0; i < STEER_NUM; i++)
  {
    if (fabsf(omega_raw[i]) < steer_wheel_params_[i].wheel_omega_deadzone) {
      idle[i] = true;
      continue;
    }

    float delta = Math_Modulus_Normalization(
      Target_Steer_Rad_Cmd[i] - Get_Now_Steer_Radian(i), 2.0f * PI);
    if (delta > PI) delta -= 2.0f * PI;

    const float pmax   = Motor_Steer[i].Get_Radian_Max();
    const float period = 2.0f * pmax;
    float anchor = fmodf(Motor_Steer[i].Get_Now_Radian() + pmax, period);
    if (anchor < 0.0f) anchor += period;
    anchor -= pmax;

    const float delta_flip = (delta >= 0.0f) ? (delta - PI) : (delta + PI);
    const float thr        = kDangerFrac * pmax;
    const float ov_no      = fmaxf(0.0f, fabsf(anchor + delta      * REDUCTION_RATIO) - thr);
    const float ov_flip    = fmaxf(0.0f, fabsf(anchor + delta_flip * REDUCTION_RATIO) - thr);
    cost_no_arr[i]   = fabsf(delta)      + kPenaltyW * ov_no   * ov_no;
    cost_flip_arr[i] = fabsf(delta_flip) + kPenaltyW * ov_flip * ov_flip;
  }

  if (pure_translation && !rotation_dominant)
  {
    // 纯平移: 聚合 4 轮 cost,统一翻态
    float total_cost_no = 0.0f, total_cost_flip = 0.0f;
    int active = 0;
    for (int i = 0; i < STEER_NUM; i++) {
      if (idle[i]) continue;
      total_cost_no   += cost_no_arr[i];
      total_cost_flip += cost_flip_arr[i];
      active++;
    }
    const float group_hyst = kHysteresis * (float)active;  // 组级滞回
    bool new_global = steer_flipped_global_;
    if (active > 0) {
      if (!steer_flipped_global_) {
        if (total_cost_flip + group_hyst < total_cost_no)
          new_global = true;
      } else {
        if (total_cost_no + group_hyst < total_cost_flip)
          new_global = false;
      }
    }
    steer_flipped_global_ = new_global;
    for (int i = 0; i < STEER_NUM; i++)
      steer_flipped[i] = idle[i] ? 0 : (new_global ? 1 : 0);
  }
  else
  {
    // rotation_dominant 或显式 omega 命令: 各轮独立决策(原行为)
    for (int i = 0; i < STEER_NUM; i++) {
      if (idle[i]) { steer_flipped[i] = 0; continue; }
      if (!steer_flipped[i]) {
        if (cost_flip_arr[i] + kHysteresis < cost_no_arr[i] && !rotation_dominant)
          steer_flipped[i] = 1;
      } else {
        if (cost_no_arr[i] + kHysteresis < cost_flip_arr[i])
          steer_flipped[i] = 0;
      }
    }
  }
}

/**
 * @brief 目标整形：slew rate → LPF → FF velocity
 *
 * 输入：Target_Steer_Rad_Cmd[i]（合成后的真命令，含翻轮）
 * 输出：Target_Steer_Rad[i]（slew+LPF 后，送 MIT 位置环）
 *       Target_Steer_Omage[i]（dθ/dt，送 MIT velocity 前馈）
 *
 * 默认 slew 500°/s（motor 物理极限 wheel-side ≈ 359°/s，留余量）；
 * 默认 LPF α=0.3 → 稳态 lag ≈ max_step/α ≈ 3.3°，约 ~7ms 命令端滞后。
 *
 * 2026-05-18: 按 drive_mode_ 切换 profile。半自动默认更柔（200°/s + α=0.15，τ ~13ms），
 * 用于路径跟随拐点段抑制过冲；手动保留原 500/0.3 响应。
 *
 * env 覆盖（一次启动加载）：
 *   STEER_SLEW_RATE_DEG_S       : MANUAL    slew, °/s, default 500
 *   STEER_LPF_ALPHA             : MANUAL    LPF α, [0,1], default 0.3
 *   STEER_SLEW_RATE_DEG_S_SEMI  : SEMI_AUTO slew, °/s, default 200
 *   STEER_LPF_ALPHA_SEMI        : SEMI_AUTO LPF α, [0,1], default 0.15
 *
 * state 收进成员（steer_shape_init_ / steer_ref_filtered / steer_ff_last_target_），重启可见。
 */
void Class_Chassis::Shape_Steer_Targets()
{
  constexpr float kDt = 0.002f;

  static const auto parse_env_f = [](const char *name, float fallback) -> float {
    const char *v = std::getenv(name);
    if (!v || v[0] == '\0') return fallback;
    char *end = nullptr;
    float val = std::strtof(v, &end);
    return (end == v) ? fallback : val;
  };
  static const float kSlewManual = parse_env_f("STEER_SLEW_RATE_DEG_S",     5000.0f) * (PI / 180.0f);
  static const float kLpfManual  = parse_env_f("STEER_LPF_ALPHA",           0.1f);
  // SEMI 默认 4500×0.1 = eff 450°/s, τ=20ms。比手动 (5000×0.1=500°/s) 略慢,同等 τ 阻尼
  // （[[project_steer_slew_lpf_bug]] τ=dt/α 经验）,在拐角软化 + cos⁴ 减速保护下足够丝滑。
  static const float kSlewSemi   = parse_env_f("STEER_SLEW_RATE_DEG_S_SEMI", 4500.0f) * (PI / 180.0f);
  static const float kLpfSemi    = parse_env_f("STEER_LPF_ALPHA_SEMI",       0.1f);

  const bool semi = (drive_mode_ == Drive_Mode_SEMI_AUTO);
  const float kSlewRadPerS = semi ? kSlewSemi : kSlewManual;
  const float kLpfAlpha    = semi ? kLpfSemi  : kLpfManual;
  const float max_step = kSlewRadPerS * kDt;

  for (int i = 0; i < STEER_NUM; i++)
  {
    if (!steer_shape_init_[i])
    {
      const float now = Get_Now_Steer_Radian(i);
      steer_ref_filtered[i] = now;
      steer_ff_last_target_[i] = now;
      Target_Steer_Rad[i] = now;
      Target_Steer_Omage[i] = 0.0f;
      steer_shape_init_[i] = true;
      continue;
    }

    // Slew
    float dtg = Math_Modulus_Normalization(
      Target_Steer_Rad_Cmd[i] - steer_ref_filtered[i], 2.0f * PI);
    if (dtg > PI) dtg -= 2.0f * PI;
    if (dtg >  max_step) dtg =  max_step;
    if (dtg < -max_step) dtg = -max_step;

    // LPF
    steer_ref_filtered[i] = Math_Modulus_Normalization(
      steer_ref_filtered[i] + kLpfAlpha * dtg, 2.0f * PI);
    Target_Steer_Rad[i] = steer_ref_filtered[i];

    // FF velocity = dθ/dt
    float dtheta = Math_Modulus_Normalization(
      Target_Steer_Rad[i] - steer_ff_last_target_[i], 2.0f * PI);
    if (dtheta > PI) dtheta -= 2.0f * PI;
    float ff = dtheta / kDt;
    if (ff >  MAX_STEER_OMEGA) ff =  MAX_STEER_OMEGA;
    if (ff < -MAX_STEER_OMEGA) ff = -MAX_STEER_OMEGA;
    Target_Steer_Omage[i] = ff;
    steer_ff_last_target_[i] = Target_Steer_Rad[i];
  }
}

/**
 * @brief 驱动轮（ODrive）动力学前馈：α 限幅 → model FF → 力矩限幅
 *        计算 Target_Wheel_torque[i]，作为 CAN 0x00D Set_Input_Vel 的 torque_ff
 */
void Class_Chassis::Compute_Wheel_Feedforward(int i, float dt)
{
  const auto &p = steer_wheel_params_[i];

  // 未标定时退回零力矩
  if (p.wheel_inertia <= 0.0f && p.wheel_friction <= 0.0f)
  {
    Target_Wheel_torque[i] = 0.0f;
    return;
  }

  // 1) α 限幅
  float omega_target = Target_Wheel_Omega[i];
  float alpha_target = (omega_target - wheel_last_omega_[i]) / dt;
  if (p.wheel_alpha_limit > 0.0f)
  {
    Math_Constrain(&alpha_target, -p.wheel_alpha_limit, p.wheel_alpha_limit);
    omega_target = wheel_last_omega_[i] + alpha_target * dt;
    Target_Wheel_Omega[i] = omega_target;
  }
  wheel_last_omega_[i] = omega_target;

  // 2) 模型前馈：T_ff = J*α + B*ω + Tc*tanh(ω/0.5)
  float T_ff = p.wheel_inertia * alpha_target
             + p.wheel_damping * omega_target
             + p.wheel_friction * tanhf(omega_target / 0.5f);

  // 3) 力矩限幅
  if (p.wheel_torque_limit > 0.0f)
    Math_Constrain(&T_ff, -p.wheel_torque_limit, p.wheel_torque_limit);

  Target_Wheel_torque[i] = T_ff;
}

/**
 * @brief 状态机：判断舵轮状态切换
 */
void Class_Chassis::Update_Steer_State(int i)
{
  float current = Get_Now_Steer_Radian(i);
  float delta =
    Math_Modulus_Normalization(Target_Steer_Rad[i] - current, 2.0f * PI);
  if (delta > PI) delta -= 2.0f * PI;

  float omega_thresh = steer_wheel_params_[i].wheel_omega_deadzone;

  switch (steer_state[i])
  {
  case STEER_STATE_IDLE:
    if (fabsf(Target_Wheel_Omega[i]) > omega_thresh)
      steer_state[i] = STEER_STATE_ALIGN;
    break;

  case STEER_STATE_ALIGN:
    if (fabsf(delta) < 0.15f)
      steer_state[i] = STEER_STATE_DRIVE;
    else if (fabsf(Target_Wheel_Omega[i]) < omega_thresh)
      steer_state[i] = STEER_STATE_IDLE;
    break;

  case STEER_STATE_DRIVE:
    if (fabsf(delta) > 0.15f && fabsf(Target_Wheel_Omega[i]) > omega_thresh)
      steer_state[i] = STEER_STATE_ALIGN;
    else if (fabsf(Target_Wheel_Omega[i]) < omega_thresh)
      steer_state[i] = STEER_STATE_IDLE;
    break;

  default:
    steer_state[i] = STEER_STATE_IDLE;
    break;
  }
}

/**
 * @brief 状态机：根据当前状态执行电机指令
 *        2026-05-09: ALIGN/DRIVE 改用 MIT 内置位置环
 *        - 位置目标 = current_motor_rad + steer_error × REDUCTION（最短路径，每 tick 重算）
 *        - mit_kp 由 steer_wheel_params_.mit_kp 提供（=15），mit_kd 提供阻尼
 *        - Target_Steer_Omage 退化为 motor velocity feedforward（dtheta/dt × REDUCTION）
 *        IDLE 保持 mit_kp=0，仅 velocity damping，让轮可被外力推动不锁死
 */
void Class_Chassis::Execute_Steer_State(int i)
{
  const float dir = steer_wheel_params_[i].wheel_direction;
  const float ff_motor_omega = Target_Steer_Omage[i] * REDUCTION_RATIO;
  const float motor_kp = steer_wheel_params_[i].mit_kp;
  const float motor_kd = steer_wheel_params_[i].mit_kd;

  if (steer_state[i] == STEER_STATE_IDLE)
  {
    /* IDLE：mit_kp=0 让舵向可被外力推动；motor_kd 提供 velocity damping 抑制自转。 */
    Motor_Steer[i].Set_Control_Torque_P_D_MIT(0.0f, 0.0f, motor_kd);
    Motor_Steer[i].Set_Control_Parameter_MIT(0.0f, 0.0f);
    ODrive_Motor_Steer[i].Set_target_omega(0.0f);
    ODrive_Motor_Steer[i].Set_target_torque(0.0f);
    wheel_omega_cmd_prev_[i] = 0.0f;  // 重置输出端 slew，避免下次 DRIVE 启动从旧值 ramp
    return;
  }

  /* ALIGN / DRIVE：MIT 位置环锁舵向 + ODrive 按【三段 gate】驱动 wheel。
   *
   * ★ Gate 设计（替代原 cos 投影）：
   *   - err < dead_rad        → gate = 1.0   全速放行
   *   - dead_rad < err < soft_rad → 线性 1→0
   *   - err > soft_rad        → gate = 0     完全切断（含 flip 期间 err≈π）
   *
   *   原 cos 投影在 err=60° 时还有 0.5 倍速 → 舵向滞后期 wheel 已弱命令运行，
   *   既偏姿态也容易撞 ODrive 静摩擦堵转。改成 hard gate 后舵向到位前 wheel 完全静止，
   *   到位后再全速冲，对 ODrive 启动友好。
   *
   *   默认 dead=10° / soft=25°；可用 env STEER_GATE_DEAD_DEG / STEER_GATE_SOFT_DEG 覆盖。
   *
   * ALIGN 与 DRIVE 唯一差别：ALIGN 不发 torque_ff（仅 omega 驱动收尾） */
  static const auto parse_gate_deg = [](const char *name, float fallback_deg) -> float {
    const char *v = std::getenv(name);
    if (!v || v[0] == '\0') return fallback_deg * (PI / 180.0f);
    char *end = nullptr;
    float deg = std::strtof(v, &end);
    return (end == v) ? fallback_deg * (PI / 180.0f) : deg * (PI / 180.0f);
  };
  static const float kGateDeadRad = parse_gate_deg("STEER_GATE_DEAD_DEG", 10.0f);
  static const float kGateSoftRad = parse_gate_deg("STEER_GATE_SOFT_DEG", 25.0f);

  const float motor_pos_target = Steer_To_Motor_Position(Target_Steer_Rad[i], i);
  Motor_Steer[i].Set_Control_Torque_P_D_MIT(
    Target_Steer_Torque[i], motor_kp, motor_kd);
  Motor_Steer[i].Set_Control_Parameter_MIT(motor_pos_target, ff_motor_omega);

  const float current = Get_Now_Steer_Radian(i);
  float err = Math_Modulus_Normalization(
    Target_Steer_Rad_Cmd[i] - current, 2.0f * PI);
  if (err > PI) err -= 2.0f * PI;
  const float abs_err = fabsf(err);

  float gate;
  if (abs_err <= kGateDeadRad)
    gate = 1.0f;
  else if (abs_err >= kGateSoftRad)
    gate = 0.0f;
  else
    gate = (kGateSoftRad - abs_err) / (kGateSoftRad - kGateDeadRad);

  // 每轮速度标定 (2026-05-17): 从 manual_correction_20260516_234114.csv 算出
  // 稳态对齐 2855 样本,4 轮实测速度 [2.807, 2.872, 2.865, 2.859] rad/s 同命令,
  // k_i = avg / s_i 让标定后 4 轮输出趋同(残余 <0.5%).
  // 修复目的: 等命令下 4 轮实测速度有 1-2% 偏差,前进/后退累计成净 yaw 漂.
  static constexpr float kWheelSpeedCalib[STEER_NUM] = {
    1.0157f,  // wheel 0
    0.9927f,  // wheel 1
    0.9951f,  // wheel 2
    0.9972f,  // wheel 3
  };

  // 输出端 slew limiter (2026-05-17): 限制下发到 ODrive 的 omega 变化率。
  // Why: 诊断数据显示 ODrive 端 vel_ramp_rate 不可靠 (实测 28 rad/s² vs 配置 10);
  //      启动瞬间 cmd 阶跃 + 速度环反向冲击 iq~36A 撞 current_lim,车窜出去 0.3cm.
  // 加减速独立 (2026-05-17): 停车 cmd→0 太快触发 ODrive vel_integrator 反向过冲 → 车回拉 1~2mm.
  //   - WHEEL_OUTPUT_ACCEL 默认 5    (启动柔和度)
  //   - WHEEL_OUTPUT_DECEL 默认 100  (用户主观最优:几乎阶跃停车响应快;客观反向位移 7~8mm
  //                                   不变 — 跟 cmd 减速形态无关,根因在 ODrive PI,等 USB 调)
  static const auto parse_env_f = [](const char *name, float fallback) -> float {
    const char *v = std::getenv(name);
    if (!v || v[0] == '\0') return fallback;
    char *end = nullptr;
    float x = std::strtof(v, &end);
    return (end == v) ? fallback : x;
  };
  static const float kWheelOutputAccel = parse_env_f("WHEEL_OUTPUT_ACCEL", 5.0f);
  static const float kWheelOutputDecel = parse_env_f("WHEEL_OUTPUT_DECEL", 100.0f);
  constexpr float kCtrlDt = 0.002f;  // TIM_2ms_Control_PeriodElapsedCallback

  const float omega_desired =
    Target_Wheel_Omega[i] * gate * dir * kWheelSpeedCalib[i];

  // gate=0 (舵向误差超 soft) 时硬清 prev,而不是按 slew 慢慢降到 0.
  // 否则 DRIVE→ALIGN 时 prev 要 ~1s 才 ramp 到 0 (5 rad/s² × 5 rad/s),
  // 此时舵向已转完 90° (180ms),wheel 全程被舵向带着画弧 → 底盘被拽走.
  // slew 仅保留"启动加速"职责防 ODrive 静摩擦堵转,不参与"停下".
  if (gate <= 0.0f)
  {
    wheel_omega_cmd_prev_[i] = 0.0f;
  }
  else
  {
    // 减速判定：|目标| < |当前缓存| (含异号反向也算先减速穿 0)
    const bool decelerating =
      fabsf(omega_desired) < fabsf(wheel_omega_cmd_prev_[i]);
    const float max_step =
      (decelerating ? kWheelOutputDecel : kWheelOutputAccel) * kCtrlDt;
    float delta = omega_desired - wheel_omega_cmd_prev_[i];
    if (delta >  max_step) delta =  max_step;
    if (delta < -max_step) delta = -max_step;
    wheel_omega_cmd_prev_[i] += delta;
  }

  ODrive_Motor_Steer[i].Set_target_omega(wheel_omega_cmd_prev_[i]);
  if (steer_state[i] == STEER_STATE_DRIVE)
    ODrive_Motor_Steer[i].Set_target_torque(Target_Wheel_torque[i] * gate * dir);
  else
    ODrive_Motor_Steer[i].Set_target_torque(0.0f);
}

/**
 * @brief 输出到电机
 */
void Class_Chassis::Output_To_Motor()
{
  for (int i = 0; i < STEER_NUM; i++)
  {
    Update_Steer_State(i);
    Compute_Wheel_Feedforward(i, 0.002f);
    Execute_Steer_State(i);
  }

  Steer_Trace_Log();

  for (int i = 0; i < STEER_NUM; i++)
    Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
  for (int i = 0; i < STEER_NUM; i++)
    ODrive_Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
      for (int i = 0; i < STEER_NUM; i++)
    Encoder_Steer[i].TIM_Query_PeriodElapsedCallback();
}

/**
 * @brief 高频 CSV trace（500Hz, 跟控制循环同节拍）
 *        env STEER_TRACE_FILE=/path.csv 触发；未设则 no-op。
 *        ofstream + 列定义封装在 SteerTrace（trace_ 成员）。
 *        本函数只采集 chassis 状态、灌进 PerWheelTrace[] 后调 LogTick。
 */
void Class_Chassis::Steer_Trace_Log()
{
  PerWheelTrace per_wheel[STEER_NUM];
  for (int i = 0; i < STEER_NUM; ++i)
  {
    per_wheel[i].target_deg       = Target_Steer_Rad[i] * 180.0f / PI;
    per_wheel[i].current_deg      = Get_Now_Steer_Radian(i) * 180.0f / PI;
    per_wheel[i].motor_target_rad = Steer_To_Motor_Position(Target_Steer_Rad[i], i);
    per_wheel[i].motor_now_rad    = Motor_Steer[i].Get_Now_Radian();
    per_wheel[i].motor_omega      = Motor_Steer[i].Get_Now_Omega();
    per_wheel[i].motor_torque     = Motor_Steer[i].Get_Now_Torque();
    per_wheel[i].state            = static_cast<int>(steer_state[i]);
    per_wheel[i].boundary_hits    = steer_boundary_hits_[i];
  }
  trace_.LogTick(chassis_now_ms(),
                 Target_Velocity_X, Target_Velocity_Y, Target_Omega,
                 per_wheel);
}
