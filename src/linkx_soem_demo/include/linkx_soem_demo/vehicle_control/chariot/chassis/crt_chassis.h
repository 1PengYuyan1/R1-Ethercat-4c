//
// Created by pzx on 2025/12/20.
//

#ifndef USTC_STEERING_1_CRT_CHASSIS_H
#define USTC_STEERING_1_CRT_CHASSIS_H

#include "alg_pid.h"
#include "math.h"
#include "dvc_encoder.h"
#include "dvc_motor_dm.h"
#include "dvc_odrive.h"
#include "dvc_ops.h"
#include "encoder_persistence.h"
#include "steer_calibration.h"
#include "steer_trace.h"

#include <string>

#ifndef STEER_NUM
#define STEER_NUM 4
#endif
#define REDUCTION_RATIO 3.5f

/** @brief 每个舵轮的可调参数 */
struct SteerWheelParams {
  float steer_pos_kp;  // 外层舵向位置环 P 增益
  float steer_pos_kd;  // 外层舵向位置环 D 增益
  float mit_kp;        // MIT 内环 Kp（位置刚度）
  float mit_kd;        // MIT 内环 Kd（速度阻尼）
  /** steer_omega_deadzone：PD 输出最小角速度，【舵向侧（输出轴）rad/s】 */
  float steer_omega_deadzone;

  float wheel_omega_deadzone;  // 轮向（驱动轮）死区 [rad/s]
  float wheel_feedforward;     // 轮向前馈系数
  float wheel_direction;       // 轮向安装方向（+1 或 -1）

  float flip_speed_threshold;  // 翻轮速度阈值
  float flip_drive_scale;      // 翻轮期间速度缩放

  // 舵向力矩前馈（动力学）参数：按轮独立
  float steer_inertia;
  float steer_damping;
  float steer_static_friction;

  // 驱动轮（ODrive）动力学参数（由 odrive_calib 标定）
  float wheel_inertia;             // J [kg·m²]
  float wheel_damping;             // B [Nm·s/rad]
  float wheel_friction;            // Tc [Nm] 库仑动摩擦
  float wheel_stiction;            // Ts [Nm] 静摩擦（breakaway）

  // 加速度/力矩限制
  float wheel_alpha_limit;         // 最大角加速度 [rad/s²]
  float wheel_torque_limit;        // 最大前馈力矩 [Nm]
};

/** @brief 校准流程状态 (定义已迁到 steer_calibration.h) */

/** @brief 底盘使能状态 */
enum Enum_Chassis_Control_Type {
  Chassis_Control_Type_DISABLE = 0,
  Chassis_Control_Type_ENABLE,
};

/** @brief 驱动模式：影响舵向整形 profile（slew + LPF）
 *   MANUAL    : F710 手动，响应优先，slew/LPF 透传快
 *   SEMI_AUTO : Auto_Pilot 路径跟随，拐点柔和，slew/LPF 加大平滑
 *  切换点：Class_Auto_Pilot::Start/Stop
 */
enum Enum_Drive_Mode {
  Drive_Mode_MANUAL = 0,
  Drive_Mode_SEMI_AUTO,
};

/** @brief 单个舵轮对准/驱动状态 */
typedef enum {
  STEER_STATE_IDLE = 0,
  STEER_STATE_ALIGN,
  STEER_STATE_DRIVE
} SteerDriveState_e;

/**
 * @brief 四舵轮底盘类
 */
class Class_Chassis {
 public:
  // 校准时序常量已迁到 Class_Steer_Calibration::CALIB_TICK_MS 等

  SteerWheelParams steer_wheel_params_[STEER_NUM];
  Class_Motor_DM_Normal Motor_Steer[STEER_NUM];
  Class_Encoder_BRT Encoder_Steer[STEER_NUM];
  Class_ODrive ODrive_Motor_Steer[STEER_NUM];
  Class_OPS OPS;

  // 初始化
  void Init(linkx_t *__LinkX_Handler);
  void Init_Motor_Params();

  // ============== 舵向编码器上层工具函数（与 dvc_encoder 解耦） ==============
  // 角度换算：由总编码值计算舵向角度
  static float SteerAngleFromTotal(int64_t total_pulses);
  static float SteerRadianFromTotal(int64_t total_pulses);
  // 上电恢复：用保存的 total、raw 和当前 raw 计算恢复后的 total
  static int64_t RestoreTotalWithRaw(int64_t saved_total, int32_t saved_raw, int32_t current_raw);
  // 异常判定：检查断电期间位移是否超阈值
  static bool CheckPowerOffDelta(int32_t delta);

  // 舵向异常锁定状态查询/设置/清除
  bool Is_Steer_Abnormal_Locked() const { return steer_abnormal_locked_; }
  void Set_Steer_Abnormal_Lock();
  void Clear_Steer_Abnormal_Lock();

  // 舵向编码器零点偏移持久化（断电后保留校准位置）
  bool Load_Steer_Zero_Offsets(const std::string &path);
  bool Capture_And_Save_Steer_Zero_Offsets(const std::string &path);
  static const char *Default_Steer_Zero_Offsets_Path();

  // 舵向编码器累计脉冲持久化（多圈解包 + 掉电恢复）
  // Load 在 Init 中自动调用一次：把磁盘里的 total_pulses 注入到各编码器 pending 状态
  bool Load_Steer_Unwrapped_Pulses(const std::string &path);
  // Save 由上层定时调用（建议 ~200ms 周期，含 200ms 限频与无变化跳过）
  bool Save_Steer_Unwrapped_Pulses(const std::string &path,
                                   uint32_t min_interval_ms = 200);
  // 事件触发立即落盘（绕过 200ms 限频）：找零捕获、降级切换、退出钩子调用。
  // reason 仅用于日志区分调用现场。
  bool Force_Save_Steer_Unwrapped_Pulses(const std::string &path,
                                         const char *reason);
  static const char *Default_Steer_Unwrapped_Pulses_Path();

  // 校准:仅保留 robot.cpp 外部入口,其他经 steer_calib_ 直接访问
  void Steer_Calibration_Init();

  // 定时器回调
  void TIM_100ms_Alive_PeriodElapsedCallback();
  void TIM_2ms_Resolution_PeriodElapsedCallback();
  void TIM_2ms_Control_PeriodElapsedCallback();

  // Getter
  inline float Get_Now_Velocity_X() { return Now_Velocity_X; }
  inline float Get_Now_Velocity_Y() { return Now_Velocity_Y; }
  inline float Get_Now_Omega() { return Now_Omega; }
  inline Enum_Chassis_Control_Type Get_Chassis_Control_Type() {
    return Chassis_Control_Type;
  }
  inline float Get_Target_Velocity_X() { return Target_Velocity_X; }
  inline float Get_Target_Velocity_Y() { return Target_Velocity_Y; }
  inline float Get_Target_Omega() { return Target_Omega; }

  // Setter
  inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type t) {
    Chassis_Control_Type = t;
  }
  inline Enum_Drive_Mode Get_Drive_Mode() const { return drive_mode_; }
  inline void Set_Drive_Mode(Enum_Drive_Mode m) { drive_mode_ = m; }
  inline void Set_Target_Velocity_X(float v) { Target_Velocity_X = v; }
  inline void Set_Target_Velocity_Y(float v) { Target_Velocity_Y = v; }
  inline void Set_Target_Omega(float v) { Target_Omega = v; }

 protected:
  linkx_t *LinkX_Handler = nullptr;

  // 调试
  float debug_wheel_error[STEER_NUM] = {0};
  float debug_encoder_rad[STEER_NUM] = {0};

  // 多圈累计值持久化辅助：上次写盘的快照、序号、限频窗口（封装为 State 结构）
  encoder_persistence::State persist_state_;

  // 500Hz CSV trace 封装（env STEER_TRACE_FILE 触发）
  SteerTrace trace_;

  // 舵向校准模块（F 方案:MIT 位置环 + slew + done 滞回 + 持续发命令）
  Class_Steer_Calibration steer_calib_;

  // 舵向异常锁定标志：断电位移超阈值时置 true，Homing 完成后清除
  bool steer_abnormal_locked_ = false;

  // 舵轮状态
  uint8_t steer_flipped[STEER_NUM] = {0};
  bool steer_flipped_global_ = false;  // 纯平移时 4 轮统一翻态(避免 3:1 力不平衡)
  float steer_ref_filtered[STEER_NUM] = {0};
  SteerDriveState_e steer_state[STEER_NUM] = {STEER_STATE_IDLE};

  // 目标整形（slew + LPF + FF）状态：替代原先散落在函数体里的 static
  bool  steer_shape_init_[STEER_NUM] = {false};
  float steer_ff_last_target_[STEER_NUM] = {0};

  // PMAX 边界截断计数 (Steer_To_Motor_Position 内累加)
  uint32_t steer_boundary_hits_[STEER_NUM] = {0};

  // 动力学辅助
  float Dynamic_Resistance_Wheel_Current[STEER_NUM] = {0};

  // 驱动轮前馈运行时状态
  float wheel_last_omega_[STEER_NUM] = {0};

  // ODrive 速度命令的输出端 slew 状态（限制下发到 ODrive 的 omega 变化率）
  // 用途：绕过 ODrive 固件 vel_ramp_rate 不可靠的问题，在 chassis 端直接限。
  // 默认每帧最大 Δω 由 env WHEEL_OUTPUT_ACCEL (rad/s²) × dt 决定；IDLE/DISABLE 必须重置。
  float wheel_omega_cmd_prev_[STEER_NUM] = {0};

  float Steer_Inertia = 0.001f;         // 舵向转动惯量，根据实际机构调整
  float Steer_Damping = 0.01f;          // 舵轴阻尼
  float Steer_Static_Friction = 0.05f;  // 舵轴静摩擦补偿

  // 目标量
  // ---------------------------------------------------------------
  // 舵向量均定义在【舵向侧（输出轴）】坐标系下：
  //   - Target_Steer_Rad    [rad]       舵向目标角度（输出轴）
  //   - Target_Steer_Omage  [rad/s]     舵向目标角速度（输出轴）
  //   - Target_Steer_Torque [N·m]       舵向目标力矩（输出轴）
  // 运行阶段发给 DM 电机时使用 MIT「速度环 + 力矩前馈」：
  // 角速度在 Execute_Steer_State() 统一乘以 REDUCTION_RATIO（舵向侧→电机轴）。
  // 校准阶段独立于此,见 steer_calibration.{h,cpp}。
  // ---------------------------------------------------------------
  float Target_Steer_Rad[STEER_NUM] = {0};
  // 未经 slew/LPF 的命令角度，由 Compute_Wheel_Raw_Targets 写、Update_Flip_Decision 翻转。
  // Execute_Steer_State 用它和 current 算 cos 投影，保证 flip 期间轮速归零。
  float Target_Steer_Rad_Cmd[STEER_NUM] = {0};
  float Target_Steer_Omage[STEER_NUM] = {0};
  float Target_Steer_Torque[STEER_NUM] = {0};
  float Target_Wheel_Omega[STEER_NUM] = {0};
  float Target_Wheel_torque[STEER_NUM] = {0};

  // 机械参数
  const float angle_tolerance = 0.05f;
  const float Wheel_Radius = 0.018f;
  const float Wheel_To_Core_Distance[STEER_NUM] = {0.707f, 0.707f, 0.707f,
                                                   0.707f};
  const float Wheel_Azimuth[STEER_NUM] = {
      PI / 4.0f,         //  45° 左前
      3.0f * PI / 4.0f,  // 135° 左后
      5.0f * PI / 4.0f,  // 225° 右后
      7.0f * PI / 4.0f,  // 315° 右前
  };

  float Wheel_Resistance_Omega_Threshold = 1.0f;
  float Wheel_Speed_Limit_Factor = 0.5f;
  /** MAX_STEER_OMEGA：舵向侧（输出轴）最大角速度 [rad/s]。
   *  下限 ~slew rate 才能正常跟踪；上限要让 MIT 位置环 ω_max 跟得上。
   *  实测：80 → 软件 PD 模式下 motor 被钉在 ±150 翻转疯转。
   *  改 12.0 后配合 MIT 位置环 (mit_kp=50/kd=0.5)：60-90° step 响应 < 200ms。 */
  float MAX_STEER_OMEGA = 12.0f;
  const float MAX_CHASSIS_SPEED = 0.1f;   // 手动中速上限 50cm/s; auto_pilot 自限 40mm/s 不受影响
  const float MAX_CHASSIS_OMEGA = 0.1f;   // 与 launch max_omega 对齐; auto_pilot 自限 0.05rad/s 不受影响

  const float Center_Height = 0.35f;
  const float Class_Mass = 30.0f;
  const float Class_Inertia = 2.0f;

  // 当前状态量
  float Now_Velocity_X = 0.0f;
  float Now_Velocity_Y = 0.0f;
  float Now_Omega = 0.0f;

  float Target_Velocity_X = 0.0f;
  float Target_Velocity_Y = 0.0f;
  float Target_Omega = 0.0f;

  Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
  Enum_Drive_Mode drive_mode_ = Drive_Mode_MANUAL;

  // 内部函数
  float Get_Now_Steer_Radian(int index);
  float Steer_To_Motor_Position(float target_steer, int index);
  void Self_Resolution();
  void Kinematics_Inverse_Resolution();
  // Kinematics_Inverse_Resolution 的子步骤
  void Compute_Wheel_Raw_Targets(float omega_raw[STEER_NUM]);
  void Update_Flip_Decision(const float omega_raw[STEER_NUM]);
  void Shape_Steer_Targets();
  void Update_Steer_State(int i);
  void Execute_Steer_State(int i);
  void Compute_Wheel_Feedforward(int i, float dt);
  void Output_To_Motor();
  void Steer_Trace_Log();          // 500Hz CSV trace, env STEER_TRACE_FILE 触发
};

#endif
