#ifndef LINKX_SOEM_DEMO_CRT_CLAMP_H
#define LINKX_SOEM_DEMO_CRT_CLAMP_H

#include "dvc_motor_dm.h"

enum Enum_Clamp_Control_Type {
  CLAMP_CONTROL_DISABLE = 0,
  CLAMP_CONTROL_ENABLE
};
enum Enum_Clamp_Pitch_Large_State {
  L_PITCH_POS1 = 0,
  L_PITCH_POS2,
  L_PITCH_POS_FOLDED  // 折叠收姿 (q1=1.555 大臂垂直向上, 末端在肩正上方 +16cm)
};
enum Enum_Clamp_Pitch_Small_State {
  S_PITCH_POS1 = 0,
  S_PITCH_POS2,
  S_PITCH_POS_FOLDED  // 折叠收姿 (q2=-3.112 小臂相对大臂折回 ~180°)
};

// 取放序列状态 (3 个 STEP + IDLE 末态;状态名表示"已进入该步、正在等待
// dwell/段满"):
//   触发瞬间            : sequence_state_=STEP1, 位置保持 POS1
//   STEP1 →[立即]→    STEP2 : 双轴目标 POS2 (抓取位)
//   STEP2 →[dwell]→   STEP3 : 双轴目标 POS_FOLDED (折叠收姿, blend 不停顿)
//   STEP3 →[段减速]→  IDLE  : 双轴目标 POS1 (回水平向后, blend 不停顿)
// 气夹爪不由本序列驱动, 完全由外部 (Key B toggle) 手动控制
enum Enum_Clamp_Sequence_State {
  CLAMP_SEQ_IDLE = 0,
  CLAMP_SEQ_STEP1,
  CLAMP_SEQ_STEP2,
  CLAMP_SEQ_STEP3
};

// 段插值模式:PTP=关节空间线性,LIN=笛卡尔空间直线 + 每 cycle IK
enum Enum_Clamp_Interp_Mode { CLAMP_INTERP_PTP = 0, CLAMP_INTERP_LIN };

// 通用 1D 梯形速度规划:位置从 0 走到 s_total,峰值速度 v_max,加速度 a_max,
// 可指定起始速度 v0 (用于 trajectory blending — 段进入减速时提前切下一段,
// 下段以上段当前速度起规划,中间无 0 速度点)。
// 自动判定三角形/梯形;v0 > √(2a·s_total) 时退化为紧急减速(允许末速度>0)。
class TrapezoidProfile1D {
 public:
  void Plan(float s_total, float v_max, float a_max, float v0 = 0.0f);
  float Sample(float t) const;
  float Sample_Velocity(float t) const;
  bool Done(float t) const { return t >= T_; }
  bool In_Deceleration(float t) const {
    return T_ > 0.0f && t >= t_acc_ + t_const_;
  }
  float Total_Time() const { return T_; }

 private:
  float s_total_ = 0.0f;
  float v0_ = 0.0f;
  float v_peak_ = 0.0f;
  float a_ = 0.0f;
  float t_acc_ = 0.0f;
  float t_const_ = 0.0f;
  float T_ = 0.0f;
};

/**
 * @brief 双 Pitch DM 电机夹爪 + 协同气夹爪 (R-R 串联肩+肘 2-DoF 平面臂)
 *        L1=320mm (肩-肘), L2=160mm (肘-夹爪);零位 (θ1=0, θ2=0) 大臂水平向后
 *        θ1 增大: 大臂顺时针绕到上/前;θ2 减小: 小臂相对大臂向下折
 *
 *        移植自 R1_robot_1 (STM32 HAL FDCAN2),改用本项目的 LinkX EtherCAT
 *        桥接器接口。默认接到第二片 LinkX(slave_id=2) 的 ch0 (CAN-FD 总线兼容
 *        经典 1Mbps)。
 *
 *        协议:DM-J4310 系列,经典 CAN,8B 帧,Master_ID=Motor_ID+0x10。
 *        工作模式:MIT 位置环,内置斜坡限速。
 *
 *        2026-05-21 重构:线性斜坡 → 梯形 profile + 双轴归一化时间同步,
 *        消除起停加速度阶跃与双轴异步到位顿挫。
 *
 *        气夹爪 (pneumatic gripper) 走同一通道 (ch0) 的 CAN ID 0x13,
 *        payload byte0 由外部直接设定 (Key Up=0x01 闭合 / Key Down=0x00 释放)。
 *        LinkX 固件无 valid bit, slot 内容每 cycle 重发,所以必须由调用方按节拍
 *        持续 push 当前状态。气夹爪与 clamp 取放序列完全解耦。
 */
class Class_Clamp {
 public:
  /// 大 Pitch 轴 (肩, Tx_ID=0x01, Rx_ID=0x11)
  Class_Motor_DM_Normal Motor_Pitch_Large;
  /// 小 Pitch 轴 (肘, Tx_ID=0x02, Rx_ID=0x12)
  Class_Motor_DM_Normal Motor_Pitch_Small;

  Enum_Clamp_Control_Type Clamp_Control_Type = CLAMP_CONTROL_DISABLE;

  /**
   * @brief 绑定 LinkX 句柄并初始化两个 DM 电机
   * @param __LinkX_Handler  CAN-FD LinkX 实例(预期 slave_id=2)
   * @param __CAN_Channel    通道(默认 0,与 r1.2 hfdcan2 对应)
   */
  void Init(linkx_t* __LinkX_Handler, uint8_t __CAN_Channel = 0);

  /// 100ms 心跳:维护 alive 滑窗、必要时重发使能帧
  void TIM_100ms_Alive_PeriodElapsedCallback();
  /// 周期(2ms)控制:推进段规划 + 下发 MIT 帧
  void TIM_Calculate_PeriodElapsedCallback();

  /// === 外部控制入口(由 robot 在按键回调中调用) ===
  void Set_Clamp_Control_Type(Enum_Clamp_Control_Type type) {
    Clamp_Control_Type = type;
  }
  void Set_Pitch_Large_State(Enum_Clamp_Pitch_Large_State state) {
    current_pitch_large_state = state;
  }
  void Set_Pitch_Small_State(Enum_Clamp_Pitch_Small_State state) {
    current_pitch_small_state = state;
  }

  inline Enum_Clamp_Pitch_Large_State Get_Pitch_Large_State() const {
    return current_pitch_large_state;
  }
  inline Enum_Clamp_Pitch_Small_State Get_Pitch_Small_State() const {
    return current_pitch_small_state;
  }

  /// 触发取放序列(按 A 调用);仅在 ENABLE 状态下生效;序列进行中重复触发被忽略
  /// 流程: POS1 → POS2(抓) → POS_FOLDED(折叠收姿) → POS1(末态)
  /// 气夹爪不参与本序列, 由 Key B 手动 toggle 独立控制
  void Trigger_Pick_Place_Sequence();
  inline bool Is_Sequence_Active() const {
    return sequence_state_ != CLAMP_SEQ_IDLE;
  }

  /// === 气夹爪 (pneumatic gripper, ch0 / id=0x13 / byte0) ===
  /// 调用方按节拍 (建议 1ms / 与 chassis 同) 重复调 Push_Gripper_Frame 以维持
  /// slot;byte0 当前由 Key Up=0x01 / Key Down=0x00 两个 rising-edge 按键设置
  void Set_Gripper_Byte0(uint8_t v) { gripper_byte0_ = v; }
  inline uint8_t Get_Gripper_Byte0() const { return gripper_byte0_; }
  /// 把当前 gripper_byte0_ 发到 ch0/id=0x13;LinkX 句柄缺失时静默跳过
  void Push_Gripper_Frame();

 private:
  Enum_Clamp_Pitch_Large_State current_pitch_large_state = L_PITCH_POS1;
  Enum_Clamp_Pitch_Small_State current_pitch_small_state = S_PITCH_POS1;

  // === 关节空间 POS 角度 (rad) ===
  // POS1 初始/回收末态 = 水平向后 (q=0,0);
  // POS2 抓取位 = 前方下沉 (q1=2.3, q2=-4.2, q2 单圈等价 +2.083 即 elbow_up);
  // POS_FOLDED 折叠收姿 = 大臂垂直向上 + 小臂相对折回 ~180°
  //   (q1=π/2-0.016≈1.555, q2≈-π+0.03≈-3.112, q2 ∈ (-π,0) 即 elbow_down)
  //   末端 FK=(-0.7cm, +16cm) 在肩正上方; POS2→POS_FOLDED 跨 elbow 支自动判 PTP
  float pitch_large_pos1_angle = 0.0f;
  float pitch_large_pos2_angle = 2.3f;
  float pitch_large_folded_angle = 1.555f;

  float pitch_small_pos1_angle = 0.0f;
  float pitch_small_pos2_angle = -4.2f;
  float pitch_small_folded_angle = -3.112f;

  // 当前下发的关节角(随段进度更新);掉线/失能时同步回真实角度防切回突变
  float current_pitch_large_angle = 0.0f;
  float current_pitch_small_angle = 0.0f;

  // === 梯形 profile 限值 ===
  // v_max 保持 r1.2 原值,a_max 按"加速段约 200ms"反算 (v_max / 0.2s)
  float max_speed_pitch_large = 3.0f;  // rad/s
  float max_speed_pitch_small = 5.0f;
  float max_accel_pitch_large = 15.0f;  // rad/s²
  float max_accel_pitch_small = 25.0f;

  // === MIT 刚度与阻尼 ===
  // 2026-05-21 trace 迭代:
  //   v0: Kp=15/Kd=1   → 跟随误差 22-27°, 到达后过冲不回 (sp 不动 act 飞 4.5°)
  //   v2: Kp=50/Kd=2   → 跟随 11°/回弹消除, 但小臂末段 omega 振荡 5↔0 周期 80ms
  //   v3: Kp=50/Kd=3.5 → 振荡更糟! tau 峰峰从 4.97 → 8.15Nm (大 64%)
  // 根因: DM 反馈 250Hz, omega 是位置差分有 ~4 rad/s 量化噪声 → Kd 越大噪声越
  // 被放大成 tau 振荡; 不是机械共振, 加 Kd 反而砸自己脚
  // 终结: Kp=50/Kd=1.5 (舵向 mit_kp=50/kd=1.5 同款经验, 见 steer_mit_params_tuning)
  float pitch_large_kp = 50.0f;
  float pitch_large_kd = 1.5f;
  float pitch_small_kp = 50.0f;
  float pitch_small_kd = 1.0f;

  // === 重力补偿 (MIT torque 前馈, 单位 Nm) ===
  // 大臂世界系水平时重力臂最长 (q1=0 即 POS1 水平向后), 重力把 q1 往负方向拽,
  //   τ1_ff = K1 · cos(q1) 在 q1=0 时给最大上抬扭矩, π/2 (顶上) 时为 0
  //   τ2_ff = K2 · cos(q1 + q2)   (小臂世界系角度跟两轴累加)
  //
  // 标定记录 2026-05-21 (带杆, K1 SWEEP 法 — 自动循环 K 候选找 q=0):
  //   K=0.00 → -11.25° (纯下垂)         K=2.00 → +1.30° (刚跨 0)
  //   K=0.50 → -11.20° (静摩擦死区)     K=2.50 → +4.93°
  //   K=1.00 → -4.89°                    K=3.00 → +8.69°
  //   K=1.50 → -2.57°                    K=3.50 → +11.79°
  //   → K1=1.83 是 q=0 线性插值点 (摩擦死区 ±1.8° 锁住实际位置)
  //   K2 未单姿态扫 (POS1 下 cos(q1+q2)≈1 但小臂自身对肘扭矩极小, 实测 ≈0.01)
  // 重标方法: kCalibMode = CALIB_K1_SWEEP, 使能后 IDLE 静等 24s, 看 SWEEP 日志
  float pitch_large_grav_K = 1.83f;
  float pitch_small_grav_K = 0.01f;

  // 调用周期(秒) -- 控制函数自身按 0.002s 推进段时钟
  static constexpr float kStepDt = 0.002f;

  // === 取放序列状态机(节拍同 TIM_Calculate_PeriodElapsedCallback,2ms) ===
  // 各步 dwell 节拍数 = 期望延时 / kStepDt
  Enum_Clamp_Sequence_State sequence_state_ = CLAMP_SEQ_IDLE;
  uint32_t sequence_tick_ = 0;
  void _Step_Sequence();

  // === 当前段执行状态 (Phase 1/2: PTP 关节空间 / LIN 笛卡尔直线 + IK) ===
  // 段开始时记录起止 + 模式,每 cycle 由 segment_profile_.Sample(t) 算归一化 s,
  // 然后 PTP 直接 q=lerp(q_start, q_end, s);
  //       LIN  p=lerp(p_start, p_end, s) → IK → unwrap 到与 q_start 同 2π 圈。
  // segment_active_=false 表示无段(到位静止),此时 setpoint 保持 current_*。
  bool segment_active_ = false;
  Enum_Clamp_Interp_Mode segment_mode_ = CLAMP_INTERP_PTP;
  float segment_t_ = 0.0f;
  // PTP / LIN 共用的关节空间起止 (LIN 模式 end 为目标姿态参考,用于 unwrap 选支)
  float segment_q1_start_ = 0.0f;
  float segment_q1_end_ = 0.0f;
  float segment_q2_start_ = 0.0f;
  float segment_q2_end_ = 0.0f;
  // LIN 模式额外: 笛卡尔起止 + 起点对应的 q (作为 unwrap 基准)
  float segment_x_start_ = 0.0f;
  float segment_y_start_ = 0.0f;
  float segment_x_end_ = 0.0f;
  float segment_y_end_ = 0.0f;
  TrapezoidProfile1D segment_profile_;  // s ∈ [0, 1]
  // trajectory blending: 上段进入减速时被 _Step_Sequence 提前切,记录当前归一化
  // 速度作为下段初速度;下段 _Start_Segment 用完后清零。
  float next_segment_v0_norm_ = 0.0f;
  // 下段速度/加速度缩放因子 (1.0=满速); _Step_Sequence 在切步前设,
  // _Start_Segment 用完即清回 1.0。用于末段回 POS1 这种"要慢要稳"的场景。
  float next_segment_speed_scale_ = 1.0f;

  // === GravComp 标定调试模式 ===
  //   OFF             - 正常运行, 用 pitch_*_grav_K 做补偿
  //   PRINT_STATIC    - 每秒打印 sp/act/Δ/K_rec (静态单点估算)
  //   K1_SWEEP        - 自动循环 K1 候选 0→3.0 每档 3 秒,找 q1=0 的 K1 (推荐)
  //   K2_SWEEP        - 同 K1 但在 POS4 状态下扫 K2 (需先 Set_Pitch_*_State 到 POS4)
  enum Enum_Calib_Mode { CALIB_OFF, CALIB_PRINT_STATIC, CALIB_K1_SWEEP,
                          CALIB_K2_SWEEP };
  static constexpr Enum_Calib_Mode kCalibMode = CALIB_OFF;
  uint32_t calib_print_tick_ = 0;
  // K1/K2 sweep 状态
  uint32_t sweep_tick_ = 0;
  uint32_t sweep_idx_ = 0;
  float sweep_q_acc_ = 0.0f;
  uint32_t sweep_q_cnt_ = 0;
  float sweep_results_[8] = {0};
  bool sweep_done_ = false;

  // 启动一段新规划:模式由 _Pick_Interp_Mode 根据 (from_state → to_state) 决定。
  // PTP: 起点 q=current_*, 终点 q=POS 表;归一化 s_dot/s_ddot 取两轴 v_i/|Δqi|、
  //      a_i/|Δqi| 的 min 后 Plan(1.0)。
  // LIN: 起点 p=FK(current_*), 终点 p=FK(POS 表);归一化 s 走梯形,采样后 IK
  //      并 unwrap 到 segment_q*_start_ 的同 2π 圈;v_norm/a_norm 用关节侧
  //      v_i/a_i 除以"该轴在直线段两端 |Δqi|" 的保守估计 (避免 Jacobian
  //      全段扫描)。
  void _Start_Segment(float q1_target, float q2_target,
                      Enum_Clamp_Interp_Mode mode);
  // 解析当前 POS state → 关节角目标 + 推荐插值模式
  void _Resolve_Pos_Targets(float& q1_target, float& q2_target) const;
  Enum_Clamp_Interp_Mode _Pick_Interp_Mode(float q1_from, float q2_from,
                                           float q1_to, float q2_to) const;

  // === FK / IK (R-R 串联,L1=320mm 肩-肘,L2=160mm 肘-夹爪,零位指 -x) ===
  // FK:  x = -L1·cos(θ1) - L2·cos(θ1+θ2)
  //      y =  L1·sin(θ1) + L2·sin(θ1+θ2)
  //   等价: 标准 2-link FK 在 (-x, y) 坐标 (零位指向后,故 x 取负)
  // IK:  c2 = ((-x)² + y² - L1² - L2²) / (2 L1 L2)
  //      θ2_raw = -|acos(c2)|   (elbow_down: 肘往下折,我们的 POS 系列都是负 θ2)
  //      θ1     = atan2(y, -x) - atan2(L2·sin(θ2_raw), L1+L2·cos(θ2_raw))
  //   注: 输出 θ2 ∈ [-π, 0],调用方负责 unwrap 到当前圈 (POS2 标定 -4.2 实为
  //   -4.2+2π=+2.083
  //       的等价,IK 出 ≈ -2.082,unwrap 到 -4.2-(-2.082)=-2.118,取
  //       -2.082-2π=-8.366? 实际用 round((q_prev-q_ik)/2π) 选最近)
  static constexpr float kL1 = 0.32f;  // 大臂 (肩→肘) 长度,m
  static constexpr float kL2 = 0.16f;  // 小臂 (肘→夹爪) 长度,m
  void FK_2DoF(float q1, float q2, float& x, float& y) const;
  // IK 返回 false = 目标点超出工作空间 (距肩 > L1+L2 或 < |L1-L2|)
  // q_ref_* 用于 unwrap,使输出 q 与参考连续
  bool IK_2DoF(float x, float y, float q1_ref, float q2_ref, float& q1,
               float& q2) const;

  // === 气夹爪共享通道 (与 DM 电机同 LinkX 同 ch) ===
  linkx_t* gripper_linkx_handler_ = nullptr;
  uint8_t gripper_can_channel_ = 0;
  static constexpr uint32_t kGripperCanId = 0x13U;
  uint8_t gripper_byte0_ = 0x00U;  // 当前要 push 的 byte0 值,上电默认 0x00
};

#endif  // LINKX_SOEM_DEMO_CRT_CLAMP_H
