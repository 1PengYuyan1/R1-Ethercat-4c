#ifndef LINKX_SOEM_DEMO_CRT_CLAMP_H
#define LINKX_SOEM_DEMO_CRT_CLAMP_H

#include "dvc_motor_dm.h"

enum Enum_Clamp_Control_Type { CLAMP_CONTROL_DISABLE = 0, CLAMP_CONTROL_ENABLE };
enum Enum_Clamp_Pitch_Large_State { L_PITCH_POS1 = 0, L_PITCH_POS2 };
enum Enum_Clamp_Pitch_Small_State { S_PITCH_POS1 = 0, S_PITCH_POS2 };

// 取放序列状态:CLOSE_1 → OPEN → CLOSE_2(末态),每段固定 dwell 秒
enum Enum_Clamp_Sequence_State {
    CLAMP_SEQ_IDLE = 0,
    CLAMP_SEQ_CLOSE_1,
    CLAMP_SEQ_OPEN,
    CLAMP_SEQ_CLOSE_2,
};

/**
 * @brief 双 Pitch DM 电机夹爪
 *        移植自 R1_robot_1 (STM32 HAL FDCAN2),改用本项目的 LinkX EtherCAT 桥接器接口。
 *        默认接到第二片 LinkX(slave_id=2) 的 ch0(CAN-FD 总线兼容经典 1Mbps)。
 *
 *        协议:DM-J4310 系列,经典 CAN,8B 帧,Master_ID=Motor_ID+0x10。
 *        工作模式:MIT 位置环,内置斜坡限速。
 */
class Class_Clamp
{
public:
    /// 大 Pitch 轴 (Tx_ID=0x01, Rx_ID=0x11)
    Class_Motor_DM_Normal Motor_Pitch_Large;
    /// 小 Pitch 轴 (Tx_ID=0x02, Rx_ID=0x12)
    Class_Motor_DM_Normal Motor_Pitch_Small;

    Enum_Clamp_Control_Type Clamp_Control_Type = CLAMP_CONTROL_DISABLE;

    /**
     * @brief 绑定 LinkX 句柄并初始化两个 DM 电机
     * @param __LinkX_Handler  CAN-FD LinkX 实例(预期 slave_id=2)
     * @param __CAN_Channel    通道(默认 0,与 r1.2 hfdcan2 对应)
     */
    void Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel = 0);

    /// 100ms 心跳:维护 alive 滑窗、必要时重发使能帧
    void TIM_100ms_Alive_PeriodElapsedCallback();
    /// 周期(2ms)控制:斜坡更新平滑目标 + 下发 MIT 帧
    void TIM_Calculate_PeriodElapsedCallback();

    /// === 外部控制入口(由 robot 在按键回调中调用) ===
    void Set_Clamp_Control_Type(Enum_Clamp_Control_Type type) { Clamp_Control_Type = type; }
    void Set_Pitch_Large_State(Enum_Clamp_Pitch_Large_State state) { current_pitch_large_state = state; }
    void Set_Pitch_Small_State(Enum_Clamp_Pitch_Small_State state) { current_pitch_small_state = state; }

    inline Enum_Clamp_Pitch_Large_State Get_Pitch_Large_State() const { return current_pitch_large_state; }
    inline Enum_Clamp_Pitch_Small_State Get_Pitch_Small_State() const { return current_pitch_small_state; }

    /// 触发"闭合-张开-闭合"取放序列(按 A 调用);仅在 ENABLE 状态下生效;序列进行中重复触发被忽略
    void Trigger_Pick_Place_Sequence();
    inline bool Is_Sequence_Active() const { return sequence_state_ != CLAMP_SEQ_IDLE; }

    /// 序列每一步的停留时间(秒),默认 1.0;运行时可调
    void Set_Sequence_Step_Dwell_S(float s) { if (s > 0.0f) sequence_step_dwell_s_ = s; }

private:
    Enum_Clamp_Pitch_Large_State current_pitch_large_state = L_PITCH_POS1;
    Enum_Clamp_Pitch_Small_State current_pitch_small_state = S_PITCH_POS1;

    // === 终点目标(rad) ===
    float pitch_large_pos1_angle = 0.0f;
    float pitch_large_pos2_angle = 2.0f;
    float pitch_small_pos1_angle = 0.0f;
    float pitch_small_pos2_angle = 1.0f;

    float smooth_pitch_large_angle = 0.0f;
    float smooth_pitch_small_angle = 0.0f;

    // 默认限速 (rad/s)
    float max_speed_pitch_large = 3.0f;
    float max_speed_pitch_small = 5.0f;

    // === MIT 刚度与阻尼 ===
    float pitch_large_kp = 15.0f;
    float pitch_large_kd = 1.0f;
    float pitch_small_kp = 15.0f;
    float pitch_small_kd = 1.0f;

    // 调用周期(秒) -- 控制函数自身按 0.002s 算斜坡步长(与 r1.2 一致)
    static constexpr float kStepDt = 0.002f;

    // === 取放序列状态机 ===
    Enum_Clamp_Sequence_State sequence_state_ = CLAMP_SEQ_IDLE;
    uint32_t sequence_tick_                   = 0;
    float    sequence_step_dwell_s_           = 1.0f;
    void _Step_Sequence();
};

#endif // LINKX_SOEM_DEMO_CRT_CLAMP_H
