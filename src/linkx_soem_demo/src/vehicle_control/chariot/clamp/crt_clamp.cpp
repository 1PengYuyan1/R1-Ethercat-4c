// 双 Pitch DM 电机夹爪
// 移植自 R1_robot_1/User_file/3_Chariot/2_Module/CLAMP/Src/crt_clamp.cpp
// 主要适配:hfdcan2 -> linkx_t*; 补 Init 末尾的 Current_Max 形参(本项目 DM Init 多此一项)

#include "crt_clamp.h"

void Class_Clamp::Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel)
{
    // 大 Pitch 轴: Tx_ID=0x01, Rx_ID=0x11; MIT 模式; PMAX=12.5 / VMAX=20 / TMAX=15 / IMAX=10.26A
    // (IMAX 沿用 r1.2 dvc_motor_dm.h 默认值 10.261194f,匹配 DM-J4310 datasheet)
    Motor_Pitch_Large.Init(__LinkX_Handler, __CAN_Channel,
                           0x11, 0x01,
                           Motor_DM_Control_Method_NORMAL_MIT,
                           12.5f, 20.0f, 15.0f, 10.261194f);
    // 小 Pitch 轴: Tx_ID=0x02, Rx_ID=0x12
    Motor_Pitch_Small.Init(__LinkX_Handler, __CAN_Channel,
                           0x12, 0x02,
                           Motor_DM_Control_Method_NORMAL_MIT,
                           12.5f, 20.0f, 15.0f, 10.261194f);

    Motor_Pitch_Large.Set_Control_Torque_P_D_MIT(0.0f, pitch_large_kp, pitch_large_kd);
    Motor_Pitch_Small.Set_Control_Torque_P_D_MIT(0.0f, pitch_small_kp, pitch_small_kd);

    smooth_pitch_large_angle = pitch_large_pos1_angle;
    smooth_pitch_small_angle = pitch_small_pos1_angle;
}

void Class_Clamp::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Motor_Pitch_Large.TIM_Alive_PeriodElapsedCallback();
    Motor_Pitch_Small.TIM_Alive_PeriodElapsedCallback();

    if (Clamp_Control_Type == CLAMP_CONTROL_ENABLE)
    {
        if (Motor_Pitch_Large.Get_Status() != Motor_DM_Control_Status_ENABLE) Motor_Pitch_Large.CAN_Send_Enter();
        if (Motor_Pitch_Small.Get_Status() != Motor_DM_Control_Status_ENABLE) Motor_Pitch_Small.CAN_Send_Enter();
    }
}

void Class_Clamp::TIM_Calculate_PeriodElapsedCallback()
{
    switch (Clamp_Control_Type)
    {
        case CLAMP_CONTROL_DISABLE:
        {
            Motor_Pitch_Large.Set_Control_Status(Motor_DM_Status_DISABLE);
            Motor_Pitch_Small.Set_Control_Status(Motor_DM_Status_DISABLE);

            if (Motor_Pitch_Large.Get_Now_Control_Status() != Motor_DM_Status_DISABLE) Motor_Pitch_Large.CAN_Send_Exit();
            if (Motor_Pitch_Small.Get_Now_Control_Status() != Motor_DM_Status_DISABLE) Motor_Pitch_Small.CAN_Send_Exit();

            // 掉线重置平滑点(读取当前绝对角度防止切入使能时突变)
            smooth_pitch_large_angle = Motor_Pitch_Large.Get_Now_Radian();
            smooth_pitch_small_angle = Motor_Pitch_Small.Get_Now_Radian();

            // 失能时强制中止序列,避免下次使能后从中间步骤继续
            sequence_state_ = CLAMP_SEQ_IDLE;
            sequence_tick_  = 0;
            return;
        }

        case CLAMP_CONTROL_ENABLE:
        {
            // 推进取放序列:每 dwell 秒切一次目标位姿,末态停在 POS2
            _Step_Sequence();

            const float final_pitch_large_target = (current_pitch_large_state == L_PITCH_POS1) ? pitch_large_pos1_angle : pitch_large_pos2_angle;
            const float final_pitch_small_target = (current_pitch_small_state == S_PITCH_POS1) ? pitch_small_pos1_angle : pitch_small_pos2_angle;

            const float pitch_large_step = max_speed_pitch_large * kStepDt;
            const float pitch_small_step = max_speed_pitch_small * kStepDt;

            // 大 Pitch 轴斜坡滤波
            if (smooth_pitch_large_angle < final_pitch_large_target - pitch_large_step)
                smooth_pitch_large_angle += pitch_large_step;
            else if (smooth_pitch_large_angle > final_pitch_large_target + pitch_large_step)
                smooth_pitch_large_angle -= pitch_large_step;
            else
                smooth_pitch_large_angle = final_pitch_large_target;

            // 小 Pitch 轴斜坡滤波
            if (smooth_pitch_small_angle < final_pitch_small_target - pitch_small_step)
                smooth_pitch_small_angle += pitch_small_step;
            else if (smooth_pitch_small_angle > final_pitch_small_target + pitch_small_step)
                smooth_pitch_small_angle -= pitch_small_step;
            else
                smooth_pitch_small_angle = final_pitch_small_target;

            // 下发平滑后的目标位置
            if (Motor_Pitch_Large.Get_Status() != Motor_DM_Control_Status_ENABLE)
                Motor_Pitch_Large.CAN_Send_Enter();
            else
                Motor_Pitch_Large.Set_Control_Parameter_MIT(smooth_pitch_large_angle, 0.0f);
            Motor_Pitch_Large.TIM_Send_PeriodElapsedCallback();

            if (Motor_Pitch_Small.Get_Status() != Motor_DM_Control_Status_ENABLE)
                Motor_Pitch_Small.CAN_Send_Enter();
            else
                Motor_Pitch_Small.Set_Control_Parameter_MIT(smooth_pitch_small_angle, 0.0f);
            Motor_Pitch_Small.TIM_Send_PeriodElapsedCallback();
            break;
        }
    }
}

// 触发"闭合(POS2) → 等 → 张开(POS1) → 等 → 闭合(POS2,末态)"取放序列
// 仅在 ENABLE 状态下生效;序列进行中重复触发被忽略,避免按键抖动复位
void Class_Clamp::Trigger_Pick_Place_Sequence()
{
    if (Clamp_Control_Type != CLAMP_CONTROL_ENABLE) return;
    if (sequence_state_ != CLAMP_SEQ_IDLE) return;

    sequence_state_           = CLAMP_SEQ_CLOSE_1;
    sequence_tick_            = 0;
    current_pitch_large_state = L_PITCH_POS2;
    current_pitch_small_state = S_PITCH_POS2;
}

// 2ms 节拍推进序列:每段累计 dwell 秒后切换到下一目标位姿
void Class_Clamp::_Step_Sequence()
{
    if (sequence_state_ == CLAMP_SEQ_IDLE) return;

    const uint32_t dwell_ticks = (sequence_step_dwell_s_ > 0.0f)
        ? static_cast<uint32_t>(sequence_step_dwell_s_ / kStepDt)
        : 1U;

    if (++sequence_tick_ < dwell_ticks) return;
    sequence_tick_ = 0;

    switch (sequence_state_)
    {
        case CLAMP_SEQ_CLOSE_1:
            sequence_state_           = CLAMP_SEQ_OPEN;
            current_pitch_large_state = L_PITCH_POS1;
            current_pitch_small_state = S_PITCH_POS1;
            break;
        case CLAMP_SEQ_OPEN:
            sequence_state_           = CLAMP_SEQ_CLOSE_2;
            current_pitch_large_state = L_PITCH_POS2;
            current_pitch_small_state = S_PITCH_POS2;
            break;
        case CLAMP_SEQ_CLOSE_2:
            // 末态保持在 POS2(闭合),释放序列锁,允许下一次 A 触发
            sequence_state_ = CLAMP_SEQ_IDLE;
            break;
        default:
            sequence_state_ = CLAMP_SEQ_IDLE;
            break;
    }
}
