#include "linkx_soem_demo/remote/dvc_logF710.h"

#include <cmath>

namespace
{
constexpr uint32_t kStartLongPressMs = 500;
constexpr uint32_t kBackLongPressMs = 100;

bool Is_Button_Pressed(const std::vector<int> &buttons, size_t idx)
{
    return (idx < buttons.size()) && (buttons[idx] != 0);
}

bool Is_DInput_Layout(const std::vector<float> &axes, const std::vector<int> &buttons)
{
    // Logitech Cordless RumblePad 2 常见: 6 axes + 12 buttons
    return (axes.size() == 6U) && (buttons.size() >= 10U);
}

bool Is_XInput_Layout(const std::vector<float> &axes, const std::vector<int> &buttons)
{
    // Logitech F710/Xbox 常见: 8 axes + >= 11 buttons
    return (axes.size() >= 8U) && (buttons.size() >= 11U);
}

bool Resolve_Dpad_From_Axes(const std::vector<float> &axes, size_t x_idx, size_t y_idx, uint16_t *key_code)
{
    if (axes.size() <= y_idx || key_code == nullptr) return false;

    const float x = axes[x_idx];
    const float y = axes[y_idx];
    if (y > 0.5f)
    {
        *key_code = LogF710_Key_Up;
        return true;
    }
    if (y < -0.5f)
    {
        *key_code = LogF710_Key_Down;
        return true;
    }
    if (x > 0.5f)
    {
        *key_code = LogF710_Key_Left;
        return true;
    }
    if (x < -0.5f)
    {
        *key_code = LogF710_Key_Right;
        return true;
    }
    return false;
}
}

void Class_LogF710::Set_Control_Params(float max_v, float max_w, float deadzone)
{
    robot_velocity_max_ = max_v;
    robot_rotation_max_ = max_w;
    deadzone_ = deadzone;
}

float Class_LogF710::Apply_Deadzone(float raw_val) const
{
    const float abs_v = std::abs(raw_val);
    if (abs_v < deadzone_) return 0.0f;
    const float sign = (raw_val > 0.0f) ? 1.0f : -1.0f;
    return sign * (abs_v - deadzone_) / (1.0f - deadzone_);
}

Struct_LogF710_Command Class_LogF710::Resolve_Chassis(const std::vector<float> &axes) const
{
    Struct_LogF710_Command cmd;

    // 默认映射：axes[1] 前后, axes[0] 左右
    // 旋转轴在不同布局下固定不同索引，避免串轴：
    // DInput(RumblePad2): axes[2] (Z)；XInput/F710: axes[3] (Rx)。
    if (axes.size() > 2)
    {
        float vx_axis = axes[1];
        float vy_axis = axes[0];
        float omega_axis = axes[2];
        if (axes.size() > 3U)
        {
            const bool dinput_like = (axes.size() == 6U);
            if (dinput_like)
            {
                // RumblePad2/旧驱动下，右摇杆X可能落在 axes[2] 或 axes[3]。
                // 优先用 axes[2]，其静止且 axes[3] 活跃时回退到 axes[3]。
                omega_axis = axes[2];
                if ((std::abs(omega_axis) < deadzone_) && (std::abs(axes[3]) >= deadzone_))
                {
                    omega_axis = axes[3];
                }
            }
            else
            {
                omega_axis = axes[3];
            }
        }

        // DInput 模式下，F710 MODE 可能导致“左摇杆与十字键互换”
        // 当主平移轴近零、而 axes[4]/axes[5] 明显变化时，回退使用 4/5 作为平移轴。
        if (axes.size() == 6U)
        {
            const bool primary_xy_idle = (std::abs(axes[0]) < deadzone_) && (std::abs(axes[1]) < deadzone_);
            const bool alt_xy_active = (std::abs(axes[4]) >= deadzone_) || (std::abs(axes[5]) >= deadzone_);
            if (primary_xy_idle && alt_xy_active)
            {
                vy_axis = axes[4];
                vx_axis = axes[5];
            }
        }

        cmd.vx = Apply_Deadzone(vx_axis) * robot_velocity_max_;
        cmd.vy = Apply_Deadzone(vy_axis) * robot_velocity_max_;
        cmd.omega = Apply_Deadzone(omega_axis) * robot_rotation_max_;
    }

    return cmd;
}

float Class_LogF710::Resolve_Speed_Scale(const std::vector<float> &axes) const
{
    // 仅 XInput (axes.size>=8): axes[2]=LT, axes[5]=RT, 静止 +1.0 / 按到底 -1.0
    // 其他布局返回 1.0 透传, 不缩放
    if (axes.size() < 8U) return 1.0f;

    const float lt = (1.0f - axes[2]) * 0.5f;  // 归一化压下量 ∈ [0, 1]
    const float rt = (1.0f - axes[5]) * 0.5f;

    // 对称双键: 基线 0.65, RT 推 +0.35 / LT 压 -0.35, 钳位 [0.30, 1.00]
    float scale = 0.65f + (rt - lt) * 0.35f;
    if (scale < 0.30f) scale = 0.30f;
    if (scale > 1.00f) scale = 1.00f;
    return scale;
}

uint16_t Class_LogF710::Resolve_Button_Code(const std::vector<float> &axes, const std::vector<int> &buttons) const
{
    const bool dinput_like = Is_DInput_Layout(axes, buttons);
    const bool xinput_like = Is_XInput_Layout(axes, buttons);

    // —— 高优先级单键: Back / Start 永远占用使能门控,LB/RB 同时按下不生 Mod ——
    if (xinput_like)
    {
        if (Is_Button_Pressed(buttons, 6U)) return LogF710_Key_Back;
        if (Is_Button_Pressed(buttons, 7U)) return LogF710_Key_Start;
    }
    else
    {
        // DInput 与未知布局都按 8/9 取 Back/Start, 与历史约定一致
        if (Is_Button_Pressed(buttons, 8U)) return LogF710_Key_Back;
        if (Is_Button_Pressed(buttons, 9U)) return LogF710_Key_Start;
    }

    // —— DInput 下 LT/RT 作单键早返回 (XInput 下 LT/RT 是 axis,在 Resolve_Speed_Scale 消费) ——
    if (dinput_like)
    {
        if (Is_Button_Pressed(buttons, 6U)) return LogF710_Key_LT;
        if (Is_Button_Pressed(buttons, 7U)) return LogF710_Key_RT;
    }

    // —— LB/RB 作 modifier 位 (与面键/方向键 OR 组合) ——
    uint16_t code = LogF710_Key_IDLE;
    if (Is_Button_Pressed(buttons, 4U)) code |= LogF710_Mod_LB;
    if (Is_Button_Pressed(buttons, 5U)) code |= LogF710_Mod_RB;

    // —— 面键 (互斥) ——
    if (xinput_like)
    {
        // XInput: 0=A, 1=B, 2=X, 3=Y
        if      (Is_Button_Pressed(buttons, 0U)) code |= LogF710_Key_A;
        else if (Is_Button_Pressed(buttons, 1U)) code |= LogF710_Key_B;
        else if (Is_Button_Pressed(buttons, 2U)) code |= LogF710_Key_X;
        else if (Is_Button_Pressed(buttons, 3U)) code |= LogF710_Key_Y;
    }
    else
    {
        // DInput: 0=X, 1=A, 2=B, 3=Y
        if      (Is_Button_Pressed(buttons, 0U)) code |= LogF710_Key_X;
        else if (Is_Button_Pressed(buttons, 1U)) code |= LogF710_Key_A;
        else if (Is_Button_Pressed(buttons, 2U)) code |= LogF710_Key_B;
        else if (Is_Button_Pressed(buttons, 3U)) code |= LogF710_Key_Y;
    }

    // —— 方向键 (互斥, OR 进 code) ——
    uint16_t dpad_key = LogF710_Key_IDLE;
    if (xinput_like)
    {
        if (Resolve_Dpad_From_Axes(axes, 6U, 7U, &dpad_key)) code |= dpad_key;
    }
    else
    {
        if (Resolve_Dpad_From_Axes(axes, 4U, 5U, &dpad_key)) code |= dpad_key;
    }

    return code;
}

void Class_LogF710::Update_Control_Enable(uint16_t key_code, uint32_t dt_ms)
{
    if (key_code == LogF710_Key_Start)
    {
        start_press_ms_ += dt_ms;
        if (start_press_ms_ >= kStartLongPressMs)
        {
            is_ros_control_enabled_ = true;
        }
    }
    else
    {
        start_press_ms_ = 0;
    }

    if (key_code == LogF710_Key_Back)
    {
        back_press_ms_ += dt_ms;
        if (back_press_ms_ >= kBackLongPressMs)
        {
            is_ros_control_enabled_ = false;
        }
    }
    else
    {
        back_press_ms_ = 0;
    }
}

bool Class_LogF710::Check_Key_Rising_Edge(uint16_t key_code, uint16_t *rising_key)
{
    const bool is_rising = (key_code != LogF710_Key_IDLE) && (key_code != last_button_code_);
    if (is_rising && rising_key)
    {
        *rising_key = key_code;
    }

    last_button_code_ = key_code;
    return is_rising;
}
