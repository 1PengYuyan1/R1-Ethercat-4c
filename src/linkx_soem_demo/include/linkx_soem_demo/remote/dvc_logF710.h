#ifndef DVC_LOGF710_H
#define DVC_LOGF710_H

#include <cstdint>
#include <vector>

#define LogF710_Key_IDLE  0x0000
#define LogF710_Key_X     0x0001
#define LogF710_Key_A     0x0002
#define LogF710_Key_B     0x0003
#define LogF710_Key_Y     0x0004
#define LogF710_Key_LB    0x0010
#define LogF710_Key_LT    0x0020
#define LogF710_Key_RB    0x0030
#define LogF710_Key_RT    0x0040
#define LogF710_Key_Back  0x0100
#define LogF710_Key_Start 0x0200
#define LogF710_Key_Right 0x1000
#define LogF710_Key_Left  0x2000
#define LogF710_Key_Up    0x3000
#define LogF710_Key_Down  0x4000

struct Struct_LogF710_Command
{
    float vx = 0.0f;
    float vy = 0.0f;
    float omega = 0.0f;
};

class Class_LogF710
{
public:
    void Set_Control_Params(float max_v, float max_w, float deadzone);
    Struct_LogF710_Command Resolve_Chassis(const std::vector<float> &axes) const;
    uint16_t Resolve_Button_Code(const std::vector<float> &axes, const std::vector<int> &buttons) const;
    // XInput 下基于 LT/RT 模拟量算速度缩放: 基线 0.65, RT 推到 1.00, LT 压到 0.30; 非 XInput 返回 1.0
    float Resolve_Speed_Scale(const std::vector<float> &axes) const;

    void Update_Control_Enable(uint16_t key_code, uint32_t dt_ms = 1U);
    bool Is_Control_Enabled() const { return is_ros_control_enabled_; }

    bool Check_Key_Rising_Edge(uint16_t key_code, uint16_t *rising_key = nullptr);

private:
    float robot_velocity_max_ = 1.0f;
    float robot_rotation_max_ = 1.0f;
    float deadzone_ = 0.05f;

    bool is_ros_control_enabled_ = false;
    uint32_t start_press_ms_ = 0;
    uint32_t back_press_ms_ = 0;
    uint16_t last_button_code_ = LogF710_Key_IDLE;

    float Apply_Deadzone(float raw_val) const;
};

#endif
