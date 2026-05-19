#ifndef DVC_LOGF710_H
#define DVC_LOGF710_H

#include <cstdint>
#include <vector>

// 按键编码 (uint16_t, 位布局如下):
//   面键   (bits 0-2, enum):  X=0x0001 A=0x0002 B=0x0003 Y=0x0004
//   Mod_LB (bit  3, flag):    0x0008   按住 LB 时置位,可 OR 进面键/方向键
//   肩键单 (bits 4-6, enum):  LT=0x0020 RT=0x0040 (DInput 才作为按键; XInput 下是 axes)
//   Mod_RB (bit  7, flag):    0x0080   按住 RB 时置位,可 OR 进面键/方向键
//   元键   (bits 8-9, enum):  Back=0x0100 Start=0x0200 (长按消费,单按预留)
//   方向键 (bits 12-14, enum):Right=0x1000 Left=0x2000 Up=0x3000 Down=0x4000
// 组合规则:
//   - LB / RB 是 modifier:与面键/方向键 OR 组合,例 LB+X=0x0009, RB+Up=0x3080
//   - LB / RB 单按 (= LogF710_Mod_LB / LogF710_Mod_RB) 不分发任何动作,只做 modifier
//   - Back/Start/LT/RT 仍是高优先级单键,按下时立即返回该值,忽略 LB/RB 状态
#define LogF710_Key_IDLE  0x0000
#define LogF710_Key_X     0x0001
#define LogF710_Key_A     0x0002
#define LogF710_Key_B     0x0003
#define LogF710_Key_Y     0x0004
#define LogF710_Mod_LB    0x0008
#define LogF710_Key_LT    0x0020
#define LogF710_Key_RT    0x0040
#define LogF710_Mod_RB    0x0080
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
