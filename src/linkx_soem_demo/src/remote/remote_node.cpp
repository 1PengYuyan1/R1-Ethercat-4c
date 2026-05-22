#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <cmath>

#include "linkx_soem_demo/remote/dvc_logF710.h"

// ROS2 节点：订阅 /joy 摇杆输入，经 Class_LogF710 解算后发布 /cmd_vel 与 /robot_buttons
class TeleopNode : public rclcpp::Node
{
public:
    // 构造时从参数服务器读取 max_speed/max_omega/deadzone 注入解算器，并建好订阅与发布
    TeleopNode() : Node("remote_node_cpp")
    {
        this->declare_parameter("max_speed", 0.5);   // 2026-05-22 二次提速 0.3→0.5 (与 MAX_CHASSIS_SPEED 一致)
        this->declare_parameter("max_omega", 0.5);   // 同步
        this->declare_parameter("deadzone", 0.03);

        const float max_speed = static_cast<float>(this->get_parameter("max_speed").as_double());
        const float max_omega = static_cast<float>(this->get_parameter("max_omega").as_double());
        const float deadzone  = static_cast<float>(this->get_parameter("deadzone").as_double());
        logf710_.Set_Control_Params(max_speed, max_omega, deadzone);

        sub_joy_      = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&TeleopNode::Joy_Callback, this, std::placeholders::_1));
        pub_chassis_  = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_buttons_  = this->create_publisher<std_msgs::msg::UInt16>("/robot_buttons", 10);

        RCLCPP_INFO(this->get_logger(),
                    "remote_node started: max_speed=%.2f, max_omega=%.2f, deadzone=%.3f",
                    max_speed, max_omega, deadzone);
    }

private:
    static constexpr bool kEnableDebugLog = false;

    Class_LogF710 logf710_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr     sub_joy_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    pub_chassis_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr        pub_buttons_;

    // /joy 回调：把摇杆轴/键码解算成底盘 Twist 与按钮位图，分别发布到两个话题
    // kEnableDebugLog=true 时仅在有非零输入时打印一行调试信息
    void Joy_Callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        const auto chassis = logf710_.Resolve_Chassis(msg->axes);
        const uint16_t buttons = logf710_.Resolve_Button_Code(msg->axes, msg->buttons);
        const float scale = logf710_.Resolve_Speed_Scale(msg->axes);

        geometry_msgs::msg::Twist twist;
        twist.linear.x  = chassis.vx * scale;
        twist.linear.y  = chassis.vy * scale;
        twist.angular.z = chassis.omega;
        pub_chassis_->publish(twist);

        std_msgs::msg::UInt16 button_msg;
        button_msg.data = buttons;
        pub_buttons_->publish(button_msg);

        if (kEnableDebugLog &&
            (std::abs(chassis.vx) > 0.01f || std::abs(chassis.vy) > 0.01f ||
             std::abs(chassis.omega) > 0.01f || buttons != 0U))
        {
            RCLCPP_INFO(this->get_logger(),
                        "vx=%.2f vy=%.2f omega=%.2f buttons=0x%04X",
                        chassis.vx, chassis.vy, chassis.omega, buttons);
        }
    }
};

// 程序入口：标准 ROS2 节点启动样板（init → spin → shutdown）
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
