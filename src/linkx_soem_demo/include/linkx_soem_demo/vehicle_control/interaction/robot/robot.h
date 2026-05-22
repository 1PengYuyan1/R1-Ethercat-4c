#ifndef ROBOT_H
#define ROBOT_H

#include "crt_chassis.h"
#include "crt_clamp.h"
#include "dvc_auto_pilot.h"
#include "dvc_manual_yaw_hold.h"
#include "linkx_soem_demo/remote/dvc_logF710.h"
#include <atomic>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <thread>

/**
 * @brief 机器人交互层
 *        整合底盘控制、ROS2 遥控桥接、CAN 接收分发与定时回调，
 *        作为 task 层与执行层（Chassis）之间的唯一入口。
 */
class Class_Robot
{
public:
    /* ===== 子系统 ===== */
    Class_Chassis Chassis;
    Class_Auto_Pilot Auto_Pilot;
    Class_Manual_Heading_Hold Manual_Yaw_Hold;
    Class_Clamp Clamp;

    /* ===== 生命周期 ===== */
    void Init(linkx_t *__LinkX_Handler, linkx_t *__LinkX_FD_Handler = nullptr);
    void Start_ROS2_Remote_Bridge();
    void Stop_ROS2_Remote_Bridge();

    /* ===== 周期回调（由 task 层按节拍触发） ===== */
    void TIM_1ms_Calculate_Callback();
    void TIM_2ms_Calculate_PeriodElapsedCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();

    /* ===== CAN 接收 =====
     * @param Module_Id  0 = 经典 LinkX(slave_id=1, DM/ODrive/Encoder/OPS)
     *                   1 = CAN-FD LinkX(slave_id=2, ch0=Clamp DM)
     */
    void CAN_Rx_Callback(uint8_t Module_Id, uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data, uint8_t CAN_DLen);

    /* ===== 调试观测接口（只读） ===== */
    inline uint16_t Get_Debug_Remote_Key_Code() const { return debug_remote_key_code_; }
    inline bool     Get_Debug_Remote_Is_Recent() const { return debug_remote_is_recent_; }
    inline bool     Get_Debug_Remote_Is_Enabled() const { return debug_remote_is_enabled_; }
    inline float    Get_Debug_Remote_Vx() const { return debug_remote_vx_; }
    inline float    Get_Debug_Remote_Vy() const { return debug_remote_vy_; }
    inline float    Get_Debug_Remote_Omega() const { return debug_remote_omega_; }

protected:
    /**
     * @brief ROS2 → 主线程之间的命令快照
     *        ros_spin_thread_ 写入，主控线程读出；通过 ros_cmd_mutex_ 保护。
     */
    struct Ros_Remote_Command
    {
        float    vx = 0.0f;
        float    vy = 0.0f;
        float    omega = 0.0f;
        uint16_t buttons = 0;
        int64_t  last_update_ns = 0;
        bool     has_twist = false;
        bool     has_buttons = false;
    };

    /* ===== 资源句柄与子设备 ===== */
    linkx_t          *LinkX_Handler    = nullptr;
    linkx_t          *LinkX_FD_Handler = nullptr;
    Class_LogF710    logf710_remote_;

    /* ===== ROS2 桥接状态 ===== */
    std::mutex             ros_cmd_mutex_;
    Ros_Remote_Command     ros_cmd_;
    std::thread            ros_spin_thread_;
    std::atomic<bool>      ros_bridge_running_{false};
    bool                   ros_initialized_here_ = false;

    /* ===== 调试观测缓存 ===== */
    uint16_t debug_remote_key_code_   = 0;
    bool     debug_remote_is_recent_  = false;
    bool     debug_remote_is_enabled_ = false;
    float    debug_remote_vx_         = 0.0f;
    float    debug_remote_vy_         = 0.0f;
    float    debug_remote_omega_      = 0.0f;

    /* ===== 私有控制流程 ===== */
    void _Chassis_Control();
    void _Clamp_Control(uint16_t key_code, bool is_enabled);
    bool _Snapshot_Remote_Command(Ros_Remote_Command &out_cmd, bool &out_is_recent);
    bool _Update_Manual_Enable_State(uint16_t key_code);
    void _Process_Function_Key_Edge(uint16_t key_code, bool is_enabled);
    void _Apply_Chassis_Velocity(float vx, float vy, float omega, bool active);

    /* ===== 手动修正 CSV 日志 ===== */
    std::ofstream manual_corr_csv_;
    uint32_t      manual_corr_tick_   = 0;
    bool          manual_corr_csv_opened_ = false;
    static constexpr uint32_t kManualCorrCsvDecimate_ = 10;   ///< 每 10 tick 记录一行 (100Hz)
    void _Open_Manual_Correction_CSV();
    void _Close_Manual_Correction_CSV();
    void _Log_Manual_Correction(float raw_vx, float raw_vy, float raw_omega,
                                float corrected_omega);

    /* ===== ROS2 桥接内部 ===== */
    void _ROS2_Remote_Spin_Loop();
    void _Update_Remote_Twist(float vx, float vy, float omega);
    void _Update_Remote_Buttons(uint16_t buttons);
};

#endif // ROBOT_H
