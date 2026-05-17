/**
 * @file    dvc_odrive.h
 * @brief   ODrive 电机控制器驱动（CAN Simple 协议完整实现）
 *
 * @details
 *  - 协议：ODrive CAN Simple（CAN 协议中文版 / ODrive 0.5.x）
 *  - CAN ID 编码：(node_id << 5) | command_id
 *  - 该类对外暴露三类 API：
 *      1. 控制类   —— 直接下发设定值（写命令）
 *      2. 请求类   —— 以 RTR 帧请求遥测（异步回填到 Struct_ODrive_Data）
 *      3. 数据类   —— 同步返回最近一次回包内容
 *  - 集成到现有框架：
 *      a. 上层每周期调用 TIM_Send_PeriodElapsedCallback 推送目标值
 *      b. CAN RX 中断/线程调用 CAN_RxCpltCallback 喂入回包
 *      c. 100 ms 调用 TIM_Alive_CheckCallback 维护 is_connected
 */

#ifndef DVC_ODRIVE_H
#define DVC_ODRIVE_H

#include <cstdint>
#include <cmath>
#include "linkx.h"
#include "linkx4c_handler.h"

/**
 * @enum   Enum_ODrive_Command
 * @brief  CAN Simple 协议命令 ID（低 5 位），与 PDF《CAN 协议中文版》一一对应
 */
typedef enum
{
    ODRIVE_CMD_HEARTBEAT                = 0x001, /**< 心跳：axis_error + axis_state */
    ODRIVE_CMD_ESTOP                    = 0x002, /**< 急停（专用单帧，立即切断） */
    ODRIVE_CMD_GET_MOTOR_ERROR          = 0x003, /**< RTR 请求电机错误寄存器 */
    ODRIVE_CMD_GET_ENCODER_ERROR        = 0x004, /**< RTR 请求编码器错误寄存器 */
    ODRIVE_CMD_GET_SENSORLESS_ERROR     = 0x005, /**< RTR 请求无传感器错误寄存器 */
    ODRIVE_CMD_SET_NODE_ID              = 0x006, /**< 修改本轴 CAN node_id */
    ODRIVE_CMD_SET_AXIS_STATE           = 0x007, /**< 切换轴状态（IDLE / CLOSED_LOOP …）*/
    ODRIVE_CMD_GET_ENCODER_ESTIMATES    = 0x009, /**< RTR 请求位置/速度估算 */
    ODRIVE_CMD_GET_ENCODER_COUNT        = 0x00A, /**< RTR 请求编码器原始计数 */
    ODRIVE_CMD_SET_CONTROLLER_MODES     = 0x00B, /**< 设置 control_mode + input_mode */
    ODRIVE_CMD_SET_INPUT_POS            = 0x00C, /**< 位置目标 + 速度前馈 */
    ODRIVE_CMD_SET_INPUT_VEL            = 0x00D, /**< 速度目标 + 力矩前馈 */
    ODRIVE_CMD_SET_INPUT_TORQUE         = 0x00E, /**< 力矩目标 */
    ODRIVE_CMD_SET_VEL_LIMIT            = 0x00F, /**< 设定速度极限 */
    ODRIVE_CMD_START_ANTICOGGING        = 0x010, /**< 启动 anticogging 标定 */
    ODRIVE_CMD_SET_TRAJ_VEL_LIMIT       = 0x011, /**< 梯形轨迹模式速度极限 */
    ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS    = 0x012, /**< 梯形轨迹模式加/减速极限 */
    ODRIVE_CMD_SET_TRAJ_INERTIA         = 0x013, /**< 梯形轨迹模式惯性补偿 */
    ODRIVE_CMD_GET_IQ                   = 0x014, /**< RTR 请求 Iq setpoint / measured */
    ODRIVE_CMD_GET_SENSORLESS_ESTIMATES = 0x015, /**< RTR 请求无传感器位置/速度估算 */
    ODRIVE_CMD_REBOOT                   = 0x016, /**< 重启 ODrive */
    ODRIVE_CMD_GET_BUS_VOLTAGE          = 0x017, /**< RTR 请求总线电压 */
    ODRIVE_CMD_CLEAR_ERRORS             = 0x018  /**< 清除全部错误寄存器 */
} Enum_ODrive_Command;

/** @brief 轴错误位掩码（axis_error 字段） */
typedef enum
{
    AXIS_ERROR_NONE = 0x00000000,
    AXIS_ERROR_INVALID_STATE = 0x00000001,
    AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 0x00000002,
    AXIS_ERROR_MIN_ENDSTOP_PRESSED = 0x00000004,
    AXIS_ERROR_MAX_ENDSTOP_PRESSED = 0x00000008,
    AXIS_ERROR_ESTOP_REQUESTED = 0x00000010,
    AXIS_ERROR_HOMING_WITHOUT_ENDSTOP = 0x00000020,
    AXIS_ERROR_OVER_TEMP = 0x00000040,
    AXIS_ERROR_UNKNOWN_POSITION = 0x00000080,
    AXIS_ERROR_BRAKE_RESISTOR_DISARMED = 0x00000100,
    AXIS_ERROR_SYSTEM_LEVEL = 0x00000200,
} Enum_ODrive_Axis_Error;

/** @brief 轴状态机状态（与 ODrive 固件保持一致） */
typedef enum
{
    ODRIVE_STATE_UNDEFINED = 0,
    ODRIVE_STATE_IDLE = 1,
    ODRIVE_STATE_STARTUP_SEQUENCE = 2,
    ODRIVE_STATE_FULL_CALIBRATION = 3,
    ODRIVE_STATE_MOTOR_CALIBRATION = 4,
    ODRIVE_STATE_ENCODER_INDEX_SEARCH = 6,
    ODRIVE_STATE_ENCODER_OFFSET_CALIB = 7,
    ODRIVE_STATE_CLOSED_LOOP_CONTROL = 8
} Enum_ODrive_Axis_State;

/** @brief 控制模式（0x00B 第一字段） */
typedef enum
{
    ODRIVE_CTRL_VOLTAGE = 0,
    ODRIVE_CTRL_TORQUE = 1,
    ODRIVE_CTRL_VELOCITY = 2,
    ODRIVE_CTRL_POSITION = 3
} Enum_ODrive_Control_Mode;

/** @brief 输入模式（0x00B 第二字段） */
typedef enum
{
    ODRIVE_INPUT_PASSTHROUGH = 1,
    ODRIVE_INPUT_VEL_RAMP = 2,
    ODRIVE_INPUT_POS_FILTER = 3,
    ODRIVE_INPUT_TRAP_TRAJ = 5
} Enum_ODrive_Input_Mode;

/** @brief 运行时数据缓冲（由 RX 回调异步写入，由 Get_* 同步读出） */
typedef struct
{
    /* 高频反馈 */
    float position;            /**< 位置 (rad)            <- 0x009 */
    float omega;               /**< 速度 (rad/s)          <- 0x009 */
    float bus_voltage;         /**< 总线电压 (V)          <- 0x017 */
    float iq_setpoint;         /**< Q 轴电流设定 (A)      <- 0x014 */
    float iq_measured;         /**< Q 轴电流测量 (A)      <- 0x014 */

    /* 编码器原始计数 */
    int32_t encoder_shadow_count;  /**< Shadow Count       <- 0x00A */
    int32_t encoder_count_in_cpr;  /**< Count in CPR       <- 0x00A */

    /* 无传感器估算 */
    float sensorless_pos;          /**< Sensorless 位置   <- 0x015 */
    float sensorless_vel;          /**< Sensorless 速度   <- 0x015 */

    /* 错误寄存器 */
    uint32_t motor_error;          /**<                    <- 0x003 */
    uint32_t encoder_error;        /**<                    <- 0x004 */
    uint32_t sensorless_error;     /**<                    <- 0x005 */

    /* 上层周期写入的目标值 */
    float target_postion;
    float target_omage;
    float target_torque;

    Enum_ODrive_Axis_Error   axis_error;   /**< 由心跳 0x001 / 0x007 回包刷新 */
    Enum_ODrive_Axis_State   axis_state;   /**< 由心跳 0x001 / 0x007 回包刷新 */
    Enum_ODrive_Control_Mode motor_mode;   /**< 本地标记，决定 TIM_Send 下发哪种命令 */
    uint32_t last_update;                  /**< 最近一次 RX 时间戳 (ms)，存活检测用 */
    uint8_t  is_connected;                 /**< 1=最近 1s 内有回包 */
} Struct_ODrive_Data;

/**
 * @class Class_ODrive
 * @brief 单台 ODrive Axis 节点的控制接口（一对一对应一个 node_id）
 */
class Class_ODrive
{
public:
    /** @brief Makerbase ODrive mini 的力矩常数 [Nm/A] */
    static constexpr float Kt = 0.0827f;

    /**
     * @brief 绑定 LinkX 句柄并初始化数据缓冲
     * @param __LinkX_Handler LinkX-4C EtherCAT/CAN 桥接句柄
     * @param __CAN_Channel   CAN 通道号（0~3）
     * @param __Node_ID       ODrive 轴节点 ID（0~0x3F）
     */
    void Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel, uint8_t __Node_ID);

    /* ============================== 控制类 ============================== */

    /** @brief  下发 0x007：切换轴状态 */
    void Set_Axis_State(Enum_ODrive_Axis_State state);
    /** @brief  下发 0x00B：同时设定控制模式 + 输入模式 */
    void Set_Control_Mode(Enum_ODrive_Control_Mode ctrl_mode,
                          Enum_ODrive_Input_Mode input_mode);
    /** @brief  下发 0x00C：位置目标 + 速度前馈（torque_ff 暂未使用） */
    void Set_Position(float position, float vel_ff = 0);
    /** @brief  下发 0x00D：速度目标 + 力矩前馈 */
    void Set_Velocity(float velocity, float torque_ff = 0);
    /** @brief  下发 0x00E：力矩目标 */
    void Set_Torque(float torque);

    /* =================== 限值与轨迹（0x00F / 0x011-0x013） =================== */

    /** @brief  下发 0x00F：设定全局速度极限（Velocity Limit）*/
    void Set_Vel_Limit(float vel_limit);
    /** @brief  下发 0x011：梯形轨迹速度极限 */
    void Set_Traj_Vel_Limit(float vel_limit);
    /** @brief  下发 0x012：梯形轨迹加 + 减速极限 */
    void Set_Traj_Accel_Limits(float accel_limit, float decel_limit);
    /** @brief  下发 0x013：梯形轨迹惯性补偿 */
    void Set_Traj_Inertia(float inertia);

    /* ================== 节点配置（0x006，运行时不常用） ================== */

    /** @brief  下发 0x006：修改本轴 node_id（之后必须用新 ID 通信） */
    void Set_Node_ID(uint32_t new_node_id);

    /* ============================ 状态请求（RTR） ============================ */

    /** @brief 0x009 RTR：请求位置/速度估算，回包写到 data.position / data.omega */
    void Request_Encoder_Data()        { Send_Command(ODRIVE_CMD_GET_ENCODER_ESTIMATES,    nullptr, 0, true); }
    /** @brief 0x00A RTR：请求 shadow_count + count_in_cpr */
    void Request_Encoder_Count()       { Send_Command(ODRIVE_CMD_GET_ENCODER_COUNT,        nullptr, 0, true); }
    /** @brief 0x017 RTR：请求总线电压 */
    void Request_Bus_Voltage()         { Send_Command(ODRIVE_CMD_GET_BUS_VOLTAGE,          nullptr, 0, true); }
    /** @brief 0x014 RTR：请求 Iq setpoint + measured */
    void Request_IQ_Data()             { Send_Command(ODRIVE_CMD_GET_IQ,                   nullptr, 0, true); }
    /** @brief 0x003 RTR：请求电机错误寄存器 */
    void Request_Motor_Error()         { Send_Command(ODRIVE_CMD_GET_MOTOR_ERROR,          nullptr, 0, true); }
    /** @brief 0x004 RTR：请求编码器错误寄存器 */
    void Request_Encoder_Error()       { Send_Command(ODRIVE_CMD_GET_ENCODER_ERROR,        nullptr, 0, true); }
    /** @brief 0x005 RTR：请求无传感器错误寄存器 */
    void Request_Sensorless_Error()    { Send_Command(ODRIVE_CMD_GET_SENSORLESS_ERROR,     nullptr, 0, true); }
    /** @brief 0x015 RTR：请求无传感器位置/速度估算 */
    void Request_Sensorless_Estimates(){ Send_Command(ODRIVE_CMD_GET_SENSORLESS_ESTIMATES, nullptr, 0, true); }

    /* ============================== 实用辅助 ============================== */

    /** @brief 进入闭环控制（0x007 → CLOSED_LOOP_CONTROL） */
    void SET_ClosedLoop()  { Set_Axis_State(ODRIVE_STATE_CLOSED_LOOP_CONTROL); }
    /** @brief 软停止：把状态切回 IDLE（0x007 → IDLE） */
    void Emergency_Stop()  { Set_Axis_State(ODRIVE_STATE_IDLE); }
    /** @brief 硬急停：使用协议专用 0x002 急停帧（不走状态机） */
    void Estop()           { Send_Command(ODRIVE_CMD_ESTOP,             nullptr, 0); }
    /** @brief 0x018：清除全部错误（不会自动回到闭环，需再发 SET_ClosedLoop） */
    void Clear_Errors()    { Send_Command(ODRIVE_CMD_CLEAR_ERRORS,      nullptr, 0); }
    /** @brief 0x016：触发 ODrive 主控重启 */
    void Reboot()          { Send_Command(ODRIVE_CMD_REBOOT,            nullptr, 0); }
    /** @brief 0x010：启动 anticogging 反齿槽标定（轴需先在 CLOSED_LOOP） */
    void Start_Anticogging(){ Send_Command(ODRIVE_CMD_START_ANTICOGGING, nullptr, 0); }

    /* ============================ 数据同步获取 ============================ */

    float    Get_Position()             const { return data.position; }            /**< rad     */
    float    Get_Omega()                const { return data.omega; }               /**< rad/s   */
    float    Get_Bus_Voltage()          const { return data.bus_voltage; }         /**< V       */
    float    Get_Axis_Error()           const { return data.axis_error; }          /**< 错误位  */
    float    Get_Axis_State()           const { return data.axis_state; }          /**< 状态机  */
    uint8_t  Is_Connected()             const { return data.is_connected; }        /**< 0/1     */
    uint8_t  Get_node_id()              const { return node_id; }                  /**< CAN ID  */
    float    Get_IQ_Measured()          const { return data.iq_measured; }         /**< A       */
    float    Get_IQ_Setpoint()          const { return data.iq_setpoint; }         /**< A       */
    float    Get_Torque_From_IQ()       const { return Kt * data.iq_measured; }    /**< Nm 估算 */
    int32_t  Get_Encoder_Shadow_Count() const { return data.encoder_shadow_count; }
    int32_t  Get_Encoder_Count_In_CPR() const { return data.encoder_count_in_cpr; }
    float    Get_Sensorless_Pos()       const { return data.sensorless_pos; }
    float    Get_Sensorless_Vel()       const { return data.sensorless_vel; }
    uint32_t Get_Motor_Error()          const { return data.motor_error; }
    uint32_t Get_Encoder_Error()        const { return data.encoder_error; }
    uint32_t Get_Sensorless_Error()     const { return data.sensorless_error; }
    /** @brief 上层最近一次写入的目标速度 (rad/s) — 标定/记录用 */
    float    Get_Target_Omega()         const { return data.target_omage; }
    /** @brief 上层最近一次写入的目标力矩 / 力矩前馈 (Nm) — 标定/记录用 */
    float    Get_Target_Torque()        const { return data.target_torque; }

    /**
     * @brief  仅设置本地 motor_mode 标记，不下发 CAN
     * @note   决定 TIM_Send_PeriodElapsedCallback 走哪条分支（速度/力矩/位置）
     */
    void Set_Motor_Mode(Enum_ODrive_Control_Mode mode) { data.motor_mode = mode; }

    /** @brief 写入下一周期目标速度（rad/s），由 TIM_Send 推送 */
    void Set_target_omega(float v)  { data.target_omage = v; }
    /** @brief 写入下一周期目标力矩（Nm），由 TIM_Send 推送 */
    void Set_target_torque(float v) { data.target_torque = v; }

    /* ============================== 回调钩子 ============================== */

    /**
     * @brief  CAN 接收回调：路由 + 解析
     * @param  rx_data   8 字节有效负载
     * @param  __can_id  原始 11-bit CAN ID（含 node << 5 | cmd）
     * @note   只接受 ((id>>5)&0x3F)==node_id 的帧；同时刷新 last_update
     */
    void CAN_RxCpltCallback(uint8_t *rx_data, uint32_t __can_id);

    /** @brief 100 ms 心跳：根据 last_update 距今是否 > 1s 翻转 is_connected */
    void TIM_Alive_CheckCallback();

    /**
     * @brief  控制周期回调：根据 motor_mode 推送 vel/torque/pos，并以 1/5 频率请求 IQ
     * @note   节奏完全由调用方决定，本函数自身不计时
     */
    void TIM_Send_PeriodElapsedCallback();

private:
    linkx_t *LinkX_Handler = nullptr;
    uint8_t  CAN_Channel = 0;
    uint8_t  node_id = 0;
    Struct_ODrive_Data data{};

    /**
     * @brief  统一的 CAN 帧发送入口
     * @param  cmd       命令 ID（低 5 位）
     * @param  send_data 有效负载指针，可为 nullptr
     * @param  len       有效字节数（>8 会被截断到 8）
     * @param  rtr       true → 远程帧（Request_*），false → 数据帧
     */
    void Send_Command(Enum_ODrive_Command cmd, const uint8_t *send_data, uint8_t len, bool rtr = false);

    /** @brief  解析 RX 回包并写入 data */
    void Process_Response(Enum_ODrive_Command cmd, const uint8_t *rx_data);
};

#endif
