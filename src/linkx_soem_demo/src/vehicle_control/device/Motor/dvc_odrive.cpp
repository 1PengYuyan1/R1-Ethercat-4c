/**
 * @file    dvc_odrive.cpp
 * @brief   ODrive CAN Simple 协议驱动实现
 *
 * @details
 *  - 所有 CAN 帧通过 linkx_quick_can_send / linkx_quick_can_send_rtr 走
 *    LinkX-4C 的 EtherCAT-CAN 桥接通道；
 *  - 浮点字段统一按 IEEE 754 小端序 memcpy 处理（与 PDF "Intel 字节序" 一致）；
 *  - 时间戳使用 std::chrono::steady_clock 转 ms（与硬件 HAL_GetTick 兼容）。
 */

#include "dvc_odrive.h"
#include <chrono>
#include <cstring>

/** @brief 1 秒内无任何 RX 回包则视为离线 */
#define ODRIVE_COMM_TIMEOUT_MS 1000

/**
 * @brief  返回当前单调时钟（毫秒），用作 last_update 时间戳
 * @note   等价于 STM32 HAL_GetTick()，这里走 std::chrono
 */
static uint32_t HAL_GetTick()
{
    auto now = std::chrono::steady_clock::now();
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return static_cast<uint32_t>(ms.count());
}

/** @brief float → 4 字节小端序写入 buffer */
static inline void float_to_bytes(float v, uint8_t *out) { std::memcpy(out, &v, 4); }
/** @brief 4 字节小端序 → float */
static inline float bytes_to_float(const uint8_t *in)    { float v; std::memcpy(&v, in, 4); return v; }

/* ================================================================== */
/*                              初始化                                  */
/* ================================================================== */

/**
 * @brief 绑定底层 CAN 句柄并清零所有运行时数据
 *
 *  Init() 不下发任何 CAN 帧，仅完成本地资源绑定。
 *  上层在 chassis Init 阶段调用，之后即可调用 Request_* / Set_* 接口。
 */
void Class_ODrive::Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel, uint8_t __Node_ID)
{
    LinkX_Handler = __LinkX_Handler;
    CAN_Channel   = __CAN_Channel;
    node_id       = __Node_ID;
    data = {};
}

/* ================================================================== */
/*                           控制写命令                                 */
/* ================================================================== */

/**
 * @brief  下发 0x007 SET_AXIS_STATE
 *
 *  ODrive 协议要求 axis_state 字段位于 byte0..3（uint32 LE）。
 *  实际有效值不超过 8，所以仅写 buf[0]，剩余 0 填充。
 */
void Class_ODrive::Set_Axis_State(Enum_ODrive_Axis_State state)
{
    uint8_t buf[8] = {0};
    buf[0] = (uint8_t)state;
    Send_Command(ODRIVE_CMD_SET_AXIS_STATE, buf, 8);
}

/**
 * @brief  下发 0x00B SET_CONTROLLER_MODES
 *
 *  byte0..3 = control_mode (uint32 LE)
 *  byte4..7 = input_mode   (uint32 LE)
 */
void Class_ODrive::Set_Control_Mode(Enum_ODrive_Control_Mode ctrl_mode,
                                    Enum_ODrive_Input_Mode input_mode)
{
    uint8_t buf[8] = {0};
    buf[0] = (uint8_t)ctrl_mode;
    buf[4] = (uint8_t)input_mode;
    Send_Command(ODRIVE_CMD_SET_CONTROLLER_MODES, buf, 8);
}

/**
 * @brief  下发 0x00C SET_INPUT_POS
 *
 *  byte0..3 = input_pos (float LE)
 *  byte4..7 = vel_ff    (float LE，本框架统一用 32-bit float)
 *
 *  @note PDF 上 vel_ff/torque_ff 标注为 int16×0.001；当前 Makerbase ODrive mini
 *        实测兼容 32-bit float 写法，且该接口内部仅以 vel_ff=0 调用。
 */
void Class_ODrive::Set_Position(float position, float vel_ff)
{
    uint8_t buf[8];
    float_to_bytes(position, &buf[0]);
    float_to_bytes(vel_ff,   &buf[4]);
    Send_Command(ODRIVE_CMD_SET_INPUT_POS, buf, 8);
}

/**
 * @brief  下发 0x00D SET_INPUT_VEL
 *
 *  byte0..3 = input_vel (float LE)
 *  byte4..7 = torque_ff (float LE)
 */
void Class_ODrive::Set_Velocity(float velocity, float torque_ff)
{
    uint8_t buf[8];
    float_to_bytes(velocity,  &buf[0]);
    float_to_bytes(torque_ff, &buf[4]);
    Send_Command(ODRIVE_CMD_SET_INPUT_VEL, buf, 8);
}

/**
 * @brief  下发 0x00E SET_INPUT_TORQUE
 *
 *  byte0..3 = input_torque (float LE)；只占 4 字节，其余无效。
 */
void Class_ODrive::Set_Torque(float torque)
{
    uint8_t buf[4];
    float_to_bytes(torque, buf);
    Send_Command(ODRIVE_CMD_SET_INPUT_TORQUE, buf, 4);
}

/* ================================================================== */
/*                       限值与轨迹（0x00F / 0x011-0x013）              */
/* ================================================================== */

/**
 * @brief  下发 0x00F SET_VEL_LIMIT
 *
 *  设定全局速度极限（rad/s）。建议在轴处于 IDLE 状态时下发。
 */
void Class_ODrive::Set_Vel_Limit(float vel_limit)
{
    uint8_t buf[4];
    float_to_bytes(vel_limit, buf);
    Send_Command(ODRIVE_CMD_SET_VEL_LIMIT, buf, 4);
}

/**
 * @brief  下发 0x011 SET_TRAJ_VEL_LIMIT
 *
 *  仅在 input_mode = TRAP_TRAJ 时生效。
 */
void Class_ODrive::Set_Traj_Vel_Limit(float vel_limit)
{
    uint8_t buf[4];
    float_to_bytes(vel_limit, buf);
    Send_Command(ODRIVE_CMD_SET_TRAJ_VEL_LIMIT, buf, 4);
}

/**
 * @brief  下发 0x012 SET_TRAJ_ACCEL_LIMITS
 *
 *  byte0..3 = accel_limit (float LE)
 *  byte4..7 = decel_limit (float LE)
 *  仅在 input_mode = TRAP_TRAJ 时生效。
 */
void Class_ODrive::Set_Traj_Accel_Limits(float accel_limit, float decel_limit)
{
    uint8_t buf[8];
    float_to_bytes(accel_limit, &buf[0]);
    float_to_bytes(decel_limit, &buf[4]);
    Send_Command(ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS, buf, 8);
}

/**
 * @brief  下发 0x013 SET_TRAJ_INERTIA
 *
 *  inertia 用于梯形轨迹力矩前馈补偿（Nm/(rad/s²)）。
 */
void Class_ODrive::Set_Traj_Inertia(float inertia)
{
    uint8_t buf[4];
    float_to_bytes(inertia, buf);
    Send_Command(ODRIVE_CMD_SET_TRAJ_INERTIA, buf, 4);
}

/* ================================================================== */
/*                       节点配置（0x006）                              */
/* ================================================================== */

/**
 * @brief  下发 0x006 SET_NODE_ID
 *
 *  byte0..3 = new_node_id (uint32 LE)。
 *  @warning 修改后必须用新 ID 通信，本对象的 node_id 不会自动同步。
 */
void Class_ODrive::Set_Node_ID(uint32_t new_node_id)
{
    uint8_t buf[4];
    std::memcpy(buf, &new_node_id, 4);
    Send_Command(ODRIVE_CMD_SET_NODE_ID, buf, 4);
}

/* ================================================================== */
/*                              回调钩子                                */
/* ================================================================== */

/**
 * @brief CAN 接收回调
 *
 *  ① 校验 (id >> 5) & 0x3F == node_id；
 *  ② 刷新 last_update 用于存活检测；
 *  ③ 把回包丢给 Process_Response 解析。
 */
void Class_ODrive::CAN_RxCpltCallback(uint8_t *rx_data, uint32_t __can_id)
{
    if (((__can_id >> 5) & 0x3F) != node_id) return;

    data.last_update = HAL_GetTick();
    Process_Response((Enum_ODrive_Command)(__can_id & 0x1F), rx_data);
}

/**
 * @brief  控制周期回调（建议 1 ms 一次）
 *
 *  - 1/5 频率请求 IQ 反馈（DOB / 标定使用）
 *  - 按 motor_mode 把 target_* 推送到 ODrive
 */
void Class_ODrive::TIM_Send_PeriodElapsedCallback()
{
    static uint8_t iq_div = 0;
    if (++iq_div >= 5) { iq_div = 0; Request_IQ_Data(); }

    Request_Encoder_Data();

    static uint8_t vbus_div = 0;
    if (++vbus_div >= 50) { vbus_div = 0; Request_Bus_Voltage(); }

    switch (data.motor_mode)
    {
    case ODRIVE_CTRL_VOLTAGE:
    case ODRIVE_CTRL_VELOCITY:
        Set_Velocity(data.target_omage, data.target_torque);
        break;
    case ODRIVE_CTRL_TORQUE:
        Set_Torque(data.target_torque);
        break;
    case ODRIVE_CTRL_POSITION:
        Set_Position(data.target_postion);
        break;
    }
}

/**
 * @brief  存活检测（建议 100 ms 一次）
 *
 *  距上次 RX 超过 ODRIVE_COMM_TIMEOUT_MS 即视为离线。
 */
void Class_ODrive::TIM_Alive_CheckCallback()
{
    data.is_connected = (HAL_GetTick() - data.last_update) <= ODRIVE_COMM_TIMEOUT_MS;
}

/* ================================================================== */
/*                              私有辅助                                */
/* ================================================================== */

/**
 * @brief 统一的 CAN 帧发送入口
 *
 *  - 帧 ID = (node_id << 5) | cmd
 *  - 数据帧固定 8 字节（不足补 0）
 *  - rtr=true 时走 linkx_quick_can_send_rtr（远程帧，不带数据，由对端回包）
 */
void Class_ODrive::Send_Command(Enum_ODrive_Command cmd, const uint8_t *send_data, uint8_t len, bool rtr)
{
    uint32_t can_id = ((uint32_t)node_id << 5) | (uint32_t)cmd;
    uint8_t buf[8] = {0};
    if (send_data && len) std::memcpy(buf, send_data, (len > 8) ? 8 : len);

    if (rtr) linkx_quick_can_send_rtr(LinkX_Handler, CAN_Channel, can_id, buf);
    else     linkx_quick_can_send    (LinkX_Handler, CAN_Channel, can_id, buf);
}

/**
 * @brief 回包解析
 *
 *  按 cmd 选择目标字段进行 memcpy / float 反序列化。
 *  未识别的 cmd 静默丢弃（default 分支）。
 */
void Class_ODrive::Process_Response(Enum_ODrive_Command cmd, const uint8_t *rx_data)
{
    if (!rx_data) return;

    switch (cmd)
    {
    case ODRIVE_CMD_HEARTBEAT:
    case ODRIVE_CMD_SET_AXIS_STATE: {
        uint32_t err, st;
        std::memcpy(&err, &rx_data[0], 4);
        std::memcpy(&st,  &rx_data[4], 4);
        data.axis_error = (Enum_ODrive_Axis_Error)err;
        data.axis_state = (Enum_ODrive_Axis_State)st;
        break;
    }
    case ODRIVE_CMD_GET_MOTOR_ERROR:
        std::memcpy(&data.motor_error, rx_data, 4);
        break;
    case ODRIVE_CMD_GET_ENCODER_ERROR:
        std::memcpy(&data.encoder_error, rx_data, 4);
        break;
    case ODRIVE_CMD_GET_SENSORLESS_ERROR:
        std::memcpy(&data.sensorless_error, rx_data, 4);
        break;
    case ODRIVE_CMD_GET_ENCODER_ESTIMATES:
        data.position = bytes_to_float(&rx_data[0]);
        data.omega    = bytes_to_float(&rx_data[4]);
        break;
    case ODRIVE_CMD_GET_ENCODER_COUNT:
        std::memcpy(&data.encoder_shadow_count, &rx_data[0], 4);
        std::memcpy(&data.encoder_count_in_cpr, &rx_data[4], 4);
        break;
    case ODRIVE_CMD_GET_IQ:
        data.iq_setpoint = bytes_to_float(&rx_data[0]);
        data.iq_measured = bytes_to_float(&rx_data[4]);
        break;
    case ODRIVE_CMD_GET_SENSORLESS_ESTIMATES:
        data.sensorless_pos = bytes_to_float(&rx_data[0]);
        data.sensorless_vel = bytes_to_float(&rx_data[4]);
        break;
    case ODRIVE_CMD_GET_BUS_VOLTAGE:
        data.bus_voltage = bytes_to_float(rx_data);
        break;
    default:
        break;
    }
}
