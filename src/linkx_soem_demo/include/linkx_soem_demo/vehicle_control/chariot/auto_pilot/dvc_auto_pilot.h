/**
 * @file  dvc_auto_pilot.h
 * @brief 半自动路径跟随 + OPS 纠偏控制器
 *
 *  作用：
 *    被遥控器按键边沿 (Y / A / LB) 触发 → 沿预先注册的世界系航点序列行驶；
 *    每段从 prev_xy → wp_xy 形成一条直线，期间用 OPS 反馈做：
 *        - 横向偏差 → 横向速度修正 (PID)
 *        - 航向偏差 → omega 修正    (PID)
 *    支持"直线段保持 yaw 不变"和"拐弯段在直线移动同时旋转 yaw"两种语义，
 *    由航点的 yaw_deg 字段决定。
 *
 *  调用契约：
 *    Init(chassis)                   一次性绑定底盘指针
 *    Register_Path(id, wps, n)       预注册若干条路径（v1 内置上限 4）
 *    Start(id)   / Stop()            按键边沿调用
 *    TIM_Tick(dt_s)                  控制循环每帧调用一次（建议 1 ms）
 *                                    返回 true 表示已 OVERRIDE chassis 速度
 *                                    → 调用方必须跳过手动 _Apply_Chassis_Velocity
 *
 *  线程模型：单线程，假设 Init / Start / Stop / TIM_Tick 全部在控制主循环里调用。
 */

#ifndef DVC_AUTO_PILOT_H
#define DVC_AUTO_PILOT_H

#include <cstdint>
#include <fstream>
#include "alg_pid.h"

class Class_Chassis;   // 前向声明，避免头文件互相依赖

/* ============================================================
 *  路径数据结构
 * ========================================================== */

/**
 * @brief 单个航点（世界系绝对坐标）
 *
 *  控制律：当前段从【上一航点 / 启动时 OPS 位姿】到本航点 (x_mm, y_mm)，
 *          沿段方向以 speed_mm_s 行驶；目标航向锁定为 yaw_deg。
 *  到点判据：沿段进度 >= 段长 - reach_tol_mm。
 */
struct Struct_Waypoint
{
    float x_mm;          ///< 世界系目标 X (mm)
    float y_mm;          ///< 世界系目标 Y (mm)
    float yaw_deg;       ///< 到达该航点时的目标航向 (deg, 与 OPS 同惯例)
    float speed_mm_s;    ///< 行驶到该航点的目标速率 (mm/s)
    float reach_tol_mm;  ///< 到点判定半径 (mm)，建议 20~50
};

/* ============================================================
 *  Class_Auto_Pilot
 * ========================================================== */

class Class_Auto_Pilot
{
public:
    /// 路径库容量
    static constexpr int kMaxPath = 4;
    /// 单条路径最大航点数
    static constexpr int kMaxWp   = 16;

    /**
     * @brief 绑定底盘指针，初始化两个 PID
     *        默认 PID 参数较保守，可上电后再调
     */
    void Init(Class_Chassis *chassis);

    /**
     * @brief 注册一条路径（重复注册会覆盖）
     * @param path_id  0..kMaxPath-1
     * @param wps      航点数组
     * @param n        航点个数 (1..kMaxWp)
     * @return 是否注册成功
     */
    bool Register_Path(int path_id, const Struct_Waypoint *wps, int n);

    /**
     * @brief 启动一条已注册的路径（按键 Y/A/LB 边沿调用）
     *        若已 active 直接返回；OPS 未就绪也直接返回
     */
    void Start(int path_id);

    /**
     * @brief 中止当前路径，写零速到底盘
     * @param disable_chassis true: 同时 DISABLE 底盘（轮舵卸劲,手可推）;
     *                        false: 保留 ENABLE,舵向 MIT 锁最后位置,原地稳住(到达终点用)
     */
    void Stop(bool disable_chassis = true);

    inline bool Is_Active() const          { return state_ == STATE_RUNNING; }
    inline int  Get_Active_Path_Id() const { return active_path_id_; }
    inline int  Get_Current_Wp_Index() const { return wp_idx_; }

    /**
     * @brief 主控制器：读 OPS → 算误差 → PID → 写 chassis 目标速度
     * @param dt_s 调用周期 (s)，建议 0.001
     * @return true：本帧已写入 chassis；调用方必须跳过手动 _Apply_Chassis_Velocity
     *         false：未 active，调用方按原流程走
     */
    bool TIM_Tick(float dt_s);

    /* ---- 调试 ---- */
    inline float Get_Lateral_Error_Mm() const  { return lat_err_mm_; }
    inline float Get_Along_Distance_Mm() const { return along_mm_; }
    inline float Get_Heading_Error_Deg() const { return head_err_deg_; }
    inline float Get_Seg_Length_Mm() const     { return seg_length_mm_; }

    /* ---- 调参接口（运行中也可调） ---- */
    // i_out_max_mm_s = 0 表示不限积分; >0 时给 I 项单独限幅,避免长直道 windup
    void Set_Lateral_PID(float kp, float ki, float kd, float out_max_mm_s,
                         float i_out_max_mm_s = 0.0f);
    void Set_Heading_PID(float kp, float ki, float kd, float out_max_rad_s);

protected:
    enum State : int
    {
        STATE_IDLE = 0,
        STATE_RUNNING,
    };

    Class_Chassis *chassis_ = nullptr;

    /* ---- 路径库 ---- */
    Struct_Waypoint paths_[kMaxPath][kMaxWp];
    int             path_len_[kMaxPath] = {0, 0, 0, 0};

    /* ---- 运行期 ---- */
    State state_          = STATE_IDLE;
    int   active_path_id_ = -1;
    int   wp_idx_         = 0;

    /* ---- 当前段（leg）几何 ---- */
    float seg_x0_mm_     = 0.0f;
    float seg_y0_mm_     = 0.0f;
    float seg_dir_x_     = 1.0f;   ///< 单位方向向量 X（世界系）
    float seg_dir_y_     = 0.0f;   ///< 单位方向向量 Y（世界系）
    float seg_length_mm_ = 0.0f;
    float seg_speed_mm_s_ = 0.0f;
    float seg_target_yaw_deg_ = 0.0f;
    float seg_reach_tol_mm_   = 30.0f;

    /* ---- 上一段方向（leaving-corner 软化用，Start 时清零，advance 时保存）---- */
    float prev_seg_dir_x_ = 0.0f;
    float prev_seg_dir_y_ = 0.0f;

    /* ---- 控制器 ---- */
    Class_PID pid_lateral_;
    Class_PID pid_heading_;

    /* ---- 调试值 ---- */
    float lat_err_mm_   = 0.0f;
    float along_mm_     = 0.0f;
    float head_err_deg_ = 0.0f;

    /* ---- CSV 日志 ---- */
    std::ofstream csv_stream_;
    uint32_t      csv_tick_    = 0;
    uint32_t      csv_decimate_ = 2;   ///< OPS 200Hz 节拍下每 N 步记一行 (2→100Hz)

    /* ---- OPS 节拍门控 ---- */
    uint32_t last_ops_frame_count_ = 0;

    /* ---- 调试：blend 触发一次性打印 ---- */
    bool blend_logged_this_leg_ = false;

    /* ---- 内部辅助 ---- */
    void Begin_Leg_(float x0_mm, float y0_mm, const Struct_Waypoint &wp);
    void Open_CSV_Log_();
    void Close_CSV_Log_();
    static float Wrap_Pm180_(float deg);
};

#endif // DVC_AUTO_PILOT_H
