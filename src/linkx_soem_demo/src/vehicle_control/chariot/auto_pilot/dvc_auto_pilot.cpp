/**
 * @file  dvc_auto_pilot.cpp
 * @brief 半自动路径跟随 + OPS 纠偏控制器实现
 *
 *  控制律（每个 tick 在世界系上算）：
 *      1) 段方向单位向量 d，左法向 n = (-d.y, d.x)
 *      2) along = (p_w - p0) · d            沿线进度（mm）
 *         lat   = (p_w - p0) · n            横向偏差（mm，左正右负）
 *      3) heading_err = wrap_pm180(yaw_t - yaw_w)        航向偏差（deg）
 *      4) v_lat_corr = pid_lateral_(target=0, now=lat)   横向修正速度（mm/s，符号已含）
 *         omega     = pid_heading_(target=head_err, now=0)  omega（rad/s）
 *      5) 世界系合速度：
 *             v_w = V·d + v_lat_corr · n
 *         （PID 输出与 (-lat) 同号，乘 n 后方向自然指向回中线）
 *      6) 世界 → 机体（chassis API 是机体系）：
 *             vx_b =  cos(yaw)·v_wx + sin(yaw)·v_wy
 *             vy_b = -sin(yaw)·v_wx + cos(yaw)·v_wy
 *      7) 写 chassis_->Set_Target_Velocity_X/Y/Omega + Set_Chassis_Control_Type(ENABLE)
 *
 *  到点判定：along >= seg_length - reach_tol → wp_idx_++。
 *      段切换在每 tick 顶部以 while 循环消化（容忍单帧跨多段）；
 *      整条路径完成 → STATE_IDLE 并下零速。
 *
 *  坐标系约定：
 *      OPS V3.4 §4.1：俯视 +Y 向前、+X 向右、yaw 由 +Y 向 +X 方向递增。
 *      本实现假设 OPS 安装与底盘前向同向、原点重合（无偏角无平移）。
 *      若实测有偏差，可在调用方把 OPS 输出做一次旋转/平移再喂进来。
 */

#include "dvc_auto_pilot.h"
#include "crt_chassis.h"
#include "math.h"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <tuple>

namespace
{
constexpr float kDegToRad = 3.14159265358979f / 180.0f;

/// 默认 PID 参数（运行后用 Set_Lateral_PID / Set_Heading_PID 覆盖更现场化）
constexpr float kDefaultLatKp     = 0.5f;     ///< 横向：mm/s per mm
constexpr float kDefaultLatKi     = 0.0f;
constexpr float kDefaultLatKd     = 0.0f;     ///< D 项关闭：OPS 200Hz 反馈下 D 易被尖峰污染
constexpr float kDefaultLatOutMax = 30.0f;    ///< 横向修正速度上限 (mm/s)；与 MAX_CHASSIS_SPEED·1000=50 留 20 给沿线分量
constexpr float kDefaultLatDeadMm = 2.0f;     ///< 横向死区 (mm)

constexpr float kDefaultHeadKp     = 0.01f;   ///< 航向 Kp：head_err 5° 才推 0.05 rad/s，给 omega·d 留余量
constexpr float kDefaultHeadKi     = 0.0f;
constexpr float kDefaultHeadKd     = 0.0f;    ///< D 关闭，避免 OPS 阶跃打尖峰
constexpr float kDefaultHeadOutMax = 0.05f;   ///< omega 上限 (rad/s)，半个 MAX_CHASSIS_OMEGA
constexpr float kDefaultHeadDeadDeg = 0.5f;   ///< 航向死区 (deg)：实测 0.2° 在 40Hz 节拍下激发航向-横向耦合振荡

constexpr float kPidDt = 0.025f;              ///< 实际推进节拍 = OPS 帧号门控后约 40 Hz（实测 ≈ 25 ms/tick）
} // namespace

/* ============================================================
 *  生命周期
 * ========================================================== */

void Class_Auto_Pilot::Init(Class_Chassis *chassis)
{
    chassis_ = chassis;

    state_          = STATE_IDLE;
    active_path_id_ = -1;
    wp_idx_         = 0;

    for (int i = 0; i < kMaxPath; ++i) path_len_[i] = 0;

    pid_lateral_.Init(kDefaultLatKp, kDefaultLatKi, kDefaultLatKd,
                      /*K_F=*/0.0f, /*I_Out_Max=*/0.0f,
                      /*Out_Max=*/kDefaultLatOutMax, /*D_T=*/kPidDt,
                      /*Dead_Zone=*/kDefaultLatDeadMm);

    pid_heading_.Init(kDefaultHeadKp, kDefaultHeadKi, kDefaultHeadKd,
                      /*K_F=*/0.0f, /*I_Out_Max=*/0.0f,
                      /*Out_Max=*/kDefaultHeadOutMax, /*D_T=*/kPidDt,
                      /*Dead_Zone=*/kDefaultHeadDeadDeg);
}

void Class_Auto_Pilot::Set_Lateral_PID(float kp, float ki, float kd, float out_max_mm_s)
{
    pid_lateral_.Set_K_P(kp);
    pid_lateral_.Set_K_I(ki);
    pid_lateral_.Set_K_D(kd);
    pid_lateral_.Set_Out_Max(out_max_mm_s);
}

void Class_Auto_Pilot::Set_Heading_PID(float kp, float ki, float kd, float out_max_rad_s)
{
    pid_heading_.Set_K_P(kp);
    pid_heading_.Set_K_I(ki);
    pid_heading_.Set_K_D(kd);
    pid_heading_.Set_Out_Max(out_max_rad_s);
}

/* ============================================================
 *  路径管理
 * ========================================================== */

bool Class_Auto_Pilot::Register_Path(int path_id, const Struct_Waypoint *wps, int n)
{
    if (path_id < 0 || path_id >= kMaxPath) return false;
    if (wps == nullptr || n <= 0 || n > kMaxWp) return false;

    for (int i = 0; i < n; ++i) paths_[path_id][i] = wps[i];
    path_len_[path_id] = n;
    std::cout << "[AUTO] path " << path_id << " registered, " << n << " waypoints" << std::endl;
    return true;
}

/* ============================================================
 *  状态切换
 * ========================================================== */

void Class_Auto_Pilot::Start(int path_id)
{
    if (chassis_ == nullptr) return;

    if (state_ == STATE_RUNNING)
    {
        std::cout << "[AUTO] already active (path " << active_path_id_
                  << ", wp " << wp_idx_ << "), ignore Start("
                  << path_id << ")" << std::endl;
        return;
    }
    if (path_id < 0 || path_id >= kMaxPath || path_len_[path_id] <= 0)
    {
        std::cout << "[AUTO] path " << path_id << " not registered" << std::endl;
        return;
    }
    // OPS 必须 ENABLE，否则纠偏闭环无反馈源
    if (chassis_->OPS.Get_Status() != OPS_Status_ENABLE)
    {
        std::cout << "[AUTO] refuse Start: OPS DISABLE (待 200Hz 数据流稳定后重试)" << std::endl;
        return;
    }

    active_path_id_ = path_id;
    wp_idx_         = 0;
    state_          = STATE_RUNNING;
    csv_tick_       = 0;
    blend_logged_this_leg_ = false;
    // 第一段没有"上一段",清零禁用 leaving-corner 软化
    prev_seg_dir_x_ = 0.0f;
    prev_seg_dir_y_ = 0.0f;
    // 切到半自动 profile：舵向 slew/LPF 走柔和档（见 Shape_Steer_Targets）
    chassis_->Set_Drive_Mode(Drive_Mode_SEMI_AUTO);
    // 门控基线对齐当前 OPS 帧号，避免 Start 当刻就消化一次 stale 数据
    last_ops_frame_count_ = chassis_->OPS.Get_Rx_Frame_Count();
    Open_CSV_Log_();

    // 第一段起点 = 当前 OPS 位姿；终点 = 第 0 个航点
    const float x0 = chassis_->OPS.Get_Pos_X_Mm();
    const float y0 = chassis_->OPS.Get_Pos_Y_Mm();
    Begin_Leg_(x0, y0, paths_[path_id][0]);

    pid_lateral_.Set_Integral_Error(0.0f);
    pid_heading_.Set_Integral_Error(0.0f);

    std::cout << "[AUTO] START path " << path_id
              << " from (" << x0 << ", " << y0 << ")"
              << " → wp0 (" << paths_[path_id][0].x_mm
              << ", "       << paths_[path_id][0].y_mm
              << ", yaw="   << paths_[path_id][0].yaw_deg << ")" << std::endl;
}

void Class_Auto_Pilot::Stop(bool disable_chassis)
{
    if (state_ == STATE_IDLE) return;

    Close_CSV_Log_();
    state_ = STATE_IDLE;
    active_path_id_ = -1;

    if (chassis_ != nullptr)
    {
        chassis_->Set_Target_Velocity_X(0.0f);
        chassis_->Set_Target_Velocity_Y(0.0f);
        chassis_->Set_Target_Omega(0.0f);
        if (disable_chassis)
            chassis_->Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        // ENABLE 保留时:速度已清零,舵向 MIT kp 锁在最后位置,原地稳住。
        // 恢复手动 profile（响应优先）
        chassis_->Set_Drive_Mode(Drive_Mode_MANUAL);
    }
    std::cout << "[AUTO] STOP " << (disable_chassis ? "(chassis DISABLE)" : "(chassis HOLD)") << std::endl;
}

/* ============================================================
 *  主控制器
 * ========================================================== */

bool Class_Auto_Pilot::TIM_Tick(float /*dt_s*/)
{
    if (state_ != STATE_RUNNING || chassis_ == nullptr) return false;

    // 1) OPS 反馈 + 失联保护
    if (chassis_->OPS.Get_Status() != OPS_Status_ENABLE)
    {
        std::cout << "[AUTO] OPS lost during run → STOP" << std::endl;
        Stop();
        return false;
    }

    // 1.5) OPS 节拍门控：5ms 一帧，1kHz 调用上 4/5 是冗余推进
    //      没有新帧时直接 return true，让 chassis 沿用上次 vx/vy/omega，
    //      避免 D 项在零误差点被 1/5ms 阶跃 punch 出尖峰 → 舵向震荡
    const uint32_t cur_ops_cnt = chassis_->OPS.Get_Rx_Frame_Count();
    if (cur_ops_cnt == last_ops_frame_count_)
    {
        return true;   // 仍保持 OVERRIDE，调用方不要走手动通道
    }
    last_ops_frame_count_ = cur_ops_cnt;

    const float x_w   = chassis_->OPS.Get_Pos_X_Mm();
    const float y_w   = chassis_->OPS.Get_Pos_Y_Mm();
    const float yaw_w = chassis_->OPS.Get_Yaw_Deg();

    // 2) 计算当前段的 along/lat/heading 误差
    auto compute_errors = [&]()
    {
        const float ex = x_w - seg_x0_mm_;
        const float ey = y_w - seg_y0_mm_;
        along_mm_   = ex * seg_dir_x_ + ey * seg_dir_y_;
        const float nx = -seg_dir_y_;
        const float ny =  seg_dir_x_;
        lat_err_mm_   = ex * nx + ey * ny;
        head_err_deg_ = Wrap_Pm180_(seg_target_yaw_deg_ - yaw_w);
    };
    compute_errors();

    // 3) 段切换：循环消化（容忍单帧跨多段，例如 OPS 大跳变或 reach_tol 较大）
    while (along_mm_ >= seg_length_mm_ - seg_reach_tol_mm_)
    {
        const int next_idx = wp_idx_ + 1;
        if (next_idx >= path_len_[active_path_id_])
        {
            std::cout << "[AUTO] path " << active_path_id_
                      << " REACHED final wp " << wp_idx_ << " → STOP" << std::endl;
            Stop(/*disable_chassis=*/false);   // 到点原地稳住,不要卸劲
            return false;
        }
        const Struct_Waypoint &prev_wp = paths_[active_path_id_][wp_idx_];
        const Struct_Waypoint &next_wp = paths_[active_path_id_][next_idx];
        wp_idx_ = next_idx;
        // 保存"刚结束这段"的方向,给新段做 leaving-corner 软化
        prev_seg_dir_x_ = seg_dir_x_;
        prev_seg_dir_y_ = seg_dir_y_;
        Begin_Leg_(prev_wp.x_mm, prev_wp.y_mm, next_wp);
        pid_lateral_.Set_Integral_Error(0.0f);
        pid_heading_.Set_Integral_Error(0.0f);
        blend_logged_this_leg_ = false;

        std::cout << "[AUTO] advance to wp " << wp_idx_
                  << " (" << next_wp.x_mm << ", " << next_wp.y_mm
                  << ", yaw=" << next_wp.yaw_deg << ")" << std::endl;

        compute_errors();
    }

    // 4) PID
    //    横向：error = Target - Now = 0 - lat = -lat → 输出与 -lat 同号
    pid_lateral_.Set_Target(0.0f);
    pid_lateral_.Set_Now(lat_err_mm_);
    pid_lateral_.TIM_Calculate_PeriodElapsedCallback();
    const float v_lat_corr_mm_s = pid_lateral_.Get_Out();

    //    航向：直接把 head_err 当目标、Now 设 0，error = head_err
    //    实测 chassis omega+ → 物理 CW → OPS ψ 减小,与 head_err+("需要 ψ 增大")反向,翻号
    pid_heading_.Set_Target(head_err_deg_);
    pid_heading_.Set_Now(0.0f);
    pid_heading_.TIM_Calculate_PeriodElapsedCallback();
    const float omega_rad_s = -pid_heading_.Get_Out();

    // 5) 世界系合速度：沿线分量 + 横向修正
    //    n = (-d.y, d.x)，v_lat_corr · n 已含正确符号（lat>0 时输出<0，乘 n 即向右推回）
    //
    // ====== 拐角软化（2026-05-18 重写：cos²(Δθ/2) + smoothstep）======
    // 概念：拐点处的"软化窗口"由 Δθ（与下一段或上一段的方向夹角）决定：
    //   - V_corner = seg_speed × cos²(Δθ/2)：直线全速,90° 半速,180° U-turn 停下转
    //   - 窗口长 L = clamp(L0 × Δθ/(π/2), Lmin, Lmax)：拐越急区间越长
    //   - 进出曲线 smoothstep (3u²−2u³)：边界 V' = 0,无加速度阶跃
    //   - dir_eff 在窗口内向"对面段方向"线性 lerp,smoothstep 权重控速,二者协同
    //   - 同时考虑上游(leaving)和下游(upcoming)拐角,取更严的 V 缩放和更近的 dir 来源
    constexpr float kSoftL0Mm   = 220.0f;   // 90° 拐时的软化窗口长度 (摊大 Δθ/dt 让 steer 跟上)
    constexpr float kSoftLMinMm = 40.0f;    // 即使 Δθ 很小也至少 40mm
    constexpr float kSoftLMaxMm = 400.0f;   // 上限 (180° U-turn 用)
    constexpr float kPiF        = 3.14159265358979f;

    auto compute_soft = [&](float other_dir_x, float other_dir_y, float dist_mm)
                            -> std::tuple<float, float, float, float, float>
    {
        // 返回 (V_scale, dir_x, dir_y, dth_rad, s_taper)；窗口外 V_scale=1, dir=seg_dir, s_taper=1
        const float cos_dth = std::max(-1.0f, std::min(1.0f,
                                seg_dir_x_ * other_dir_x + seg_dir_y_ * other_dir_y));
        const float dth = std::acos(cos_dth);
        const float L_raw = kSoftL0Mm * (dth / (kPiF * 0.5f));
        const float L = std::max(kSoftLMinMm, std::min(kSoftLMaxMm, L_raw));
        if (dist_mm >= L) return {1.0f, seg_dir_x_, seg_dir_y_, dth, 1.0f};
        const float u = std::max(0.0f, dist_mm / L);
        const float s = u * u * (3.0f - 2.0f * u);                 // smoothstep
        const float half = dth * 0.5f;
        // V_scale 在拐角处 = cos⁴(Δθ/2):90° 拐 → 25%,降低 steer 旋转率需求 (V×Δθ/L)
        // 让舵向 slew (eff 300°/s) 有 ~3× 余量,根除"轮向冲在前舵向追不上"现象。
        const float c = std::cos(half);
        const float cos4_half = c * c * c * c;
        const float V_scale = cos4_half + (1.0f - cos4_half) * s;
        // dir_eff lerp：拐角处 50%/50% 混合 seg_dir 与 other_dir,边界全 seg_dir
        const float w_other = (1.0f - s) * 0.5f;
        float dx = (1.0f - w_other) * seg_dir_x_ + w_other * other_dir_x;
        float dy = (1.0f - w_other) * seg_dir_y_ + w_other * other_dir_y;
        const float dlen = std::sqrt(dx * dx + dy * dy);
        if (dlen > 1e-6f) { dx /= dlen; dy /= dlen; }
        else { dx = seg_dir_x_; dy = seg_dir_y_; }
        return {V_scale, dx, dy, dth, s};
    };

    // 5a) 下游拐角（end of leg → 下一段）
    float V_scale_up = 1.0f, dir_up_x = seg_dir_x_, dir_up_y = seg_dir_y_, dth_up = 0.0f;
    float s_up = 1.0f;
    const float dist_up_mm = std::max(0.0f, seg_length_mm_ - along_mm_);
    const int next_idx = wp_idx_ + 1;
    if (next_idx < path_len_[active_path_id_])
    {
        const Struct_Waypoint &cur_wp = paths_[active_path_id_][wp_idx_];
        const Struct_Waypoint &nxt_wp = paths_[active_path_id_][next_idx];
        const float ndx = nxt_wp.x_mm - cur_wp.x_mm;
        const float ndy = nxt_wp.y_mm - cur_wp.y_mm;
        const float nlen = std::sqrt(ndx * ndx + ndy * ndy);
        if (nlen > 1.0f)
        {
            std::tie(V_scale_up, dir_up_x, dir_up_y, dth_up, s_up) =
                compute_soft(ndx / nlen, ndy / nlen, dist_up_mm);
        }
    }

    // 5b) 上游拐角（start of leg ← 上一段）
    float V_scale_lv = 1.0f, dir_lv_x = seg_dir_x_, dir_lv_y = seg_dir_y_, dth_lv = 0.0f;
    float s_lv = 1.0f;
    const float dist_lv_mm = std::max(0.0f, along_mm_);
    if (prev_seg_dir_x_ != 0.0f || prev_seg_dir_y_ != 0.0f)
    {
        std::tie(V_scale_lv, dir_lv_x, dir_lv_y, dth_lv, s_lv) =
            compute_soft(prev_seg_dir_x_, prev_seg_dir_y_, dist_lv_mm);
    }

    // 5c) 合成：V 取更严缩放,dir_eff 取距拐角更近一侧的 lerp 结果
    float V = seg_speed_mm_s_ * std::min(V_scale_up, V_scale_lv);
    // 底盘 |v| < 0.02 m/s 会整车冻结 + 锁舵 (crt_chassis.cpp:541)。
    // cos⁴ × seg_speed=40 在拐角处 =10 mm/s 必然踩死区。floor 在 25 mm/s 给 5 余量。
    // floor 不能超过 seg_speed_,避免在低 seg_speed 路径上反而加速。
    constexpr float kChassisDeadzoneFloorMmS = 25.0f;
    V = std::max(V, std::min(seg_speed_mm_s_, kChassisDeadzoneFloorMmS));
    float dir_eff_x, dir_eff_y;
    if (dist_up_mm <= dist_lv_mm) { dir_eff_x = dir_up_x; dir_eff_y = dir_up_y; }
    else                          { dir_eff_x = dir_lv_x; dir_eff_y = dir_lv_y; }

    // 5d) 软化期 lat PID 衰减:拐角处 dir_eff 朝 bisector,直线段 lat_err 没意义会和 dir_eff 打架
    //     用 smoothstep 权重 min(s_up, s_lv) 在拐角→0、远离拐角→1。
    //     这样圆弧轨迹自由发展,直到回到段中间才让 lat PID 把残余偏置吃掉。
    const float lat_taper = std::min(s_up, s_lv);
    const float v_lat_corr_eff = v_lat_corr_mm_s * lat_taper;

    // 一次性日志:任一侧进入软化时打印一次
    if (!blend_logged_this_leg_ && (V_scale_up < 0.999f || V_scale_lv < 0.999f))
    {
        const bool up_active = (V_scale_up < V_scale_lv);
        std::cout << "[AUTO] soften ENTER wp_idx=" << wp_idx_
                  << " side=" << (up_active ? "UP" : "LV")
                  << " dth_deg=" << (up_active ? dth_up : dth_lv) * 180.0f / kPiF
                  << " V_scale=" << std::min(V_scale_up, V_scale_lv)
                  << " lat_taper=" << lat_taper
                  << " dir=(" << dir_eff_x << "," << dir_eff_y << ")"
                  << std::endl;
        blend_logged_this_leg_ = true;
    }

    const float nx = -dir_eff_y;
    const float ny =  dir_eff_x;
    const float v_wx = V * dir_eff_x + v_lat_corr_eff * nx;
    const float v_wy = V * dir_eff_y + v_lat_corr_eff * ny;

    // 6) 世界 → 机体
    //    实测物理映射(推 vx 前 → OPS -Y, 推 vy 右 → OPS -X, 物理 CCW → ψ+):
    //    机体 +X_body 在 OPS 世界系里指向 -90° 方向 → 机体相对世界的安装偏角 β = ψ - 90°
    const float yaw_rad = (yaw_w - 90.0f) * kDegToRad;
    const float c = std::cos(yaw_rad);
    const float s = std::sin(yaw_rad);
    const float vx_body =  c * v_wx + s * v_wy;
    const float vy_body = -s * v_wx + c * v_wy;

    // 7) 写底盘（chassis API 单位是 m/s，本类内部用 mm/s，下发前做一次 /1000）
    chassis_->Set_Target_Velocity_X(vx_body * 0.001f);
    chassis_->Set_Target_Velocity_Y(vy_body * 0.001f);
    chassis_->Set_Target_Omega(omega_rad_s);
    chassis_->Set_Chassis_Control_Type(Chassis_Control_Type_ENABLE);

    // 8) CSV 日志（每 csv_decimate_ tick 记录一行，默认 100Hz）
    if (csv_stream_.is_open() && (csv_tick_ % csv_decimate_) == 0)
    {
        csv_stream_ << csv_tick_
                    << "," << active_path_id_
                    << "," << wp_idx_
                    << "," << x_w
                    << "," << y_w
                    << "," << yaw_w
                    << "," << along_mm_
                    << "," << lat_err_mm_
                    << "," << head_err_deg_
                    << "," << v_lat_corr_mm_s
                    << "," << omega_rad_s
                    << "," << v_wx
                    << "," << v_wy
                    << "," << vx_body
                    << "," << vy_body
                    << "," << seg_speed_mm_s_
                    << "," << seg_length_mm_
                    << "," << seg_target_yaw_deg_
                    << "," << dir_eff_x
                    << "," << dir_eff_y
                    << "," << std::min(V_scale_up, V_scale_lv)
                    << "\n";
    }
    ++csv_tick_;

    return true;
}

/* ============================================================
 *  内部辅助
 * ========================================================== */

/**
 * @brief 配置一段：起点 (x0, y0) → 终点 wp，计算单位方向向量与段长
 *        起终点重合时（len < 1mm）退化为 0 长度段、方向取 +X，避免 NaN
 */
void Class_Auto_Pilot::Begin_Leg_(float x0_mm, float y0_mm, const Struct_Waypoint &wp)
{
    seg_x0_mm_          = x0_mm;
    seg_y0_mm_          = y0_mm;
    seg_target_yaw_deg_ = wp.yaw_deg;
    seg_speed_mm_s_     = wp.speed_mm_s;
    seg_reach_tol_mm_   = (wp.reach_tol_mm > 1.0f) ? wp.reach_tol_mm : 30.0f;

    const float dx = wp.x_mm - x0_mm;
    const float dy = wp.y_mm - y0_mm;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1.0f)
    {
        seg_dir_x_     = 1.0f;
        seg_dir_y_     = 0.0f;
        seg_length_mm_ = 0.0f;
    }
    else
    {
        seg_dir_x_     = dx / len;
        seg_dir_y_     = dy / len;
        seg_length_mm_ = len;
    }
}

void Class_Auto_Pilot::Open_CSV_Log_()
{
    Close_CSV_Log_();

    auto now = std::chrono::system_clock::now();
    auto tt  = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&tt, &tm_buf);

    std::ostringstream fname;
    fname << "var_data/auto_pilot_"
          << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
          << "_path" << active_path_id_
          << ".csv";

    csv_stream_.open(fname.str(), std::ios::out | std::ios::trunc);
    if (!csv_stream_.is_open())
    {
        std::cerr << "[AUTO] WARN: failed to open CSV log: " << fname.str() << std::endl;
        return;
    }

    csv_stream_ << std::fixed << std::setprecision(3);
    csv_stream_ << "tick,path_id,wp_idx,"
                   "ops_x_mm,ops_y_mm,ops_yaw_deg,"
                   "along_mm,lat_err_mm,head_err_deg,"
                   "v_lat_corr_mm_s,omega_rad_s,"
                   "v_world_x,v_world_y,vx_body,vy_body,"
                   "seg_speed_mm_s,seg_length_mm,seg_target_yaw_deg,"
                   "dir_eff_x,dir_eff_y,blend_active\n";

    std::cout << "[AUTO] CSV log started: " << fname.str() << std::endl;
}

void Class_Auto_Pilot::Close_CSV_Log_()
{
    if (csv_stream_.is_open())
    {
        csv_stream_.flush();
        csv_stream_.close();
        std::cout << "[AUTO] CSV log closed (" << csv_tick_ << " ticks)" << std::endl;
    }
}

float Class_Auto_Pilot::Wrap_Pm180_(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}
