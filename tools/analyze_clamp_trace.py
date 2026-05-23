#!/usr/bin/env python3
"""
analyze_clamp_trace.py — Class_Clamp 13 列 CSV 分析 (重力补偿 / FK 几何 / 执行跟踪)

CSV 表头 (crt_clamp.cpp:55):
  ts_ms,seq,seg,sp_q1,act_q1,sp_q2,act_q2,omega_q1,omega_q2,torque_q1,torque_q2,tau1_ff,tau2_ff

Usage:
  python3 tools/analyze_clamp_trace.py /tmp/clamp_xxx.csv [--plot]

输出三大块:
  1. SEGMENT TABLE      — 每个段 (seg=1) 起止 sp/act、ω/τ 峰值、跟踪 RMS、settling、过冲
  2. STATIC HOLD TABLE  — 段间静止 (seg=0 ≥0.5s, sp 不变) 偏差 + 推荐 K1
  3. FK / INTERP DECISION — POS 末端坐标 + PTP/LIN 选择是否合理

依赖: pandas, numpy, (optional) matplotlib
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass

import numpy as np
import pandas as pd

L1 = 0.32  # m, 大臂
L2 = 0.16  # m, 小臂
KP_L = 50.0  # crt_clamp.h pitch_large_kp
KD_L = 1.5
KP_S = 50.0
KD_S = 1.0
K1_RUN = 1.83  # 当前运行 K1
K2_RUN = 0.01  # 当前运行 K2
V_MAX_L = 3.0  # rad/s
V_MAX_S = 5.0
A_MAX_L = 15.0  # rad/s²
A_MAX_S = 25.0
T_MAX = 15.0  # Nm, DM PMAX
SETTLE_BAND_DEG = 1.0  # ±1° 算 settled


def fk(q1: float, q2: float) -> tuple[float, float]:
    x = -L1 * math.cos(q1) - L2 * math.cos(q1 + q2)
    y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    return x, y


def wrap_pi(a: float) -> float:
    return math.remainder(a, 2 * math.pi)


def find_segments(seg_col: np.ndarray) -> list[tuple[int, int]]:
    """段 = seg 列连续 1 的区间"""
    s = seg_col.astype(int)
    edges = np.diff(s)
    starts = list(np.where(edges == 1)[0] + 1)
    ends = list(np.where(edges == -1)[0] + 1)
    if s[0] == 1:
        starts.insert(0, 0)
    if s[-1] == 1:
        ends.append(len(s))
    return list(zip(starts, ends))


def find_static_holds(df: pd.DataFrame, min_ms: float = 500) -> list[tuple[int, int]]:
    """静止段 = seg=0 连续 + sp 变化 ≤ 1e-4 + 时长 ≥ min_ms"""
    seg0 = (df.seg.values == 0).astype(int)
    edges = np.diff(seg0)
    starts = list(np.where(edges == 1)[0] + 1)
    ends = list(np.where(edges == -1)[0] + 1)
    if seg0[0] == 1:
        starts.insert(0, 0)
    if seg0[-1] == 1:
        ends.append(len(seg0))
    out = []
    for s, e in zip(starts, ends):
        if (df.ts_ms.iloc[e - 1] - df.ts_ms.iloc[s]) < min_ms:
            continue
        # 丢掉前 100ms (settling)
        trim = min(50, (e - s) // 4)
        s2 = s + trim
        if e - s2 < 50:
            continue
        # sp 在窗口内必须基本不变
        if df.sp_q1.iloc[s2:e].std() > 1e-3 or df.sp_q2.iloc[s2:e].std() > 1e-3:
            continue
        out.append((s2, e))
    return out


def settling_time_ms(t: np.ndarray, sp: np.ndarray, act: np.ndarray,
                     band_rad: float) -> float:
    """从段末态反查,找最后一次离开 band 的时刻; 返回 t[end-1] - t[that]"""
    end_sp = sp[-1]
    err = np.abs(act - end_sp)
    inside = err <= band_rad
    if inside[-1] is False or inside[-1] == 0:
        return float('nan')
    # 从尾往头找最后一个 outside
    last_out = np.where(~inside)[0]
    if len(last_out) == 0:
        return 0.0
    return float(t[-1] - t[last_out[-1]])


def _overshoot_deg(sp: np.ndarray, act: np.ndarray) -> float:
    """末态 sp 之后,act 是否冲过头。
    sp 向上: ovs = max(0, act.max - sp_end)
    sp 向下: ovs = max(0, sp_end - act.min)
    sp 几乎不变: 取 act 偏 sp_end 的绝对值最大。"""
    sp_end = sp[-1]
    delta = sp_end - sp[0]
    if abs(delta) < 1e-3:
        return math.degrees(np.abs(act - sp_end).max())
    if delta > 0:
        return math.degrees(max(0.0, act.max() - sp_end))
    return math.degrees(max(0.0, sp_end - act.min()))


def analyze_segments(df: pd.DataFrame, fs: float) -> None:
    segs = find_segments(df.seg.values)
    if not segs:
        print("  (no segments found — seg 列恒 0,可能没触发任何 RB+A/Right/B 按键)")
        return
    print(f"\n{'='*108}")
    print(f"1. SEGMENT TABLE — {len(segs)} 段")
    print(f"{'='*108}")
    hdr = (f"{'#':<3} {'t_s':>6} {'dur':>5} "
           f"{'q1: sp_st→sp_ed':>17} {'act_ed':>7} {'err_pk':>7} "
           f"{'ω_pk':>5} {'τ_pk':>5} {'τ_ff':>5} {'ovs°':>5} {'set_ms':>7} | "
           f"{'q2: sp_st→sp_ed':>17} {'act_ed':>7} {'err_pk':>7} "
           f"{'ω_pk':>5} {'τ_pk':>5} {'τ_ff':>5} {'ovs°':>5} {'set_ms':>7}")
    print(hdr)
    print('-' * len(hdr))
    band = math.radians(SETTLE_BAND_DEG)
    for i, (s, e) in enumerate(segs):
        t = df.ts_ms.values[s:e] / 1000.0
        dur = t[-1] - t[0]
        # large
        sp1 = df.sp_q1.values[s:e]
        ac1 = df.act_q1.values[s:e]
        w1 = df.omega_q1.values[s:e]
        tq1 = df.torque_q1.values[s:e]
        ff1 = df.tau1_ff.values[s:e]
        err1 = ac1 - sp1
        ovs1_deg = _overshoot_deg(sp1, ac1)
        set1 = settling_time_ms(t * 1000, sp1, ac1, band)
        # small
        sp2 = df.sp_q2.values[s:e]
        ac2 = df.act_q2.values[s:e]
        w2 = df.omega_q2.values[s:e]
        tq2 = df.torque_q2.values[s:e]
        ff2 = df.tau2_ff.values[s:e]
        err2 = ac2 - sp2
        ovs2_deg = _overshoot_deg(sp2, ac2)
        set2 = settling_time_ms(t * 1000, sp2, ac2, band)
        print(f"{i:<3} {t[0]:>6.2f} {dur:>5.2f} "
              f"{sp1[0]:>7.3f}→{sp1[-1]:>7.3f} {ac1[-1]:>7.3f} "
              f"{math.degrees(np.abs(err1).max()):>6.2f}° "
              f"{abs(w1).max():>5.2f} {abs(tq1).max():>5.2f} "
              f"{abs(ff1).max():>5.2f} {ovs1_deg:>5.2f} {set1:>7.0f} | "
              f"{sp2[0]:>7.3f}→{sp2[-1]:>7.3f} {ac2[-1]:>7.3f} "
              f"{math.degrees(np.abs(err2).max()):>6.2f}° "
              f"{abs(w2).max():>5.2f} {abs(tq2).max():>5.2f} "
              f"{abs(ff2).max():>5.2f} {ovs2_deg:>5.2f} {set2:>7.0f}")

    # 警示
    print()
    print("Hints:")
    print(f"  · v_max:   L1 ω 应 ≤ {V_MAX_L:.1f} rad/s, L2 ≤ {V_MAX_S:.1f} rad/s")
    print(f"  · τ limit: |τ| 接近 {T_MAX:.0f} 表示撞限幅 (TMAX),应降 Kp 或加大段时长")
    print(f"  · settling 用 ±{SETTLE_BAND_DEG:.1f}° band; NaN = 末态尚未稳进 band")
    print(f"  · err_pk: 段内 |sp-act| 最大角度差; tracking lag 主要来自梯形规划")


def analyze_static_holds(df: pd.DataFrame) -> None:
    holds = find_static_holds(df)
    if not holds:
        print("\n  (no static-hold windows ≥0.5s — 多按几下 RB+A/Right/B 然后停 1s+)")
        return
    print(f"\n{'='*108}")
    print(f"2. STATIC HOLD TABLE — {len(holds)} 窗口 (seg=0, sp 不变, ≥0.5s)")
    print(f"{'='*108}")
    print(f"{'#':<3} {'t_s':>6} {'dur':>5} "
          f"{'sp_q1':>7} {'mean_err1_deg':>14} {'std1_deg':>9} "
          f"{'τ1_avg':>7} {'τ1_ff':>6} {'K1_rec':>7} | "
          f"{'sp_q2':>7} {'mean_err2_deg':>14} {'std2_deg':>9} "
          f"{'τ2_avg':>7} {'τ2_ff':>6} {'K2_rec':>7}")
    for i, (s, e) in enumerate(holds):
        t = df.ts_ms.values[s:e] / 1000.0
        sp1 = df.sp_q1.values[s:e].mean()
        sp2 = df.sp_q2.values[s:e].mean()
        err1 = df.act_q1.values[s:e] - sp1
        err2 = df.act_q2.values[s:e] - sp2
        tau1 = df.torque_q1.values[s:e].mean()
        tau2 = df.torque_q2.values[s:e].mean()
        ff1 = df.tau1_ff.values[s:e].mean()
        ff2 = df.tau2_ff.values[s:e].mean()
        # 推荐 K: 静态平衡 τ_motor + τ_grav = 0
        #   τ_motor = τ_ff + Kp·(sp - act) = τ_ff - Kp·err
        #   τ_grav  = -K_true·cos(q)
        #   => K_true·cos(q) = τ_ff - Kp·err = K_run·cos(q) - Kp·err
        #   => K_rec = K_run - Kp·err / cos(q)
        # 注: cos(q) 接近 0 时分母放大噪声 → 标 NaN
        c1 = math.cos(sp1)
        c12 = math.cos(sp1 + sp2)
        k1_rec = (K1_RUN - KP_L * err1.mean() / c1) if abs(c1) > 0.1 else float('nan')
        k2_rec = (K2_RUN - KP_S * err2.mean() / c12) if abs(c12) > 0.1 else float('nan')
        print(f"{i:<3} {t[0]:>6.2f} {t[-1]-t[0]:>5.2f} "
              f"{sp1:>7.3f} {math.degrees(err1.mean()):>14.3f} "
              f"{math.degrees(err1.std()):>9.4f} "
              f"{tau1:>7.3f} {ff1:>6.3f} {k1_rec:>7.3f} | "
              f"{sp2:>7.3f} {math.degrees(err2.mean()):>14.3f} "
              f"{math.degrees(err2.std()):>9.4f} "
              f"{tau2:>7.3f} {ff2:>6.3f} {k2_rec:>7.3f}")
    print()
    print("Hints:")
    print(f"  · mean_err: 静摩擦死区 + grav 残余;|err| ≤ ~2° 通常即"
          f"达 MIT 位置环精度上限")
    print(f"  · K1_rec: 用 K1_new=K1+Kp·err/cos(q1) 反推, 多个窗口取均值更稳"
          f" (当前 K1={K1_RUN}, K2={K2_RUN})")
    print(f"  · τ_avg vs τ_ff: 二者差距 = Kp·err + Kd·ω, 若差距 ≫ K·cos 重力项,"
          f"说明 FF 不够,被 P 项接管")


def analyze_fk_per_segment(df: pd.DataFrame) -> None:
    segs = find_segments(df.seg.values)
    if not segs:
        return
    print(f"\n{'='*108}")
    print(f"3. FK / 段几何 — 每段起止末端坐标 + elbow 支 + 跨原点判定")
    print(f"{'='*108}")
    print(f"{'#':<3} {'sp_q1_st':>9} {'sp_q2_st':>9} {'x_st':>7} {'y_st':>7} {'elb_st':>7} | "
          f"{'sp_q1_ed':>9} {'sp_q2_ed':>9} {'x_ed':>7} {'y_ed':>7} {'elb_ed':>7} | "
          f"{'cross':>5} {'near_sh':>7} {'flip':>5}")
    for i, (s, e) in enumerate(segs):
        q1s = df.sp_q1.iloc[s]
        q2s = df.sp_q2.iloc[s]
        q1e = df.sp_q1.iloc[e - 1]
        q2e = df.sp_q2.iloc[e - 1]
        xs, ys = fk(q1s, q2s)
        xe, ye = fk(q1e, q2e)
        # elbow 支: q2 mod 2π ∈ (0, π) = up, (-π, 0) = down
        q2sw = wrap_pi(q2s)
        q2ew = wrap_pi(q2e)
        elb_s = 'up' if q2sw > 0 else 'down'
        elb_e = 'up' if q2ew > 0 else 'down'
        cross = (xs * xe) < 0
        near_sh = math.hypot(xs, ys) < 0.13 or math.hypot(xe, ye) < 0.13
        flip = (q2sw * q2ew) < 0
        mode = 'PTP' if (cross or near_sh or flip) else 'LIN'
        print(f"{i:<3} {q1s:>9.3f} {q2s:>9.3f} "
              f"{xs*1000:>6.1f} {ys*1000:>6.1f} {elb_s:>7} | "
              f"{q1e:>9.3f} {q2e:>9.3f} "
              f"{xe*1000:>6.1f} {ye*1000:>6.1f} {elb_e:>7} | "
              f"{str(cross):>5} {str(near_sh):>7} {str(flip):>5} → {mode}")
    print()
    print("Hints:")
    print("  · 坐标系: 零位 q=(0,0) 末端在 (-L1-L2, 0) = (-480mm, 0); "
          "x>0 = 前方,y>0 = 上方")
    print("  · cross/near_sh/flip 任一 true → 选 PTP (避免穿肩 / 支切换)")


def main():
    plot = '--plot' in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if not args:
        print(__doc__)
        sys.exit(1)
    csv_path = args[0]
    df = pd.read_csv(csv_path)
    n = len(df)
    if n < 100:
        print(f"  too few rows ({n})")
        sys.exit(1)
    dur_s = (df.ts_ms.iloc[-1] - df.ts_ms.iloc[0]) / 1000.0
    dt_ms = np.diff(df.ts_ms.values).mean()
    fs = 1000.0 / dt_ms
    print(f"\n=== {csv_path} ===")
    print(f"  rows={n}, dur={dur_s:.1f}s, fs≈{fs:.0f}Hz  "
          f"({'OK' if 480 < fs < 520 else 'WARN: 期望 500Hz'})")

    analyze_segments(df, fs)
    analyze_static_holds(df)
    analyze_fk_per_segment(df)

    if plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available, skipping --plot")
            return
        t = (df.ts_ms.values - df.ts_ms.values[0]) / 1000.0
        fig, axes = plt.subplots(4, 2, figsize=(15, 11), sharex=True)
        # 大臂列
        axes[0, 0].plot(t, df.sp_q1, 'b-', lw=0.6, label='sp')
        axes[0, 0].plot(t, df.act_q1, 'r-', lw=0.6, label='act')
        axes[0, 0].set_ylabel('q1 [rad]')
        axes[0, 0].set_title('Pitch_Large')
        axes[0, 0].legend(loc='upper right', fontsize=8)
        axes[1, 0].plot(t, df.omega_q1, 'g-', lw=0.6)
        axes[1, 0].axhline(V_MAX_L, color='k', ls=':', lw=0.5)
        axes[1, 0].axhline(-V_MAX_L, color='k', ls=':', lw=0.5)
        axes[1, 0].set_ylabel('ω1 [rad/s]')
        axes[2, 0].plot(t, df.torque_q1, 'm-', lw=0.6, label='τ_motor')
        axes[2, 0].plot(t, df.tau1_ff, 'c-', lw=0.6, label='τ_ff')
        axes[2, 0].axhline(T_MAX, color='k', ls=':', lw=0.5)
        axes[2, 0].axhline(-T_MAX, color='k', ls=':', lw=0.5)
        axes[2, 0].set_ylabel('τ1 [Nm]')
        axes[2, 0].legend(loc='upper right', fontsize=8)
        axes[3, 0].plot(t, np.degrees(df.act_q1 - df.sp_q1), 'r-', lw=0.6)
        axes[3, 0].set_ylabel('err1 [°]')
        axes[3, 0].set_xlabel('t [s]')
        # 小臂列
        axes[0, 1].plot(t, df.sp_q2, 'b-', lw=0.6, label='sp')
        axes[0, 1].plot(t, df.act_q2, 'r-', lw=0.6, label='act')
        axes[0, 1].set_ylabel('q2 [rad]')
        axes[0, 1].set_title('Pitch_Small')
        axes[1, 1].plot(t, df.omega_q2, 'g-', lw=0.6)
        axes[1, 1].axhline(V_MAX_S, color='k', ls=':', lw=0.5)
        axes[1, 1].axhline(-V_MAX_S, color='k', ls=':', lw=0.5)
        axes[1, 1].set_ylabel('ω2 [rad/s]')
        axes[2, 1].plot(t, df.torque_q2, 'm-', lw=0.6, label='τ_motor')
        axes[2, 1].plot(t, df.tau2_ff, 'c-', lw=0.6, label='τ_ff')
        axes[2, 1].set_ylabel('τ2 [Nm]')
        axes[3, 1].plot(t, np.degrees(df.act_q2 - df.sp_q2), 'r-', lw=0.6)
        axes[3, 1].set_ylabel('err2 [°]')
        axes[3, 1].set_xlabel('t [s]')
        for ax in axes.flat:
            ax.grid(alpha=0.3)
        plt.tight_layout()
        out = csv_path.replace('.csv', '_plot.png')
        plt.savefig(out, dpi=120)
        print(f"\n  plot saved: {out}")


if __name__ == '__main__':
    main()
