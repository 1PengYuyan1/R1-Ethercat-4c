#!/usr/bin/env python3
"""
2-DOF Pitch 夹爪机械臂实时仿真 / 实机回放 (matplotlib 2D 侧视)

两种模式:
  (1) 离线仿真 (默认): 复刻 crt_clamp.cpp 控制环, 按 a 触发取放序列.
  (2) 实机 UDP live: --udp-port N, 接收 build/.../clamp_telemetry 广播的 28B
      二进制包 (magic 'CLMP' + seq + t_sec + tL/tS/wL/wS, 单精度 LE), 实时绘
      制电机反馈位置. 此时电机仅被 Enter 帧使能、无力矩, 可手动反推臂感受动画.

杆长 CLI 给定, 默认 L1=0.32m / L2=0.16m. 两轴都是 pitch 共面摆动, 用 2D
侧视图 (X=forward, Z=up) + 地面参考线渲染.

约定 (与实际机械标定可能反向, 看动画方向再决定是否给角度加负号):
  theta1 = pitch_large 角度, 从 +X 起, 正方向 = 向上 (+Z)
  theta2 = pitch_small 角度 (相对 L1), 同方向为正
  tip_xz = L1*(cos t1, sin t1) + L2*(cos(t1+t2), sin(t1+t2))

键盘:
  a         触发 Pick&Place 序列 (POS1->2->3->4->1, 共 ~8s)
  e         使能 / 失能切换
  1 2 3 4   强制双轴目标到 POSk (仅 ENABLE 下)
  r         打断序列, 回 POS1
  q / Esc   退出

依赖: matplotlib + numpy (项目已装).
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


# ------- 与 crt_clamp.cpp 完全一致的常量 -------
PITCH_LARGE_POS = (0.0, 2.3, 1.6, 1.2)   # POS1..POS4, rad
PITCH_SMALL_POS = (0.0, -4.2, -3.5, -2.0)
MAX_SPEED_LARGE = 3.0   # rad/s
MAX_SPEED_SMALL = 5.0
STEP_DT = 0.002         # s, 控制环节拍

# _Step_Sequence dwell (单位: 2ms tick)
DWELL_TICKS = (500, 1500, 1000, 1000)   # STEP1->2, 2->3, 3->4, 4->IDLE

# 序列各 STEP 进入时的 (large_pos_idx, small_pos_idx); 触发瞬间已经是 STEP1, 维持 POS1
SEQ_STEP_TARGETS = {
    1: (1, 1),  # STEP1 dwell 完 -> 进入 STEP2 时切到 POS2
    2: (2, 2),  # -> STEP3 -> POS3
    3: (3, 3),  # -> STEP4 -> POS4
    4: (0, 0),  # -> IDLE  -> POS1
}


@dataclass
class ArmGeom:
    L1: float
    L2: float


class ClampSim:
    """无电机版本的 crt_clamp 控制环: 每 STEP_DT 推进一次状态."""

    def __init__(self) -> None:
        self.enabled = True
        self.large_state = 0          # 0..3 -> POS1..POS4
        self.small_state = 0
        self.smooth_large = PITCH_LARGE_POS[0]
        self.smooth_small = PITCH_SMALL_POS[0]
        self.seq_state = 0            # 0 = IDLE, 1..4 = STEP1..STEP4
        self.seq_tick = 0

    # ---- 外部入口 ----
    def trigger_sequence(self) -> None:
        if not self.enabled or self.seq_state != 0:
            return
        self.seq_state = 1
        self.seq_tick = 0
        # 位置保持当前 (POS1), 切换由 _step_sequence 完成

    def toggle_enable(self) -> None:
        self.enabled = not self.enabled
        if not self.enabled:
            self.seq_state = 0
            self.seq_tick = 0

    def jump_to_pos(self, idx: int) -> None:
        """1..4"""
        if not self.enabled or not 1 <= idx <= 4:
            return
        self.large_state = idx - 1
        self.small_state = idx - 1
        # 不打断已有序列; 序列下一节拍会覆盖

    def reset_to_pos1(self) -> None:
        self.large_state = 0
        self.small_state = 0
        self.seq_state = 0
        self.seq_tick = 0

    # ---- 内部 ----
    def _step_sequence(self) -> None:
        if self.seq_state == 0:
            return
        dwell = DWELL_TICKS[self.seq_state - 1]
        self.seq_tick += 1
        if self.seq_tick < dwell:
            return
        self.seq_tick = 0
        next_large, next_small = SEQ_STEP_TARGETS[self.seq_state]
        self.large_state = next_large
        self.small_state = next_small
        self.seq_state = 0 if self.seq_state == 4 else self.seq_state + 1

    def step(self) -> None:
        """推进一个 STEP_DT (2ms) 控制周期."""
        if not self.enabled:
            return
        self._step_sequence()

        tgt_l = PITCH_LARGE_POS[self.large_state]
        tgt_s = PITCH_SMALL_POS[self.small_state]
        step_l = MAX_SPEED_LARGE * STEP_DT
        step_s = MAX_SPEED_SMALL * STEP_DT

        if self.smooth_large < tgt_l - step_l:
            self.smooth_large += step_l
        elif self.smooth_large > tgt_l + step_l:
            self.smooth_large -= step_l
        else:
            self.smooth_large = tgt_l

        if self.smooth_small < tgt_s - step_s:
            self.smooth_small += step_s
        elif self.smooth_small > tgt_s + step_s:
            self.smooth_small -= step_s
        else:
            self.smooth_small = tgt_s

    # ---- 输出 ----
    def joint_positions(self, geom: ArmGeom) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """返回 base/elbow/tip 在 XZ 侧视平面的 2D 坐标 (Y 恒 0)."""
        return forward_kinematics(self.smooth_large, self.smooth_small, geom)


def forward_kinematics(
    theta_large: float, theta_small: float, geom: ArmGeom
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """两轴角度 -> base/elbow/tip 2D 坐标 (XZ 侧视面)."""
    base = np.array([0.0, 0.0])
    elbow = base + np.array(
        [geom.L1 * math.cos(theta_large), geom.L1 * math.sin(theta_large)]
    )
    tip = elbow + np.array(
        [
            geom.L2 * math.cos(theta_large + theta_small),
            geom.L2 * math.sin(theta_large + theta_small),
        ]
    )
    return base, elbow, tip


# ===== Live (UDP) 接收: 与 clamp_telemetry_main.cpp UdpPacket 对齐 =====
UDP_MAGIC = 0x434C4D50  # 'CLMP'
UDP_FMT = "<IIfffff"    # magic, seq, t_sec, tL, tS, wL, wS
UDP_SIZE = struct.calcsize(UDP_FMT)
assert UDP_SIZE == 28


class LiveReceiver:
    """后台 UDP 接收线程, 把最新一帧角度暴露给主线程渲染."""

    def __init__(self, bind_ip: str, port: int) -> None:
        self._bind = (bind_ip, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(self._bind)
        self._sock.settimeout(0.2)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        # 最新一帧
        self.theta_L = 0.0
        self.theta_S = 0.0
        self.omega_L = 0.0
        self.omega_S = 0.0
        self.t_sec = 0.0
        self.seq = 0
        self.recv_count = 0
        self.bad_count = 0
        self.last_recv_monotonic = 0.0
        self._thread = threading.Thread(target=self._loop, name="UdpRecv", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(UDP_SIZE * 4)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) != UDP_SIZE:
                self.bad_count += 1
                continue
            magic, seq, t_sec, tL, tS, wL, wS = struct.unpack(UDP_FMT, data)
            if magic != UDP_MAGIC:
                self.bad_count += 1
                continue
            now_mono = time.monotonic()
            with self._lock:
                self.theta_L = tL
                self.theta_S = tS
                self.omega_L = wL
                self.omega_S = wS
                self.t_sec = t_sec
                self.seq = seq
                self.recv_count += 1
                self.last_recv_monotonic = now_mono

    def snapshot(self) -> tuple[float, float, float, float, float, int, int, float]:
        with self._lock:
            return (
                self.theta_L, self.theta_S, self.omega_L, self.omega_S,
                self.t_sec, self.seq, self.recv_count, self.last_recv_monotonic,
            )


def _state_label(sim: ClampSim) -> str:
    seq = "IDLE" if sim.seq_state == 0 else f"STEP{sim.seq_state}"
    en = "ENABLE" if sim.enabled else "DISABLE"
    return (
        f"{en} | seq={seq} (tick {sim.seq_tick:>4d})\n"
        f"target  POS{sim.large_state + 1} / POS{sim.small_state + 1}\n"
        f"theta_L = {math.degrees(sim.smooth_large):+7.2f} deg "
        f"({sim.smooth_large:+6.3f} rad)\n"
        f"theta_S = {math.degrees(sim.smooth_small):+7.2f} deg "
        f"({sim.smooth_small:+6.3f} rad)"
    )


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--L1", type=float, default=0.32, help="大轴到小轴杆长 (m), 默认 0.32")
    p.add_argument("--L2", type=float, default=0.16, help="小轴到末端杆长 (m), 默认 0.16")
    p.add_argument("--fps", type=float, default=50.0, help="渲染帧率, 默认 50")
    p.add_argument(
        "--speed", type=float, default=1.0,
        help="(仅离线仿真) 时间倍率: 1.0=实时, 0.25=慢放, 4.0=快进",
    )
    p.add_argument(
        "--udp-port", type=int, default=None,
        help="给定端口后进入 LIVE 模式, 接收 clamp_telemetry 广播的 UDP 包",
    )
    p.add_argument("--bind-ip", type=str, default="0.0.0.0",
                   help="(LIVE) 绑定 IP, 默认 0.0.0.0")
    p.add_argument(
        "--invert-large", action="store_true",
        help="反转 theta_large 符号 (机械朝向与默认约定相反时用)",
    )
    p.add_argument(
        "--invert-small", action="store_true",
        help="反转 theta_small 符号",
    )
    args = p.parse_args()

    geom = ArmGeom(L1=args.L1, L2=args.L2)
    live_mode = args.udp_port is not None

    sim: ClampSim | None = None
    receiver: LiveReceiver | None = None
    if live_mode:
        receiver = LiveReceiver(args.bind_ip, args.udp_port)
        receiver.start()
        print(f"[LIVE] listening on udp://{args.bind_ip}:{args.udp_port}")
    else:
        sim = ClampSim()
        steps_per_frame = max(1, int(round((1.0 / args.fps) * args.speed / STEP_DT)))
        print("Keys:  a=trigger seq   e=enable toggle   1..4=jump POS   r=reset   q/Esc=quit")
        print(f"speed={args.speed}x  fps={args.fps:.0f}  steps/frame={steps_per_frame}")

    interval_ms = 1000.0 / args.fps

    reach = geom.L1 + geom.L2
    span = reach * 1.15

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_xlim(-span, span)
    ax.set_ylim(-span, span)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (m)  forward")
    ax.set_ylabel("Z (m)  up")
    mode_label = "LIVE (UDP)" if live_mode else "SIM"
    ax.set_title(
        f"Clamp Arm  L1={geom.L1*100:.0f}cm  L2={geom.L2*100:.0f}cm  [{mode_label}]"
    )
    ax.grid(True, alpha=0.3)
    ax.axhline(0.0, color="gray", lw=0.8, alpha=0.5)
    ax.axvline(0.0, color="gray", lw=0.8, alpha=0.5)

    (link1_line,) = ax.plot([], [], "-", lw=4, color="#1f77b4", label="L1 (large pitch)")
    (link2_line,) = ax.plot([], [], "-", lw=4, color="#ff7f0e", label="L2 (small pitch)")
    (joints_pts,) = ax.plot([], [], "o", ms=7, color="black")
    (tip_pt,) = ax.plot([], [], "o", ms=10, color="red", label="end-effector")
    (trail_line,) = ax.plot([], [], "-", lw=1, color="red", alpha=0.35, label="tip trail")
    ax.legend(loc="upper left", fontsize=8)

    info_text = ax.text(0.02, 0.02, "", transform=ax.transAxes,
                        family="monospace", fontsize=9,
                        verticalalignment="bottom",
                        bbox=dict(boxstyle="round", facecolor="white", alpha=0.75))
    tip_text = ax.text(0.98, 0.98, "", transform=ax.transAxes,
                       family="monospace", fontsize=9,
                       horizontalalignment="right", verticalalignment="top",
                       bbox=dict(boxstyle="round", facecolor="white", alpha=0.75))

    trail: list[np.ndarray] = []
    TRAIL_MAX = 400
    sign_L = -1.0 if args.invert_large else 1.0
    sign_S = -1.0 if args.invert_small else 1.0

    def on_key(event):
        k = (event.key or "").lower()
        if k in ("q", "escape"):
            plt.close(fig)
            return
        if sim is None:
            return  # LIVE 模式下控制键无效
        if k == "a":
            sim.trigger_sequence()
        elif k == "e":
            sim.toggle_enable()
        elif k in ("1", "2", "3", "4"):
            sim.jump_to_pos(int(k))
        elif k == "r":
            sim.reset_to_pos1()

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        if sim is not None:
            for _ in range(steps_per_frame):
                sim.step()
            tL = sim.smooth_large * sign_L
            tS = sim.smooth_small * sign_S
            label = _state_label(sim)
        else:
            assert receiver is not None
            tL_raw, tS_raw, wL, wS, t_sec, seq, cnt, last_mono = receiver.snapshot()
            tL = tL_raw * sign_L
            tS = tS_raw * sign_S
            age_ms = (time.monotonic() - last_mono) * 1000.0 if last_mono > 0 else float("inf")
            stale = age_ms > 500.0
            label = (
                f"LIVE  recv={cnt}  seq={seq}  "
                f"{'STALE' if stale else f'age={age_ms:5.0f}ms'}\n"
                f"t_src = {t_sec:7.2f} s\n"
                f"theta_L = {math.degrees(tL):+7.2f} deg "
                f"({tL:+6.3f} rad)  w={wL:+6.2f} rad/s\n"
                f"theta_S = {math.degrees(tS):+7.2f} deg "
                f"({tS:+6.3f} rad)  w={wS:+6.2f} rad/s"
            )

        base, elbow, tip = forward_kinematics(tL, tS, geom)
        link1_line.set_data([base[0], elbow[0]], [base[1], elbow[1]])
        link2_line.set_data([elbow[0], tip[0]], [elbow[1], tip[1]])
        joints_pts.set_data([base[0], elbow[0]], [base[1], elbow[1]])
        tip_pt.set_data([tip[0]], [tip[1]])

        trail.append(tip.copy())
        if len(trail) > TRAIL_MAX:
            del trail[: len(trail) - TRAIL_MAX]
        if len(trail) >= 2:
            arr = np.asarray(trail)
            trail_line.set_data(arr[:, 0], arr[:, 1])

        info_text.set_text(label)
        tip_text.set_text(
            f"tip   X={tip[0]*100:+6.1f}cm  Z={tip[1]*100:+6.1f}cm\n"
            f"elbow X={elbow[0]*100:+6.1f}cm  Z={elbow[1]*100:+6.1f}cm"
        )
        return link1_line, link2_line, joints_pts, tip_pt, trail_line, info_text, tip_text

    _ = FuncAnimation(fig, update, interval=interval_ms, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        if receiver is not None:
            receiver.stop()


if __name__ == "__main__":
    main()
