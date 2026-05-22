#!/usr/bin/env python3
"""
wheel_breakaway_scan.py — 4 轮全向底盘破静速度扫描

前置:
  1. ./start_full_system.sh 主程序在跑
  2. 摇杆按 Start 长按使能后, 把 4 轮朝向摆稳(随便方向都行, 脚本自适应)
  3. 把车放在地面、留 0.5m+ 余量
  4. 摇杆放开 (脚本会注入 /cmd_vel)

流程:
  顺序扫 X+, X-, Y+, Y- 四段:
    cmd 从 START_SPEED 起每 STEP_TIME_S 增加 STEP_SPEED
    1s 内 OPS 位移 magnitude > THRESH_MM 记 breakaway → 进入下一段
    超过 MAX_SPEED 还没动 → 记 FAIL, 进入下一段
  段间冷却 COOLDOWN_S, cmd 归 0

依赖: rclpy, geometry_msgs (主程序 ros2 环境已经齐了)

用法:
  ros2 run rclpy ... 不需要, 直接:
  source install/setup.bash
  python3 tools/wheel_breakaway_scan.py
"""

import csv
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ---- 扫描参数 ----
START_SPEED = 0.002   # m/s, 起始 cmd
STEP_SPEED  = 0.002   # m/s, 每步加
STEP_TIME_S = 1.0     # s, 每步持续 (OPS 1s 内位移)
THRESH_MM   = 5.0     # mm, 1s 位移 > 这值 = 动了
MAX_SPEED   = 0.030   # m/s, 上限保护 (= 15 步)
COOLDOWN_S  = 1.5     # s, 段间停车

PUB_HZ      = 50      # /cmd_vel 发布频率


@dataclass
class OpsSnap:
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_deg: float = 0.0
    received: bool = False


class Scanner(Node):
    def __init__(self):
        super().__init__('wheel_breakaway_scan')
        self.ops = OpsSnap()
        self.create_subscription(Twist, '/ops', self._on_ops, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 20)

    def _on_ops(self, msg: Twist):
        self.ops.x_mm = msg.linear.x
        self.ops.y_mm = msg.linear.y
        self.ops.yaw_deg = msg.angular.z
        self.ops.received = True

    def send_cmd(self, vx, vy, omega=0.0):
        m = Twist()
        m.linear.x = float(vx)
        m.linear.y = float(vy)
        m.angular.z = float(omega)
        self.pub.publish(m)

    def hold_cmd(self, vx, vy, dur_s):
        """以 PUB_HZ 持续发同一 cmd 维持 dur_s, 边发边 spin 接收 OPS."""
        t_end = time.monotonic() + dur_s
        period = 1.0 / PUB_HZ
        while time.monotonic() < t_end and rclpy.ok():
            self.send_cmd(vx, vy)
            rclpy.spin_once(self, timeout_sec=period)

    def wait_ops(self, timeout_s=5.0):
        t_end = time.monotonic() + timeout_s
        while not self.ops.received and time.monotonic() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ops.received

    def scan_one(self, label, axis):
        """axis: 'x+' 'x-' 'y+' 'y-'  → 返回 breakaway cmd 速度或 None."""
        sign = +1.0 if axis.endswith('+') else -1.0
        is_x = axis.startswith('x')

        self.get_logger().info(f'===== SCAN {label} (axis={axis}) =====')
        # 段前 cooldown
        self.hold_cmd(0.0, 0.0, COOLDOWN_S)

        cmd = START_SPEED
        while cmd <= MAX_SPEED + 1e-9 and rclpy.ok():
            x0, y0 = self.ops.x_mm, self.ops.y_mm
            vx = sign * cmd if is_x else 0.0
            vy = sign * cmd if not is_x else 0.0
            self.hold_cmd(vx, vy, STEP_TIME_S)
            dx = self.ops.x_mm - x0
            dy = self.ops.y_mm - y0
            disp = math.hypot(dx, dy)
            self.get_logger().info(
                f'  cmd={cmd*1000:5.1f} mm/s  Δ=({dx:+6.2f},{dy:+6.2f}) mm  |Δ|={disp:5.2f} mm')
            if disp > THRESH_MM:
                self.get_logger().info(f'  >> BREAKAWAY @ {cmd*1000:.1f} mm/s')
                self.hold_cmd(0.0, 0.0, COOLDOWN_S)
                return cmd
            cmd += STEP_SPEED

        self.get_logger().warn(f'  >> FAIL: no breakaway within {MAX_SPEED*1000:.0f} mm/s')
        self.hold_cmd(0.0, 0.0, COOLDOWN_S)
        return None


def main():
    rclpy.init()
    node = Scanner()
    if not node.wait_ops():
        node.get_logger().error('No /ops msg in 5s. 主程序起来了吗?')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info(
        f'OPS first frame: x={node.ops.x_mm:.1f}mm y={node.ops.y_mm:.1f}mm yaw={node.ops.yaw_deg:.2f}deg')
    node.get_logger().info('开始扫描. 请勿动摇杆.')

    results = {}
    for label, axis in [('X+', 'x+'), ('X-', 'x-'), ('Y+', 'y+'), ('Y-', 'y-')]:
        results[label] = node.scan_one(label, axis)

    # 安全停车
    node.hold_cmd(0.0, 0.0, 0.5)

    # 汇总
    print('\n========== BREAKAWAY SUMMARY ==========')
    print(f'  X+ : {results["X+"]*1000:.1f} mm/s' if results['X+'] else '  X+ : FAIL')
    print(f'  X- : {results["X-"]*1000:.1f} mm/s' if results['X-'] else '  X- : FAIL')
    print(f'  Y+ : {results["Y+"]*1000:.1f} mm/s' if results['Y+'] else '  Y+ : FAIL')
    print(f'  Y- : {results["Y-"]*1000:.1f} mm/s' if results['Y-'] else '  Y- : FAIL')

    ok = [v for v in results.values() if v is not None]
    if ok:
        v_min = min(ok)
        v_max = max(ok)
        print(f'\n  范围: {v_min*1000:.1f} ~ {v_max*1000:.1f} mm/s')
        print(f'  建议 v_mod 门槛 = {v_min*0.6*1000:.2f} mm/s = {v_min*0.6:.4f} m/s')
        print(f'  建议 wheel_omega_dz = {v_min*0.6/0.018:.3f} rad/s  (Wheel_Radius=0.018)')

    # csv
    csv_path = '/home/rc/Ethercat_R1/var_data/breakaway_result.csv'
    with open(csv_path, 'a', newline='') as f:
        w = csv.writer(f)
        w.writerow([time.strftime('%Y-%m-%d %H:%M:%S'),
                    results['X+'], results['X-'], results['Y+'], results['Y-']])
    print(f'\nCSV 追加到 {csv_path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
