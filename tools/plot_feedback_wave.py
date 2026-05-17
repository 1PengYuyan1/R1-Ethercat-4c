#!/usr/bin/env python3
"""
Stream parser + ASCII waveform plotter for key=value telemetry lines.

Typical input is the vehicle_control live dashboard text stream.
Examples of supported series keys:
  now_vx, target_vx, now_omega
  M0.omega, M1.torque
  E0.wheel_rad, E2.omega_rpm
  O0.pos, O3.busV
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import select
import shutil
import sys
import time
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

ANSI_RE = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")
KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([-+]?0x[0-9A-Fa-f]+|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)")
PREFIX_RE = re.compile(r"^([A-Za-z]\d+)\b")

SERIES_CHARS = "123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ*+xo#@"


class SeriesBuffer:
    def __init__(self, maxlen: int) -> None:
        self.values: Deque[float] = deque(maxlen=maxlen)
        self.last_ts: float = 0.0

    def append(self, ts: float, v: float) -> None:
        self.values.append(v)
        self.last_ts = ts


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Parse stdin key=value telemetry and render ASCII waveforms."
    )
    parser.add_argument(
        "--series",
        action="append",
        default=[],
        help="Series key to plot. Repeat for multiple series, e.g. --series now_vx --series M0.omega",
    )
    parser.add_argument(
        "--history",
        type=int,
        default=400,
        help="Number of recent samples to keep per series (default: 400)",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=0,
        help="Plot width in characters (0 = auto terminal width)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=18,
        help="Plot height in lines (default: 18)",
    )
    parser.add_argument(
        "--refresh-hz",
        type=float,
        default=8.0,
        help="Screen refresh rate (default: 8 Hz)",
    )
    parser.add_argument(
        "--csv",
        default="",
        help="Optional CSV output path",
    )
    parser.add_argument(
        "--line-filter",
        default="",
        help="Optional regex: only parse matching lines",
    )
    parser.add_argument(
        "--list-keys",
        action="store_true",
        help="Print newly discovered keys to stderr",
    )
    parser.add_argument(
        "--y-min",
        type=float,
        default=math.nan,
        help="Optional fixed y min",
    )
    parser.add_argument(
        "--y-max",
        type=float,
        default=math.nan,
        help="Optional fixed y max",
    )
    return parser.parse_args()


def strip_ansi(s: str) -> str:
    return ANSI_RE.sub("", s)


def parse_numeric(s: str) -> float:
    if s.lower().startswith("0x"):
        return float(int(s, 16))
    return float(s)


def line_to_kv(line: str) -> Dict[str, float]:
    line = strip_ansi(line).strip()
    if not line:
        return {}

    m = PREFIX_RE.match(line)
    prefix = m.group(1) if m else ""

    out: Dict[str, float] = {}
    for k, raw_v in KV_RE.findall(line):
        try:
            v = parse_numeric(raw_v)
        except ValueError:
            continue

        if prefix:
            out[f"{prefix}.{k}"] = v
        else:
            out[k] = v
    return out


def auto_width(user_width: int) -> int:
    if user_width > 10:
        return user_width
    cols = shutil.get_terminal_size((120, 40)).columns
    return max(40, cols - 2)


def y_to_row(v: float, ymin: float, ymax: float, h: int) -> int:
    if ymax <= ymin:
        return h // 2
    t = (v - ymin) / (ymax - ymin)
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return int(round((1.0 - t) * (h - 1)))


def bresenham(grid: List[List[str]], x0: int, y0: int, x1: int, y1: int, ch: str) -> None:
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    x, y = x0, y0
    while True:
        if 0 <= y < len(grid) and 0 <= x < len(grid[0]):
            if grid[y][x] == " ":
                grid[y][x] = ch
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


def resample(values: List[float], width: int) -> List[Optional[float]]:
    if not values:
        return [None] * width
    if width <= 1:
        return [values[-1]]
    if len(values) == 1:
        return [values[0]] * width

    n = len(values)
    out: List[Optional[float]] = []
    for i in range(width):
        pos = i * (n - 1) / (width - 1)
        lo = int(pos)
        hi = lo + 1 if lo + 1 < n else lo
        frac = pos - lo
        v = values[lo] * (1.0 - frac) + values[hi] * frac
        out.append(v)
    return out


def render(
    series_order: List[str],
    buffers: Dict[str, SeriesBuffer],
    discovered: List[str],
    width: int,
    height: int,
    fixed_ymin: float,
    fixed_ymax: float,
) -> None:
    grid = [[" "] * width for _ in range(height)]

    all_vals: List[float] = []
    resampled: Dict[str, List[Optional[float]]] = {}

    for name in series_order:
        vals = list(buffers[name].values)
        if vals:
            all_vals.extend(vals)
        resampled[name] = resample(vals, width)

    if math.isfinite(fixed_ymin) and math.isfinite(fixed_ymax) and fixed_ymax > fixed_ymin:
        ymin = fixed_ymin
        ymax = fixed_ymax
    elif all_vals:
        ymin = min(all_vals)
        ymax = max(all_vals)
        if abs(ymax - ymin) < 1e-9:
            pad = 1.0 if abs(ymin) < 1.0 else abs(ymin) * 0.1
            ymin -= pad
            ymax += pad
    else:
        ymin, ymax = -1.0, 1.0

    if ymin < 0.0 < ymax:
        zero_row = y_to_row(0.0, ymin, ymax, height)
        for x in range(width):
            grid[zero_row][x] = "-"

    for idx, name in enumerate(series_order):
        ch = SERIES_CHARS[idx % len(SERIES_CHARS)]
        pts = resampled[name]
        prev: Optional[Tuple[int, int]] = None
        for x, v in enumerate(pts):
            if v is None:
                prev = None
                continue
            y = y_to_row(v, ymin, ymax, height)
            if prev is not None:
                bresenham(grid, prev[0], prev[1], x, y, ch)
            if 0 <= y < height and 0 <= x < width:
                grid[y][x] = ch
            prev = (x, y)

    now = time.monotonic()

    lines: List[str] = []
    lines.append("ASCII Feedback Waveform (Ctrl+C exit)")
    lines.append(f"y_range=[{ymin:.6g}, {ymax:.6g}]  samples/history={max((len(buffers[s].values) for s in series_order), default=0)}")

    for idx, name in enumerate(series_order):
        vals = list(buffers[name].values)
        ch = SERIES_CHARS[idx % len(SERIES_CHARS)]
        if vals:
            age = now - buffers[name].last_ts
            lines.append(
                f" {ch} {name:16s} last={vals[-1]:.6g} min={min(vals):.6g} max={max(vals):.6g} n={len(vals)} age={age:.2f}s"
            )
        else:
            lines.append(f" {ch} {name:16s} (no data yet)")

    if discovered:
        preview = ", ".join(discovered[:12])
        suffix = " ..." if len(discovered) > 12 else ""
        lines.append(f"keys_seen: {preview}{suffix}")

    lines.append("=" * width)
    lines.extend("".join(row) for row in grid)

    sys.stdout.write("\033[H\033[J")
    sys.stdout.write("\n".join(lines))
    sys.stdout.write("\n")
    sys.stdout.flush()


def main() -> int:
    args = parse_args()

    if not args.series:
        print(
            "[ERROR] --series is required. Example: --series now_vx or --series E0.wheel_rad",
            file=sys.stderr,
        )
        return 2

    if args.history < 10:
        print("[ERROR] --history must be >= 10", file=sys.stderr)
        return 2

    if args.refresh_hz <= 0.0:
        print("[ERROR] --refresh-hz must be > 0", file=sys.stderr)
        return 2

    line_filter = re.compile(args.line_filter) if args.line_filter else None

    series_order = args.series
    buffers: Dict[str, SeriesBuffer] = {name: SeriesBuffer(args.history) for name in series_order}

    discovered_keys: List[str] = []
    discovered_set = set()

    csv_fp = None
    csv_writer = None
    if args.csv:
        os.makedirs(os.path.dirname(os.path.abspath(args.csv)) or ".", exist_ok=True)
        csv_fp = open(args.csv, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_fp)
        csv_writer.writerow(["t_monotonic", "series", "value"])

    refresh_dt = 1.0 / args.refresh_hz
    last_render = 0.0
    eof = False

    try:
        while True:
            timeout = max(0.0, refresh_dt - (time.monotonic() - last_render))
            readable, _, _ = select.select([sys.stdin], [], [], timeout)

            if readable:
                line = sys.stdin.readline()
                if line == "":
                    eof = True
                else:
                    if (line_filter is None) or line_filter.search(line):
                        kv = line_to_kv(line)
                        if kv:
                            for k in kv.keys():
                                if k not in discovered_set:
                                    discovered_set.add(k)
                                    discovered_keys.append(k)
                                    if args.list_keys:
                                        print(f"[KEY] {k}", file=sys.stderr)

                            ts = time.monotonic()
                            for name in series_order:
                                if name in kv:
                                    v = kv[name]
                                    buffers[name].append(ts, v)
                                    if csv_writer is not None:
                                        csv_writer.writerow([f"{ts:.9f}", name, f"{v:.12g}"])

                while True:
                    more, _, _ = select.select([sys.stdin], [], [], 0)
                    if not more:
                        break
                    line = sys.stdin.readline()
                    if line == "":
                        eof = True
                        break
                    if (line_filter is None) or line_filter.search(line):
                        kv = line_to_kv(line)
                        if kv:
                            for k in kv.keys():
                                if k not in discovered_set:
                                    discovered_set.add(k)
                                    discovered_keys.append(k)
                                    if args.list_keys:
                                        print(f"[KEY] {k}", file=sys.stderr)

                            ts = time.monotonic()
                            for name in series_order:
                                if name in kv:
                                    v = kv[name]
                                    buffers[name].append(ts, v)
                                    if csv_writer is not None:
                                        csv_writer.writerow([f"{ts:.9f}", name, f"{v:.12g}"])

            now = time.monotonic()
            if now - last_render >= refresh_dt:
                w = auto_width(args.width)
                render(
                    series_order=series_order,
                    buffers=buffers,
                    discovered=discovered_keys,
                    width=w,
                    height=max(6, args.height),
                    fixed_ymin=args.y_min,
                    fixed_ymax=args.y_max,
                )
                last_render = now

            if eof:
                break

    except KeyboardInterrupt:
        pass
    finally:
        if csv_fp is not None:
            csv_fp.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
