#!/usr/bin/env python3
"""Analyze steer trace CSV — per-segment jitter (not aggregate)."""
import sys, os, numpy as np, pandas as pd
from scipy import signal

PLOT = "--plot" in sys.argv
csv_path = next((a for a in sys.argv[1:] if not a.startswith("--")), None)
if not csv_path:
    print(__doc__); sys.exit(1)

df = pd.read_csv(csv_path)
n = len(df)
ts = df.ts_ms.values
dt_ms = np.diff(ts).mean()
fs = 1000.0 / dt_ms
print(f"\n=== {csv_path} : {n} rows, {(ts[-1]-ts[0])/1000:.1f}s, fs={fs:.1f}Hz ===\n")

cmd_mag = np.sqrt(df.vx**2 + df.vy**2 + df.omega**2).values
static = cmd_mag < 0.005

# Find contiguous static segments
edges = np.diff(static.astype(int))
starts = list(np.where(edges == 1)[0] + 1)
ends   = list(np.where(edges == -1)[0] + 1)
if static[0]: starts.insert(0, 0)
if static[-1]: ends.append(n)
segs = [(s, e) for s, e in zip(starts, ends) if e - s > 300]   # ≥ 0.6s
# Drop first 250 ms of each segment (transient settling)
trim = int(0.25 * fs)
segs = [(s + trim, e) for s, e in segs if (e - s - trim) > 200]

print(f"Found {len(segs)} usable static segments (each ≥ 0.6s, trimmed first 250ms):")
for k, (s, e) in enumerate(segs):
    print(f"  seg {k}: t={ts[s]/1000:.1f}-{ts[e-1]/1000:.1f}s  ({(e-s)/fs:.1f}s)")
print()

print("=" * 95)
print("PER-SEGMENT STATIC JITTER (each segment = a 'parked' wheel position)")
print("=" * 95)
print(f"{'seg':<5} {'wheel':<6} {'mean_deg':>10} {'std_deg':>9} {'pp_deg':>9} {'dom_Hz':>8} {'pwr_dom':>9}  motor")
for k, (s, e) in enumerate(segs):
    for i in range(4):
        c = df[f"c_deg_{i}"].values[s:e]
        cd = c - c.mean()
        if len(cd) >= 64:
            f, P = signal.welch(cd, fs=fs, nperseg=min(256, len(cd)//2))
            dom_idx = 1 + np.argmax(P[1:])
            dom_hz = f[dom_idx]; pwr = P[dom_idx]
        else:
            dom_hz = 0; pwr = 0
        mq_max = np.abs(df[f"mq_{i}"].values[s:e]).max()
        mo_max = np.abs(df[f"mo_{i}"].values[s:e]).max()
        print(f"  {k:<3} E{i}  {c.mean():>10.3f} {c.std():>9.4f} {c.max()-c.min():>9.4f}"
              f" {dom_hz:>8.2f} {pwr:>9.2e}  τ≤{mq_max:.3f} ω≤{mo_max:.2f}")

print()
print("=" * 95)
print("DYNAMIC PERIODS (target changing) — tracking quality")
print("=" * 95)
dyn = ~static
if dyn.sum() > 100:
    for i in range(4):
        t = df[f"t_deg_{i}"].values[dyn]
        c = df[f"c_deg_{i}"].values[dyn]
        err = ((t - c + 180) % 360) - 180
        print(f"  E{i}  RMS_err={np.sqrt((err**2).mean()):.2f}°  "
              f"max_err={abs(err).max():.2f}°  "
              f"|τ|≤{abs(df[f'mq_{i}'].values[dyn]).max():.3f}Nm  "
              f"|ω|≤{abs(df[f'mo_{i}'].values[dyn]).max():.2f}rad/s")
else:
    print("  not enough dynamic data")

if PLOT:
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 2, figsize=(16, 11), sharex='col')
    t_s = (df.ts_ms - df.ts_ms.iloc[0]) / 1000
    # Shade static segments on time-series
    for k, (s, e) in enumerate(segs):
        for ax_row in range(4):
            axes[ax_row, 0].axvspan(t_s[s], t_s[e-1], alpha=0.12, color='green')
    for i in range(4):
        axes[i, 0].plot(t_s, df[f"t_deg_{i}"], 'b-', lw=0.5, alpha=0.8, label='target')
        axes[i, 0].plot(t_s, df[f"c_deg_{i}"], 'r-', lw=0.5, label='current')
        axes[i, 0].set_ylabel(f"E{i} deg"); axes[i, 0].grid(alpha=0.3)
        if i == 0: axes[i, 0].legend(loc='upper right', fontsize=8)
        # PSD: average across all static segments (use fixed nperseg for consistency)
        npfix = 128
        Ps = []
        for s, e in segs:
            c = df[f"c_deg_{i}"].values[s:e]
            cd = c - c.mean()
            if len(cd) >= npfix:
                f, P = signal.welch(cd, fs=fs, nperseg=npfix)
                Ps.append(P)
        if Ps:
            P_avg = np.mean(np.stack(Ps), axis=0)
            axes[i, 1].semilogy(f, P_avg)
            axes[i, 1].set_ylabel(f"E{i} PSD (avg static)")
            axes[i, 1].grid(alpha=0.3)
    axes[-1, 0].set_xlabel("time (s)  [green=static segment]")
    axes[-1, 1].set_xlabel("freq (Hz)")
    fig.suptitle(os.path.basename(csv_path))
    fig.tight_layout()
    out = csv_path.rsplit(".", 1)[0] + "_analysis.png"
    plt.savefig(out, dpi=120)
    print(f"\nplot saved: {out}")
