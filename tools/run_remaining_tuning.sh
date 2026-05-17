#!/usr/bin/env bash
# 续跑剩余 3 组测试 (E / step / high-freq sine)
# 全部带温度保护 (TEMP_LIMIT=70°C)、舵向速度限 (OMEGA_MAX=30)、FF 打开
#
# 用法（必须 sudo）：
#   sudo IFNAME=enp86s0 ./tools/run_remaining_tuning.sh [duration_sec]
#
# 默认每组测 12 秒（约 6 个正弦周期），中间冷却 8 秒。
# 任意一组温度 >70°C 会自动暂停并 hold 0°；脚本仍会等到时间到再切换。
#
set -e
IFNAME=${IFNAME:-enp86s0}
DUR=${1:-12}
COOL=${COOL:-8}
BIN=/home/rc/Ethercat_R1/install/linkx_soem_demo/lib/linkx_soem_demo/steer_tuning
OUTDIR=/home/rc/Ethercat_R1/var_data
RAW=$OUTDIR/steer_tuning.csv

if [ "$(id -u)" != "0" ]; then
  echo "[ERR] 必须用 sudo 运行 (EtherCAT 需要原始套接字权限)"
  exit 1
fi

# 网卡 up
if ! ip link show "$IFNAME" | grep -q LOWER_UP; then
  ip link set "$IFNAME" up || true
  sleep 1
fi
if ! ip link show "$IFNAME" | grep -q LOWER_UP; then
  echo "[ERR] $IFNAME 仍 NO-CARRIER，请检查网线/从站电源"
  exit 1
fi

run_one() {
  local label=$1; shift
  local outname=$1; shift
  echo
  echo "=========================================="
  echo "[RUN] $label  -> $outname  (duration=${DUR}s)"
  echo "=========================================="
  rm -f "$RAW"

  # 用临时 env 启动后台进程，定时杀掉
  env "$@" \
      TUNE_ALL_WHEELS=1 \
      TUNE_OMEGA_MAX=30 \
      TUNE_TEMP_LIMIT=70 \
      TUNE_TEMP_HYST=5 \
      TUNE_VEL_FF=1 \
      "$BIN" "$IFNAME" &
  local pid=$!
  sleep "$DUR"
  # 优雅退出（程序里 q 等 stdin 输入；这里直接 SIGTERM）
  kill -TERM "$pid" 2>/dev/null || true
  sleep 2
  kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true

  if [ -s "$RAW" ]; then
    cp "$RAW" "$OUTDIR/$outname"
    echo "[OK] saved $OUTDIR/$outname  ($(wc -l < "$OUTDIR/$outname") lines)"
  else
    echo "[WARN] $RAW 为空，未保存 $outname"
  fi

  echo "[COOL] cooling ${COOL}s ..."
  sleep "$COOL"
}

# E: kp=100 sine, period=2000ms (vs B/C/D 同节奏)
run_one "E: kp=100 sine FF=1 OMEGA=30" "sine_E_kp100_ff.csv" \
    TUNE_W0_POS_KP=100 TUNE_W1_POS_KP=100 TUNE_W2_POS_KP=100 TUNE_W3_POS_KP=100 \
    TUNE_W0_POS_KD=0.5 TUNE_W1_POS_KD=0.5 TUNE_W2_POS_KD=0.5 TUNE_W3_POS_KD=0.5 \
    TUNE_W0_FF=1 TUNE_W1_FF=1 TUNE_W2_FF=1 TUNE_W3_FF=1 \
    TUNE_PROFILE=1 TUNE_TARGET_DEG=30 TUNE_PERIOD_MS=2000

# step at kp=80（profile=0，方波 step）
run_one "step kp=80 FF=1 OMEGA=30" "step_kp80_ff.csv" \
    TUNE_W0_POS_KP=80 TUNE_W1_POS_KP=80 TUNE_W2_POS_KP=80 TUNE_W3_POS_KP=80 \
    TUNE_W0_POS_KD=0.5 TUNE_W1_POS_KD=0.5 TUNE_W2_POS_KD=0.5 TUNE_W3_POS_KD=0.5 \
    TUNE_W0_FF=1 TUNE_W1_FF=1 TUNE_W2_FF=1 TUNE_W3_FF=1 \
    TUNE_PROFILE=0 TUNE_TARGET_DEG=30 TUNE_PERIOD_MS=4000

# 高频 sine：period=750ms
run_one "high-freq sine kp=80 period=750ms FF=1 OMEGA=30" "sine_F_kp80_per750_ff.csv" \
    TUNE_W0_POS_KP=80 TUNE_W1_POS_KP=80 TUNE_W2_POS_KP=80 TUNE_W3_POS_KP=80 \
    TUNE_W0_POS_KD=0.5 TUNE_W1_POS_KD=0.5 TUNE_W2_POS_KD=0.5 TUNE_W3_POS_KD=0.5 \
    TUNE_W0_FF=1 TUNE_W1_FF=1 TUNE_W2_FF=1 TUNE_W3_FF=1 \
    TUNE_PROFILE=1 TUNE_TARGET_DEG=20 TUNE_PERIOD_MS=750

echo
echo "=== 全部完成。下一步分析: ==="
echo "python3 $OUTDIR/compare_sine.py \\"
echo "  $OUTDIR/sine_B_with_ff.csv $OUTDIR/sine_C_kp60_ff.csv \\"
echo "  $OUTDIR/sine_D_kp80_ff.csv $OUTDIR/sine_E_kp100_ff.csv \\"
echo "  $OUTDIR/sine_F_kp80_per750_ff.csv"
