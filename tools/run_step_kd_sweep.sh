#!/usr/bin/env bash
# 快速验证 kp=60 + kd=0.8 在 step 下能否抑制超调（生产候选 G）
# 同时跑一组 sine 看 ripple 是否变化
set -e
IFNAME=${IFNAME:-enp86s0}
DUR=12
COOL=8
BIN=/home/rc/Ethercat_R1/install/linkx_soem_demo/lib/linkx_soem_demo/steer_tuning
OUTDIR=/home/rc/Ethercat_R1/var_data
RAW=$OUTDIR/steer_tuning.csv

if [ "$(id -u)" != "0" ]; then echo "[ERR] need sudo"; exit 1; fi
if ! ip link show "$IFNAME" | grep -q LOWER_UP; then ip link set "$IFNAME" up || true; sleep 1; fi

run_one() {
  local label=$1; shift; local outname=$1; shift
  echo
  echo "[RUN] $label  -> $outname"
  rm -f "$RAW"
  env "$@" \
      TUNE_ALL_WHEELS=1 TUNE_OMEGA_MAX=30 TUNE_TEMP_LIMIT=70 TUNE_TEMP_HYST=5 TUNE_VEL_FF=1 \
      "$BIN" "$IFNAME" &
  local pid=$!
  sleep "$DUR"
  kill -TERM "$pid" 2>/dev/null || true
  sleep 2; kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  if [ -s "$RAW" ]; then cp "$RAW" "$OUTDIR/$outname"; echo "[OK] saved $outname"; fi
  echo "[COOL] ${COOL}s..."
  sleep "$COOL"
}

# G_step: kp=60 kd=0.8 step 30°
run_one "G_step kp=60 kd=0.8 step" "step_kp60_kd08_ff.csv" \
    TUNE_W0_POS_KP=60 TUNE_W1_POS_KP=60 TUNE_W2_POS_KP=60 TUNE_W3_POS_KP=60 \
    TUNE_W0_POS_KD=0.8 TUNE_W1_POS_KD=0.8 TUNE_W2_POS_KD=0.8 TUNE_W3_POS_KD=0.8 \
    TUNE_W0_FF=1 TUNE_W1_FF=1 TUNE_W2_FF=1 TUNE_W3_FF=1 \
    TUNE_PROFILE=0 TUNE_TARGET_DEG=30 TUNE_PERIOD_MS=4000

# G_sine: kp=60 kd=0.8 sine
run_one "G_sine kp=60 kd=0.8 sine" "sine_G_kp60_kd08_ff.csv" \
    TUNE_W0_POS_KP=60 TUNE_W1_POS_KP=60 TUNE_W2_POS_KP=60 TUNE_W3_POS_KP=60 \
    TUNE_W0_POS_KD=0.8 TUNE_W1_POS_KD=0.8 TUNE_W2_POS_KD=0.8 TUNE_W3_POS_KD=0.8 \
    TUNE_W0_FF=1 TUNE_W1_FF=1 TUNE_W2_FF=1 TUNE_W3_FF=1 \
    TUNE_PROFILE=1 TUNE_TARGET_DEG=30 TUNE_PERIOD_MS=2000

# G2_step: kp=80 kd=1.0 step（看是否能在保留高 kp 跟踪的同时压住超调）
run_one "G2_step kp=80 kd=1.0 step" "step_kp80_kd10_ff.csv" \
    TUNE_W0_POS_KP=80 TUNE_W1_POS_KP=80 TUNE_W2_POS_KP=80 TUNE_W3_POS_KP=80 \
    TUNE_W0_POS_KD=1.0 TUNE_W1_POS_KD=1.0 TUNE_W2_POS_KD=1.0 TUNE_W3_POS_KD=1.0 \
    TUNE_W0_FF=1 TUNE_W1_FF=1 TUNE_W2_FF=1 TUNE_W3_FF=1 \
    TUNE_PROFILE=0 TUNE_TARGET_DEG=30 TUNE_PERIOD_MS=4000

echo
echo "=== done ==="
