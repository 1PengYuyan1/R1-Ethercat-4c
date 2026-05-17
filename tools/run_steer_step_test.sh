#!/usr/bin/env bash
# 4 轮并行 step 阶跃响应测试
# 用法：sudo ./tools/run_steer_step_test.sh [TARGET_DEG] [PERIOD_MS]
#
# 示例：
#   sudo ./tools/run_steer_step_test.sh          # 30°, 4000ms（默认）
#   sudo ./tools/run_steer_step_test.sh 10 4000  # 小幅试拍
#   sudo ./tools/run_steer_step_test.sh 30 6000  # 大周期看稳态
set -e

TARGET_DEG=${1:-30}
PERIOD_MS=${2:-4000}
IFNAME=${IFNAME:-enp86s0}
BIN=/home/rc/Ethercat_R1/install/linkx_soem_demo/lib/linkx_soem_demo/steer_tuning

# 网卡预检
if ! ip link show "$IFNAME" | grep -q LOWER_UP; then
    echo "[WARN] $IFNAME 不是 LOWER_UP 状态，尝试 up ..."
    ip link set "$IFNAME" up || true
    sleep 1
    ip link show "$IFNAME"
    if ! ip link show "$IFNAME" | grep -q LOWER_UP; then
        echo "[ERR] $IFNAME 仍无载波（NO-CARRIER）。请检查网线 / 从站电源。"
        exit 1
    fi
fi

# 备份旧 CSV
CSV=/home/rc/Ethercat_R1/var_data/steer_tuning.csv
if [ -f "$CSV" ]; then
    mv "$CSV" "${CSV%.csv}.$(date +%H%M%S).bak.csv"
    echo "[INFO] 旧 CSV 已备份"
fi

echo "[INFO] target=${TARGET_DEG}deg  period=${PERIOD_MS}ms  ifname=$IFNAME"
echo "[INFO] 按 space 急停 / q 退出 / l 切换日志"

exec env \
    TUNE_ALL_WHEELS=1 \
    TUNE_PROFILE=0 \
    TUNE_TARGET_DEG="$TARGET_DEG" \
    TUNE_PERIOD_MS="$PERIOD_MS" \
    "$BIN" "$IFNAME"
