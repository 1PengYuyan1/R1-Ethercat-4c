#!/usr/bin/env bash
# 单轮电机参数标定（动摩擦/静摩擦/转动惯量）
# 必须 sudo（EtherCAT 需要原始套接字权限）
#
# 用法：
#   sudo ./tools/run_motor_calib.sh <wheel> [test] [extra args...]
#
# 例子：
#   sudo ./tools/run_motor_calib.sh 0                       # 默认全部 (friction+stiction+inertia)
#   sudo ./tools/run_motor_calib.sh 0 friction              # 只测动摩擦
#   sudo ./tools/run_motor_calib.sh 0 stiction              # 只测静摩擦
#   sudo ./tools/run_motor_calib.sh 0 inertia --inertia_t 0.5 --friction 0.05
#   sudo ./tools/run_motor_calib.sh 1 friction --omega 3.0 --measure 3.0
#
# ★ 必须在测试前把对应车轮架空（轮胎离地）★
#
set -e
IFNAME=${IFNAME:-enp86s0}
BIN=/home/rc/Ethercat_R1/install/linkx_soem_demo/lib/linkx_soem_demo/motor_calib

if [ "$(id -u)" != "0" ]; then
  echo "[ERR] 必须用 sudo 运行 (EtherCAT 需要原始套接字权限)"
  echo "  例子: sudo IFNAME=$IFNAME $0 0 friction"
  exit 1
fi

WHEEL=${1:-0}
TEST=${2:-all}
shift 2 2>/dev/null || true

# 网卡 up
if ! ip link show "$IFNAME" | grep -q LOWER_UP; then
  ip link set "$IFNAME" up || true
  for i in 1 2 3 4 5; do
    sleep 1
    ip link show "$IFNAME" | grep -q LOWER_UP && break
  done
fi
if ! ip link show "$IFNAME" | grep -q LOWER_UP; then
  echo "[ERR] $IFNAME 仍 NO-CARRIER，请检查网线/从站电源"
  exit 1
fi

cd /home/rc/Ethercat_R1
exec "$BIN" --wheel "$WHEEL" --test "$TEST" "$@"
