#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

IFNAME="enp86s0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ifname)
      IFNAME="${2:-}"
      shift 2
      ;;
    -h|--help)
      echo "Usage: $0 [--ifname <name>]"
      echo "  --ifname <name>   EtherCAT NIC name (default: enp86s0)"
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

# var_data 目录: C++ 程序写 live_variables.log / steer_trace CSV / manual_correction CSV
VAR_DATA_DIR="${WS_DIR}/var_data"
mkdir -p "${VAR_DATA_DIR}"
export VAR_DATA_DIR
export VAR_DATA_FILE="${VAR_DATA_DIR}/live_variables.log"
: > "${VAR_DATA_FILE}"
export STEER_TRACE_FILE="${VAR_DATA_DIR}/steer_trace_$(date +%Y%m%d_%H%M%S).csv"
echo "[INFO] var_data: ${VAR_DATA_DIR}"

# ROS 2 source
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[ERROR] ROS 2 Humble not found: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
set +u; source /opt/ros/humble/setup.bash; set -u

# 网卡 UP (EtherCAT 必需)
if ! ip link show "${IFNAME}" >/dev/null 2>&1; then
  echo "[ERROR] Interface not found: ${IFNAME}" >&2
  ip -br link >&2
  exit 1
fi
if [[ "$(ip -br link show "${IFNAME}" | awk '{print $2}')" != "UP" ]]; then
  echo "[INFO] Bringing ${IFNAME} UP..."
  sudo ip link set "${IFNAME}" up
fi
echo "[INFO] EtherCAT ifname=${IFNAME}"

echo "[1/3] Building workspace..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[2/3] Sourcing workspace..."
set +u; source install/setup.bash; set -u

# EtherCAT 二进制需要 cap_net_raw + cap_net_admin 才能开 raw socket
# setcap 不能操作 symlink, 解析到真实路径 (build/.../linkx_soem_demo)
VEHICLE_BIN="${WS_DIR}/install/linkx_soem_demo/lib/linkx_soem_demo/linkx_soem_demo"
if [[ ! -e "${VEHICLE_BIN}" ]]; then
  echo "[ERROR] Vehicle binary not found: ${VEHICLE_BIN}" >&2
  exit 1
fi
REAL_BIN="$(readlink -f "${VEHICLE_BIN}")"
CAPS="$(getcap "${REAL_BIN}" 2>/dev/null || true)"
if [[ "${CAPS}" != *cap_net_raw* || "${CAPS}" != *cap_net_admin* ]]; then
  echo "[INFO] Granting cap_net_raw,cap_net_admin to ${REAL_BIN}..."
  sudo -n setcap cap_net_raw,cap_net_admin=eip "${REAL_BIN}" 2>/dev/null \
    || sudo setcap cap_net_raw,cap_net_admin=eip "${REAL_BIN}"
  CAPS="$(getcap "${REAL_BIN}" 2>/dev/null || true)"
  if [[ "${CAPS}" != *cap_net_raw* ]]; then
    echo "[ERROR] Failed to set capabilities on ${REAL_BIN}" >&2
    exit 1
  fi
fi

echo "[3/3] Launching full system..."
exec ros2 launch linkx_bringup full_system.launch.py "ifname:=${IFNAME}"
