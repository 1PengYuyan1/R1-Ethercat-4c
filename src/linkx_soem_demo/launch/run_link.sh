#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOT'
Usage:
  ./run_link.sh [options]

Options:
  --ifname <name>       EtherCAT network interface (default: enp3s0)
  --max-speed <value>   Remote max speed (default: 1.5)
  --no-vehicle          Do not start vehicle main control executable
  --sudo                Start vehicle main control with sudo and ROS env passthrough
  -h, --help            Show this help
EOT
}

IFNAME="enp3s0"
MAX_SPEED="1.5"
START_VEHICLE="true"
USE_SUDO="false"
VEHICLE_PREFIX=""
ROS_NODES_PREFIX=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ifname)
      IFNAME="${2:-}"
      shift 2
      ;;
    --max-speed)
      MAX_SPEED="${2:-}"
      shift 2
      ;;
    --no-vehicle)
      START_VEHICLE="false"
      shift
      ;;
    --sudo)
      USE_SUDO="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "${IFNAME}" ]]; then
  echo "[ERROR] --ifname requires a non-empty value." >&2
  exit 1
fi

if [[ -z "${MAX_SPEED}" ]]; then
  echo "[ERROR] --max-speed requires a non-empty value." >&2
  exit 1
fi

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
PKG_DIR="$(cd "$(dirname "${SCRIPT_PATH}")/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"
VEHICLE_BIN="${WS_DIR}/install/linkx_soem_demo/lib/linkx_soem_demo/linkx_soem_demo"
FASTRTPS_PROFILE_FILE="${WS_DIR}/src/linkx_bringup/config/fastrtps_profiles.xml"

if [[ ! -f "${WS_DIR}/src/linkx_soem_demo/CMakeLists.txt" ]]; then
  echo "[ERROR] Workspace layout not detected at: ${WS_DIR}" >&2
  echo "Expected: ${WS_DIR}/src/linkx_soem_demo/CMakeLists.txt" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

echo "[INFO] Workspace: ${WS_DIR}"
echo "[INFO] Build: Release"
cd "${WS_DIR}"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

set +u
source "${WS_DIR}/install/setup.bash"
set -u

# 避免 Fast DDS 在当前工作目录探测默认 XML（DEFAULT_FASTRTPS_PROFILES.xml）失败报错
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  if [[ -f "${FASTRTPS_PROFILE_FILE}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_PROFILE_FILE}"
  else
    echo "[WARN] Fast DDS profile file not found: ${FASTRTPS_PROFILE_FILE}" >&2
  fi
fi

if [[ "${USE_SUDO}" == "true" ]]; then
  echo "[INFO] Validating sudo credential..."
  sudo -v
  VEHICLE_PREFIX="sudo -E env \
LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-} \
AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-} \
CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-} \
PYTHONPATH=${PYTHONPATH:-} \
PATH=${PATH} \
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-} \
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-} \
ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-} \
FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-} \
CYCLONEDDS_URI=${CYCLONEDDS_URI:-}"
  ROS_NODES_PREFIX="${VEHICLE_PREFIX}"
fi

if [[ "${START_VEHICLE}" == "true" && -z "${VEHICLE_PREFIX}" ]]; then
  if [[ "${EUID}" -ne 0 ]]; then
    caps=""
    if command -v getcap >/dev/null 2>&1 && [[ -f "${VEHICLE_BIN}" ]]; then
      caps="$(getcap "${VEHICLE_BIN}" 2>/dev/null || true)"
    fi

    if [[ "${caps}" != *cap_net_raw* || "${caps}" != *cap_net_admin* ]]; then
      echo "[ERROR] vehicle_control needs raw Ethernet permission for EtherCAT." >&2
      echo "[ERROR] Run with --sudo, for example:" >&2
      echo "        ./run_link.sh --sudo --ifname ${IFNAME}" >&2
      echo "[ERROR] Or grant capabilities to the executable after build:" >&2
      echo "        sudo setcap 'cap_net_raw,cap_net_admin+ep' ${VEHICLE_BIN}" >&2
      exit 1
    fi
  fi
fi

echo "[INFO] Launch full system..."
echo "[INFO] ifname=${IFNAME}, max_speed=${MAX_SPEED}, start_vehicle_control=${START_VEHICLE}"
if [[ -n "${VEHICLE_PREFIX}" ]]; then
  echo "[INFO] vehicle_prefix=${VEHICLE_PREFIX}"
fi

LAUNCH_ARGS=(
  "ifname:=${IFNAME}"
  "max_speed:=${MAX_SPEED}"
  "start_vehicle_control:=${START_VEHICLE}"
)

if [[ -n "${VEHICLE_PREFIX}" ]]; then
  LAUNCH_ARGS+=("vehicle_prefix:=${VEHICLE_PREFIX}")
fi
LAUNCH_ARGS+=("ros_nodes_prefix:=${ROS_NODES_PREFIX}")

ros2 launch linkx_bringup full_system.launch.py "${LAUNCH_ARGS[@]}"
