#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOMAIN_ID="${ROS_DOMAIN_ID:-221}"
RESULT_FILE="${SWERVE_RESULT_FILE:-/tmp/ats_swerve_dynamics_${DOMAIN_ID}.json}"
LAUNCH_LOG="${SWERVE_LAUNCH_LOG:-/tmp/ats_swerve_dynamics_${DOMAIN_ID}.log}"

if (( DOMAIN_ID < 0 || DOMAIN_ID > 232 )); then
  echo "ROS_DOMAIN_ID must be in [0, 232] for Fast DDS" >&2
  exit 2
fi

cd "${ROOT_DIR}"
set +u
source install/setup.bash
set -u
export ROS_DOMAIN_ID="${DOMAIN_ID}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ats_ros_logs_${DOMAIN_ID}}"
# The managed test environment blocks UDP sockets; SHM also keeps this test isolated locally.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-SHM}"

ros2 launch ats_mujoco_sim ats_mujoco_sim.launch.py \
  model_path:="${ROOT_DIR}/src/sim/ats_mujoco_sim/models/swerve_chassis.xml" \
  use_viewer:=false show_viewer:=false launch_mujoco_rviz:=false \
  enable_lidar:=false enable_tof:=false >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 3
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  cat "${LAUNCH_LOG}" >&2
  exit 1
fi

python3 scripts/evaluate_mujoco_swerve_dynamics.py --output "${RESULT_FILE}"
echo "Swerve dynamics result: ${RESULT_FILE}"
