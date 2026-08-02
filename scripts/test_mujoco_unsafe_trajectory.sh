#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAULT_CASE="${P4_UNSAFE_FAULT_CASE:-map_after_commit}"
DOMAIN_ID="${ROS_DOMAIN_ID:-201}"
RESULT_FILE="${P4_UNSAFE_RESULT_FILE:-/tmp/ats_p4_unsafe_${FAULT_CASE}_${DOMAIN_ID}.json}"
LAUNCH_LOG="${P4_UNSAFE_LAUNCH_LOG:-/tmp/ats_p4_unsafe_${FAULT_CASE}_${DOMAIN_ID}.log}"

case "${FAULT_CASE}" in
  mid_segment|pure_rotation|unknown|outside|map_after_commit|old_generation|repair_after_unsafe) ;;
  *)
    echo "Unsupported P4_UNSAFE_FAULT_CASE=${FAULT_CASE}" >&2
    exit 2
    ;;
esac
if (( DOMAIN_ID < 0 || DOMAIN_ID > 232 )); then
  echo "ROS_DOMAIN_ID must be in [0, 232]" >&2
  exit 2
fi

cd "${ROOT_DIR}"
set +u
source install/setup.bash
set -u
export ROS_DOMAIN_ID="${DOMAIN_ID}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ats_p4_unsafe_ros_logs_${DOMAIN_ID}}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-SHM}"

python3 scripts/evaluate_mujoco_unsafe_trajectory.py \
  --fault "${FAULT_CASE}" --output "${RESULT_FILE}" &
EVALUATOR_PID=$!

setsid ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py \
  use_viewer:=false show_viewer:=false launch_mujoco_rviz:=false \
  enable_lidar:=true lidar_backend:=cpu lidar_downsample:=24 enable_tof:=false \
  start_x:=-10.66 start_y:=1.47 start_z:=0.42 start_yaw:=0.0 \
  nav_start_delay_sec:=9.0 rog_map_start_delay_sec:=15.0 map_start_delay_sec:=2.0 \
  rviz_delay_sec:=1000.0 log_level:=warn >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "${EVALUATOR_PID}" 2>/dev/null || true
  wait "${EVALUATOR_PID}" 2>/dev/null || true
  kill -INT "-${LAUNCH_PID}" 2>/dev/null || true
  sleep 2
  kill -TERM "-${LAUNCH_PID}" 2>/dev/null || true
  sleep 1
  kill -KILL "-${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

deadline=$((SECONDS + 240))
while kill -0 "${EVALUATOR_PID}" 2>/dev/null && (( SECONDS < deadline )); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "MuJoCo launch exited during ${FAULT_CASE}" >&2
    tail -n 200 "${LAUNCH_LOG}" >&2 || true
    exit 1
  fi
  sleep 1
done

if kill -0 "${EVALUATOR_PID}" 2>/dev/null; then
  echo "Unsafe trajectory evaluator timed out: ${FAULT_CASE}" >&2
  tail -n 200 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi
wait "${EVALUATOR_PID}"
cat "${RESULT_FILE}"
echo "PASS: P4 unsafe trajectory fault ${FAULT_CASE}; result=${RESULT_FILE}"
