#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAULT_CASE="${P4_FAULT_CASE:-odometry_stale}"
DOMAIN_ID="${ROS_DOMAIN_ID:-210}"
RESULT_FILE="${P4_RESULT_FILE:-/tmp/ats_p4_localization_${FAULT_CASE}_${DOMAIN_ID}.json}"
LAUNCH_LOG="${P4_LAUNCH_LOG:-/tmp/ats_p4_localization_${FAULT_CASE}_${DOMAIN_ID}.log}"
RELOCALIZATION_MODE="${P4_RELOCALIZATION_MODE:-real}"
if [[ "${RELOCALIZATION_MODE}" != real && "${RELOCALIZATION_MODE}" != synthetic ]]; then
  echo "P4_RELOCALIZATION_MODE must be real or synthetic" >&2
  exit 2
fi
if [[ "${RELOCALIZATION_MODE}" == real && "${FAULT_CASE}" != odometry_stale ]]; then
  echo "Mutation faults require explicit P4_RELOCALIZATION_MODE=synthetic" >&2
  exit 2
fi
LAUNCH_GICP=true
[[ "${RELOCALIZATION_MODE}" != synthetic ]] || LAUNCH_GICP=false
if [[ "${RELOCALIZATION_MODE}" == synthetic ]]; then
  echo "Synthetic-observation-assisted fixture; this does not validate real GICP recovery."
fi

case "${FAULT_CASE}" in
  odometry_stale|delayed|gicp_rejected|false_match|epoch_jump|tf_loss) ;;
  *)
    echo "Unsupported P4_FAULT_CASE=${FAULT_CASE}" >&2
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
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ats_p4_ros_logs_${DOMAIN_ID}}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-SHM}"

python3 scripts/evaluate_mujoco_localization_fault.py \
  --fault "${FAULT_CASE}" --relocalization-mode "${RELOCALIZATION_MODE}" --output "${RESULT_FILE}" &
EVALUATOR_PID=$!

setsid env --default-signal=INT ros2 launch ats_mujoco_sim rmuc_2025_mujoco.launch.py \
  planning_grid_owner:=rog_map launch_small_gicp_relocalization:="${LAUNCH_GICP}" \
  mujoco_odom_topic:=/odometry_raw fusion_odom_topic:=/odometry \
  use_viewer:=false show_viewer:=false launch_mujoco_rviz:=false \
  enable_lidar:=true lidar_backend:=cpu lidar_downsample:=24 enable_tof:=false \
  start_x:=-0.18 start_y:=0.06 start_z:=0.381 start_yaw:=0.0 \
  nav_start_delay_sec:=9.0 rog_map_start_delay_sec:=15.0 map_start_delay_sec:=2.0 \
  rviz_delay_sec:=1000.0 log_level:=warn >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "${EVALUATOR_PID}" 2>/dev/null || true
  wait "${EVALUATOR_PID}" 2>/dev/null || true
  # Launch forwards SIGINT once; group signals are only for escalation.
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
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
  echo "Localization fault evaluator timed out: ${FAULT_CASE}" >&2
  tail -n 200 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi
wait "${EVALUATOR_PID}"
cat "${RESULT_FILE}"
echo "PASS: P4 localization fault ${FAULT_CASE}; observation_mode=${RELOCALIZATION_MODE}; result=${RESULT_FILE}"
