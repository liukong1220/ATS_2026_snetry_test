#!/usr/bin/env bash
set -Eeuo pipefail

# S3/O1 smoke test: run the MuJoCo LIO chain without a static PGM or prior PCD.
# This is an interface/startup test, not the full navigation or map-quality gate.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOMAIN_ID="${ROS_DOMAIN_ID:-227}"
SCENE="${SCENE:-rmuc_2025}"
RUN_ID="$(date +%Y%m%d_%H%M%S)_${DOMAIN_ID}"
LOG_DIR="${ROOT_DIR}/log/mujoco_lio_mapping/${RUN_ID}"
mkdir -p "${LOG_DIR}"

export ROS_DOMAIN_ID="${DOMAIN_ID}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1
# ROS 2 setup scripts from this workspace read COLCON_TRACE without a default.
: "${COLCON_TRACE:=}"
set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LAUNCH_ARGS=(
  ats_mujoco_sim "${SCENE}_mujoco.launch.py"
  localization_source:=lio
  slam:=true
  launch_small_gicp_relocalization:=false
  planning_grid_owner:=rog_map
  use_viewer:=false show_viewer:=false use_rviz:=false launch_mujoco_rviz:=false
  launch_terrain_analysis:=false
  enable_lidar:=true lidar_backend:=cpu lidar_downsample:=1 lidar_rate_hz:=20.0 sim_rate_hz:=400.0
  map_start_delay_sec:=1.0 rog_map_start_delay_sec:=8.0 nav_start_delay_sec:=4.0
  log_level:=info
  save_odom_layer_on_shutdown:=true
  mapping_output_prefix:="${LOG_DIR}/odom_static_layer"
)

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting MuJoCo LIO mapping smoke test"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Log: ${LOG_DIR}"
setsid ros2 launch "${LAUNCH_ARGS[@]}" >"${LOG_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

wait_topic() {
  local topic="$1" timeout_sec="${2:-45}"
  timeout "${timeout_sec}" bash -c \
    "until ros2 topic list --no-daemon | grep -qx '${topic}'; do sleep 1; done"
}

wait_node() {
  local node="$1" timeout_sec="${2:-45}"
  timeout "${timeout_sec}" bash -c \
    "until ros2 node list --no-daemon | grep -qx '${node}'; do sleep 1; done"
}

wait_node /ats_mujoco_sim
wait_node /point_lio
wait_node /ats_rog_map
wait_node /ats_rog_map_adapter
wait_topic /registered_scan
wait_topic /localization
wait_topic /rc_esdf/planning_grid
wait_topic /rog_map_adapter/ready

if ros2 node list --no-daemon | grep -qx '/static_map_publisher'; then
  echo "ERROR: static_map_publisher is active in slam mode" >&2
  exit 1
fi
if ros2 node list --no-daemon | grep -qx '/small_gicp_relocalization'; then
  echo "ERROR: small_gicp_relocalization is active in no-prior mapping mode" >&2
  exit 1
fi

timeout 45 ros2 topic echo --once --no-daemon /rc_esdf/planning_grid >"${LOG_DIR}/planning_grid.txt" || {
  echo "ERROR: no planning grid message within 45 seconds" >&2
  exit 1
}
grep -q 'frame_id: odom' "${LOG_DIR}/planning_grid.txt" || {
  echo "ERROR: planning grid is not in odom frame" >&2
  cat "${LOG_DIR}/planning_grid.txt" >&2
  exit 1
}

for _ in $(seq 1 20); do
  timeout 5 ros2 topic echo --once --no-daemon /rog_map_adapter/ready >"${LOG_DIR}/ready.txt" || true
  grep -q 'data: true' "${LOG_DIR}/ready.txt" && break
  sleep 1
done
if ! grep -q 'data: true' "${LOG_DIR}/ready.txt"; then
  echo "ERROR: adapter ready remained false; navigation is not ready" >&2
  cat "${LOG_DIR}/ready.txt" >&2 || true
  exit 1
fi

for _ in $(seq 1 10); do
  ros2 topic info --no-daemon /rc_esdf/planning_grid >"${LOG_DIR}/planning_grid_info.txt" || true
  grep -q 'Publisher count: 1' "${LOG_DIR}/planning_grid_info.txt" && break
  sleep 1
done
grep -q 'Publisher count: 1' "${LOG_DIR}/planning_grid_info.txt" || {
  echo "ERROR: planning grid does not have exactly one publisher" >&2
  cat "${LOG_DIR}/planning_grid_info.txt" >&2
  exit 1
}

# Move through a conservative four-leg odom rectangle while the LIO and
# projection chain run. This exercises continuous map updates without relying
# on a pre-existing navigation goal or static map connectivity.
drive_leg() {
  local vx="$1" vy="$2" seconds="$3"
  timeout "$seconds" ros2 topic pub -r 10 --qos-reliability best_effort \
    /cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${vx}, y: ${vy}, z: 0.0}, angular: {z: 0.0}}" \
    >"${LOG_DIR}/cmd_${vx}_${vy}.txt" 2>&1 || true
}
drive_leg 0.35 0.00 8
drive_leg 0.00 0.35 8
drive_leg -0.35 0.00 8
drive_leg 0.00 -0.35 8

for _ in $(seq 1 20); do
  timeout 5 ros2 topic echo --once --no-daemon /rog_map_adapter/generation >"${LOG_DIR}/generation.txt" || true
  grep -Eq 'data: [1-9][0-9]*' "${LOG_DIR}/generation.txt" && break
  sleep 1
done
grep -Eq 'data: [1-9][0-9]*' "${LOG_DIR}/generation.txt" || {
  echo "ERROR: map generation did not advance during motion" >&2
  cat "${LOG_DIR}/generation.txt" >&2
  exit 1
}

cleanup
for suffix in pgm yaml pcd; do
  test -s "${LOG_DIR}/odom_static_layer.${suffix}" || {
    echo "ERROR: missing saved map file ${LOG_DIR}/odom_static_layer.${suffix}" >&2
    exit 1
  }
done
grep -q 'resolution: 0.100000' "${LOG_DIR}/odom_static_layer.yaml" || exit 1
grep -q 'frame_id: odom' "${LOG_DIR}/odom_static_layer.yaml" || exit 1
grep -Eq '^POINTS [1-9]' "${LOG_DIR}/odom_static_layer.pcd" || {
  echo "ERROR: saved PCD contains no persistent occupied cells" >&2
  exit 1
}
echo "Saved map verified: ${LOG_DIR}/odom_static_layer.{pgm,yaml,pcd}"
echo "PASS: startup and save smoke checks; vehicle displacement and coverage not verified"
echo "Evidence: ${LOG_DIR}"
