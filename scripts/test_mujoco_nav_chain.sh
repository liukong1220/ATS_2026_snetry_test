#!/usr/bin/env bash
set -u

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="/tmp/ats_nav_chain_test_logs"
LAUNCH_LOG="/tmp/ats_nav_chain_test_launch.log"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
MAP_CONFIG="${MAP_CONFIG:-}"
MAP_NAME="${MAP_NAME:-}"
SEED="${SEED:-7}"
START_X="${START_X:-0.0}"
START_Y="${START_Y:-0.0}"
START_Z="${START_Z:-0.18}"
START_YAW="${START_YAW:-0.0}"
GOAL_X="${GOAL_X:-2.0}"
GOAL_Y="${GOAL_Y:-0.0}"
GOAL_YAW_W="${GOAL_YAW_W:-1.0}"
GOAL_TIMEOUT="${GOAL_TIMEOUT:-20}"

set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID
export ROS_LOG_DIR="${LOG_DIR}"

rm -rf "${LOG_DIR}"
mkdir -p "${LOG_DIR}"
: > "${LAUNCH_LOG}"

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT "-${LAUNCH_PID}" 2>/dev/null || true
    sleep 3
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

LAUNCH_ARGS=(
  ats_mujoco_sim
  mujoco_navigation.launch.py
  use_viewer:=false
  show_viewer:=false
  use_rviz:=false
  launch_nav2:=true
  launch_twist_bridge:=true
  enable_lidar:=true
  lidar_backend:=cpu
  lidar_downsample:=64
  enable_tof:=false
  seed:="${SEED}"
  start_x:="${START_X}"
  start_y:="${START_Y}"
  start_z:="${START_Z}"
  start_yaw:="${START_YAW}"
  nav_start_delay_sec:=9.0
  map_start_delay_sec:=2.0
  rviz_delay_sec:=1000.0
  log_level:=warn
)
if [[ -n "${MAP_CONFIG}" ]]; then
  LAUNCH_ARGS+=(map_config:="${MAP_CONFIG}")
fi
if [[ -n "${MAP_NAME}" ]]; then
  LAUNCH_ARGS+=(map_name:="${MAP_NAME}")
fi

setsid ros2 launch "${LAUNCH_ARGS[@]}" > "${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

fail() {
  echo "FAIL: $1"
  echo "launch_log=${LAUNCH_LOG}"
  tail -n 120 "${LAUNCH_LOG}" || true
  exit 1
}

wait_for_command() {
  local label="$1"
  local timeout_sec="$2"
  shift 2
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if "$@" >/tmp/ats_nav_chain_check.out 2>/tmp/ats_nav_chain_check.err; then
      echo "OK: ${label}"
      cat /tmp/ats_nav_chain_check.out
      return 0
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      fail "launch exited while waiting for ${label}"
    fi
    sleep 1
  done
  echo "last stderr for ${label}:"
  cat /tmp/ats_nav_chain_check.err || true
  fail "timeout waiting for ${label}"
}

wait_for_topic_once() {
  local topic="$1"
  local timeout_sec="$2"
  wait_for_command "topic ${topic}" "${timeout_sec}" timeout 4 ros2 topic echo --once "${topic}"
}

wait_for_tf() {
  local parent="$1"
  local child="$2"
  local timeout_sec="$3"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    timeout 4 ros2 run tf2_ros tf2_echo "${parent}" "${child}" \
      >/tmp/ats_nav_chain_tf.out 2>/tmp/ats_nav_chain_tf.err || true
    if grep -Eq "At time|Transform|Translation" /tmp/ats_nav_chain_tf.out; then
      echo "OK: TF ${parent} -> ${child}"
      sed -n '1,12p' /tmp/ats_nav_chain_tf.out
      return 0
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      fail "launch exited while waiting for TF ${parent} -> ${child}"
    fi
    sleep 1
  done
  echo "last stderr for TF ${parent} -> ${child}:"
  cat /tmp/ats_nav_chain_tf.err || true
  fail "timeout waiting for TF ${parent} -> ${child}"
}

wait_for_lifecycle_active() {
  local node="$1"
  local deadline=$((SECONDS + 90))
  while (( SECONDS < deadline )); do
    if ros2 lifecycle get "${node}" 2>/tmp/ats_nav_chain_lifecycle.err | tee /tmp/ats_nav_chain_lifecycle.out | grep -q "active"; then
      echo "OK: lifecycle ${node} active"
      return 0
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      fail "launch exited while waiting for lifecycle ${node}"
    fi
    sleep 1
  done
  cat /tmp/ats_nav_chain_lifecycle.err || true
  fail "timeout waiting for lifecycle ${node}"
}

wait_for_command "node graph" 60 timeout 4 ros2 node list
wait_for_topic_once /localization 70
wait_for_tf odom gimbal_yaw_odom 70
wait_for_tf gimbal_yaw_odom front_mid360 70
wait_for_topic_once /local_pointcloud 90
wait_for_topic_once /registered_scan 90
wait_for_topic_once /terrain_map 120
wait_for_topic_once /terrain_map_ext 120
wait_for_topic_once /traversability_grid 120
wait_for_topic_once /traversability_slope_grid 120

for node in /controller_server /planner_server /behavior_server /bt_navigator /velocity_smoother; do
  wait_for_lifecycle_active "${node}"
done

timeout "${GOAL_TIMEOUT}" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {w: ${GOAL_YAW_W}}}}}" \
  >/tmp/ats_nav_chain_goal.out 2>/tmp/ats_nav_chain_goal.err || true
cat /tmp/ats_nav_chain_goal.out
if ! grep -q "Goal accepted" /tmp/ats_nav_chain_goal.out; then
  cat /tmp/ats_nav_chain_goal.err || true
  fail "NavigateToPose goal was not accepted"
fi

wait_for_topic_once /cmd_vel_nav2_result 30
wait_for_topic_once /motion_control 30

echo "PASS: MuJoCo navigation chain is active, publishing sensor data, accepting Nav2 goals, and forwarding motion commands."
