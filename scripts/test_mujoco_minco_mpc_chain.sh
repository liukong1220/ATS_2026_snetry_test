#!/usr/bin/env bash
set -u

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="/tmp/ats_minco_mpc_test_logs"
LAUNCH_LOG="/tmp/ats_minco_mpc_test_launch.log"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
START_X="${START_X:--10.66}"
START_Y="${START_Y:-1.47}"
START_Z="${START_Z:-0.42}"
START_YAW="${START_YAW:-0.0}"
GOAL_X="${GOAL_X:--9.0}"
GOAL_Y="${GOAL_Y:-1.47}"
GOAL_YAW_W="${GOAL_YAW_W:-1.0}"
GOAL_TIMEOUT="${GOAL_TIMEOUT:-60}"

set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID
export ROS_LOG_DIR="${LOG_DIR}"

rm -rf "${LOG_DIR}"
mkdir -p "${LOG_DIR}"
: > "${LAUNCH_LOG}"

CAPTURE_PIDS=()

cleanup() {
  for pid in "${CAPTURE_PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  done
  if [[ -n "${GOAL_PID:-}" ]]; then
    kill "${GOAL_PID}" 2>/dev/null || true
    wait "${GOAL_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT "-${LAUNCH_PID}" 2>/dev/null || true
    sleep 3
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

fail() {
  echo "FAIL: $1"
  echo "launch_log=${LAUNCH_LOG}"
  tail -n 160 "${LAUNCH_LOG}" || true
  exit 1
}

wait_for_command() {
  local label="$1"
  local timeout_sec="$2"
  shift 2
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if "$@" >/tmp/ats_minco_mpc_check.out 2>/tmp/ats_minco_mpc_check.err; then
      echo "OK: ${label}"
      return 0
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      fail "launch exited while waiting for ${label}"
    fi
    sleep 1
  done
  cat /tmp/ats_minco_mpc_check.err || true
  fail "timeout waiting for ${label}"
}

wait_for_topic_once() {
  local topic="$1"
  local timeout_sec="$2"
  wait_for_command "topic ${topic}" "${timeout_sec}" timeout 4 ros2 topic echo --once "${topic}"
}

wait_for_lifecycle_active() {
  local node="$1"
  local deadline=$((SECONDS + 90))
  while (( SECONDS < deadline )); do
    if timeout 4 ros2 lifecycle get "${node}" 2>/dev/null | grep -q "active"; then
      echo "OK: lifecycle ${node} active"
      return 0
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      fail "launch exited while waiting for lifecycle ${node}"
    fi
    sleep 1
  done
  fail "timeout waiting for lifecycle ${node}"
}

capture_pose() {
  local output_file="$1"
  timeout 8 ros2 topic echo --once /localization --field pose.pose.position \
    >"${output_file}" 2>/dev/null || return 1
  grep -q '^x:' "${output_file}" && grep -q '^y:' "${output_file}"
}

pose_axis() {
  local output_file="$1"
  local axis="$2"
  awk -v key="${axis}:" '$1 == key {print $2; exit}' "${output_file}"
}

assert_nonzero_stream() {
  local label="$1"
  local output_file="$2"
  if ! awk '
    /^[[:space:]]*(x|y|z|linear_x|linear_y|angular_z):/ {
      value = $2 + 0.0;
      if (value < 0.0) value = -value;
      if (value > 0.001) found = 1;
    }
    END {exit found ? 0 : 1}
  ' "${output_file}"; then
    fail "${label} never carried a non-zero command"
  fi
  echo "OK: ${label} carried a non-zero command"
}

LAUNCH_ARGS=(
  ats_mujoco_sim
  rmuc_2026_mujoco.launch.py
  launch_swerve_mpc:=true
  use_viewer:=false
  show_viewer:=false
  launch_mujoco_rviz:=false
  launch_nav2:=true
  launch_trajectory_optimizer:=true
  launch_twist_bridge:=true
  enable_lidar:=true
  lidar_backend:=cpu
  lidar_downsample:=64
  enable_tof:=false
  start_x:="${START_X}"
  start_y:="${START_Y}"
  start_z:="${START_Z}"
  start_yaw:="${START_YAW}"
  nav_start_delay_sec:=9.0
  map_start_delay_sec:=2.0
  rviz_delay_sec:=1000.0
  log_level:=warn
)

setsid ros2 launch "${LAUNCH_ARGS[@]}" >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

wait_for_command "node graph" 60 timeout 4 ros2 node list
wait_for_topic_once /localization 70
wait_for_topic_once /traversability_grid 120
for node in /controller_server /planner_server /bt_navigator; do
  wait_for_lifecycle_active "${node}"
done

NODE_LIST="$(ros2 node list)"
grep -q '^/minco_planner$' <<<"${NODE_LIST}" || fail "minco_planner is absent"
grep -q '^/ats_swerve_mpc$' <<<"${NODE_LIST}" || fail "ats_swerve_mpc is absent"
grep -q '^/twist_to_motion_ctrl$' <<<"${NODE_LIST}" || fail "twist bridge is absent"
if grep -q '^/fake_vel_transform$' <<<"${NODE_LIST}"; then
  fail "fake_vel_transform must be disabled in swerve MPC mode"
fi
echo "OK: MPC nodes present and fake_vel_transform absent"

TOPIC_INFO="$(ros2 topic info --verbose /cmd_vel_mpc)"
grep -q '^Publisher count: 1$' <<<"${TOPIC_INFO}" || fail "/cmd_vel_mpc publisher count is not one"
grep -q '^Subscription count: 1$' <<<"${TOPIC_INFO}" || fail "/cmd_vel_mpc subscription count is not one"
grep -q 'Node name: twist_to_motion_ctrl' <<<"${TOPIC_INFO}" || \
  fail "/cmd_vel_mpc subscriber is not twist_to_motion_ctrl"
echo "OK: /cmd_vel_mpc has one publisher and one bridge subscriber"

capture_pose /tmp/ats_minco_mpc_initial_pose.out || fail "cannot capture initial pose"
INITIAL_X="$(pose_axis /tmp/ats_minco_mpc_initial_pose.out x)"
INITIAL_Y="$(pose_axis /tmp/ats_minco_mpc_initial_pose.out y)"

declare -a ONE_SHOT_TOPICS=(
  /plan
  /minco/raw_path
  /minco/reference_path
  /ats_swerve_mpc/reference_horizon
  /ats_swerve_mpc/predicted_path
)
for topic in "${ONE_SHOT_TOPICS[@]}"; do
  output_file="/tmp/ats_minco_mpc_${topic//\//_}.out"
  timeout "${GOAL_TIMEOUT}" ros2 topic echo --once "${topic}" >"${output_file}" 2>/dev/null &
  CAPTURE_PIDS+=("$!")
done
timeout "${GOAL_TIMEOUT}" ros2 topic echo /cmd_vel_mpc \
  >/tmp/ats_minco_mpc_cmd_vel_stream.out 2>/dev/null &
CMD_STREAM_PID=$!
CAPTURE_PIDS+=("${CMD_STREAM_PID}")
timeout "${GOAL_TIMEOUT}" ros2 topic echo /motion_control \
  >/tmp/ats_minco_mpc_motion_stream.out 2>/dev/null &
MOTION_STREAM_PID=$!
CAPTURE_PIDS+=("${MOTION_STREAM_PID}")

timeout "${GOAL_TIMEOUT}" ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {w: ${GOAL_YAW_W}}}}}" \
  >/tmp/ats_minco_mpc_goal.out 2>/tmp/ats_minco_mpc_goal.err &
GOAL_PID=$!

GOAL_DEADLINE=$((SECONDS + GOAL_TIMEOUT + 5))
while kill -0 "${GOAL_PID}" 2>/dev/null && (( SECONDS < GOAL_DEADLINE )); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    fail "launch exited while navigation goal was active"
  fi
  sleep 1
done
if kill -0 "${GOAL_PID}" 2>/dev/null; then
  fail "NavigateToPose did not finish before timeout"
fi
wait "${GOAL_PID}" 2>/dev/null || true
unset GOAL_PID
cat /tmp/ats_minco_mpc_goal.out
grep -q 'Goal accepted' /tmp/ats_minco_mpc_goal.out || fail "NavigateToPose goal was not accepted"
grep -q 'Goal finished with status: SUCCEEDED' /tmp/ats_minco_mpc_goal.out || \
  fail "NavigateToPose did not succeed"
echo "OK: NavigateToPose returned SUCCEEDED"

sleep 1
kill "${CMD_STREAM_PID}" "${MOTION_STREAM_PID}" 2>/dev/null || true
wait "${CMD_STREAM_PID}" 2>/dev/null || true
wait "${MOTION_STREAM_PID}" 2>/dev/null || true

for topic in "${ONE_SHOT_TOPICS[@]}"; do
  output_file="/tmp/ats_minco_mpc_${topic//\//_}.out"
  [[ -s "${output_file}" ]] || fail "${topic} did not publish after the goal"
  echo "OK: ${topic} published"
done
assert_nonzero_stream /cmd_vel_mpc /tmp/ats_minco_mpc_cmd_vel_stream.out
assert_nonzero_stream /motion_control /tmp/ats_minco_mpc_motion_stream.out

capture_pose /tmp/ats_minco_mpc_final_pose.out || fail "cannot capture final pose"
FINAL_X="$(pose_axis /tmp/ats_minco_mpc_final_pose.out x)"
FINAL_Y="$(pose_axis /tmp/ats_minco_mpc_final_pose.out y)"
if ! awk -v ix="${INITIAL_X}" -v iy="${INITIAL_Y}" -v fx="${FINAL_X}" -v fy="${FINAL_Y}" '
  BEGIN {
    dx = fx - ix;
    dy = fy - iy;
    exit (dx * dx + dy * dy >= 0.09) ? 0 : 1;
  }
'; then
  fail "robot pose advanced by less than 0.30 m"
fi
if ! awk -v gx="${GOAL_X}" -v gy="${GOAL_Y}" -v fx="${FINAL_X}" -v fy="${FINAL_Y}" '
  BEGIN {
    dx = fx - gx;
    dy = fy - gy;
    exit (dx * dx + dy * dy <= 0.09) ? 0 : 1;
  }
'; then
  fail "final robot pose is farther than 0.30 m from the goal"
fi
echo "OK: pose advanced (${INITIAL_X}, ${INITIAL_Y}) -> (${FINAL_X}, ${FINAL_Y})"

echo "PASS: MuJoCo JPS/MINCO/clearance-aware yaw/SE2 MPC chain completed an autonomous goal."
