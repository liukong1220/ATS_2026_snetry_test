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
TEST_PROFILE="${TEST_PROFILE:-single}"
MIN_LEG_PROGRESS="${MIN_LEG_PROGRESS:-0.20}"

case "${TEST_PROFILE}" in
  single)
    GOAL_NAMES=(single)
    GOAL_XS=("${GOAL_X}")
    GOAL_YS=("${GOAL_Y}")
    ;;
  rectangle)
    # Stage into the map's east-side free area, then close a 0.68 m x 0.49 m loop.
    # Keeping yaw at zero makes the north/south legs exercise true lateral motion.
    GOAL_NAMES=(stage east south west north)
    GOAL_XS=(-9.50 -8.82 -8.82 -9.50 -9.50)
    GOAL_YS=(1.47 1.47 0.98 0.98 1.47)
    ;;
  red_box)
    # Screenshot red box center converted from rmuc_2026.pgm/yaml:
    # pixel=(510,425) -> map=(-0.043,-4.082). Stage through the existing
    # east-side free area first, then run the 10 m-class route to the target.
    GOAL_NAMES=(stage_red_box red_box)
    GOAL_XS=(-8.88 -0.04)
    GOAL_YS=(1.47 -4.08)
    ;;
  *)
    echo "Unsupported TEST_PROFILE='${TEST_PROFILE}'; use 'single', 'rectangle', or 'red_box'."
    exit 2
    ;;
esac

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

assert_lateral_stream() {
  local label="$1"
  local output_file="$2"
  if ! awk '
    $1 == "linear:" {in_linear = 1; next}
    $1 == "angular:" {in_linear = 0; next}
    in_linear && $1 == "y:" {
      value = $2 + 0.0;
      if (value < 0.0) value = -value;
      if (value > 0.02) found = 1;
    }
    END {exit found ? 0 : 1}
  ' "${output_file}"; then
    fail "${label} did not produce a lateral MPC command"
  fi
  echo "OK: ${label} produced a lateral MPC command"
}

assert_pose_progress() {
  local label="$1"
  local before_file="$2"
  local after_file="$3"
  local before_x before_y after_x after_y
  before_x="$(pose_axis "${before_file}" x)"
  before_y="$(pose_axis "${before_file}" y)"
  after_x="$(pose_axis "${after_file}" x)"
  after_y="$(pose_axis "${after_file}" y)"
  if ! awk \
    -v ix="${before_x}" -v iy="${before_y}" -v fx="${after_x}" -v fy="${after_y}" \
    -v minimum="${MIN_LEG_PROGRESS}" '
      BEGIN {
        dx = fx - ix;
        dy = fy - iy;
        exit (dx * dx + dy * dy >= minimum * minimum) ? 0 : 1;
      }
    '; then
    fail "${label} advanced by less than ${MIN_LEG_PROGRESS} m"
  fi
  echo "OK: ${label} pose advanced (${before_x}, ${before_y}) -> (${after_x}, ${after_y})"
}

assert_pose_near_goal() {
  local label="$1"
  local pose_file="$2"
  local goal_x="$3"
  local goal_y="$4"
  local final_x final_y
  final_x="$(pose_axis "${pose_file}" x)"
  final_y="$(pose_axis "${pose_file}" y)"
  if ! awk -v gx="${goal_x}" -v gy="${goal_y}" -v fx="${final_x}" -v fy="${final_y}" '
    BEGIN {
      dx = fx - gx;
      dy = fy - gy;
      exit (dx * dx + dy * dy <= 0.09) ? 0 : 1;
    }
  '; then
    fail "${label} final pose is farther than 0.30 m from its goal"
  fi
}

wait_for_capture() {
  local label="$1"
  local pid="$2"
  local output_file="$3"
  local deadline=$((SECONDS + 10))
  while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 0.2
  done
  if kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
    fail "timeout capturing ${label}"
  fi
  wait "${pid}" 2>/dev/null || true
  [[ -s "${output_file}" ]] || fail "${label} did not publish after the goal"
  echo "OK: ${label} published"
}

run_navigation_goal() {
  local index="$1"
  local name="${GOAL_NAMES[index]}"
  local goal_x="${GOAL_XS[index]}"
  local goal_y="${GOAL_YS[index]}"
  local prefix="/tmp/ats_minco_mpc_${TEST_PROFILE}_${index}_${name}"
  local before_pose="${prefix}_before_pose.out"
  local after_pose="${prefix}_after_pose.out"
  local goal_output="${prefix}_goal.out"
  local goal_error="${prefix}_goal.err"
  local command_output="${prefix}_cmd_vel.out"
  local topic output_file pid
  local -a topic_pids=()

  capture_pose "${before_pose}" || fail "cannot capture pose before ${name}"
  for topic in /plan /minco/raw_path /minco/reference_path; do
    output_file="${prefix}_${topic//\//_}.out"
    timeout "${GOAL_TIMEOUT}" ros2 topic echo --once "${topic}" >"${output_file}" 2>/dev/null &
    pid=$!
    CAPTURE_PIDS+=("${pid}")
    topic_pids+=("${pid}:${topic}:${output_file}")
  done
  timeout "${GOAL_TIMEOUT}" ros2 topic echo /cmd_vel_mpc >"${command_output}" 2>/dev/null &
  local leg_command_pid=$!
  CAPTURE_PIDS+=("${leg_command_pid}")

  # The MINCO paths are event-driven. Let the one-shot subscriptions complete
  # DDS discovery before the goal causes all three paths to publish in a burst.
  sleep 1

  timeout "${GOAL_TIMEOUT}" ros2 action send_goal /navigate_to_pose \
    nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: 0.0}, orientation: {w: ${GOAL_YAW_W}}}}}" \
    >"${goal_output}" 2>"${goal_error}" &
  GOAL_PID=$!

  local goal_deadline=$((SECONDS + GOAL_TIMEOUT + 5))
  while kill -0 "${GOAL_PID}" 2>/dev/null && (( SECONDS < goal_deadline )); do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      fail "launch exited while ${name} goal was active"
    fi
    sleep 1
  done
  if kill -0 "${GOAL_PID}" 2>/dev/null; then
    fail "${name} NavigateToPose did not finish before timeout"
  fi
  wait "${GOAL_PID}" 2>/dev/null || true
  unset GOAL_PID
  cat "${goal_output}"
  grep -q 'Goal accepted' "${goal_output}" || fail "${name} goal was not accepted"
  grep -q 'Goal finished with status: SUCCEEDED' "${goal_output}" || \
    fail "${name} NavigateToPose did not succeed"
  echo "OK: ${name} NavigateToPose returned SUCCEEDED"

  for capture in "${topic_pids[@]}"; do
    pid="${capture%%:*}"
    capture="${capture#*:}"
    topic="${capture%%:*}"
    output_file="${capture#*:}"
    wait_for_capture "${name} ${topic}" "${pid}" "${output_file}"
  done

  sleep 0.5
  kill "${leg_command_pid}" 2>/dev/null || true
  wait "${leg_command_pid}" 2>/dev/null || true
  if [[ "${name}" == "south" || "${name}" == "north" ]]; then
    assert_lateral_stream "${name} leg" "${command_output}"
  fi

  capture_pose "${after_pose}" || fail "cannot capture pose after ${name}"
  assert_pose_progress "${name} leg" "${before_pose}" "${after_pose}"
  assert_pose_near_goal "${name} leg" "${after_pose}" "${goal_x}" "${goal_y}"
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

declare -a DEBUG_TOPICS=(
  /ats_swerve_mpc/reference_horizon
  /ats_swerve_mpc/predicted_path
)
for topic in "${DEBUG_TOPICS[@]}"; do
  output_file="/tmp/ats_minco_mpc_${topic//\//_}.out"
  timeout "$((GOAL_TIMEOUT * ${#GOAL_NAMES[@]}))" ros2 topic echo --once "${topic}" \
    >"${output_file}" 2>/dev/null &
  CAPTURE_PIDS+=("$!")
done
timeout "$((GOAL_TIMEOUT * ${#GOAL_NAMES[@]}))" ros2 topic echo /cmd_vel_mpc \
  >/tmp/ats_minco_mpc_cmd_vel_stream.out 2>/dev/null &
CMD_STREAM_PID=$!
CAPTURE_PIDS+=("${CMD_STREAM_PID}")
timeout "$((GOAL_TIMEOUT * ${#GOAL_NAMES[@]}))" ros2 topic echo /motion_control \
  >/tmp/ats_minco_mpc_motion_stream.out 2>/dev/null &
MOTION_STREAM_PID=$!
CAPTURE_PIDS+=("${MOTION_STREAM_PID}")

for index in "${!GOAL_NAMES[@]}"; do
  echo "RUN: ${TEST_PROFILE} goal $((index + 1))/${#GOAL_NAMES[@]} '${GOAL_NAMES[index]}' -> " \
    "(${GOAL_XS[index]}, ${GOAL_YS[index]})"
  run_navigation_goal "${index}"
done

sleep 1
kill "${CMD_STREAM_PID}" "${MOTION_STREAM_PID}" 2>/dev/null || true
wait "${CMD_STREAM_PID}" 2>/dev/null || true
wait "${MOTION_STREAM_PID}" 2>/dev/null || true

for topic in "${DEBUG_TOPICS[@]}"; do
  output_file="/tmp/ats_minco_mpc_${topic//\//_}.out"
  [[ -s "${output_file}" ]] || fail "${topic} did not publish after the goal"
  echo "OK: ${topic} published"
done
assert_nonzero_stream /cmd_vel_mpc /tmp/ats_minco_mpc_cmd_vel_stream.out
assert_nonzero_stream /motion_control /tmp/ats_minco_mpc_motion_stream.out

echo "PASS: MuJoCo JPS/MINCO/clearance-aware yaw/SE2 MPC '${TEST_PROFILE}' profile completed."
