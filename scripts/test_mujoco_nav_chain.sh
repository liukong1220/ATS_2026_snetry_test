#!/usr/bin/env bash
set -u

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="/tmp/ats_nav_chain_test_logs"
LAUNCH_LOG="/tmp/ats_nav_chain_test_launch.log"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
START_X="${START_X:--10.66}"
START_Y="${START_Y:-1.47}"
START_Z="${START_Z:-0.42}"
START_YAW="${START_YAW:-0.0}"
GOAL_X="${GOAL_X:--9.0}"
GOAL_Y="${GOAL_Y:-1.47}"
GOAL_YAW_W="${GOAL_YAW_W:-1.0}"
GOAL_TIMEOUT="${GOAL_TIMEOUT:-20}"
VERIFY_EVENT_DRIVEN_REPLAN="${VERIFY_EVENT_DRIVEN_REPLAN:-true}"
PLANNING_GRID_OWNER="${PLANNING_GRID_OWNER:-rc_esdf}"
LAUNCH_ROG_MAP="${LAUNCH_ROG_MAP:-false}"
ROG_MAP_CONFIG_FILE="${ROG_MAP_CONFIG_FILE:-${WORKSPACE_DIR}/src/ats_sentry_nav/ats_rog_map/config/rog_map_ground_planning_mujoco.yaml}"

case "${PLANNING_GRID_OWNER}" in
  rc_esdf|rog_map) ;;
  *)
    echo "Unsupported PLANNING_GRID_OWNER='${PLANNING_GRID_OWNER}'; use 'rc_esdf' or 'rog_map'."
    exit 2
    ;;
esac
if [[ "${PLANNING_GRID_OWNER}" == "rog_map" || "${LAUNCH_ROG_MAP,,}" == "true" ]]; then
  [[ -r "${ROG_MAP_CONFIG_FILE}" ]] || {
    echo "ROGMap config is not readable: ${ROG_MAP_CONFIG_FILE}"
    exit 2
  }
fi

set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID
export ROS_LOG_DIR="${LOG_DIR}"

rm -rf "${LOG_DIR}"
mkdir -p "${LOG_DIR}"
: > "${LAUNCH_LOG}"

cleanup() {
  if [[ -n "${ELASTIC_PATH_PID:-}" ]]; then
    kill "${ELASTIC_PATH_PID}" 2>/dev/null || true
    wait "${ELASTIC_PATH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT "-${LAUNCH_PID}" 2>/dev/null || true
    sleep 3
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || true
    sleep 1
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

LAUNCH_ARGS=(
  ats_mujoco_sim
  rmuc_2026_mujoco.launch.py
  use_viewer:=false
  show_viewer:=false
  launch_mujoco_rviz:=false
  launch_nav2:=true
  launch_trajectory_optimizer:=true
  launch_twist_bridge:=true
  launch_rog_map:="${LAUNCH_ROG_MAP}"
  planning_grid_owner:="${PLANNING_GRID_OWNER}"
  rog_map_config_file:="${ROG_MAP_CONFIG_FILE}"
  enable_lidar:=true
  lidar_backend:=cpu
  lidar_downsample:=64
  enable_tof:=false
  start_x:="${START_X}"
  start_y:="${START_Y}"
  start_z:="${START_Z}"
  start_yaw:="${START_YAW}"
  nav_start_delay_sec:=9.0
  rog_map_start_delay_sec:=15.0
  map_start_delay_sec:=2.0
  rviz_delay_sec:=1000.0
  log_level:=warn
)

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
      sed -n '1,24p' /tmp/ats_nav_chain_check.out
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

read_topic_field() {
  local topic="$1"
  local field="$2"
  timeout 5 ros2 topic echo --once "${topic}" --field "${field}" 2>/dev/null | awk '
    $1 ~ /^[[:alnum:]_]+:$/ && NF >= 2 {print $2; exit}
    NF == 1 && $1 != "---" {print $1; exit}
  '
}

topic_field_equals() {
  local topic="$1"
  local field="$2"
  local expected="$3"
  local value
  value="$(read_topic_field "${topic}" "${field}")"
  [[ "${value,,}" == "${expected,,}" ]]
}

topic_field_positive() {
  local topic="$1"
  local field="$2"
  local value
  value="$(read_topic_field "${topic}" "${field}")"
  [[ "${value}" =~ ^[0-9]+$ ]] && awk -v value="${value}" 'BEGIN {exit value > 0 ? 0 : 1}'
}

path_capture_has_poses() {
  local output_file="$1"
  grep -q '^poses:$' "${output_file}" && grep -q '^- header:$' "${output_file}"
}

wait_for_generation_advance() {
  local baseline="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))
  local current
  while (( SECONDS < deadline )); do
    current="$(read_topic_field /rog_map_adapter/generation data)"
    if [[ "${current}" =~ ^[0-9]+$ ]] &&
      awk -v current="${current}" -v baseline="${baseline}" \
        'BEGIN {exit current > baseline ? 0 : 1}'
    then
      echo "OK: ROGMap adapter generation advanced ${baseline} -> ${current}"
      return 0
    fi
    sleep 1
  done
  fail "ROGMap adapter generation did not advance beyond ${baseline}"
}

assert_single_publisher_owner() {
  local topic="$1"
  local expected_node="$2"
  local topic_info publisher_block
  topic_info="$(ros2 topic info --verbose "${topic}")"
  grep -q '^Publisher count: 1$' <<<"${topic_info}" || \
    fail "${topic} publisher count is not one"
  publisher_block="$(sed -n '/^Publisher count:/,/^Subscription count:/p' <<<"${topic_info}")"
  grep -q "Node name: ${expected_node}$" <<<"${publisher_block}" || \
    fail "${topic} publisher is not ${expected_node}"
  echo "OK: ${topic} has one publisher owned by ${expected_node}"
}

verify_rog_map_planning_interface() {
  local topic node_info generation
  for topic in /rog_map/occ /rog_map/inf_occ /rog_map/unk /rog_map/esdf; do
    wait_for_command "non-empty ${topic}" 120 topic_field_positive "${topic}" width
  done
  wait_for_command "ROGMap input fresh" 30 topic_field_equals /rog_map/stale data false
  wait_for_command "ROGMap adapter ready" 120 \
    topic_field_equals /rog_map_adapter/ready data true
  wait_for_command "positive ROGMap adapter generation" 30 \
    topic_field_positive /rog_map_adapter/generation data
  wait_for_command "non-empty planning grid width" 30 \
    topic_field_positive /rc_esdf/planning_grid info.width
  wait_for_command "non-empty planning grid height" 30 \
    topic_field_positive /rc_esdf/planning_grid info.height

  assert_single_publisher_owner /rc_esdf/planning_grid ats_rog_map_adapter
  node_info="$(ros2 node info /ats_rog_map_adapter)"
  if grep -q '/rog_map/esdf' <<<"${node_info}"; then
    fail "ats_rog_map_adapter must not subscribe to the ROGMap visualization ESDF cloud"
  fi
  grep -q '/rog_map/get_ground_projection' <<<"${node_info}" || \
    fail "ats_rog_map_adapter does not expose the numeric projection service client"
  echo "OK: adapter consumes the numeric projection service without /rog_map/esdf subscription"

  generation="$(read_topic_field /rog_map_adapter/generation data)"
  [[ "${generation}" =~ ^[0-9]+$ ]] || fail "cannot read ROGMap adapter generation"
  wait_for_generation_advance "${generation}" 30
}

assert_plan_is_event_driven() {
  local observation_sec="$1"
  local output_file="/tmp/ats_nav_chain_plan_observation.out"

  timeout "${observation_sec}" ros2 topic echo /plan >"${output_file}" 2>/dev/null || true
  local message_count
  message_count="$(grep -c '^header:$' "${output_file}" || true)"
  message_count="${message_count:-0}"
  if (( message_count > 1 )); then
    echo "observed /plan messages: ${message_count}"
    cat "${output_file}" || true
    fail "global planner is still publishing periodically in the static test"
  fi
  echo "OK: event-driven global plan observation (${message_count} message in ${observation_sec}s)"
}

assert_controller_never_received_empty_path() {
  if grep -q "Resulting plan has 0 poses" "${LAUNCH_LOG}"; then
    fail "controller_server received an empty FollowPath goal"
  fi
  echo "OK: controller never received an empty FollowPath goal"
}

assert_controller_never_aborted_follow_path() {
  if grep -q "\[follow_path\] \[ActionServer\] Aborting handle\." "${LAUNCH_LOG}"; then
    fail "controller_server aborted FollowPath; a recovery completed the goal instead of a clean track"
  fi
  echo "OK: controller completed FollowPath without aborting"
}

assert_navigation_succeeded() {
  if ! grep -q "Goal finished with status: SUCCEEDED" /tmp/ats_nav_chain_goal.out; then
    cat /tmp/ats_nav_chain_goal.err || true
    fail "NavigateToPose did not finish successfully before the timeout"
  fi
  echo "OK: NavigateToPose returned SUCCEEDED"
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
    if timeout 4 ros2 lifecycle get "${node}" 2>/tmp/ats_nav_chain_lifecycle.err | \
      tee /tmp/ats_nav_chain_lifecycle.out | grep -q "active"; then
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
wait_for_topic_once /rc_esdf/planning_grid 120
wait_for_topic_once /rc_esdf/signed_distance_grid 120
wait_for_topic_once /rc_esdf/footprint_clearance_grid 120

if [[ "${PLANNING_GRID_OWNER}" == "rog_map" ]]; then
  verify_rog_map_planning_interface
fi

for node in /controller_server /planner_server /behavior_server /bt_navigator /velocity_smoother; do
  wait_for_lifecycle_active "${node}"
done

ELASTIC_PATH_OUTPUT=/tmp/ats_nav_chain_local_elastic_path.out
: >"${ELASTIC_PATH_OUTPUT}"
timeout "$((GOAL_TIMEOUT + 30))" ros2 topic echo --once \
  --qos-reliability reliable --qos-durability transient_local \
  /local_elastic_path >"${ELASTIC_PATH_OUTPUT}" 2>/tmp/ats_nav_chain_local_elastic_path.err &
ELASTIC_PATH_PID=$!
sleep 2

timeout "${GOAL_TIMEOUT}" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {w: ${GOAL_YAW_W}}}}}" \
  >/tmp/ats_nav_chain_goal.out 2>/tmp/ats_nav_chain_goal.err || true
cat /tmp/ats_nav_chain_goal.out
if ! grep -q "Goal accepted" /tmp/ats_nav_chain_goal.out; then
  cat /tmp/ats_nav_chain_goal.err || true
  fail "NavigateToPose goal was not accepted"
fi
assert_navigation_succeeded

wait_for_topic_once /cmd_vel_nav2_result 30
wait_for_topic_once /motion_control 30
wait_for_command "non-empty topic /local_elastic_path" 30 \
  path_capture_has_poses "${ELASTIC_PATH_OUTPUT}"
wait "${ELASTIC_PATH_PID}" 2>/dev/null || true
unset ELASTIC_PATH_PID
sed -n '1,24p' "${ELASTIC_PATH_OUTPUT}"

if [[ "${VERIFY_EVENT_DRIVEN_REPLAN}" == "true" ]]; then
  assert_plan_is_event_driven 4
fi
assert_controller_never_received_empty_path
assert_controller_never_aborted_follow_path

echo "PASS: MuJoCo navigation chain is active, publishing RC-ESDF local elastic paths, accepting Nav2 goals, and forwarding motion commands."
