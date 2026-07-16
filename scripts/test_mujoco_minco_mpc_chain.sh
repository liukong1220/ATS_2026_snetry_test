#!/usr/bin/env bash
set -u

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="/tmp/ats_minco_mpc_test_logs"
LAUNCH_LOG="/tmp/ats_minco_mpc_test_launch.log"
# ROS 领域号；默认 88，避免回归测试与其他 ROS 进程串话。
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
# MuJoCo 初始位姿（map/odom 平面坐标，单位 m；yaw 单位 rad）。
START_X="${START_X:--10.66}"
START_Y="${START_Y:-1.47}"
START_Z="${START_Z:-0.42}"
START_YAW="${START_YAW:-0.0}"
GOAL_X="${GOAL_X:--9.0}"
GOAL_Y="${GOAL_Y:-1.47}"
# 单点测试的目标朝向四元数 w；当前只使用零 yaw 的 w=1。
GOAL_YAW_W="${GOAL_YAW_W:-1.0}"
# 每个目标允许的最长执行时间（s）。
GOAL_TIMEOUT="${GOAL_TIMEOUT:-60}"
# 回归路线：single、rectangle（验证横移）、red_box（长距离路线）。
TEST_PROFILE="${TEST_PROFILE:-single}"
# 每一段至少应产生的位姿位移（m），低于此值视为控制未真正跟随。
MIN_LEG_PROGRESS="${MIN_LEG_PROGRESS:-0.20}"
# 每段终点的平面位置误差门限（m）。
GOAL_TOLERANCE="${GOAL_TOLERANCE:-0.30}"
# P2 回归默认由 ROGMap adapter 唯一发布规划栅格；可设为 rc_esdf 做对照。
PLANNING_GRID_OWNER="${PLANNING_GRID_OWNER:-rog_map}"
LAUNCH_ROG_MAP="${LAUNCH_ROG_MAP:-true}"
ROG_MAP_CONFIG_FILE="${ROG_MAP_CONFIG_FILE:-${WORKSPACE_DIR}/src/ats_sentry_nav/ats_rog_map/config/rog_map_ground_planning_mujoco.yaml}"
LIDAR_DOWNSAMPLE="${LIDAR_DOWNSAMPLE:-24}"
# 故障注入必须单独启动一套 MuJoCo，避免目标与机器人状态跨用例污染。
P2_FAULT_CASE="${P2_FAULT_CASE:-none}"

case "${PLANNING_GRID_OWNER}" in
  rc_esdf|rog_map) ;;
  *)
    echo "Unsupported PLANNING_GRID_OWNER='${PLANNING_GRID_OWNER}'; use 'rc_esdf' or 'rog_map'."
    exit 2
    ;;
esac
case "${P2_FAULT_CASE}" in
  none|adapter_lease|service_timeout|input_stale|unknown|unreachable) ;;
  *)
    echo "Unsupported P2_FAULT_CASE='${P2_FAULT_CASE}'; use 'none', 'adapter_lease', " \
      "'service_timeout', 'input_stale', 'unknown', or 'unreachable'."
    exit 2
    ;;
esac
if [[ "${P2_FAULT_CASE}" != "none" && "${PLANNING_GRID_OWNER}" != "rog_map" ]]; then
  echo "P2_FAULT_CASE='${P2_FAULT_CASE}' requires PLANNING_GRID_OWNER='rog_map'."
  exit 2
fi
if [[ "${PLANNING_GRID_OWNER}" == "rog_map" || "${LAUNCH_ROG_MAP,,}" == "true" ]]; then
  [[ -r "${ROG_MAP_CONFIG_FILE}" ]] || {
    echo "ROGMap config is not readable: ${ROG_MAP_CONFIG_FILE}"
    exit 2
  }
fi

case "${TEST_PROFILE}" in
  single)
    GOAL_NAMES=(single)
    GOAL_XS=("${GOAL_X}")
    GOAL_YS=("${GOAL_Y}")
    ;;
  rectangle)
    # 先进入东侧空旷区，再闭合 0.68 m x 0.32 m 矩形；南边与保守聚合后的
    # 静态墙保持完整 footprint 余量。保持 yaw=0，使南北两段必须产生真实
    # 横移速度，而不是差速式原地转向。
    GOAL_NAMES=(stage east south west north)
    GOAL_XS=(-9.50 -8.82 -8.82 -9.50 -9.50)
    GOAL_YS=(1.47 1.47 1.15 1.15 1.47)
    ;;
  red_box)
    # 截图红框中心由 rmuc_2026.pgm/yaml 换算：pixel=(510,425) ->
    # map=(-0.043,-4.082)。先经东侧空旷区，再执行约 10 m 的长路线。
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
STOPPED_PIDS=()

cleanup() {
  for pid in "${STOPPED_PIDS[@]:-}"; do
    kill -CONT "${pid}" 2>/dev/null || true
  done
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
    sleep 1
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || true
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
      P2_LAST_GENERATION="${current}"
      return 0
    fi
    sleep 1
  done
  fail "ROGMap adapter generation did not advance beyond ${baseline}"
}

assert_topic_ownership() {
  local topic="$1"
  local expected_publisher="$2"
  local expected_subscriber="${3:-}"
  local topic_info publisher_block subscription_block
  topic_info="$(ros2 topic info --verbose "${topic}")"
  grep -q '^Publisher count: 1$' <<<"${topic_info}" || \
    fail "${topic} publisher count is not one"
  publisher_block="$(sed -n '/^Publisher count:/,/^Subscription count:/p' <<<"${topic_info}")"
  grep -q "Node name: ${expected_publisher}$" <<<"${publisher_block}" || \
    fail "${topic} publisher is not ${expected_publisher}"
  if [[ -n "${expected_subscriber}" ]]; then
    grep -q '^Subscription count: 1$' <<<"${topic_info}" || \
      fail "${topic} subscription count is not one"
    subscription_block="$(sed -n '/^Subscription count:/,$p' <<<"${topic_info}")"
    grep -q "Node name: ${expected_subscriber}$" <<<"${subscription_block}" || \
      fail "${topic} subscriber is not ${expected_subscriber}"
    echo "OK: ${topic} ownership ${expected_publisher} -> ${expected_subscriber} is unique"
    return
  fi
  echo "OK: ${topic} has one publisher owned by ${expected_publisher}"
}

verify_rog_map_planning_interface() {
  local topic node_info generation node_list
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

  assert_topic_ownership /rc_esdf/planning_grid ats_rog_map_adapter
  node_list="$(ros2 node list)"
  grep -q '^/ats_rog_map$' <<<"${node_list}" || fail "ats_rog_map is absent"
  grep -q '^/ats_rog_map_adapter$' <<<"${node_list}" || fail "ats_rog_map_adapter is absent"
  if grep -q '^/rc_esdf_map$' <<<"${node_list}"; then
    fail "rc_esdf_map must not run while ROGMap owns the planning grid"
  fi
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

path_pose_count() {
  local output_file="$1"
  awk '/^[[:space:]]*-[[:space:]]+header:$/ {count += 1} END {print count + 0}' \
    "${output_file}"
}

assert_path_has_poses() {
  local label="$1"
  local output_file="$2"
  local pose_count
  pose_count="$(path_pose_count "${output_file}")"
  if ! awk -v count="${pose_count}" 'BEGIN {exit count > 0 ? 0 : 1}'; then
    fail "${label} published an empty path"
  fi
  echo "OK: ${label} published ${pose_count} poses"
}

assert_nonzero_stream() {
  local label="$1"
  local output_file="$2"
  if ! awk '
    function finite_number(value) {
      return value ~ /^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$/;
    }
    /^[[:space:]]*(x|y|z|linear_x|linear_y|angular_z):/ {
      seen = 1;
      if (!finite_number($2)) {
        invalid = 1;
        next;
      }
      value = $2 + 0.0;
      if (value < 0.0) value = -value;
      if (value > 0.001) found = 1;
    }
    END {exit seen && !invalid && found ? 0 : 1}
  ' "${output_file}"; then
    fail "${label} did not contain finite non-zero commands"
  fi
  echo "OK: ${label} carried a non-zero command"
}

assert_zero_stream() {
  local label="$1"
  local output_file="$2"
  if ! awk '
    function finite_number(value) {
      return value ~ /^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$/;
    }
    /^[[:space:]]*(x|y|z|linear_x|linear_y|angular_z):/ {
      found = 1;
      if (!finite_number($2)) {
        invalid = 1;
        next;
      }
      value = $2 + 0.0;
      if (value < 0.0) value = -value;
      if (value > 0.001) nonzero = 1;
    }
    END {exit found && !invalid && !nonzero ? 0 : 1}
  ' "${output_file}"; then
    fail "${label} contained a non-finite or non-zero command"
  fi
  echo "OK: ${label} remained zero"
}

stream_has_nonzero_command() {
  local output_file="$1"
  awk '
    function finite_number(value) {
      return value ~ /^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$/;
    }
    /^[[:space:]]*(x|y|z|linear_x|linear_y|angular_z):/ {
      seen = 1;
      if (!finite_number($2)) {
        invalid = 1;
        next;
      }
      value = $2 + 0.0;
      if (value < 0.0) value = -value;
      if (value > 0.001) found = 1;
    }
    END {exit seen && !invalid && found ? 0 : 1}
  ' "${output_file}"
}

capture_zero_outputs() {
  local label="$1"
  local cmd_file="/tmp/ats_p2_fault_${label}_cmd.out"
  local motion_file="/tmp/ats_p2_fault_${label}_motion.out"
  sleep 0.5
  capture_numeric_stream /cmd_vel_mpc "${cmd_file}" || \
    fail "${label} did not publish /cmd_vel_mpc during stop"
  capture_numeric_stream /motion_control "${motion_file}" || \
    fail "${label} did not publish /motion_control during stop"
  assert_zero_stream "${label} /cmd_vel_mpc" "${cmd_file}"
  assert_zero_stream "${label} /motion_control" "${motion_file}"
}

capture_numeric_stream() {
  local topic="$1"
  local output_file="$2"
  local attempt
  for attempt in 1 2 3; do
    : >"${output_file}"
    timeout 3 ros2 topic echo "${topic}" >"${output_file}" 2>/dev/null || true
    if grep -Eq '^[[:space:]]*(x|y|z|linear_x|linear_y|angular_z):' "${output_file}"; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

publish_relative_fault_goal() {
  local label="$1"
  local pose_file="/tmp/ats_p2_fault_${label}_pose.out"
  local command_file="/tmp/ats_p2_fault_${label}_motion_start.out"
  local goal_output="/tmp/ats_p2_fault_${label}_goal.out"
  local current_x current_y goal_x monitor_pid
  capture_pose "${pose_file}" || fail "cannot capture pose for ${label}"
  current_x="$(pose_axis "${pose_file}" x)"
  current_y="$(pose_axis "${pose_file}" y)"
  goal_x="$(awk -v x="${current_x}" 'BEGIN {printf "%.6f", x + 0.60}')"
  : >"${command_file}"
  timeout 15 ros2 topic echo /cmd_vel_mpc >"${command_file}" 2>/dev/null &
  monitor_pid=$!
  CAPTURE_PIDS+=("${monitor_pid}")
  sleep 2
  timeout 8 ros2 topic pub --rate 2 --times 3 --wait-matching-subscriptions 2 \
    --qos-durability volatile \
    /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: ${goal_x}, y: ${current_y}, z: 0.0}, orientation: {w: 1.0}}}" \
    >"${goal_output}" 2>&1 || fail "cannot publish relative goal for ${label}"
  wait_for_command "${label} clears emergency stop" 12 \
    topic_field_equals /planner/emergency_stop data false
  wait_for_command "${label} produces MPC motion" 12 \
    stream_has_nonzero_command "${command_file}"
  kill "${monitor_pid}" 2>/dev/null || true
  wait "${monitor_pid}" 2>/dev/null || true
}

resume_process() {
  local pid="$1"
  local label="$2"
  kill -CONT "${pid}" || fail "cannot resume ${label} process ${pid}"
  echo "OK: resumed ${label} process ${pid}"
}

find_unknown_goal() {
  local query_output query_error="/tmp/ats_p2_fault_unknown_query.err"
  local attempt
  for attempt in $(seq 1 12); do
    if query_output="$(timeout 15 python3 "${WORKSPACE_DIR}/scripts/query_occupancy_grid.py" \
      --topic /rc_esdf/planning_grid --timeout 10 value --value -1 2>"${query_error}")"; then
      break
    fi
    sleep 0.5
  done
  [[ -n "${query_output:-}" ]] || fail "cannot select unknown goal: $(<"${query_error}")"
  read -r UNKNOWN_GOAL_X UNKNOWN_GOAL_Y UNKNOWN_GOAL_INDEX UNKNOWN_GOAL_VALUE \
    UNKNOWN_GOAL_FRAME UNKNOWN_GOAL_STAMP <<<"${query_output}"
  [[ "${UNKNOWN_GOAL_INDEX}" =~ ^[0-9]+$ && "${UNKNOWN_GOAL_VALUE}" == "-1" ]] || \
    fail "unknown-grid query returned malformed data: ${query_output}"
  echo "OK: selected ${UNKNOWN_GOAL_FRAME} unknown cell ${UNKNOWN_GOAL_INDEX} " \
    "at (${UNKNOWN_GOAL_X}, ${UNKNOWN_GOAL_Y}) stamp=${UNKNOWN_GOAL_STAMP}"
}

find_unreachable_goal() {
  local pose_file="/tmp/ats_p2_fault_unreachable_pose.out"
  local query_error="/tmp/ats_p2_fault_unreachable_query.err"
  local query_output start_x start_y
  capture_pose "${pose_file}" || fail "cannot capture start pose for unreachable goal"
  start_x="$(pose_axis "${pose_file}" x)"
  start_y="$(pose_axis "${pose_file}" y)"
  query_output="$(timeout 20 python3 "${WORKSPACE_DIR}/scripts/query_occupancy_grid.py" \
    --topic /rc_esdf/planning_grid --timeout 10 unreachable \
    --start-x "${start_x}" --start-y "${start_y}" --threshold 50 --clearance 0.57 \
    2>"${query_error}")" || fail "cannot select free unreachable goal: $(<"${query_error}")"
  read -r UNREACHABLE_GOAL_X UNREACHABLE_GOAL_Y UNREACHABLE_GOAL_INDEX \
    UNREACHABLE_GOAL_VALUE UNREACHABLE_GOAL_FRAME UNREACHABLE_GOAL_STAMP <<<"${query_output}"
  [[ "${UNREACHABLE_GOAL_INDEX}" =~ ^[0-9]+$ && \
    "${UNREACHABLE_GOAL_VALUE}" =~ ^[0-9]+$ && "${UNREACHABLE_GOAL_VALUE}" -lt 50 ]] || \
    fail "unreachable-grid query returned malformed data: ${query_output}"
  echo "OK: selected free ${UNREACHABLE_GOAL_FRAME} cell ${UNREACHABLE_GOAL_INDEX} " \
    "at (${UNREACHABLE_GOAL_X}, ${UNREACHABLE_GOAL_Y}) stamp=${UNREACHABLE_GOAL_STAMP}"
}

run_p2_fault_injection() {
  local fault_case="$1"
  local process_pid baseline log_start_line

  case "${fault_case}" in
    adapter_lease)
      process_pid="$(pgrep -P "${LAUNCH_PID}" -f 'ats_rog_map_adapter_node' | head -n 1 || true)"
      [[ -n "${process_pid}" ]] || fail "cannot locate adapter process for fault injection"
      publish_relative_fault_goal adapter_lease
      baseline="$(read_topic_field /rog_map_adapter/generation data)"
      [[ "${baseline}" =~ ^[0-9]+$ ]] || fail "cannot read generation before adapter lease fault"
      kill -STOP "${process_pid}" || fail "cannot pause adapter process"
      STOPPED_PIDS+=("${process_pid}")
      wait_for_command "adapter heartbeat lease triggers emergency stop" 8 \
        topic_field_equals /planner/emergency_stop data true
      capture_zero_outputs adapter_lease
      resume_process "${process_pid}" adapter
      wait_for_command "adapter recovers ready heartbeat" 12 \
        topic_field_equals /rog_map_adapter/ready data true
      wait_for_generation_advance "${baseline}" 15
      capture_zero_outputs adapter_lease_recovery
      ;;
    service_timeout)
      process_pid="$(pgrep -P "${LAUNCH_PID}" -f 'ats_rog_map_node' | head -n 1 || true)"
      [[ -n "${process_pid}" ]] || fail "cannot locate ROGMap process for fault injection"
      publish_relative_fault_goal service_timeout
      baseline="$(read_topic_field /rog_map_adapter/generation data)"
      [[ "${baseline}" =~ ^[0-9]+$ ]] || fail "cannot read generation before service timeout"
      log_start_line=$(( $(wc -l < "${LAUNCH_LOG}") + 1 ))
      kill -STOP "${process_pid}" || fail "cannot pause ROGMap process"
      STOPPED_PIDS+=("${process_pid}")
      wait_for_command "ROGMap service timeout publishes adapter not-ready" 10 \
        topic_field_equals /rog_map_adapter/ready data false
      wait_for_command "ROGMap service timeout is logged" 6 \
        bash -c "tail -n +${log_start_line} '${LAUNCH_LOG}' | grep -q 'projection request timed out'"
      wait_for_command "ROGMap service timeout triggers emergency stop" 8 \
        topic_field_equals /planner/emergency_stop data true
      capture_zero_outputs service_timeout
      resume_process "${process_pid}" ROGMap
      wait_for_command "ROGMap service recovers adapter ready" 15 \
        topic_field_equals /rog_map_adapter/ready data true
      wait_for_generation_advance "${baseline}" 15
      capture_zero_outputs service_timeout_recovery
      ;;
    input_stale)
      process_pid="$(pgrep -P "${LAUNCH_PID}" -f 'ats_mujoco_sim/lib/ats_mujoco_sim/ats_mujoco_sim' | head -n 1 || true)"
      [[ -n "${process_pid}" ]] || fail "cannot locate MuJoCo process for fault injection"
      publish_relative_fault_goal input_stale
      baseline="$(read_topic_field /rog_map_adapter/generation data)"
      [[ "${baseline}" =~ ^[0-9]+$ ]] || fail "cannot read generation before input stale"
      kill -STOP "${process_pid}" || fail "cannot pause MuJoCo input process"
      STOPPED_PIDS+=("${process_pid}")
      wait_for_command "ROGMap detects stale Point-LIO-compatible inputs" 8 \
        topic_field_equals /rog_map/stale data true
      wait_for_command "input stale publishes adapter not-ready" 8 \
        topic_field_equals /rog_map_adapter/ready data false
      wait_for_command "input stale triggers emergency stop" 8 \
        topic_field_equals /planner/emergency_stop data true
      capture_zero_outputs input_stale
      resume_process "${process_pid}" MuJoCo
      wait_for_command "ROGMap inputs recover fresh" 15 \
        topic_field_equals /rog_map/stale data false
      wait_for_command "input recovery restores adapter ready" 15 \
        topic_field_equals /rog_map_adapter/ready data true
      wait_for_generation_advance "${baseline}" 15
      capture_zero_outputs input_stale_recovery
      ;;
    unknown)
      wait_for_command "nominal emergency stop is clear" 10 \
        topic_field_equals /planner/emergency_stop data false
      timeout 15 python3 "${WORKSPACE_DIR}/scripts/query_occupancy_grid.py" \
        --topic /map --timeout 10 publish-unknown --output-topic /map --count 3 --period 0.2 \
        >/tmp/ats_p2_fault_unknown_static_map.out 2>&1 || \
        fail "cannot inject an unknown static-map snapshot"
      find_unknown_goal
      log_start_line=$(( $(wc -l < "${LAUNCH_LOG}") + 1 ))
      timeout 8 ros2 topic pub --rate 2 --times 3 --wait-matching-subscriptions 2 \
        --qos-durability volatile \
        /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: ${UNKNOWN_GOAL_FRAME}}, pose: {position: {x: ${UNKNOWN_GOAL_X}, y: ${UNKNOWN_GOAL_Y}, z: 0.0}, orientation: {w: 1.0}}}" \
        >/tmp/ats_p2_fault_unknown_pub.out 2>&1 || fail "cannot publish unknown goal"
      wait_for_command "unknown goal triggers emergency stop" 10 \
        topic_field_equals /planner/emergency_stop data true
      wait_for_command "unknown goal is rejected as occupied" 6 \
        bash -c "tail -n +${log_start_line} '${LAUNCH_LOG}' | grep -q 'goal is occupied'"
      capture_zero_outputs unknown
      ;;
    unreachable)
      wait_for_command "nominal emergency stop is clear" 10 \
        topic_field_equals /planner/emergency_stop data false
      find_unreachable_goal
      log_start_line=$(( $(wc -l < "${LAUNCH_LOG}") + 1 ))
      timeout 8 ros2 topic pub --rate 2 --times 3 --wait-matching-subscriptions 2 \
        --qos-durability volatile \
        /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: ${UNREACHABLE_GOAL_FRAME}}, pose: {position: {x: ${UNREACHABLE_GOAL_X}, y: ${UNREACHABLE_GOAL_Y}, z: 0.0}, orientation: {w: 1.0}}}" \
        >/tmp/ats_p2_fault_unreachable_pub.out 2>&1 || fail "cannot publish unreachable goal"
      wait_for_command "unreachable goal triggers emergency stop" 10 \
        topic_field_equals /planner/emergency_stop data true
      wait_for_command "free unreachable goal is classified no-path" 8 \
        bash -c "tail -n +${log_start_line} '${LAUNCH_LOG}' | grep -q 'failed.*no path'"
      capture_zero_outputs unreachable
      ;;
  esac
  echo "PASS: independent P2 '${fault_case}' fault gate completed."
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
  local final_x final_y error
  final_x="$(pose_axis "${pose_file}" x)"
  final_y="$(pose_axis "${pose_file}" y)"
  error="$(awk -v gx="${goal_x}" -v gy="${goal_y}" -v fx="${final_x}" -v fy="${final_y}" '
    BEGIN {
      dx = fx - gx;
      dy = fy - gy;
      printf "%.6f", sqrt(dx * dx + dy * dy);
    }
  ')"
  echo "RESULT: ${label} final=(${final_x}, ${final_y}) goal=(${goal_x}, ${goal_y}) error=${error} m"
  if ! awk -v error="${error}" -v tolerance="${GOAL_TOLERANCE}" \
    'BEGIN {exit error <= tolerance ? 0 : 1}'
  then
    fail "${label} final pose error ${error} m exceeds ${GOAL_TOLERANCE} m"
  fi
}

assert_minco_plan_record() {
  local label="$1"
  local start_line="$2"
  local deadline=$((SECONDS + 10))
  local record generation raw_points reference_points collisions
  while (( SECONDS < deadline )); do
    record="$(tail -n +"${start_line}" "${LAUNCH_LOG}" | \
      grep 'planned generation=' | tail -n 1 || true)"
    if [[ "${record}" =~ generation=([0-9]+).*raw_points=([0-9]+).*reference_points=([0-9]+).*collisions=([0-9]+) ]]; then
      generation="${BASH_REMATCH[1]}"
      raw_points="${BASH_REMATCH[2]}"
      reference_points="${BASH_REMATCH[3]}"
      collisions="${BASH_REMATCH[4]}"
      if ! awk -v raw="${raw_points}" -v reference="${reference_points}" \
        'BEGIN {exit raw > 0 && reference > 0 ? 0 : 1}'
      then
        fail "${label} MINCO log contains an empty raw/reference path"
      fi
      [[ "${collisions}" == "0" ]] || \
        fail "${label} MINCO footprint gate reported ${collisions} collisions"
      echo "RESULT: ${label} generation=${generation} raw_points=${raw_points} " \
        "reference_points=${reference_points} footprint_collisions=${collisions}"
      return 0
    fi
    sleep 0.2
  done
  fail "${label} has no complete MINCO planning record in the launch log"
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
  assert_path_has_poses "${label}" "${output_file}"
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
  local topic output_file pid log_line_count log_start_line
  local -a topic_pids=()

  log_line_count="$(wc -l < "${LAUNCH_LOG}")"
  log_start_line=$((log_line_count + 1))
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

  # MINCO 路径为事件触发发布；先等待 DDS 单次订阅发现完成，
  # 避免目标触发后多个路径话题瞬时发布而被测试遗漏。
  sleep 2

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
  assert_minco_plan_record "${name}" "${log_start_line}"

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
  # 启用 JPS/MINCO/全向 MPC 旁路；当前脚本仍用 Nav2 action 下发目标。
  launch_swerve_mpc:=true
  use_viewer:=false
  show_viewer:=false
  launch_mujoco_rviz:=false
  # 当前兼容回归需 Nav2 生成 /plan；Nav2-free 回归应另建脚本，不能复用这里的 action。
  launch_nav2:=true
  launch_trajectory_optimizer:=true
  launch_twist_bridge:=true
  launch_rog_map:="${LAUNCH_ROG_MAP}"
  planning_grid_owner:="${PLANNING_GRID_OWNER}"
  rog_map_config_file:="${ROG_MAP_CONFIG_FILE}"
  enable_lidar:=true
  lidar_backend:=cpu
  lidar_downsample:="${LIDAR_DOWNSAMPLE}"
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

setsid ros2 launch "${LAUNCH_ARGS[@]}" >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

wait_for_command "node graph" 60 timeout 4 ros2 node list
wait_for_topic_once /localization 70
wait_for_topic_once /traversability_grid 120
wait_for_topic_once /rc_esdf/planning_grid 120
for node in /controller_server /planner_server /bt_navigator; do
  wait_for_lifecycle_active "${node}"
done

if [[ "${PLANNING_GRID_OWNER}" == "rog_map" ]]; then
  verify_rog_map_planning_interface
fi

NODE_LIST="$(ros2 node list)"
grep -q '^/minco_planner$' <<<"${NODE_LIST}" || fail "minco_planner is absent"
grep -q '^/ats_swerve_mpc$' <<<"${NODE_LIST}" || fail "ats_swerve_mpc is absent"
grep -q '^/twist_to_motion_ctrl$' <<<"${NODE_LIST}" || fail "twist bridge is absent"
if grep -q '^/fake_vel_transform$' <<<"${NODE_LIST}"; then
  fail "fake_vel_transform must be disabled in swerve MPC mode"
fi
echo "OK: MPC nodes present and fake_vel_transform absent"

assert_topic_ownership /cmd_vel_mpc ats_swerve_mpc twist_to_motion_ctrl
assert_topic_ownership /motion_control twist_to_motion_ctrl ats_mujoco_sim

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
  assert_path_has_poses "${topic}" "${output_file}"
done
assert_nonzero_stream /cmd_vel_mpc /tmp/ats_minco_mpc_cmd_vel_stream.out
assert_nonzero_stream /motion_control /tmp/ats_minco_mpc_motion_stream.out

if [[ "${PLANNING_GRID_OWNER}" == "rog_map" ]]; then
  wait_for_generation_advance "${P2_LAST_GENERATION}" 30
  if [[ "${P2_FAULT_CASE}" != "none" ]]; then
    run_p2_fault_injection "${P2_FAULT_CASE}"
  fi
fi

echo "PASS: MuJoCo JPS/MINCO/clearance-aware yaw/SE2 MPC '${TEST_PROFILE}' profile completed."
