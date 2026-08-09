#!/usr/bin/env bash
set -u

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="/tmp/ats_minco_mpc_test_logs"
# ROS 领域号；默认 88，避免回归测试与其他 ROS 进程串话。
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
# 每个隔离 domain 使用独立 launch 日志，避免异常遗留的旧进程污染本轮验收记录。
LAUNCH_LOG="/tmp/ats_minco_mpc_test_launch_${ROS_DOMAIN_ID}.log"
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
# RViz must be optional in the same scenario runner so a display-only change
# can be checked against the identical closed-loop route and fault gates.
USE_RVIZ="${USE_RVIZ:-false}"
if [[ "${USE_RVIZ}" == "true" ]]; then
  RVIZ_DELAY_SEC="${RVIZ_DELAY_SEC:-18.0}"
else
  RVIZ_DELAY_SEC="${RVIZ_DELAY_SEC:-1000.0}"
fi
# 每一段至少应产生的位姿位移（m），低于此值视为控制未真正跟随。
MIN_LEG_PROGRESS="${MIN_LEG_PROGRESS:-0.20}"
# 每段终点的平面位置误差门限（m）。
GOAL_TOLERANCE="${GOAL_TOLERANCE:-0.30}"
LIDAR_DOWNSAMPLE="${LIDAR_DOWNSAMPLE:-24}"
# 故障注入必须单独启动一套 MuJoCo，避免目标与机器人状态跨用例污染。
P2_FAULT_CASE="${P2_FAULT_CASE:-none}"
# Planning-grid ownership is a launch-time contract; it is intentionally not
# changed while a robot is running.  The current MuJoCo chain implements the
# ROGMap adapter owner only.
PLANNING_GRID_OWNER="${PLANNING_GRID_OWNER:-rog_map}"
# P3 action 生命周期故障；每次也必须使用新的 ROS_DOMAIN_ID 和 MuJoCo launch。
P3_FAULT_CASE="${P3_FAULT_CASE:-none}"
# Route-profile input for narrow/slope/contact-sensitive BODY_YAW_FOLLOW
# regressions. This is not a runtime authority hot switch.
FORCE_BODY_YAW_FOLLOW="${FORCE_BODY_YAW_FOLLOW:-false}"
BODY_YAW_FOLLOW_CLEARANCE="${BODY_YAW_FOLLOW_CLEARANCE:-0.55}"
# auto records the selected immutable policy; gimbal/body assert the complete
# execute lease and simulated gimbal acknowledgement contract.
YAW_AUTHORITY_EXPECTED="${YAW_AUTHORITY_EXPECTED:-auto}"
# MPC solver mode; default preserves the production iLQR chain. qp_shadow only
# records OSQP diagnostics and never becomes the command publisher.
SOLVER_MODE="${SOLVER_MODE:-ilqr}"
# 默认仍为 warn；qp_shadow 观察可显式传 LOG_LEVEL=info 以保存有界 telemetry。
LOG_LEVEL="${LOG_LEVEL:-warn}"
# 仅在回归调用方显式给出路径时导出控制遥测。文件 I/O 由独立 Python 客户端执行，绝不进入
# MPC control timer；空值保持既有回归行为不变。
QP_TELEMETRY_OUTPUT="${QP_TELEMETRY_OUTPUT:-}"
QP_TELEMETRY_MANIFEST="${QP_TELEMETRY_MANIFEST:-}"
QP_TELEMETRY_WINDOW_CYCLES="${QP_TELEMETRY_WINDOW_CYCLES:-0}"
QP_TELEMETRY_RUN_START_EPOCH_NS="$(date +%s%N)"
case "${SOLVER_MODE}" in
  ilqr|qp_shadow) ;;
  *)
    echo "Unsupported SOLVER_MODE='${SOLVER_MODE}'; use 'ilqr' or 'qp_shadow'."
    exit 2
    ;;
esac
case "${LOG_LEVEL}" in
  debug|info|warn|error|fatal) ;;
  *)
    echo "Unsupported LOG_LEVEL='${LOG_LEVEL}'; use debug, info, warn, error, or fatal."
    exit 2
    ;;
esac
if ! [[ "${QP_TELEMETRY_WINDOW_CYCLES}" =~ ^[0-9]+$ ]] ||
  (( QP_TELEMETRY_WINDOW_CYCLES > 128 )); then
  echo "QP_TELEMETRY_WINDOW_CYCLES must be an integer in [0, 128]."
  exit 2
fi
if [[ -n "${QP_TELEMETRY_OUTPUT}" && "${QP_TELEMETRY_WINDOW_CYCLES}" == "0" ]]; then
  echo "QP_TELEMETRY_OUTPUT requires QP_TELEMETRY_WINDOW_CYCLES in [1, 128]."
  exit 2
fi

case "${P2_FAULT_CASE}" in
  none|adapter_lease|service_timeout|input_stale|unknown|unreachable|freeze) ;;
  *)
    echo "Unsupported P2_FAULT_CASE='${P2_FAULT_CASE}'; use 'none', 'adapter_lease', " \
      "'service_timeout', 'input_stale', 'unknown', 'unreachable', or 'freeze'."
    exit 2
    ;;
esac
case "${PLANNING_GRID_OWNER}" in
  rog_map) ;;
  rc_esdf)
    echo "PLANNING_GRID_OWNER=rc_esdf is not implemented by the current MuJoCo launch; refusing to claim a publisher."
    exit 2
    ;;
  *)
    echo "Unsupported PLANNING_GRID_OWNER='${PLANNING_GRID_OWNER}'; use 'rog_map' or 'rc_esdf'."
    exit 2
    ;;
esac
case "${P3_FAULT_CASE}" in
  none|cancel|preempt|timeout|tf_failure) ;;
  *)
    echo "Unsupported P3_FAULT_CASE='${P3_FAULT_CASE}'; use 'none', 'cancel', " \
      "'preempt', 'timeout', or 'tf_failure'."
    exit 2
    ;;
esac
case "${FORCE_BODY_YAW_FOLLOW,,}" in
  true|false) ;;
  *)
    echo "FORCE_BODY_YAW_FOLLOW must be true or false."
    exit 2
    ;;
esac
case "${YAW_AUTHORITY_EXPECTED}" in
  auto|gimbal|body) ;;
  *)
    echo "YAW_AUTHORITY_EXPECTED must be auto, gimbal, or body."
    exit 2
    ;;
esac
if [[ "${FORCE_BODY_YAW_FOLLOW,,}" == "true" &&
  "${YAW_AUTHORITY_EXPECTED}" == "gimbal" ]]; then
  echo "FORCE_BODY_YAW_FOLLOW=true cannot expect GIMBAL_COMPENSATED."
  exit 2
fi
P3_GOAL_FRAME="${P3_GOAL_FRAME:-map}"

case "${TEST_PROFILE}" in
  default|single)
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
    echo "Unsupported TEST_PROFILE='${TEST_PROFILE}'; use 'default', 'single', 'rectangle', or 'red_box'."
    exit 2
    ;;
esac
case "${USE_RVIZ}" in
  true|false) ;;
  *)
    echo "Unsupported USE_RVIZ='${USE_RVIZ}'; use 'true' or 'false'."
    exit 2
    ;;
esac

set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID
export ROS_LOG_DIR="${LOG_DIR}"
# 沙箱禁止 ros2cli 的 XML-RPC daemon socket；直接通过当前 DDS domain 查询图。
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"

rm -rf "${LOG_DIR}"
mkdir -p "${LOG_DIR}"
: > "${LAUNCH_LOG}"

CAPTURE_PIDS=()
STOPPED_PIDS=()

stop_capture_process() {
  local pid="$1"
  local deadline
  [[ -n "${pid}" ]] || return
  kill -TERM "${pid}" 2>/dev/null || true
  deadline=$((SECONDS + 3))
  while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 0.1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    kill -KILL "${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  for pid in "${STOPPED_PIDS[@]:-}"; do
    kill -CONT "${pid}" 2>/dev/null || true
  done
  for pid in "${CAPTURE_PIDS[@]:-}"; do
    stop_capture_process "${pid}"
  done
  if [[ -n "${GOAL_PID:-}" ]]; then
    stop_capture_process "${GOAL_PID}"
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
  local qos_reliability="${3:-}"
  local -a echo_args=(ros2 topic echo --no-daemon --once)
  if [[ -n "${qos_reliability}" ]]; then
    echo_args+=(--qos-reliability "${qos_reliability}")
  fi
  echo_args+=("${topic}")
  wait_for_command "topic ${topic}" "${timeout_sec}" timeout 4 "${echo_args[@]}"
}

read_topic_field() {
  local topic="$1"
  local field="$2"
  local qos_reliability="${3:-}"
  local -a echo_args=(ros2 topic echo --no-daemon --once)
  if [[ -n "${qos_reliability}" ]]; then
    echo_args+=(--qos-reliability "${qos_reliability}")
  fi
  echo_args+=("${topic}" --field "${field}")
  timeout 5 "${echo_args[@]}" 2>/dev/null | awk '
    $1 ~ /^[[:alnum:]_]+:$/ && NF >= 2 {print $2; exit}
    NF == 1 && $1 != "---" {print $1; exit}
  '
}

topic_field_equals() {
  local topic="$1"
  local field="$2"
  local expected="$3"
  local qos_reliability="${4:-}"
  local value
  value="$(read_topic_field "${topic}" "${field}" "${qos_reliability}")"
  [[ "${value,,}" == "${expected,,}" ]]
}

topic_field_positive() {
  local topic="$1"
  local field="$2"
  local qos_reliability="${3:-}"
  local value
  value="$(read_topic_field "${topic}" "${field}" "${qos_reliability}")"
  [[ "${value}" =~ ^[0-9]+$ ]] && awk -v value="${value}" 'BEGIN {exit value > 0 ? 0 : 1}'
}

read_positive_topic_field() {
  local topic="$1"
  local field="$2"
  local timeout_sec="$3"
  local deadline=$((SECONDS + timeout_sec))
  local value
  while (( SECONDS < deadline )); do
    value="$(read_topic_field "${topic}" "${field}")"
    if [[ "${value}" =~ ^[0-9]+$ ]] &&
      awk -v value="${value}" 'BEGIN {exit value > 0 ? 0 : 1}'
    then
      printf '%s\n' "${value}"
      return 0
    fi
    sleep 1
  done
  return 1
}

read_swerve_telemetry_sequence() {
  local attempt output
  for attempt in 1 2 3; do
    output="$(timeout 5 ros2 topic echo --no-daemon --once /swerve/telemetry \
      ats_navigation_interfaces/msg/SwerveTelemetry 2>/dev/null || true)"
    output="$(awk '$1 == "sequence:" {print $2; exit}' <<<"${output}")"
    if [[ "${output}" =~ ^[0-9]+$ ]]; then
      printf '%s\n' "${output}"
      return 0
    fi
    sleep 0.5
  done
  return 1
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
  local allowed_observer="${4:-}"
  local topic_info publisher_block subscription_block deadline
  deadline=$((SECONDS + 30))
  while (( SECONDS < deadline )); do
    topic_info="$(timeout 5 ros2 topic info --no-daemon --verbose "${topic}" 2>/dev/null || true)"
    publisher_block="$(sed -n '/^Publisher count:/,/^Subscription count:/p' <<<"${topic_info}")"
    if ! grep -q '^Publisher count: 1$' <<<"${topic_info}" ||
      ! grep -q "Node name: ${expected_publisher}$" <<<"${publisher_block}"
    then
      sleep 1
      continue
    fi
    if [[ -z "${expected_subscriber}" ]]; then
      echo "OK: ${topic} has one publisher owned by ${expected_publisher}"
      return
    fi
    subscription_block="$(sed -n '/^Subscription count:/,$p' <<<"${topic_info}")"
    if [[ -z "${allowed_observer}" ]] &&
      grep -q '^Subscription count: 1$' <<<"${topic_info}" &&
      [[ "$(grep -c "^Node name: ${expected_subscriber}$" <<<"${subscription_block}")" -eq 1 ]]
    then
      echo "OK: ${topic} ownership ${expected_publisher} -> ${expected_subscriber} is unique"
      return
    fi
    if [[ -n "${allowed_observer}" ]] &&
      grep -q '^Subscription count: 2$' <<<"${topic_info}" &&
      [[ "$(grep -c "^Node name: ${expected_subscriber}$" <<<"${subscription_block}")" -eq 1 ]] &&
      [[ "$(grep -c "^Node name: ${allowed_observer}$" <<<"${subscription_block}")" -eq 1 ]]
    then
      echo "OK: ${topic} ownership ${expected_publisher} -> ${expected_subscriber}; observer ${allowed_observer} is read-only"
      return
    fi
    sleep 1
  done
  echo "${topic_info}" >&2
  fail "${topic} ownership did not converge to ${expected_publisher} -> ${expected_subscriber:-<none>}${allowed_observer:+ with observer ${allowed_observer}}"
}

node_is_present() {
  local node_name="$1"
  ros2 node list --no-daemon 2>/dev/null | grep -qx "${node_name}"
}

node_exposes_endpoint() {
  local node_name="$1"
  local endpoint="$2"
  ros2 node info --no-daemon "${node_name}" 2>/dev/null | grep -Fq "${endpoint}"
}

assert_final_swerve_telemetry() {
  local output_file="$1"
  local minimum_sequence="$2"
  local minimum_stamp_sec="$3"
  local minimum_stamp_nanosec="$4"
  local raw_file="${output_file}.raw"
  local contact_count drive_count sequence stamp_sec stamp_nanosec

  # A one-shot subscriber can consume a queued sample that predates the last
  # action. Capture a short stream and keep only telemetry newer than both the
  # pre-goal sequence and the wall-clock action submission time.
  : >"${raw_file}"
  timeout 4 ros2 topic echo --no-daemon /swerve/telemetry \
    ats_navigation_interfaces/msg/SwerveTelemetry >"${raw_file}" 2>/dev/null || true
  awk -v min_sequence="${minimum_sequence}" \
    -v min_sec="${minimum_stamp_sec}" \
    -v min_nanosec="${minimum_stamp_nanosec}" '
    function reset_message() {
      block = ""
      sequence = -1
      stamp_sec = -1
      stamp_nanosec = -1
      expect_stamp = 0
    }
    function keep_message() {
      if (block == "" || sequence <= min_sequence || stamp_sec < 0 || stamp_nanosec < 0) {
        return
      }
      if (stamp_sec > min_sec || (stamp_sec == min_sec && stamp_nanosec > min_nanosec)) {
        latest = block
      }
    }
    $0 == "---" {
      keep_message()
      reset_message()
      next
    }
    {
      block = block $0 ORS
      if ($1 == "stamp:") {
        expect_stamp = 1
      } else if (expect_stamp && $1 == "sec:") {
        stamp_sec = $2
      } else if (expect_stamp && $1 == "nanosec:") {
        stamp_nanosec = $2
        expect_stamp = 0
      } else if ($1 == "sequence:") {
        sequence = $2
      }
    }
    END {
      keep_message()
      printf "%s", latest
    }
  ' "${raw_file}" >"${output_file}"
  [[ -s "${output_file}" ]] || fail "cannot capture post-action /swerve/telemetry"

  sequence="$(awk '$1 == "sequence:" {print $2; exit}' "${output_file}")"
  stamp_sec="$(awk '$1 == "sec:" {print $2; exit}' "${output_file}")"
  stamp_nanosec="$(awk '$1 == "nanosec:" {print $2; exit}' "${output_file}")"
  [[ "${sequence}" =~ ^[0-9]+$ ]] &&
    awk -v current="${sequence}" -v baseline="${minimum_sequence}" \
      'BEGIN {exit current > baseline ? 0 : 1}' ||
    fail "final telemetry sequence did not advance after the last action"
  [[ "${stamp_sec}" =~ ^[0-9]+$ && "${stamp_nanosec}" =~ ^[0-9]+$ ]] ||
    fail "final telemetry has no valid header stamp"
  contact_count="$(awk '$1 == "contact_violation_count:" {print $2}' "${output_file}")"
  [[ "${contact_count}" =~ ^[0-9]+$ ]] || \
    fail "final telemetry has no valid contact_violation_count"
  [[ "${contact_count}" == "0" ]] || \
    fail "MuJoCo contact evaluator reported ${contact_count} violation samples"
  drive_count="$(awk '
    /^drive_rpm:/ {in_drive=1; next}
    in_drive && /^- / {
      value=$2 + 0.0
      if (value < 0.0) value=-value
      if (value >= 2.0) exit 2
      count++
      next
    }
    in_drive {in_drive=0}
    END {if (count != 4) exit 3; print count}
  ' "${output_file}")" || fail "final four-wheel drive RPM did not settle below 2 rpm"
  [[ "${drive_count}" == "4" ]] || fail "final telemetry did not contain four drive RPM values"
  echo "RESULT: MuJoCo post-action telemetry sequence=${sequence} " \
    "contact_violation_count=0 and final four-wheel drive RPM is below 2 rpm"
  sed -n '/^drive_rpm:/,/^command_vx:/p; /^contact_violation_count:/,/^max_contact_force:/p' \
    "${output_file}"
}

assert_p3_process_graph() {
  local node_list goal_manager_info forbidden
  wait_for_command "ats_goal_manager node" 30 node_is_present /ats_goal_manager
  wait_for_command "ATS P3 Navigate action" 30 \
    node_exposes_endpoint /ats_goal_manager /ats_navigate_to_pose
  node_list="$(ros2 node list --no-daemon)"
  for forbidden in /bt_navigator /planner_server /controller_server /behavior_server \
    /velocity_smoother /lifecycle_manager_rmuc_2026_map /map_server; do
    if grep -q "^${forbidden}$" <<<"${node_list}"; then
      fail "P3 graph unexpectedly contains ${forbidden}"
    fi
  done
  if ros2 topic list --no-daemon | grep -qx '/plan'; then
    fail "P3 graph unexpectedly exposes /plan"
  fi
  goal_manager_info="$(ros2 node info --no-daemon /ats_goal_manager)"
  echo "OK: P3 graph has ATS action and no Nav2 process or /plan dependency"
}

verify_rog_map_planning_interface() {
  local topic node_info generation node_list
  # /rog_map/unk is an on-demand visualization/audit cloud.  A nominal map
  # can correctly fuse to known free while its debug unknown payload is empty.
  for topic in /rog_map/occ /rog_map/inf_occ /rog_map/esdf; do
    wait_for_command "non-empty ${topic}" 120 topic_field_positive "${topic}" width best_effort
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
  wait_for_command "ats_rog_map node" 30 node_is_present /ats_rog_map
  wait_for_command "ats_rog_map_adapter node" 30 node_is_present /ats_rog_map_adapter
  node_list="$(ros2 node list --no-daemon)"
  if grep -q '^/rc_esdf_map$' <<<"${node_list}"; then
    fail "rc_esdf_map must not run while ROGMap owns the planning grid"
  fi
  wait_for_command "ROGMap numeric projection client" 30 \
    node_exposes_endpoint /ats_rog_map_adapter /rog_map/get_ground_projection
  # Discovery can briefly lose a node between the presence and endpoint checks
  # under the CPU-heavy MuJoCo/ROGMap startup.  Retry the same bounded query
  # instead of treating that DDS window as an adapter contract failure.
  wait_for_command "ROGMap adapter node info" 30 \
    node_exposes_endpoint /ats_rog_map_adapter /rog_map/get_ground_projection
  node_info=""
  local node_info_deadline=$((SECONDS + 30))
  while (( SECONDS < node_info_deadline )); do
    node_info="$(timeout 5 ros2 node info --no-daemon /ats_rog_map_adapter 2>/dev/null || true)"
    [[ -n "${node_info}" ]] && break
    sleep 0.5
  done
  [[ -n "${node_info}" ]] || fail "ROGMap adapter node info disappeared after bounded discovery retry"
  if grep -q '/rog_map/esdf' <<<"${node_info}"; then
    fail "ats_rog_map_adapter must not subscribe to the ROGMap visualization ESDF cloud"
  fi
  echo "OK: adapter consumes the numeric projection service without /rog_map/esdf subscription"

  generation="$(read_positive_topic_field /rog_map_adapter/generation data 30)" || \
    fail "cannot read ROGMap adapter generation"
  wait_for_generation_advance "${generation}" 30
}

read_rog_numeric_projection_generation() {
  local output_file="${1:-/tmp/ats_p2_rog_numeric_projection.out}"
  timeout 15 ros2 service call /rog_map/get_ground_projection \
    ats_rog_map_interfaces/srv/GetRogMapProjection \
    "{min_height: 0.1, max_height: 0.8, resolution: 0.1}" >"${output_file}" 2>&1 || return 1
  grep -q '^ready: true$' "${output_file}" || return 1
  grep -q '^stale: false$' "${output_file}" || return 1
  awk '$1 == "generation:" && $2 ~ /^[0-9]+$/ {print $2; exit}' "${output_file}"
}

read_rog_numeric_unknown_generation() {
  local output_file="${1:-/tmp/ats_p2_rog_numeric_unknown.out}"
  local generation
  generation="$(read_rog_numeric_projection_generation "${output_file}")" || return 1
  awk '
    $1 == "data:" {in_occupancy_data = 1}
    $1 == "generation:" {exit}
    in_occupancy_data && /(^|[[:space:],\[])-1([[:space:],\]]|$)/ {found = 1}
    END {exit found ? 0 : 1}
  ' "${output_file}" || return 1
  [[ "${generation}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${generation}"
}

wait_for_rog_numeric_unknown() {
  local output_file="$1"
  local generation
  generation="$(read_rog_numeric_unknown_generation "${output_file}")" || return 1
  ROG_NUMERIC_UNKNOWN_GENERATION="${generation}"
}

verify_rog_unknown_visualization() {
  local header_file="$1"
  local topic_info publisher_block
  timeout 8 ros2 topic echo --no-daemon --once --qos-reliability best_effort \
    /rog_map/unk --field header >"${header_file}" 2>&1 || return 1
  awk '
    $1 == "frame_id:" && $2 == "odom" {frame = 1}
    $1 == "sec:" && $2 ~ /^[0-9]+$/ && $2 > 0 {stamp = 1}
    END {exit frame && stamp ? 0 : 1}
  ' "${header_file}" || return 1
  topic_info="$(timeout 6 ros2 topic info --no-daemon --verbose /rog_map/unk 2>/dev/null || true)"
  publisher_block="$(sed -n '/^Publisher count:/,/^Subscription count:/p' <<<"${topic_info}")"
  grep -q '^Publisher count: 1$' <<<"${topic_info}" &&
    grep -q '^Node name: /ats_rog_map$' <<<"${publisher_block}" &&
    awk '
      $0 == "Node name: /ats_rog_map" {in_node = 1; next}
      /^Node name:/ {in_node = 0}
      in_node && $1 == "Reliability:" && $2 == "BEST_EFFORT" {found = 1}
      END {exit found ? 0 : 1}
    ' <<<"${publisher_block}"
}

record_unknown_timeline() {
  local key="$1"
  local value="${2:-$(date +%s%N)}"
  printf '%s=%s\n' "${key}" "${value}" >>"${UNKNOWN_TIMELINE}"
}

assert_no_old_reference_revival() {
  local output_file="$1"
  : >"${output_file}"
  timeout 4 ros2 topic echo --no-daemon --field poses /minco/reference_path \
    nav_msgs/msg/Path >"${output_file}" 2>/dev/null || true
  if grep -q 'position:' "${output_file}"; then
    fail "old reference revived after map recovery without a new goal"
  fi
  echo "OK: no non-empty old /minco/reference_path revived after recovery"
}

assert_rviz_best_effort_observer() {
  local topic="$1"
  local topic_info reliabilities deadline
  deadline=$((SECONDS + 30))
  while (( SECONDS < deadline )); do
    topic_info="$(timeout 5 ros2 topic info --no-daemon --verbose "${topic}" 2>/dev/null || true)"
    printf '\n=== %s ===\n%s\n' "${topic}" "${topic_info}" >>"${RVIZ_QOS_LOG}"
    reliabilities="$(awk -v node='mujoco_navigation_rviz2' '
      $0 == "Node name: " node {in_node = 1; next}
      /^Node name:/ {in_node = 0}
      in_node && $1 == "Reliability:" {print $2}
    ' <<<"${topic_info}")"
    if [[ -n "${reliabilities}" ]] && ! grep -qvx 'BEST_EFFORT' <<<"${reliabilities}"; then
      echo "OK: RViz observer uses best-effort QoS on ${topic}"
      return
    fi
    sleep 1
  done
  cat "${RVIZ_QOS_LOG}" >&2
  fail "RViz observer on ${topic} did not converge to best-effort QoS"
}

assert_rviz_runtime_contract() {
  local topic
  [[ "${USE_RVIZ}" == "true" ]] || return
  : >"${RVIZ_QOS_LOG}"
  wait_for_command "MuJoCo navigation RViz node" 30 \
    node_is_present /mujoco_navigation_rviz2
  for topic in /rog_map/occ /rog_map/inf_occ /rog_map/viz /rog_map/bounds /localization; do
    assert_rviz_best_effort_observer "${topic}"
  done
  wait_for_command "non-empty ROGMap local voxel visualization" 30 \
    topic_field_positive /rog_map/viz width best_effort
  wait_for_command "ROGMap local voxel visualization frame" 30 \
    topic_field_equals /rog_map/viz header.frame_id odom best_effort
  echo "OK: RViz runtime QoS contract saved to ${RVIZ_QOS_LOG}"
}

capture_rviz_screenshot() {
  local window_id xwd_file deadline
  [[ "${USE_RVIZ}" == "true" ]] || return
  [[ -n "${DISPLAY:-}" ]] || fail "USE_RVIZ=true requires DISPLAY for screenshot capture"
  deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    window_id="$(xwininfo -root -tree 2>/dev/null | awk '/mujoco_navigation\.rviz - RViz/ {print $1; exit}')"
    [[ "${window_id}" =~ ^0x[[:xdigit:]]+$ ]] && break
    sleep 0.5
  done
  [[ "${window_id}" =~ ^0x[[:xdigit:]]+$ ]] || fail "cannot find MuJoCo navigation RViz window"
  xwd_file="${RVIZ_SCREENSHOT%.png}.xwd"
  xwd -id "${window_id}" -silent -out "${xwd_file}" || \
    fail "cannot capture MuJoCo navigation RViz window"
  ffmpeg -y -v error -f xwd_pipe -i "${xwd_file}" -frames:v 1 "${RVIZ_SCREENSHOT}" || \
    fail "cannot convert MuJoCo navigation RViz screenshot"
  [[ -s "${RVIZ_SCREENSHOT}" ]] || fail "MuJoCo navigation RViz screenshot is empty"
  echo "OK: RViz screenshot saved to ${RVIZ_SCREENSHOT}"
}

capture_pose() {
  local output_file="$1"
  local attempt
  # ros2cli discovery is independent from the already-verified localization
  # lease. Retry boundedly so one missed transient-local discovery window is
  # not reported as a control or yaw-authority failure.
  for attempt in 1 2 3; do
    timeout 5 ros2 topic echo --no-daemon --once --qos-reliability best_effort \
      /localization --field pose.pose.position \
      >"${output_file}" 2>/dev/null || true
    if grep -q '^x:' "${output_file}" && grep -q '^y:' "${output_file}"; then
      return 0
    fi
    sleep 1
  done
  return 1
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
    timeout 3 ros2 topic echo --no-daemon "${topic}" >"${output_file}" 2>/dev/null || true
    if grep -Eq '^[[:space:]]*(x|y|z|linear_x|linear_y|angular_z):' "${output_file}"; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

send_fault_goal() {
  local label="$1"
  local frame="$2"
  local goal_x="$3"
  local goal_y="$4"
  local timeout_sec="${5:-30}"
  FAULT_ACTION_OUTPUT="/tmp/ats_p3_fault_${label}_action.out"
  FAULT_ACTION_ERROR="/tmp/ats_p3_fault_${label}_action.err"
  timeout "$((timeout_sec + 10))" ros2 action send_goal --feedback /ats_navigate_to_pose \
    ats_navigation_interfaces/action/NavigateToPose \
    "{goal_pose: {header: {frame_id: ${frame}}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: 0.0}, orientation: {w: 1.0}}}, timeout: {sec: ${timeout_sec}, nanosec: 0}}" \
    >"${FAULT_ACTION_OUTPUT}" 2>"${FAULT_ACTION_ERROR}" &
  FAULT_ACTION_PID=$!
  CAPTURE_PIDS+=("${FAULT_ACTION_PID}")
}

wait_for_fault_action_result() {
  local label="$1"
  local result_code="$2"
  local timeout_sec="${3:-15}"
  wait_for_command "${label} action result code ${result_code}" "${timeout_sec}" \
    bash -c "grep -q 'result_code: ${result_code}' '${FAULT_ACTION_OUTPUT}' && grep -q 'Goal finished with status:' '${FAULT_ACTION_OUTPUT}'"
}

publish_relative_fault_goal() {
  local label="$1"
  local pose_file="/tmp/ats_p2_fault_${label}_pose.out"
  local command_file="/tmp/ats_p2_fault_${label}_motion_start.out"
  local goal_output="/tmp/ats_p2_fault_${label}_goal.out"
  local current_x current_y p3_goal_x monitor_pid
  capture_pose "${pose_file}" || fail "cannot capture pose for ${label}"
  current_x="$(pose_axis "${pose_file}" x)"
  current_y="$(pose_axis "${pose_file}" y)"
  : >"${command_file}"
  # 重定向到文件时 ros2 Python CLI 会块缓冲；强制无缓冲才能在 tracking 期间
  # 立即观察到非零控制量，而不是等采样 timeout 后才注入故障。
  timeout 15 env PYTHONUNBUFFERED=1 ros2 topic echo --no-daemon /cmd_vel_mpc >"${command_file}" 2>/dev/null &
  monitor_pid=$!
  CAPTURE_PIDS+=("${monitor_pid}")
  sleep 2
  # nominal single 由西向东完成；故障前置段改为反向 1.80 m，复用已经通过
  # footprint gate 的自由走廊，并给 DDS 采样与故障注入保留稳定 tracking 窗口。
  p3_goal_x="$(awk -v x="${current_x}" 'BEGIN {printf "%.6f", x - 1.80}')"
  send_fault_goal "${label}" odom "${p3_goal_x}" "${current_y}" 30
  wait_for_command "${label} clears emergency stop" 12 \
    topic_field_equals /planner/emergency_stop data false
  wait_for_command "${label} produces MPC motion" 12 \
    stream_has_nonzero_command "${command_file}"
  stop_capture_process "${monitor_pid}"
}

resume_process() {
  local pid="$1"
  local label="$2"
  kill -CONT "${pid}" || fail "cannot resume ${label} process ${pid}"
  echo "OK: resumed ${label} process ${pid}"
}

# [Dead Code Suggestion] 旧 Nav2/static-map unknown 注入路径的查询 helper。
# P3 action-only 回归不再调用；待 P3 运行验收完成后再单独清理。
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
  local process_pid baseline log_start_line replan_count

  case "${fault_case}" in
    adapter_lease)
      process_pid="$(pgrep -P "${LAUNCH_PID}" -f 'ats_rog_map_adapter_node' | head -n 1 || true)"
      [[ -n "${process_pid}" ]] || fail "cannot locate adapter process for fault injection"
      publish_relative_fault_goal adapter_lease
      baseline="$(read_positive_topic_field /rog_map_adapter/generation data 10)" || \
        fail "cannot read generation before adapter lease fault"
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
      baseline="$(read_positive_topic_field /rog_map_adapter/generation data 10)" || \
        fail "cannot read generation before service timeout"
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
      baseline="$(read_positive_topic_field /rog_map_adapter/generation data 10)" || \
        fail "cannot read generation before input stale"
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
      # 真实 unknown 必须来自 ROGMap 的数值状态：先用持续发布的零回波
      # 模拟 LiDAR 全遮挡，再由 ROGMap owner 清空既有观测。adapter 只在
      # 融合前遮蔽辅助证据，绝不从 debug PointCloud2 或融合后栅格伪造结果。
      local source_before source_after source_recovered publication_before publication_fault publication_after
      local localization_epoch request_identity zero_window_started zero_window_finished
      local unknown_header="/tmp/ats_p2_unknown_unk_header_${ROS_DOMAIN_ID}.out"
      local numeric_unknown="/tmp/ats_p2_unknown_numeric_${ROS_DOMAIN_ID}.out"
      UNKNOWN_TIMELINE="/tmp/ats_p2_unknown_timeline_${ROS_DOMAIN_ID}.log"
      : >"${UNKNOWN_TIMELINE}"
      publish_relative_fault_goal unknown
      wait_for_command "unknown action accepted" 8 \
        bash -c "grep -q 'Goal accepted' '${FAULT_ACTION_OUTPUT}'"
      request_identity="$(sed -n 's/^Goal accepted with ID: //p' "${FAULT_ACTION_OUTPUT}" | head -n 1)"
      [[ "${request_identity}" =~ ^[[:xdigit:]]{32}$ ]] || \
        fail "cannot capture unknown action request identity"
      source_before="$(read_rog_numeric_projection_generation \
        "/tmp/ats_p2_unknown_numeric_before_${ROS_DOMAIN_ID}.out")" || \
        fail "cannot capture healthy ROGMap numeric source generation"
      publication_before="$(read_topic_field /rog_map_adapter/status publication_sequence)"
      [[ "${publication_before}" =~ ^[0-9]+$ ]] || \
        fail "cannot capture adapter publication sequence before unknown fault"
      localization_epoch="$(read_topic_field /rog_map_adapter/status localization_epoch)"
      [[ "${localization_epoch}" =~ ^[0-9]+$ ]] || \
        fail "cannot capture localization epoch before unknown fault"
      record_unknown_timeline pre_fault_nonzero true
      record_unknown_timeline source_generation_before "${source_before}"
      record_unknown_timeline adapter_publication_sequence_before "${publication_before}"
      record_unknown_timeline localization_epoch "${localization_epoch}"
      record_unknown_timeline plan_request_identity "${request_identity}"
      record_unknown_timeline fault_injected_ns
      timeout 8 ros2 param set /ats_mujoco_sim lidar_occlusion_enabled true \
        >/tmp/ats_p2_fault_unknown_occlusion_enable.out 2>&1 || \
        fail "cannot enable MuJoCo LiDAR occlusion"
      timeout 8 ros2 param set /ats_rog_map test_reset_to_unknown true \
        >/tmp/ats_p2_fault_unknown_source_reset.out 2>&1 || \
        fail "cannot reset ROGMap source to unknown"
      wait_for_command "unknown fault keeps ROGMap input fresh" 12 \
        topic_field_equals /rog_map/stale data false
      wait_for_command "unknown fault obtains a ROGMap numeric unknown projection" 20 \
        wait_for_rog_numeric_unknown "${numeric_unknown}"
      source_after="${ROG_NUMERIC_UNKNOWN_GENERATION}"
      awk -v current="${source_after}" -v baseline="${source_before}" \
        'BEGIN {exit current > baseline ? 0 : 1}' || \
        fail "ROGMap source generation did not advance into the unknown fixture"
      record_unknown_timeline first_unknown_ns
      record_unknown_timeline source_generation_unknown "${source_after}"
      wait_for_command "unknown fault publishes non-empty ROGMap audit cloud" 12 \
        topic_field_positive /rog_map/unk width best_effort
      wait_for_command "unknown audit cloud frame stamp and QoS" 12 \
        verify_rog_unknown_visualization "${unknown_header}"
      timeout 8 ros2 param set /ats_rog_map_adapter test_mask_secondary_evidence true \
        >/tmp/ats_p2_fault_unknown_mask_enable.out 2>&1 || \
        fail "cannot enable adapter secondary-evidence mask"
      wait_for_command "unknown fault publishes all-unknown planning grid" 10 \
        timeout 6 python3 "${WORKSPACE_DIR}/scripts/query_occupancy_grid.py" \
        --topic /rc_esdf/planning_grid --timeout 4 all-value --value -1
      wait_for_command "unknown fault publishes adapter not-ready" 8 \
        topic_field_equals /rog_map_adapter/ready data false
      publication_fault="$(read_topic_field /rog_map_adapter/status publication_sequence)"
      [[ "${publication_fault}" =~ ^[0-9]+$ ]] || \
        fail "cannot capture adapter publication sequence during unknown fault"
      awk -v current="${publication_fault}" -v baseline="${publication_before}" \
        'BEGIN {exit current > baseline ? 0 : 1}' || \
        fail "adapter publication sequence did not advance into unknown fault"
      record_unknown_timeline ready_false_ns
      record_unknown_timeline adapter_publication_sequence_unknown "${publication_fault}"
      wait_for_fault_action_result unknown 4 12
      wait_for_command "unknown fault triggers emergency stop" 8 \
        topic_field_equals /planner/emergency_stop data true
      record_unknown_timeline emergency_stop_true_ns
      zero_window_started="$(date +%s%N)"
      sleep 0.5
      capture_numeric_stream /cmd_vel_mpc "/tmp/ats_p2_fault_unknown_cmd.out" || \
        fail "unknown fault did not publish /cmd_vel_mpc during stop"
      assert_zero_stream "unknown /cmd_vel_mpc" "/tmp/ats_p2_fault_unknown_cmd.out"
      record_unknown_timeline cmd_vel_zero_ns
      capture_numeric_stream /motion_control "/tmp/ats_p2_fault_unknown_motion.out" || \
        fail "unknown fault did not publish /motion_control during stop"
      assert_zero_stream "unknown /motion_control" "/tmp/ats_p2_fault_unknown_motion.out"
      record_unknown_timeline motion_control_zero_ns
      zero_window_finished="$(date +%s%N)"
      record_unknown_timeline zero_window_sec \
        "$(awk -v start="${zero_window_started}" -v end="${zero_window_finished}" 'BEGIN {printf "%.3f", (end - start) / 1e9}')"
      timeout 8 ros2 param set /ats_rog_map_adapter test_mask_secondary_evidence false \
        >/tmp/ats_p2_fault_unknown_mask_disable.out 2>&1 || \
        fail "cannot disable adapter secondary-evidence mask"
      timeout 8 ros2 param set /ats_rog_map test_reset_to_unknown false \
        >/tmp/ats_p2_fault_unknown_source_rearm.out 2>&1 || \
        fail "cannot rearm ROGMap source reset fixture"
      timeout 8 ros2 param set /ats_mujoco_sim lidar_occlusion_enabled false \
        >/tmp/ats_p2_fault_unknown_occlusion_disable.out 2>&1 || \
        fail "cannot restore MuJoCo LiDAR raycast"
      wait_for_command "unknown recovery restores adapter ready" 15 \
        topic_field_equals /rog_map_adapter/ready data true
      source_recovered="$(read_rog_numeric_projection_generation \
        "/tmp/ats_p2_unknown_numeric_recovery_${ROS_DOMAIN_ID}.out")" || \
        fail "cannot capture recovered ROGMap numeric source generation"
      awk -v current="${source_recovered}" -v baseline="${source_after}" \
        'BEGIN {exit current > baseline ? 0 : 1}' || \
        fail "ROGMap source generation did not advance after unknown recovery"
      publication_after="$(read_topic_field /rog_map_adapter/status publication_sequence)"
      [[ "${publication_after}" =~ ^[0-9]+$ ]] || \
        fail "cannot capture recovered adapter publication sequence"
      awk -v current="${publication_after}" -v baseline="${publication_before}" \
        'BEGIN {exit current > baseline ? 0 : 1}' || \
        fail "adapter publication sequence did not advance after unknown recovery"
      record_unknown_timeline source_generation_recovered "${source_recovered}"
      record_unknown_timeline adapter_publication_sequence_recovered "${publication_after}"
      assert_no_old_reference_revival \
        "/tmp/ats_p2_unknown_old_reference_${ROS_DOMAIN_ID}.out"
      capture_zero_outputs unknown_recovery
      # Recovery must not replay the failed request.  A new independent goal
      # is the only permitted way to re-establish planning and motion.
      publish_relative_fault_goal unknown_recovery_new_goal
      wait_for_fault_action_result unknown_recovery_new_goal 0 40
      echo "RESULT: unknown timeline=${UNKNOWN_TIMELINE} source=${source_before}->${source_after}->${source_recovered} publication=${publication_before}->${publication_after} request=${request_identity}"
      ;;
    unreachable)
      # 该查询会在大规划栅格上保守展开，先在静止状态找目标；随后立即用
      # free-unreachable action 抢占 tracking 任务，避免查询耗时让前置目标自然完成。
      find_unreachable_goal
      publish_relative_fault_goal unreachable_precondition
      log_start_line=$(( $(wc -l < "${LAUNCH_LOG}") + 1 ))
      send_fault_goal unreachable "${UNREACHABLE_GOAL_FRAME}" "${UNREACHABLE_GOAL_X}" "${UNREACHABLE_GOAL_Y}" 30
      wait_for_fault_action_result unreachable 5 12
      wait_for_command "unreachable goal triggers emergency stop" 10 \
        topic_field_equals /planner/emergency_stop data true
      wait_for_command "free unreachable goal is classified no-path" 8 \
        bash -c "tail -n +${log_start_line} '${LAUNCH_LOG}' | grep -q 'failed.*no path'"
      capture_zero_outputs unreachable
      ;;
    freeze)
      # 机器人保持静止，但 MuJoCo、LiDAR、里程计和 ROGMap 继续运行；这只
      # 覆盖健康输入下的真实无进展，不把输入 stale 混入同一故障用例。
      log_start_line=$(( $(wc -l < "${LAUNCH_LOG}") + 1 ))
      timeout 8 ros2 param set /ats_mujoco_sim freeze_motion true \
        >/tmp/ats_p2_fault_freeze_enable.out 2>&1 || \
        fail "cannot enable MuJoCo freeze_motion fault"
      wait_for_command "freeze fault keeps localization tracking" 8 \
        topic_field_equals /localization/status state 1
      wait_for_command "freeze fault keeps ROGMap inputs fresh" 8 \
        topic_field_equals /rog_map/stale data false
      wait_for_command "freeze fault keeps adapter ready" 8 \
        topic_field_equals /rog_map_adapter/ready data true
      baseline="$(read_positive_topic_field /rog_map_adapter/generation data 10)" || \
        fail "cannot read generation before freeze fault"
      publish_relative_fault_goal freeze
      wait_for_fault_action_result freeze 5 35
      wait_for_command "freeze fault triggers emergency stop" 8 \
        topic_field_equals /planner/emergency_stop data true
      wait_for_command "freeze fault logs bounded replan" 12 \
        bash -c "tail -n +${log_start_line} '${LAUNCH_LOG}' | grep -q 'Progress watchdog replan='"
      wait_for_command "freeze fault logs bounded exhaustion" 12 \
        bash -c "tail -n +${log_start_line} '${LAUNCH_LOG}' | grep -q 'progress watchdog exhausted bounded replans'"
      replan_count="$(tail -n +"${log_start_line}" "${LAUNCH_LOG}" | grep -c 'Progress watchdog replan=' || true)"
      [[ "${replan_count}" =~ ^[0-9]+$ && "${replan_count}" -ge 1 && "${replan_count}" -le 2 ]] || \
        fail "freeze fault exceeded bounded replan count: ${replan_count}"
      capture_zero_outputs freeze
      wait_for_generation_advance "${baseline}" 15
      # 没有新目标时，旧急停前 reference 不得恢复运动。
      capture_zero_outputs freeze_recovery
      echo "OK: freeze fault observed ${replan_count} bounded replans before terminal failure"
      ;;
  esac
  echo "PASS: independent P2 '${fault_case}' fault gate completed."
}

run_p3_action_fault_injection() {
  local fault_case="$1"
  local pose_file="/tmp/ats_p3_fault_${fault_case}_pose.out"
  local current_x current_y first_x second_x cancel_x
  capture_pose "${pose_file}" || fail "cannot capture pose for P3 ${fault_case} fault"
  current_x="$(pose_axis "${pose_file}" x)"
  current_y="$(pose_axis "${pose_file}" y)"
  # 选择 nominal single 已验证的西侧空旷区，确保 timeout/preempt 先进入 tracking，
  # 而不是被碰撞 gate 提前归类为 planning failure。
  first_x="$(awk -v x="${current_x}" 'BEGIN {printf "%.6f", x - 0.90}')"
  second_x="$(awk -v x="${current_x}" 'BEGIN {printf "%.6f", x - 0.45}')"
  # 取消用例需覆盖 Humble 独立 service 客户端的 DDS 发现时间，因此沿 nominal
  # 已通过的返程自由走廊使用更长目标，避免在 CancelGoal 到达前自然成功。
  cancel_x="$(awk -v x="${current_x}" 'BEGIN {printf "%.6f", x - 1.80}')"

  case "${fault_case}" in
    cancel)
      send_fault_goal cancel odom "${cancel_x}" "${current_y}" 30
      wait_for_command "cancel action accepted" 8 \
        bash -c "grep -q 'Goal accepted' '${FAULT_ACTION_OUTPUT}'"
      # Humble 的 ros2 action CLI 没有 cancel 子命令。零 UUID 的“取消全部”在
      # 本环境可能返回空目标列表，因此从客户端输出提取已接受目标的 UUID 并精确取消。
      local cancel_goal_id cancel_goal_uuid cancel_byte cancel_index
      cancel_goal_id="$(sed -n 's/^Goal accepted with ID: //p' "${FAULT_ACTION_OUTPUT}" | head -n 1)"
      [[ "${cancel_goal_id}" =~ ^[[:xdigit:]]{32}$ ]] || \
        fail "cannot parse the accepted ATS action UUID for cancellation"
      cancel_goal_uuid=""
      for ((cancel_index = 0; cancel_index < 32; cancel_index += 2)); do
        cancel_byte="$((16#${cancel_goal_id:cancel_index:2}))"
        cancel_goal_uuid+="${cancel_goal_uuid:+, }${cancel_byte}"
      done
      timeout 8 ros2 service call /ats_navigate_to_pose/_action/cancel_goal \
        action_msgs/srv/CancelGoal \
        "{goal_info: {goal_id: {uuid: [${cancel_goal_uuid}]}, stamp: {sec: 0, nanosec: 0}}}" \
        >/tmp/ats_p3_fault_cancel_request.out 2>&1 || \
        fail "cannot request ATS action cancellation"
      grep -q 'goals_canceling=\[' /tmp/ats_p3_fault_cancel_request.out && \
        ! grep -q 'goals_canceling=\[\]' /tmp/ats_p3_fault_cancel_request.out || \
        fail "ATS action cancel request did not return a cancellation acknowledgement"
      wait_for_fault_action_result cancel 1 12
      wait_for_command "cancel triggers emergency stop" 8 \
        topic_field_equals /planner/emergency_stop data true
      capture_zero_outputs cancel
      sleep 2
      capture_zero_outputs cancel_recovery
      ;;
    timeout)
      send_fault_goal timeout odom "${first_x}" "${current_y}" 1
      wait_for_fault_action_result timeout 3 12
      wait_for_command "action timeout triggers emergency stop" 8 \
        topic_field_equals /planner/emergency_stop data true
      capture_zero_outputs timeout
      sleep 2
      capture_zero_outputs timeout_recovery
      ;;
    tf_failure)
      send_fault_goal tf_failure p3_missing_goal_frame "${first_x}" "${current_y}" 10
      wait_for_fault_action_result tf_failure 6 12
      wait_for_command "TF failure triggers emergency stop" 8 \
        topic_field_equals /planner/emergency_stop data true
      capture_zero_outputs tf_failure
      ;;
    preempt)
      # 第二个独立 CLI 客户端需要 DDS 发现时间；使用较长返程目标确保其到达
      # action server 时第一个任务仍处于 tracking，从而实际覆盖 preempt 路径。
      send_fault_goal preempt_first odom "${cancel_x}" "${current_y}" 30
      local first_output="${FAULT_ACTION_OUTPUT}"
      wait_for_command "preempt first action accepted" 8 \
        bash -c "grep -q 'Goal accepted' '${first_output}'"
      sleep 1
      send_fault_goal preempt_second odom "${second_x}" "${current_y}" 1
      wait_for_command "preempted action result" 12 \
        bash -c "grep -q 'result_code: 2' '${first_output}' && grep -q 'Goal finished with status:' '${first_output}'"
      wait_for_fault_action_result preempt_second 3 12
      wait_for_command "preempt chain ends in emergency stop" 8 \
        topic_field_equals /planner/emergency_stop data true
      capture_zero_outputs preempt
      sleep 2
      capture_zero_outputs preempt_recovery
      ;;
  esac
  echo "PASS: independent P3 action '${fault_case}' fault gate completed."
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

report_execution_yaw_authority() {
  local output_file="$1"
  awk '
    function emit() {
      if (mode == 1 && yaw != "") {
        printf "RESULT: execute_yaw_authority=%s requires_gimbal_lock=%s request_sequence=%s feedback_sequence=%s\n", \
          yaw, locked, request, feedback
      }
    }
    $1 == "mode:" {mode = $2}
    $1 == "yaw_authority:" {yaw = $2}
    $1 == "requires_gimbal_lock:" {locked = $2}
    $1 == "gimbal_request_sequence:" {request = $2}
    $1 == "gimbal_feedback_sequence:" {feedback = $2}
    $1 == "---" {emit(); mode = ""; yaw = ""; locked = ""; request = ""; feedback = ""}
    END {emit()}
  ' "${output_file}" | tail -n 1
}

assert_execution_yaw_authority() {
  local output_file="$1"
  local expected="$2"
  local expected_value expected_lock
  case "${expected}" in
    gimbal)
      expected_value=1
      expected_lock=false
      ;;
    body)
      expected_value=2
      expected_lock=true
      ;;
    *)
      report_execution_yaw_authority "${output_file}"
      return
      ;;
  esac
  if ! awk -v expected="${expected_value}" -v expected_lock="${expected_lock}" '
    function matches() {
      return mode == 1 && yaw == expected && locked == expected_lock &&
        request ~ /^[1-9][0-9]*$/ && feedback ~ /^[1-9][0-9]*$/
    }
    $1 == "mode:" {mode = $2}
    $1 == "yaw_authority:" {yaw = $2}
    $1 == "requires_gimbal_lock:" {locked = $2}
    $1 == "gimbal_request_sequence:" {request = $2}
    $1 == "gimbal_feedback_sequence:" {feedback = $2}
    $1 == "---" {
      if (matches()) found = 1
      mode = ""; yaw = ""; locked = ""; request = ""; feedback = ""
    }
    END {if (matches()) found = 1; exit found ? 0 : 1}
  ' "${output_file}"; then
    fail "no EXECUTE lease matched yaw authority '${expected}' with a fresh gimbal acknowledgement"
  fi
  report_execution_yaw_authority "${output_file}"
  echo "OK: execute lease and gimbal acknowledgement match '${expected}' authority"
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

# [Dead Code Suggestion] 旧 /goal_pose topic fallback 的到达检查。
# P3 action-only 回归只由 action result 驱动；待 P3 运行验收完成后再单独清理。
wait_for_pose_near_goal() {
  local label="$1"
  local goal_x="$2"
  local goal_y="$3"
  local output_file="$4"
  local deadline=$((SECONDS + GOAL_TIMEOUT))
  local final_x final_y error
  while (( SECONDS < deadline )); do
    if capture_pose "${output_file}"; then
      final_x="$(pose_axis "${output_file}" x)"
      final_y="$(pose_axis "${output_file}" y)"
      error="$(awk -v gx="${goal_x}" -v gy="${goal_y}" -v fx="${final_x}" -v fy="${final_y}" '
        BEGIN {printf "%.6f", sqrt((fx - gx) * (fx - gx) + (fy - gy) * (fy - gy))}
      ')"
      if awk -v error="${error}" -v tolerance="${GOAL_TOLERANCE}" \
        'BEGIN {exit error <= tolerance ? 0 : 1}'
      then
        echo "RESULT: ${label} final=(${final_x}, ${final_y}) goal=(${goal_x}, ${goal_y}) error=${error} m"
        return 0
      fi
    fi
    sleep 1
  done
  fail "${label} did not reach its goal before timeout"
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
  while (( SECONDS < deadline )); do
    if [[ -s "${output_file}" ]] && \
      awk '/^[[:space:]]*-[[:space:]]+header:$/ {found = 1} END {exit found ? 0 : 1}' \
        "${output_file}"
    then
      stop_capture_process "${pid}"
      assert_path_has_poses "${label}" "${output_file}"
      return
    fi
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      break
    fi
    sleep 0.2
  done
  stop_capture_process "${pid}"
  fail "${label} did not publish a non-empty path after the goal"
}

start_topic_capture() {
  local topic="$1"
  local message_type="$2"
  local output_file="$3"
  : >"${output_file}"
  # The event-driven diagnostic and authorization topics must have a live
  # observer before the next execute cycle is published. Providing the type
  # avoids ros2cli graph discovery being the condition for creating it.
  timeout "${GOAL_TIMEOUT}" ros2 topic echo --no-daemon --qos-reliability reliable "${topic}" \
    "${message_type}" \
    >"${output_file}" 2>/dev/null &
  STARTED_CAPTURE_PID=$!
}

ensure_topic_capture_ready() {
  local label="$1"
  local topic="$2"
  local message_type="$3"
  local output_file="$4"
  local pid_variable="$5"
  local attempt
  for attempt in 1 2 3; do
    start_topic_capture "${topic}" "${message_type}" "${output_file}"
    # ros2cli can lose a one-shot graph discovery race even when the producer
    # was verified earlier. Detect that exit before issuing the goal; a retry
    # after the event-driven volatile publication would be too late.
    sleep 1
    if kill -0 "${STARTED_CAPTURE_PID}" 2>/dev/null; then
      printf -v "${pid_variable}" '%s' "${STARTED_CAPTURE_PID}"
      return 0
    fi
    wait "${STARTED_CAPTURE_PID}" 2>/dev/null || true
    sleep 1
  done
  fail "${label} topic capture could not remain subscribed before the goal"
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
  local topic output_file pid log_line_count log_start_line telemetry_sequence goal_send_stamp
  local -a topic_pids=()

  log_line_count="$(wc -l < "${LAUNCH_LOG}")"
  log_start_line=$((log_line_count + 1))
  capture_pose "${before_pose}" || fail "cannot capture pose before ${name}"
  telemetry_sequence="$(read_swerve_telemetry_sequence)"
  [[ "${telemetry_sequence}" =~ ^[0-9]+$ ]] || \
    fail "cannot capture /swerve/telemetry sequence before ${name}"
  local -a path_topics=(/minco/raw_path /minco/reference_path)
  for topic in "${path_topics[@]}"; do
    output_file="${prefix}_${topic//\//_}.out"
    ensure_topic_capture_ready "${name} ${topic}" "${topic}" nav_msgs/msg/Path \
      "${output_file}" pid
    CAPTURE_PIDS+=("${pid}")
    topic_pids+=("${pid}:${topic}:${output_file}")
  done
  timeout "${GOAL_TIMEOUT}" ros2 topic echo --no-daemon /cmd_vel_mpc >"${command_output}" 2>/dev/null &
  local leg_command_pid=$!
  CAPTURE_PIDS+=("${leg_command_pid}")

  # MINCO 路径为事件触发发布；先等待 DDS 单次订阅发现完成，
  # 避免目标触发后多个路径话题瞬时发布而被测试遗漏。
  sleep 2

  goal_send_stamp="$(date +%s.%N)"
  LAST_GOAL_SEND_EPOCH_SEC="${goal_send_stamp%%.*}"
  LAST_GOAL_SEND_EPOCH_NANOSEC="${goal_send_stamp##*.}"
  LAST_GOAL_TELEMETRY_SEQUENCE="${telemetry_sequence}"
  timeout "${GOAL_TIMEOUT}" ros2 action send_goal --feedback /ats_navigate_to_pose \
    ats_navigation_interfaces/action/NavigateToPose \
    "{goal_pose: {header: {frame_id: ${P3_GOAL_FRAME}}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: 0.0}, orientation: {w: ${GOAL_YAW_W}}}}, timeout: {sec: ${GOAL_TIMEOUT}, nanosec: 0}}" \
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
    fail "${name} ATS action did not finish before timeout"
  fi
  wait "${GOAL_PID}" 2>/dev/null || true
  unset GOAL_PID
  cat "${goal_output}"
  grep -q 'Goal accepted' "${goal_output}" || fail "${name} ATS action was not accepted"
  grep -q 'Goal finished with status: SUCCEEDED' "${goal_output}" || \
    fail "${name} ATS action did not succeed"
  grep -q 'Feedback:' "${goal_output}" || fail "${name} ATS action did not return feedback"
  echo "OK: ${name} ATS Navigate action returned feedback and SUCCEEDED"

  for capture in "${topic_pids[@]}"; do
    pid="${capture%%:*}"
    capture="${capture#*:}"
    topic="${capture%%:*}"
    output_file="${capture#*:}"
    wait_for_capture "${name} ${topic}" "${pid}" "${output_file}"
  done
  assert_minco_plan_record "${name}" "${log_start_line}"

  sleep 0.5
  stop_capture_process "${leg_command_pid}"
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
  use_viewer:=false
  show_viewer:=false
  use_rviz:="${USE_RVIZ}"
  launch_mujoco_rviz:=false
  force_body_yaw_follow:="${FORCE_BODY_YAW_FOLLOW}"
  body_yaw_follow_clearance:="${BODY_YAW_FOLLOW_CLEARANCE}"
  enable_lidar:=true
  lidar_backend:=cpu
  lidar_downsample:="${LIDAR_DOWNSAMPLE}"
  enable_tof:=false
  freeze_motion:=false
  start_x:="${START_X}"
  start_y:="${START_Y}"
  start_z:="${START_Z}"
  start_yaw:="${START_YAW}"
  nav_start_delay_sec:=9.0
  rog_map_start_delay_sec:=15.0
  map_start_delay_sec:=2.0
  rviz_delay_sec:="${RVIZ_DELAY_SEC}"
  planning_grid_owner:="${PLANNING_GRID_OWNER}"
  solver_mode:="${SOLVER_MODE}"
  telemetry_sampling_window_cycles:="${QP_TELEMETRY_WINDOW_CYCLES}"
  log_level:="${LOG_LEVEL}"
)

setsid ros2 launch "${LAUNCH_ARGS[@]}" >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

wait_for_command "node graph" 60 timeout 4 ros2 node list --no-daemon
wait_for_topic_once /localization 70 best_effort
wait_for_topic_once /odometry 30 best_effort
wait_for_topic_once /localization/status 30
wait_for_command "localization tracking" 30 \
  topic_field_equals /localization/status state 1
wait_for_command "MuJoCo map->odom disabled" 10 \
  timeout --kill-after=1 4 bash -c \
  "ros2 param get --no-daemon /ats_mujoco_sim publish_map_to_odom_tf | grep -q 'Boolean value is: False'"
wait_for_command "fusion map->odom enabled" 10 \
  timeout --kill-after=1 4 bash -c \
  "ros2 param get --no-daemon /localization_fusion publish_tf | grep -q 'Boolean value is: True'"
assert_topic_ownership /odometry ats_mujoco_sim localization_fusion
assert_topic_ownership /localization localization_fusion
assert_topic_ownership /localization/status localization_fusion
echo "OK: /odometry -> fusion -> /localization contract is active"
wait_for_topic_once /traversability_grid 120
wait_for_topic_once /rc_esdf/planning_grid 120
assert_p3_process_graph

verify_rog_map_planning_interface

wait_for_command "minco_planner node" 30 node_is_present /minco_planner
wait_for_command "ats_swerve_mpc node" 30 node_is_present /ats_swerve_mpc
wait_for_command "twist bridge node" 30 node_is_present /twist_to_motion_ctrl
NODE_LIST="$(ros2 node list --no-daemon)"
if grep -q '^/fake_vel_transform$' <<<"${NODE_LIST}"; then
  fail "fake_vel_transform must be disabled in swerve MPC mode"
fi
echo "OK: MPC nodes present and fake_vel_transform absent"

assert_topic_ownership /cmd_vel_mpc ats_swerve_mpc twist_to_motion_ctrl
assert_topic_ownership /motion_control twist_to_motion_ctrl ats_mujoco_sim
assert_topic_ownership /ats_goal_manager/planner_goal ats_goal_manager minco_planner
# RViz may observe the reference in the UI, but it must never gain execution
# authority.  The assertion still accepts exactly one controller and one named
# display subscriber; command topics below remain strictly single-subscriber.
if [[ "${USE_RVIZ}" == "true" ]]; then
  assert_topic_ownership /minco/reference_path ats_goal_manager ats_swerve_mpc mujoco_navigation_rviz2
else
  assert_topic_ownership /minco/reference_path ats_goal_manager ats_swerve_mpc
fi
assert_topic_ownership /planner/execution_command ats_goal_manager ats_swerve_mpc
# MPC 清 tracker，MuJoCo 执行器做硬零速；安全信号允许多个 consumer，但只有
# Goal Manager 可以发布最终急停权威。
assert_topic_ownership /planner/emergency_stop ats_goal_manager
wait_for_topic_once /gimbal/yaw_status 20
wait_for_topic_once /swerve/telemetry 20

declare -a DEBUG_TOPICS=(
  /ats_swerve_mpc/reference_horizon
  /ats_swerve_mpc/predicted_path
)
declare -a DEBUG_CAPTURE_PIDS=()
for topic in "${DEBUG_TOPICS[@]}"; do
  output_file="/tmp/ats_minco_mpc_${topic//\//_}.out"
  # Both debug topics have transient-local QoS and initially publish an empty
  # path while MPC is fail-stopped.  Keep the capture open through the goal so
  # an initial empty latched sample cannot hide the non-empty tracking output.
  ensure_topic_capture_ready "MPC debug ${topic}" "${topic}" nav_msgs/msg/Path \
    "${output_file}" debug_pid
  DEBUG_CAPTURE_PIDS+=("${debug_pid}")
  CAPTURE_PIDS+=("${debug_pid}")
done
ensure_topic_capture_ready "MPC command stream" /cmd_vel_mpc geometry_msgs/msg/Twist \
  /tmp/ats_minco_mpc_cmd_vel_stream.out CMD_STREAM_PID
CAPTURE_PIDS+=("${CMD_STREAM_PID}")
ensure_topic_capture_ready "Motion control stream" /motion_control manda_can_control/msg/MotionCtrl \
  /tmp/ats_minco_mpc_motion_stream.out MOTION_STREAM_PID
CAPTURE_PIDS+=("${MOTION_STREAM_PID}")
EXECUTION_STREAM="/tmp/ats_minco_mpc_${TEST_PROFILE}_${ROS_DOMAIN_ID}_execution_command.out"
RVIZ_QOS_LOG="/tmp/ats_minco_mpc_${TEST_PROFILE}_${ROS_DOMAIN_ID}_rviz_qos.out"
RVIZ_SCREENSHOT="/tmp/ats_minco_mpc_${TEST_PROFILE}_${ROS_DOMAIN_ID}_rviz.png"
assert_rviz_runtime_contract
capture_rviz_screenshot
ensure_topic_capture_ready "P3 ExecutionCommand" /planner/execution_command \
  ats_navigation_interfaces/msg/ExecutionCommand "${EXECUTION_STREAM}" EXECUTION_STREAM_PID
CAPTURE_PIDS+=("${EXECUTION_STREAM_PID}")

for index in "${!GOAL_NAMES[@]}"; do
  echo "RUN: ${TEST_PROFILE} goal $((index + 1))/${#GOAL_NAMES[@]} '${GOAL_NAMES[index]}' -> " \
    "(${GOAL_XS[index]}, ${GOAL_YS[index]})"
  run_navigation_goal "${index}"
done

sleep 1
stop_capture_process "${CMD_STREAM_PID}"
stop_capture_process "${MOTION_STREAM_PID}"
stop_capture_process "${EXECUTION_STREAM_PID}"
[[ -s "${EXECUTION_STREAM}" ]] || fail "ExecutionCommand did not publish during P3 run"
assert_execution_yaw_authority "${EXECUTION_STREAM}" "${YAW_AUTHORITY_EXPECTED}"
if [[ "${YAW_AUTHORITY_EXPECTED}" == "body" ]]; then
  wait_for_command "BODY_YAW_FOLLOW gimbal lock acknowledgement" 10 \
    topic_field_equals /gimbal/yaw_status locked true
fi

for index in "${!DEBUG_TOPICS[@]}"; do
  topic="${DEBUG_TOPICS[index]}"
  debug_pid="${DEBUG_CAPTURE_PIDS[index]}"
  stop_capture_process "${debug_pid}"
  output_file="/tmp/ats_minco_mpc_${topic//\//_}.out"
  [[ -s "${output_file}" ]] || fail "${topic} did not publish after the goal"
  assert_path_has_poses "${topic}" "${output_file}"
done
assert_nonzero_stream /cmd_vel_mpc /tmp/ats_minco_mpc_cmd_vel_stream.out
assert_nonzero_stream /motion_control /tmp/ats_minco_mpc_motion_stream.out
assert_final_swerve_telemetry \
  "/tmp/ats_minco_mpc_${TEST_PROFILE}_${ROS_DOMAIN_ID}_final_swerve_telemetry.out" \
  "${LAST_GOAL_TELEMETRY_SEQUENCE}" \
  "${LAST_GOAL_SEND_EPOCH_SEC}" \
  "${LAST_GOAL_SEND_EPOCH_NANOSEC}"

if [[ -n "${QP_TELEMETRY_OUTPUT}" ]]; then
  [[ -n "${QP_TELEMETRY_MANIFEST}" ]] || \
    fail "QP_TELEMETRY_OUTPUT requires QP_TELEMETRY_MANIFEST"
  python3 "${WORKSPACE_DIR}/scripts/dump_ats_swerve_mpc_telemetry.py" \
    --output "${QP_TELEMETRY_OUTPUT}" \
    --manifest "${QP_TELEMETRY_MANIFEST}" \
    --workspace "${WORKSPACE_DIR}" \
    --solver-mode "${SOLVER_MODE}" \
    --log-level "${LOG_LEVEL}" \
    --test-profile "${TEST_PROFILE}" \
    --planning-grid-owner "${PLANNING_GRID_OWNER}" \
    --p2-fault-case "${P2_FAULT_CASE}" \
    --p3-fault-case "${P3_FAULT_CASE}" \
    --ros-domain-id "${ROS_DOMAIN_ID}" \
    --params-file "${WORKSPACE_DIR}/src/ats_sentry_bringup/params/node_params.yaml" \
    --start-x "${START_X}" --start-y "${START_Y}" \
    --start-z "${START_Z}" --start-yaw "${START_YAW}" \
    --goal-x "${GOAL_X}" --goal-y "${GOAL_Y}" \
    --goal-yaw-w "${GOAL_YAW_W}" \
    --run-start-epoch-ns "${QP_TELEMETRY_RUN_START_EPOCH_NS}" \
    --sampling-window-cycles "${QP_TELEMETRY_WINDOW_CYCLES}" || \
    fail "cannot export /ats_swerve_mpc/dump_control_telemetry"
  [[ -s "${QP_TELEMETRY_OUTPUT}" ]] || fail "control telemetry output is empty"
  [[ -s "${QP_TELEMETRY_MANIFEST}" ]] || fail "control telemetry manifest is empty"
  echo "OK: exported control telemetry to ${QP_TELEMETRY_OUTPUT}"
fi

wait_for_generation_advance "${P2_LAST_GENERATION}" 30
if [[ "${P2_FAULT_CASE}" != "none" ]]; then
  run_p2_fault_injection "${P2_FAULT_CASE}"
fi
if [[ "${P3_FAULT_CASE}" != "none" ]]; then
  run_p3_action_fault_injection "${P3_FAULT_CASE}"
fi

echo "PASS: MuJoCo JPS/MINCO/clearance-aware yaw/SE2 MPC '${TEST_PROFILE}' profile completed."
