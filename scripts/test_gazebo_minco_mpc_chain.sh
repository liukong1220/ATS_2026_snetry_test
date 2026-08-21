#!/usr/bin/env bash
# Gazebo -> ATS navigation chain regression driver.
#
# The chain under test is:
#   Gazebo (SwerveDrive4WS chassis, mid360 gpu_lidar + imu, /clock)
#     -> ros_gz_bridge -> gz_livox_bridge -> point_lio -> loam_interface
#     -> sensor_scan_generation -> localization_fusion (/localization)
#     -> ats_rog_map -> ats_rog_map_adapter (/rc_esdf/planning_grid)
#     -> minco_planner (JPS + MINCO) -> ats_swerve_mpc (/cmd_vel_mpc)
#     -> velocity transform -> lower-controller velocity boundary
#
# The script records evidence; it does not decide "pass" from the mere presence
# of a topic or from a successful launch. Every metric that could not be
# measured is written as "unverified" rather than defaulted to a passing value.
#
# Usage:
#   ROS_DOMAIN_ID=91 PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
#   TEST_PROFILE=nominal WORLD=rmuc_2025 \
#   MAP_YAML=src/ats_sentry_bringup/map/rmuc_2025.yaml \
#   USE_RVIZ=false USE_VIEWER=false \
#   scripts/test_gazebo_minco_mpc_chain.sh

set -u -o pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"
# shellcheck source=scripts/gazebo_freshness_classifier.sh
source "$WORKSPACE_ROOT/scripts/gazebo_freshness_classifier.sh"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
PLANNING_GRID_OWNER="${PLANNING_GRID_OWNER:-rog_map}"
P2_FAULT_CASE="${P2_FAULT_CASE:-none}"
TEST_PROFILE="${TEST_PROFILE:-nominal}"
USE_RVIZ="${USE_RVIZ:-false}"
USE_VIEWER="${USE_VIEWER:-false}"
HEADLESS="${HEADLESS:-true}"
HEADLESS_RENDERING="${HEADLESS_RENDERING:-true}"
ENABLE_CAMERA_SENSORS="${ENABLE_CAMERA_SENSORS:-false}"
LIVOX_UPDATE_RATE_HZ="${LIVOX_UPDATE_RATE_HZ:-10.0}"
LIVOX_HORIZONTAL_SAMPLES="${LIVOX_HORIZONTAL_SAMPLES:-625}"
OBSERVE_GAZEBO_TRANSPORT_LIDAR="${OBSERVE_GAZEBO_TRANSPORT_LIDAR:-false}"
USE_DIRECT_GAZEBO_LIDAR_BRIDGE="${USE_DIRECT_GAZEBO_LIDAR_BRIDGE:-false}"
LIDAR_BRIDGE_PUBLISHER_DEPTH="${LIDAR_BRIDGE_PUBLISHER_DEPTH:-10}"
LIDAR_BRIDGE_PUBLISHER_RELIABILITY="${LIDAR_BRIDGE_PUBLISHER_RELIABILITY:-reliable}"
WORLD="${WORLD:-rmuc_2025}"
WORLD_SDF_PATH="${WORLD_SDF_PATH:-}"
MAP_YAML="${MAP_YAML:-src/ats_sentry_bringup/map/rmuc_2025.yaml}"
ROBOT_NAME="${ROBOT_NAME:-red_standard_robot1}"
SOLVER_MODE="${SOLVER_MODE:-ilqr}"

# Discovery is pinned to the loopback interface. Both DDS and ign-transport pick
# a non-loopback NIC by default; on a host whose wired NIC is NO-CARRIER every
# discovery datagram fails with "Network is unreachable" and no topic is ever
# matched, which looks like a dead simulator rather than a network fault. The
# whole chain runs in one machine, so loopback is also the correct scope.
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
IGN_IP="${IGN_IP:-127.0.0.1}"
# ros2cli's daemon binds its first domain at process start. A regression run
# must not query an old daemon from another ROS_DOMAIN_ID, otherwise topic list
# can be stale while echo/action/service calls silently inspect the wrong graph.
ROS2CLI_DAEMON="${ROS2CLI_DAEMON:-false}"
export ROS_LOCALHOST_ONLY IGN_IP ROS2CLI_DAEMON

# Timings. Gazebo + Point-LIO need a long warm-up before the planning map is
# usable, so the defaults are generous rather than optimistic.
STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-120}"
RUN_DURATION_SEC="${RUN_DURATION_SEC:-90}"
FAULT_INJECTION_DELAY_SEC="${FAULT_INJECTION_DELAY_SEC:-$((RUN_DURATION_SEC / 3))}"
SHUTDOWN_GRACE_SEC="${SHUTDOWN_GRACE_SEC:-10}"
SAMPLE_SEC="${SAMPLE_SEC:-5}"
COUNT_EXTRA_TIMEOUT_SEC="${COUNT_EXTRA_TIMEOUT_SEC:-10}"
# The graph is only a discovery aid.  A long ros2cli spin sends a fresh DDS
# participant into a resource-constrained simulation, so retain just enough
# time for loopback discovery and cache the result per observation phase.
GRAPH_SPIN_TIME_SEC="${GRAPH_SPIN_TIME_SEC:-1}"

GOAL_X="${GOAL_X:-}"
GOAL_Y="${GOAL_Y:-}"
GOAL_YAW="${GOAL_YAW:-0.0}"
GOAL_FRAME="${GOAL_FRAME:-map}"
GOAL_TIMEOUT_SEC="${GOAL_TIMEOUT_SEC:-90}"
ACTION_SERVER_TIMEOUT_SEC="${ACTION_SERVER_TIMEOUT_SEC:-30}"
GOAL_RESULT_WAIT_SEC="${GOAL_RESULT_WAIT_SEC:-$GOAL_TIMEOUT_SEC}"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_ROOT="${LOG_ROOT:-$WORKSPACE_ROOT/log/gazebo_minco_mpc_chain}"
RUN_DIR="$LOG_ROOT/${STAMP}_${TEST_PROFILE}_${P2_FAULT_CASE}_domain${ROS_DOMAIN_ID}"
mkdir -p "$RUN_DIR"

SUMMARY="$RUN_DIR/summary.txt"
LAUNCH_LOG="$RUN_DIR/launch.log"
TOPIC_LOG="$RUN_DIR/topics.log"
TOPIC_CACHE="$RUN_DIR/topic_cache.txt"
METRIC_LOG="$RUN_DIR/metrics.log"
CAPTURE_RVIZ_SCREENSHOT="${CAPTURE_RVIZ_SCREENSHOT:-false}"
RVIZ_CAPTURE_TRACKING_TIMEOUT_SEC="${RVIZ_CAPTURE_TRACKING_TIMEOUT_SEC:-20}"
RVIZ_CAPTURE_WINDOW_TIMEOUT_SEC="${RVIZ_CAPTURE_WINDOW_TIMEOUT_SEC:-10}"
RVIZ_CAPTURE_SETTLE_SEC="${RVIZ_CAPTURE_SETTLE_SEC:-1.5}"
RVIZ_WINDOW_TITLE="${RVIZ_WINDOW_TITLE:-ATS Gazebo Navigation - RViz}"
RVIZ_CAPTURE_X="${RVIZ_CAPTURE_X:-60}"
RVIZ_CAPTURE_Y="${RVIZ_CAPTURE_Y:-60}"
RVIZ_CAPTURE_WIDTH="${RVIZ_CAPTURE_WIDTH:-1720}"
RVIZ_CAPTURE_HEIGHT="${RVIZ_CAPTURE_HEIGHT:-1000}"
RVIZ_SCREENSHOT="$RUN_DIR/rviz_navigation_active.png"
RVIZ_SCREENSHOT_XWD="$RUN_DIR/rviz_navigation_active.xwd"

export ROS_DOMAIN_ID

FIRST_FAILURE=""
FAILURE_COUNT=0
GOAL_PID=""
GOAL_SESSION_ID=""
GOAL_OUTPUT="$RUN_DIR/goal_action.log"
GOAL_ERROR="$RUN_DIR/goal_action.err"
STOPPED_NODE_PIDS=""
ACTIVE_OBSERVER_WINDOW_SEC="${ACTIVE_OBSERVER_WINDOW_SEC:-$RUN_DURATION_SEC}"
ACTIVE_EVIDENCE_DURATION_SEC="$(
  awk -v value="$ACTIVE_OBSERVER_WINDOW_SEC" 'BEGIN {printf "%.6f", value + 0.0}'
)"
ACTIVE_OBSERVER_PIDS=()
ACTIVE_EVIDENCE_LOG="$RUN_DIR/active_navigation_evidence.log"
ACTIVE_EVIDENCE_PID=""
ACTIVE_EVIDENCE_SESSION_ID=""
ACTIVE_EVIDENCE_DEADLINE_SEC=""
ACTIVE_OWNERSHIP_LOG="$RUN_DIR/active_ownership.log"
PROCESS_RESOURCE_LOG="$RUN_DIR/launch_process_resources.log"
P1_ADMISSION_EVIDENCE="false"
P1_ADMISSION_REASON="not_evaluated"
RUNTIME_PREFLIGHT_LOG="$RUN_DIR/runtime_preflight.txt"
HEALTH_PROBE_LOG="$RUN_DIR/navigation_health_probe.log"
RECOVERY_CANCEL_LOG="$RUN_DIR/recovery_cancel_on_command.log"
RECOVERY_CANCEL_RESULT=""

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$SUMMARY"; }
metric() { printf '%-42s %s\n' "$1" "$2" | tee -a "$METRIC_LOG" >>"$SUMMARY"; }

fail() {
  FAILURE_COUNT=$((FAILURE_COUNT + 1))
  if [ -z "$FIRST_FAILURE" ]; then FIRST_FAILURE="$1"; fi
  log "FAIL: $1"
}

run_runtime_preflight() {
  local residual_processes
  if ! [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || [ "$ROS_DOMAIN_ID" -gt 232 ]; then
    fail "runtime_invalid_ros_domain_${ROS_DOMAIN_ID}"
    return 1
  fi
  residual_processes="$(ps -eo pid=,comm=,args= | awk '
    $2 ~ /^(ign|gazebo|gzserver|gzclient|mujoco|pointlio|loam_interface|sensor_scan_gene|localization_fu|ats_rog_map|ats_navigation|gz_livox_bridge|gz_clock_relay|gz_chassis_cmd)/ {print}
  ')"
  {
    date --iso-8601=seconds
    printf 'ros_domain=%s\n' "$ROS_DOMAIN_ID"
    printf 'root_head=%s\n' "$(git rev-parse HEAD)"
    printf 'gazebo_head=%s\n' "$(git -C src/sim/gazebo_simulator rev-parse HEAD)"
    if [ -n "$residual_processes" ]; then
      printf 'residual_navigation_or_simulator_processes=detected\n'
      printf '%s\n' "$residual_processes"
    else
      printf 'residual_navigation_or_simulator_processes=none_detected\n'
    fi
  } | tee "$RUNTIME_PREFLIGHT_LOG" >>"$SUMMARY"
  metric "runtime_preflight_artifact" "$RUNTIME_PREFLIGHT_LOG"
  metric "runtime_preflight_residual_processes" \
    "$( [ -n "$residual_processes" ] && printf detected || printf none_detected )"
  if [ -n "$residual_processes" ]; then
    fail "runtime_residual_navigation_or_simulator_process"
    return 1
  fi
}

# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------

if ! run_runtime_preflight; then
  exit 3
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not on PATH; source the ROS 2 and workspace setup first." >&2
  exit 2
fi

# The regression is normally launched from a shell that already sourced the
# overlay.  Make the checked-out workspace self-contained when the underlay is
# present but the local package index is not, while still failing explicitly
# instead of starting a dead launch session.
if ! ros2 pkg prefix rmu_gazebo_simulator >/dev/null 2>&1 && \
   [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u
  source "$WORKSPACE_ROOT/install/setup.bash"
  set -u
fi
if ! ros2 pkg prefix rmu_gazebo_simulator >/dev/null 2>&1; then
  echo "rmu_gazebo_simulator is not available; build and source install/setup.bash." >&2
  exit 2
fi

if [ ! -f "$MAP_YAML" ]; then
  # Fall back to the installed share path so the script also works after a
  # colcon install without a source checkout in the current directory.
  INSTALLED_MAP="$(ros2 pkg prefix ats_sentry_bringup 2>/dev/null || true)"
  if [ -n "$INSTALLED_MAP" ] && \
     [ -f "$INSTALLED_MAP/share/ats_sentry_bringup/map/$(basename "$MAP_YAML")" ]; then
    MAP_YAML="$INSTALLED_MAP/share/ats_sentry_bringup/map/$(basename "$MAP_YAML")"
  else
    echo "Map YAML not found: $MAP_YAML" >&2
    exit 2
  fi
fi
MAP_YAML="$(cd "$(dirname "$MAP_YAML")" && pwd)/$(basename "$MAP_YAML")"

MAP_IMAGE_REL="$(awk -F': *' '/^image:/ {print $2}' "$MAP_YAML" | tr -d '\r')"
MAP_IMAGE="$(dirname "$MAP_YAML")/$MAP_IMAGE_REL"

{
  echo "=========================================================="
  echo " Gazebo -> ATS navigation chain run"
  echo "=========================================================="
  echo "timestamp             : $STAMP"
  echo "workspace             : $WORKSPACE_ROOT"
  echo "ROS_DOMAIN_ID         : $ROS_DOMAIN_ID"
  echo "TEST_PROFILE          : $TEST_PROFILE"
  echo "P2_FAULT_CASE         : $P2_FAULT_CASE"
  echo "PLANNING_GRID_OWNER   : $PLANNING_GRID_OWNER"
  echo "WORLD                 : $WORLD"
  echo "WORLD_SDF_PATH        : ${WORLD_SDF_PATH:-<world-default>}"
  echo "MAP_YAML              : $MAP_YAML"
  echo "MAP_PGM               : $MAP_IMAGE"
  echo "SOLVER_MODE           : $SOLVER_MODE"
  echo "HEADLESS/USE_VIEWER   : $HEADLESS / $USE_VIEWER"
  echo "HEADLESS_RENDERING    : $HEADLESS_RENDERING"
  echo "USE_RVIZ              : $USE_RVIZ"
  echo "ENABLE_CAMERA_SENSORS : $ENABLE_CAMERA_SENSORS"
  echo "LIVOX_UPDATE_RATE_HZ  : $LIVOX_UPDATE_RATE_HZ"
  echo "LIVOX_HORIZONTAL_SAMPLES : $LIVOX_HORIZONTAL_SAMPLES"
  echo "OBSERVE_GAZEBO_TRANSPORT_LIDAR : $OBSERVE_GAZEBO_TRANSPORT_LIDAR"
  echo "USE_DIRECT_GAZEBO_LIDAR_BRIDGE : $USE_DIRECT_GAZEBO_LIDAR_BRIDGE"
  echo "LIDAR_BRIDGE_PUBLISHER_DEPTH : $LIDAR_BRIDGE_PUBLISHER_DEPTH"
  echo "LIDAR_BRIDGE_PUBLISHER_RELIABILITY : $LIDAR_BRIDGE_PUBLISHER_RELIABILITY"
  echo "=========================================================="
} | tee "$SUMMARY"

if [ ! -f "$MAP_IMAGE" ]; then
  fail "map PGM referenced by $MAP_YAML not found: $MAP_IMAGE"
  exit 2
fi

if [ "$SOLVER_MODE" = "qp" ]; then
  fail "solver_mode=qp is refused; only ilqr (control) and qp_shadow (diagnostic)"
  exit 2
fi

# ---------------------------------------------------------------------------
# Fault-case wiring
# ---------------------------------------------------------------------------

EXTRA_LAUNCH_ARGS=()
FAULT_NOTE=""
FAULT_EXECUTED="yes"

# ros2 launch treats an empty ``name:=`` as malformed. Let the Gazebo launch
# resolve the normal world name unless an A/B explicitly supplies an SDF file.
if [ -n "$WORLD_SDF_PATH" ]; then
  EXTRA_LAUNCH_ARGS+=("world_sdf_path:=$WORLD_SDF_PATH")
fi

case "$P2_FAULT_CASE" in
  none) ;;
  all-unknown)
    EXTRA_LAUNCH_ARGS+=("enable_test_fault_injection:=true")
    FAULT_NOTE="ROGMap starts with every cell unknown; unknown counts as obstacle."
    ;;
  map-unready)
    FAULT_NOTE="adapter heartbeat will be interrupted after nominal startup."
    ;;
  map-stale|input-stale)
    # The bridge is killed mid-run so the sensor/grid stream goes stale.
    FAULT_NOTE="gz_livox_bridge killed mid-run to starve the localization input."
    ;;
  emergency-stop-recovery)
    FAULT_NOTE="Active action is canceled by its client; a fresh goal must re-authorize motion."
    ;;
  goal-unreachable)
    GOAL_X="${GOAL_X:-999.0}"
    GOAL_Y="${GOAL_Y:-999.0}"
    FAULT_NOTE="Goal placed outside the map so JPS must fail safely."
    ;;
  adapter-lease|projection-timeout)
    FAULT_NOTE="${P2_FAULT_CASE} will be injected by stopping the owning node."
    ;;
  *)
    echo "Unknown P2_FAULT_CASE: $P2_FAULT_CASE" >&2
    exit 2
    ;;
esac

# ---------------------------------------------------------------------------
# Launch
# ---------------------------------------------------------------------------

# The gz_sim wrapper can exit before its `ign gazebo` server.  The server keeps
# the launch session ID after re-parenting, so it must be reaped explicitly or
# the next isolated ROS domain connects to the old Gazebo transport world.
reap_launch_gazebo_servers() {
  local sid="$1" signal pid
  local -a server_pids=()
  for signal in INT TERM KILL; do
    mapfile -t server_pids < <(
      ps -eo pid=,sid=,args= | awk -v sid="$sid" '
        $2 == sid && $0 ~ /(^|[[:space:]])ign[[:space:]]+gazebo([[:space:]]|$)/ {print $1}
      '
    )
    [ "${#server_pids[@]}" -eq 0 ] && return 0
    for pid in "${server_pids[@]}"; do
      kill -"$signal" "$pid" 2>/dev/null || true
    done
    [ "$signal" = "KILL" ] && break
    for _ in $(seq "${GAZEBO_SERVER_REAP_GRACE_SEC:-5}"); do
      sleep 1
      mapfile -t server_pids < <(
        ps -eo pid=,sid=,args= | awk -v sid="$sid" '
          $2 == sid && $0 ~ /(^|[[:space:]])ign[[:space:]]+gazebo([[:space:]]|$)/ {print $1}
        '
      )
      [ "${#server_pids[@]}" -eq 0 ] && return 0
    done
  done
  mapfile -t server_pids < <(
    ps -eo pid=,sid=,args= | awk -v sid="$sid" '
      $2 == sid && $0 ~ /(^|[[:space:]])ign[[:space:]]+gazebo([[:space:]]|$)/ {print $1}
    '
  )
  [ "${#server_pids[@]}" -eq 0 ] || log "failed to reap Gazebo server pid(s): ${server_pids[*]}"
}

cleanup() {
  stop_active_observers
  if [ -n "${GOAL_PID:-}" ] && kill -0 "$GOAL_PID" 2>/dev/null; then
    local goal_sid="${GOAL_SESSION_ID:-$GOAL_PID}"
    kill -TERM -- -"$goal_sid" 2>/dev/null || kill -TERM "$GOAL_PID" 2>/dev/null || true
    sleep 1
    kill -KILL -- -"$goal_sid" 2>/dev/null || kill -KILL "$GOAL_PID" 2>/dev/null || true
  fi
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    # ats_gazebo_nav is started in its own session below.  Signal that session
    # rather than using a broad pkill pattern which can match this shell or an
    # unrelated Gazebo run owned by another test.
    local launch_sid="${LAUNCH_SESSION_ID:-$LAUNCH_PID}"
    kill -INT -- -"$launch_sid" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq "$SHUTDOWN_GRACE_SEC"); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 1
    done
    kill -KILL -- -"$launch_sid" 2>/dev/null || kill -9 "$LAUNCH_PID" 2>/dev/null || true
    reap_launch_gazebo_servers "$launch_sid"
  fi
}
trap cleanup EXIT INT TERM

log "launching ats_gazebo_nav.launch.py"
setsid ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py \
  world:="$WORLD" \
  map_yaml:="$MAP_YAML" \
  use_sim_time:=true \
  headless:="$HEADLESS" \
  headless_rendering:="$HEADLESS_RENDERING" \
  use_viewer:="$USE_VIEWER" \
  use_rviz:="$USE_RVIZ" \
  enable_camera_sensors:="$ENABLE_CAMERA_SENSORS" \
  use_direct_gazebo_lidar_bridge:="$USE_DIRECT_GAZEBO_LIDAR_BRIDGE" \
  livox_update_rate_hz:="$LIVOX_UPDATE_RATE_HZ" \
  livox_horizontal_samples:="$LIVOX_HORIZONTAL_SAMPLES" \
  lidar_bridge_publisher_depth:="$LIDAR_BRIDGE_PUBLISHER_DEPTH" \
  lidar_bridge_publisher_reliability:="$LIDAR_BRIDGE_PUBLISHER_RELIABILITY" \
  planning_grid_owner:="$PLANNING_GRID_OWNER" \
  robot_name:="$ROBOT_NAME" \
  solver_mode:="$SOLVER_MODE" \
  "${EXTRA_LAUNCH_ARGS[@]}" \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
LAUNCH_SESSION_ID="$(ps -o sid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ' || true)"

# Stop only a node that belongs to this launch session.  A fault run is not a
# test when the intended target was absent or stayed alive: both cases must be
# observable failures, never a successful no-op.
stop_launch_node() {
  local name="$1" sid="${LAUNCH_SESSION_ID:-}"
  STOPPED_NODE_PIDS=""
  if [ -z "$sid" ]; then
    log "fault injection cannot find launch session for $name"
    return 1
  fi

  local -a pids=()
  local pid remaining deadline
  mapfile -t pids < <(
    ps -eo pid=,sid=,args= | awk -v sid="$sid" -v name="$name" '
      $2 == sid && $0 ~ ("(^|[[:space:]/])" name "([[:space:]]|$)") {print $1}
    '
  )
  if [ "${#pids[@]}" -eq 0 ]; then
    log "fault injection target not found in launch session: $name"
    return 1
  fi

  for pid in "${pids[@]}"; do
    if ! kill -TERM "$pid" 2>/dev/null; then
      log "fault injection could not signal $name pid=$pid"
      return 1
    fi
  done
  STOPPED_NODE_PIDS="${pids[*]}"

  deadline=$((SECONDS + ${FAULT_STOP_GRACE_SEC:-10}))
  while [ "$SECONDS" -lt "$deadline" ]; do
    remaining=0
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        remaining=1
        break
      fi
    done
    if [ "$remaining" -eq 0 ]; then
      log "fault injection stopped $name pid(s)=$STOPPED_NODE_PIDS"
      return 0
    fi
    sleep 1
  done

  log "fault injection target remained alive after TERM: $name pid(s)=$STOPPED_NODE_PIDS"
  return 1
}

start_goal_action() {
  local goal_x="$1" goal_y="$2" output="$3" error="$4" session_file deadline
  session_file="${output}.session_pid"
  : >"$session_file"
  # GNU setsid can fork when the background shell child is a process-group
  # leader. --wait keeps GOAL_PID attached to that child, while the inner shell
  # records the actual isolated session used for a narrowly targeted SIGINT.
  setsid --wait bash -c '
    session_file="$1"
    shift
    printf "%s\\n" "$$" >"$session_file"
    exec "$@"
  ' bash "$session_file" \
    timeout --foreground -k "$KILL_GRACE_SEC" "$GOAL_TIMEOUT_SEC" \
    ros2 action send_goal --feedback /ats_navigate_to_pose \
    ats_navigation_interfaces/action/NavigateToPose \
    "{goal_pose: {header: {frame_id: $GOAL_FRAME}, pose: {position: {x: $goal_x, y: $goal_y, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}, timeout: {sec: $GOAL_TIMEOUT_SEC, nanosec: 0}}" \
    >"$output" 2>"$error" &
  GOAL_PID=$!
  GOAL_SESSION_ID=""
  deadline=$((SECONDS + ${GOAL_SESSION_START_TIMEOUT_SEC:-5}))
  while [ ! -s "$session_file" ] && kill -0 "$GOAL_PID" 2>/dev/null && \
        [ "$SECONDS" -lt "$deadline" ]; do
    sleep 0.1
  done
  GOAL_SESSION_ID="$(awk 'NR == 1 {print; exit}' "$session_file" 2>/dev/null || true)"
  [[ "$GOAL_SESSION_ID" =~ ^[0-9]+$ ]]
}

# This is opt-in because window capture needs a local X display.  Capture only
# after action feedback proves the goal entered tracking; a launch-only RViz
# image is not navigation evidence.
capture_tracking_rviz_screenshot() {
  [[ "$CAPTURE_RVIZ_SCREENSHOT" = "true" ]] || return 0
  [[ "$USE_RVIZ" = "true" ]] || {
    fail "CAPTURE_RVIZ_SCREENSHOT=true requires USE_RVIZ=true"
    return 1
  }
  [[ -n "${DISPLAY:-}" ]] || {
    fail "CAPTURE_RVIZ_SCREENSHOT=true requires DISPLAY"
    return 1
  }
  command -v xwininfo >/dev/null 2>&1 && command -v xwd >/dev/null 2>&1 && \
    command -v ffmpeg >/dev/null 2>&1 || {
    fail "RViz capture needs xwininfo, xwd and ffmpeg"
    return 1
  }

  local deadline window_id window_tree
  deadline=$((SECONDS + RVIZ_CAPTURE_TRACKING_TIMEOUT_SEC))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if grep -q 'status: tracking' "$GOAL_OUTPUT" 2>/dev/null; then
      break
    fi
    sleep 0.2
  done
  if ! grep -q 'status: tracking' "$GOAL_OUTPUT" 2>/dev/null; then
    fail "RViz screenshot skipped because action never entered tracking"
    metric "rviz_screenshot" "not_captured_no_tracking"
    return 1
  fi

  sleep "$RVIZ_CAPTURE_SETTLE_SEC"

  deadline=$((SECONDS + RVIZ_CAPTURE_WINDOW_TIMEOUT_SEC))
  window_id=""
  while [ "$SECONDS" -lt "$deadline" ]; do
    window_tree="$RUN_DIR/rviz_windows_at_capture.txt"
    xwininfo -root -tree >"$window_tree" 2>&1 || true
    window_id="$(awk -v title="$RVIZ_WINDOW_TITLE" \
      'index($0, title) {print $1; exit}' "$window_tree")"
    [ -n "$window_id" ] && break
    sleep 0.2
  done
  if [ -z "$window_id" ]; then
    fail "RViz screenshot skipped because window '$RVIZ_WINDOW_TITLE' was not found"
    metric "rviz_screenshot" "not_captured_window_missing"
    return 1
  fi

  if ! python3 - "$window_id" "$RVIZ_CAPTURE_X" "$RVIZ_CAPTURE_Y" \
      "$RVIZ_CAPTURE_WIDTH" "$RVIZ_CAPTURE_HEIGHT" <<'PY'
import ctypes
import sys

window, x, y, width, height = (int(value, 0) for value in sys.argv[1:])
x11 = ctypes.CDLL("libX11.so.6")
x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
x11.XOpenDisplay.restype = ctypes.c_void_p
x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
x11.XCloseDisplay.restype = ctypes.c_int
x11.XMoveResizeWindow.argtypes = [
    ctypes.c_void_p,
    ctypes.c_ulong,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_uint,
]
x11.XMoveResizeWindow.restype = ctypes.c_int
x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
x11.XRaiseWindow.restype = ctypes.c_int
x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
x11.XSync.restype = ctypes.c_int
display = x11.XOpenDisplay(None)
if not display:
    raise SystemExit("cannot open X display")
try:
    x11.XMoveResizeWindow(display, ctypes.c_ulong(window), x, y, width, height)
    x11.XRaiseWindow(display, ctypes.c_ulong(window))
    x11.XSync(display, False)
finally:
    x11.XCloseDisplay(display)
PY
  then
    fail "RViz screenshot window resize failed"
    metric "rviz_screenshot" "not_captured_resize_failed"
    return 1
  fi
  sleep 1

  if ! xwd -id "$window_id" -silent -out "$RVIZ_SCREENSHOT_XWD" || \
     ! ffmpeg -y -v error -f xwd_pipe -i "$RVIZ_SCREENSHOT_XWD" -frames:v 1 "$RVIZ_SCREENSHOT" || \
     [ ! -s "$RVIZ_SCREENSHOT" ]; then
    fail "RViz screenshot capture failed"
    metric "rviz_screenshot" "not_captured_conversion_failed"
    return 1
  fi
  metric "rviz_screenshot" "$RVIZ_SCREENSHOT"
  log "RViz navigation screenshot saved to $RVIZ_SCREENSHOT"
}

wait_for_goal_action() {
  local label="$1" timeout_sec="$2"
  if [ -z "${GOAL_PID:-}" ]; then
    log "$label action has no tracked client PID"
    return 1
  fi
  local deadline=$((SECONDS + timeout_sec))
  while kill -0 "$GOAL_PID" 2>/dev/null && [ "$SECONDS" -lt "$deadline" ]; do
    sleep 1
  done
  if kill -0 "$GOAL_PID" 2>/dev/null; then
    log "$label action exceeded ${timeout_sec}s"
    return 1
  fi
  wait "$GOAL_PID" 2>/dev/null || true
  GOAL_PID=""
  GOAL_SESSION_ID=""
  return 0
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

refresh_topic_cache() {
  ros2 topic list --no-daemon --spin-time "$GRAPH_SPIN_TIME_SEC" \
    >"$TOPIC_CACHE" 2>/dev/null || :
}

topic_exists() {
  [ -f "$TOPIC_CACHE" ] && grep -Fxq "$1" "$TOPIC_CACHE"
}

# Sampling subscribes as BEST_EFFORT throughout. /localization, /odometry and
# the registered-scan stream are published BEST_EFFORT, so a default RELIABLE
# sampler requests an incompatible QoS, matches nothing and reports an empty
# topic that is in fact streaming. BEST_EFFORT also matches RELIABLE publishers,
# so the same setting is correct for every topic sampled here.
# `ros2 topic hz` already subscribes with the sensor-data profile.
ECHO_QOS=(--qos-reliability best_effort)

# Every sampler is killed, not merely asked to stop. `timeout N` sends SIGTERM
# only, and `ros2 topic echo --once` on a topic that never publishes blocks
# inside rclpy with the signal swallowed: one such call held this script for
# 20 minutes instead of the 10 seconds requested. `-k` escalates to SIGKILL, so
# a silent topic costs one sample window and is recorded as "no message".
KILL_GRACE_SEC="${KILL_GRACE_SEC:-5}"
sample_timeout() { timeout -k "$KILL_GRACE_SEC" "$@"; }

# Count real messages, not just publisher presence.
count_messages() {
  local topic="$1" seconds="${2:-$SAMPLE_SEC}" type
  type="$(topic_type "$topic")"
  if [ -n "$type" ]; then
    sample_timeout "$((seconds + COUNT_EXTRA_TIMEOUT_SEC))" \
      ros2 topic echo --no-daemon "$topic" "$type" "${ECHO_QOS[@]}" --once \
      >/dev/null 2>&1 && echo 1 || echo 0
  else
    sample_timeout "$((seconds + COUNT_EXTRA_TIMEOUT_SEC))" \
      ros2 topic echo --no-daemon "$topic" "${ECHO_QOS[@]}" --once \
      >/dev/null 2>&1 && echo 1 || echo 0
  fi
}

topic_hz() {
  local topic="$1" seconds="${2:-$SAMPLE_SEC}"
  sample_timeout "$((seconds + 3))" ros2 topic hz "$topic" --window 20 2>/dev/null \
    | awk '/average rate/ {rate=$3} END {print (rate == "" ? "unverified" : rate)}'
}

# Query both ownership counts from one graph participant.  Calling `ros2 topic
# info` separately for publisher and subscriber counts doubles discovery load
# without yielding an independent observation.
topic_counts() {
  ros2 topic info --no-daemon --spin-time "$GRAPH_SPIN_TIME_SEC" "$1" 2>/dev/null \
    | awk -F': *' '
      /Publisher count/ {pub=$2}
      /Subscription count/ {subscriber_count=$2}
      END {
        if (pub == "") pub = "unverified"
        if (subscriber_count == "") subscriber_count = "unverified"
        gsub(/[ \r]/, "", pub)
        gsub(/[ \r]/, "", subscriber_count)
        print pub, subscriber_count
      }
    '
}

# Explicit types keep the sampler independent of a stale ros2cli graph cache,
# especially for transient-local adapter heartbeat topics.
topic_type() {
  case "$1" in
    /clock) echo rosgraph_msgs/msg/Clock ;;
    /localization/status) echo ats_navigation_interfaces/msg/LocalizationStatus ;;
    /rog_map_adapter/ready|/planner/emergency_stop) echo std_msgs/msg/Bool ;;
    /rog_map_adapter/status) echo ats_navigation_interfaces/msg/PlanningMapStatus ;;
    /rog_map_adapter/planning_snapshot) echo ats_navigation_interfaces/msg/PlanningMapSnapshot ;;
    /rog_map/occ|/rog_map/inf_occ|/rog_map/unk|/rog_map/esdf|/traversability_grid|/rc_esdf/planning_grid) echo nav_msgs/msg/OccupancyGrid ;;
    /minco/raw_path|/minco/preprocessed_guide|/minco/esdf_refined_guide|/minco/reference_path|/ats_swerve_mpc/predicted_path|/ats_swerve_mpc/executed_path) echo nav_msgs/msg/Path ;;
    /localization|/odometry) echo nav_msgs/msg/Odometry ;;
    /registered_scan|/livox/lidar|*/livox/lidar) echo sensor_msgs/msg/PointCloud2 ;;
    /livox/imu|*/livox/imu) echo sensor_msgs/msg/Imu ;;
    /cmd_vel_mpc|*/cmd_vel) echo geometry_msgs/msg/Twist ;;
    *) echo "" ;;
  esac
}

echo_once() {
  local type
  type="$(topic_type "$1")"
  if [ -n "$type" ]; then
    sample_timeout "$((SAMPLE_SEC + 5))" ros2 topic echo --no-daemon "$1" "$type" "${ECHO_QOS[@]}" --once 2>/dev/null
  else
    sample_timeout "$((SAMPLE_SEC + 5))" ros2 topic echo --no-daemon "$1" "${ECHO_QOS[@]}" --once 2>/dev/null
  fi
}

# The goal manager is the sole owner of this transient-local stop heartbeat.
# Use its offered QoS for the health-gate evidence rather than publishing a
# second test writer on the same safety topic.
safety_echo_once() {
  sample_timeout "$((SAMPLE_SEC + 5))" ros2 topic echo --no-daemon /planner/emergency_stop \
    std_msgs/msg/Bool --qos-reliability reliable --qos-durability transient_local \
    --once 2>/dev/null
}

# Topic existence and a final terminal sample cannot prove a short trajectory
# ever ran: successful goals legitimately end with a cleared reference and
# deterministic zero velocity. A single C++ recorder retains the largest path,
# command, ground-truth and joint-state evidence over the action lifetime. It
# subscribes only; it has no publishers and uses a wall-clock deadline so a
# use_sim_time jump cannot end the collection immediately.
start_active_observers() {
  local duration_ceiling
  duration_ceiling="$(awk -v value="$ACTIVE_EVIDENCE_DURATION_SEC" \
    'BEGIN { if (value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0.0) { printf "%d", int(value + 0.999999) } }')"
  if [ -z "$duration_ceiling" ]; then
    fail "invalid active evidence duration: $ACTIVE_EVIDENCE_DURATION_SEC"
    return 1
  fi
  setsid ros2 run rmu_gazebo_simulator ats_navigation_evidence_recorder --ros-args \
    -p robot_name:="$ROBOT_NAME" \
    -p duration_sec:="$ACTIVE_EVIDENCE_DURATION_SEC" \
    -p observe_gazebo_transport_lidar:="$OBSERVE_GAZEBO_TRANSPORT_LIDAR" \
    >"$ACTIVE_EVIDENCE_LOG" 2>&1 &
  ACTIVE_EVIDENCE_PID=$!
  ACTIVE_EVIDENCE_SESSION_ID="$(ps -o sid= -p "$ACTIVE_EVIDENCE_PID" 2>/dev/null | tr -d ' ' || true)"
  ACTIVE_EVIDENCE_DEADLINE_SEC=$((SECONDS + duration_ceiling + 5))
}

wait_for_active_evidence_window() {
  if [ -z "${ACTIVE_EVIDENCE_PID:-}" ] || [ -z "${ACTIVE_EVIDENCE_DEADLINE_SEC:-}" ]; then
    return 0
  fi
  log "waiting for active evidence recorder to complete its ${ACTIVE_EVIDENCE_DURATION_SEC}s window"
  while kill -0 "$ACTIVE_EVIDENCE_PID" 2>/dev/null && [ "$SECONDS" -lt "$ACTIVE_EVIDENCE_DEADLINE_SEC" ]; do
    sleep 1
  done
}

# Snapshot only known nodes inside this run's launch session.  It never matches
# across sessions and never signals a process, so an unrelated user's ROS or
# simulator process cannot become test input or a cleanup target.  The raw
# CPU ticks/RSS/thread/context-switch values deliberately remain raw: two
# snapshots can be compared offline without pretending that a point sample is
# a process-rate or DDS drop measurement.
capture_launch_process_resources() {
  local phase="$1" pid sid args stat_path status_path cpu_ticks rss_kb threads vctx ivctx
  sid="${LAUNCH_SESSION_ID:-}"
  {
    printf 'phase=%s steady_epoch_ns=%s launch_session=%s\n' \
      "$phase" "$(date +%s%N)" "${sid:-unverified}"
    if [ -z "$sid" ]; then
      printf 'resource_capture=unverified_missing_launch_session\n'
      return 0
    fi
    while IFS=$'\t' read -r pid args; do
      [ -n "$pid" ] || continue
      stat_path="/proc/$pid/stat"
      status_path="/proc/$pid/status"
      if [ ! -r "$stat_path" ] || [ ! -r "$status_path" ]; then
        printf 'pid=%s resource_capture=unverified_process_exited args=%s\n' "$pid" "$args"
        continue
      fi
      cpu_ticks="$(awk '{print $14 + $15}' "$stat_path" 2>/dev/null || true)"
      rss_kb="$(awk '/^VmRSS:/ {print $2; exit}' "$status_path" 2>/dev/null || true)"
      threads="$(awk '/^Threads:/ {print $2; exit}' "$status_path" 2>/dev/null || true)"
      vctx="$(awk '/^voluntary_ctxt_switches:/ {print $2; exit}' "$status_path" 2>/dev/null || true)"
      ivctx="$(awk '/^nonvoluntary_ctxt_switches:/ {print $2; exit}' "$status_path" 2>/dev/null || true)"
      printf 'pid=%s cpu_ticks=%s rss_kb=%s threads=%s voluntary_ctxt=%s nonvoluntary_ctxt=%s args=%s\n' \
        "$pid" "${cpu_ticks:-unverified}" "${rss_kb:-unverified}" \
        "${threads:-unverified}" "${vctx:-unverified}" "${ivctx:-unverified}" "$args"
    done < <(
      ps -eo pid=,sid=,args= | awk -v sid="$sid" '
        $2 == sid &&
        $0 ~ /(gz_livox_bridge_node|pointlio_mapping|loam_interface_node|sensor_scan_generation_node|localization_fusion_node|ats_rog_map_node|ats_rog_map_adapter_node|ign gazebo)/ {
          pid = $1
          $1 = ""
          $2 = ""
          sub(/^[[:space:]]+/, "")
          print pid "\t" $0
        }'
    )
    printf 'dds_queue_drop_counter=unverified_no_portable_rmw_counter\n'
  } >>"$PROCESS_RESOURCE_LOG"
}

# The recovery client owns the exact action it cancels. Unlike a shell SIGINT,
# its command callback and GoalHandle share one rclcpp executor, so an early
# progress watchdog cannot win merely because a CLI process was re-parented by
# setsid. It has no command publisher.
run_recovery_cancel_on_command() {
  local wall_timeout goal_timeout
  wall_timeout="$(awk -v value="${RECOVERY_CANCEL_WALL_TIMEOUT_SEC:-20}" 'BEGIN {printf "%.6f", value + 0.0}')"
  goal_timeout="$(awk -v value="$GOAL_TIMEOUT_SEC" 'BEGIN {printf "%.6f", value + 0.0}')"
  sample_timeout "$(( ${RECOVERY_CANCEL_WALL_TIMEOUT_SEC:-20} + KILL_GRACE_SEC ))" \
    ros2 run rmu_gazebo_simulator ats_navigation_cancel_on_command_client --ros-args \
      -p goal_x:="$GOAL_X" \
      -p goal_y:="$GOAL_Y" \
      -p goal_yaw:="$GOAL_YAW" \
      -p goal_frame:="$GOAL_FRAME" \
      -p goal_timeout_sec:="$goal_timeout" \
      -p wall_timeout_sec:="$wall_timeout" \
      >"$RECOVERY_CANCEL_LOG" 2>&1
}

recovery_cancel_value() {
  local key="$1"
  awk -v key="$key=" '
    /^ATS_CANCEL_ON_COMMAND_RESULT / {
      for (i = 1; i <= NF; ++i) {
        if (index($i, key) == 1) value = substr($i, length(key) + 1)
      }
    }
    END {print value}
  ' "$RECOVERY_CANCEL_LOG"
}

# Record the graph while the launched nodes are still alive. A terminal query
# after a fast action can race launch teardown and falsely report zero writers.
# This function is intentionally read-only and observes only ownership that the
# ATS profile is required to keep unique.
capture_active_ownership() {
  local spin_time="${ACTIVE_OWNER_GRAPH_SPIN_TIME_SEC:-1}" attempt
  sleep "${ACTIVE_OWNER_DELAY_SEC:-1}"
  {
    for attempt in $(seq "${ACTIVE_OWNER_ATTEMPTS:-3}"); do
      printf 'attempt=%s\n' "$attempt"
      for topic in /cmd_vel_mpc; do
        printf 'topic=%s\n' "$topic"
        ros2 topic info --no-daemon --spin-time "$spin_time" "$topic" 2>&1 || true
      done
      [ "$attempt" = "${ACTIVE_OWNER_ATTEMPTS:-3}" ] || sleep 1
    done
  } >"$ACTIVE_OWNERSHIP_LOG"
}

stop_active_observers() {
  local pid
  if [ -n "${ACTIVE_EVIDENCE_PID:-}" ] && kill -0 "$ACTIVE_EVIDENCE_PID" 2>/dev/null; then
    local evidence_sid="${ACTIVE_EVIDENCE_SESSION_ID:-$ACTIVE_EVIDENCE_PID}"
    # rclcpp handles SIGINT by returning from spin; its main then writes the
    # single structured witness line before exiting. SIGKILL is retained only
    # as a bounded cleanup fallback.
    kill -INT -- -"$evidence_sid" 2>/dev/null || kill -INT "$ACTIVE_EVIDENCE_PID" 2>/dev/null || true
    local evidence_deadline=$((SECONDS + KILL_GRACE_SEC))
    while kill -0 "$ACTIVE_EVIDENCE_PID" 2>/dev/null && [ "$SECONDS" -lt "$evidence_deadline" ]; do
      sleep 1
    done
    if kill -0 "$ACTIVE_EVIDENCE_PID" 2>/dev/null; then
      kill -KILL -- -"$evidence_sid" 2>/dev/null || kill -KILL "$ACTIVE_EVIDENCE_PID" 2>/dev/null || true
    fi
    wait "$ACTIVE_EVIDENCE_PID" 2>/dev/null || true
  fi
  ACTIVE_EVIDENCE_PID=""
  ACTIVE_EVIDENCE_SESSION_ID=""
  for pid in "${ACTIVE_OBSERVER_PIDS[@]:-}"; do
    [ -n "$pid" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  for pid in "${ACTIVE_OBSERVER_PIDS[@]:-}"; do
    [ -n "$pid" ] || continue
    wait "$pid" 2>/dev/null || true
  done
  ACTIVE_OBSERVER_PIDS=()
}

evidence_value() {
  local key="$1"
  awk -v key="$key=" '
    /^ATS_NAVIGATION_EVIDENCE_RESULT / {
      for (i = 1; i <= NF; ++i) {
        if (index($i, key) == 1) value = substr($i, length(key) + 1)
      }
    }
    END {print value}
  ' "$ACTIVE_EVIDENCE_LOG"
}

evidence_bool_as_int() {
  case "$1" in
    yes) echo 1 ;;
    no) echo 0 ;;
    *) echo unverified ;;
  esac
}

set_p1_admission_evidence() {
  P1_ADMISSION_EVIDENCE="false"
  P1_ADMISSION_REASON="not_eligible"
  if [ "$P2_FAULT_CASE" != "none" ]; then
    P1_ADMISSION_REASON="fault_case_${P2_FAULT_CASE}"
    return 0
  fi
  if ! awk -v duration="$ACTIVE_OBSERVER_WINDOW_SEC" 'BEGIN {exit !(duration + 0.0 >= 60.0)}'; then
    P1_ADMISSION_REASON="observer_window_shorter_than_60s"
    return 0
  fi
  if [ "$(evidence_value completed)" != "yes" ]; then
    P1_ADMISSION_REASON="observer_not_completed"
    return 0
  fi
  if ! awk -v observed="$(evidence_value duration_s)" -v required="$ACTIVE_EVIDENCE_DURATION_SEC" '
      BEGIN {
        observed_numeric = observed ~ /^[0-9]+([.][0-9]+)?$/
        required_numeric = required ~ /^[0-9]+([.][0-9]+)?$/
        exit !(observed_numeric && required_numeric && observed + 0.0 >= required + 0.0)
      }'; then
    P1_ADMISSION_REASON="observer_duration_shorter_than_requested"
    return 0
  fi
  if [ "$(freshness_metric_value "$FRESHNESS_CLASSIFICATION" first_violation)" != "none" ]; then
    P1_ADMISSION_REASON="freshness_$(freshness_metric_value "$FRESHNESS_CLASSIFICATION" first_violation)"
    return 0
  fi
  if [ "$(evidence_value localization_status_non_tracking_samples)" != "0" ]; then
    P1_ADMISSION_REASON="localization_status_not_continuously_tracking"
    return 0
  fi
  if [ "$(evidence_value tf_lookup_failures)" != "0" ]; then
    P1_ADMISSION_REASON="tf_lookup_failures_$(evidence_value tf_lookup_failures)"
    return 0
  fi
  if [ "${GOAL_SUCCEEDED:-0}" != "1" ]; then
    P1_ADMISSION_REASON="straight_action_not_succeeded"
    return 0
  fi
  P1_ADMISSION_EVIDENCE="true"
  P1_ADMISSION_REASON="all_p1_gates_passed"
}

active_owner_count() {
  local topic="$1" field="$2"
  awk -v topic="$topic" -v field="$field" '
    $0 == "topic=" topic { current = 1; next }
    /^topic=/ { current = 0 }
    current && $0 ~ field ":" {
      split($0, values, ": *")
      value = values[2]
    }
    END {print value}
  ' "$ACTIVE_OWNERSHIP_LOG" | tr -d ' \r'
}

twist_is_zero() {
  awk '
    /^[[:space:]]+[xyz]:/ {
      value = $2 + 0.0
      if (value < -1e-9 || value > 1e-9) nonzero = 1
      fields++
    }
    END { exit !(fields == 6 && !nonzero) }
  '
}

twist_is_nonzero() {
  awk '
    /^[[:space:]]+[xyz]:/ {
      value = $2 + 0.0
      if (value < -1e-9 || value > 1e-9) nonzero = 1
      fields++
    }
    END { exit !(fields == 6 && nonzero) }
  '
}

# Take several fresh command samples after an emergency stop.  One message is
# insufficient: a stale command can be followed by a single zero and then
# resume after the safety edge.  Sampling with a short interval observes the
# command lease while no replacement goal has been submitted.
observe_zero_window() {
  local label="$1" topic="$2" validator="$3" samples="$4" output="$5"
  local index raw
  : >"$output"
  for index in $(seq "$samples"); do
    raw="$(echo_once "$topic")"
    {
      printf 'sample=%s topic=%s\n' "$index" "$topic"
      printf '%s\n---\n' "$raw"
    } >>"$output"
    if [ -z "$raw" ] || ! printf '%s\n' "$raw" | "$validator"; then
      log "$label observed a missing or non-zero command on $topic at sample $index"
      return 1
    fi
    if [ "$index" -lt "$samples" ]; then
      sleep "${ZERO_WINDOW_SAMPLE_INTERVAL_SEC:-1}"
    fi
  done
  return 0
}

status_value() {
  local field="$1"
  echo_once /rog_map_adapter/status | awk -v field="$field" '
    $1 == field ":" {print $2; exit}
  '
}

strictly_increases() {
  local before="$1" after="$2"
  [[ "$before" =~ ^[0-9]+$ && "$after" =~ ^[0-9]+$ ]] &&
    [ "$after" -gt "$before" ]
}

write_artifact_summary() {
  {
    echo
    echo "p1 admission evidence: $P1_ADMISSION_EVIDENCE"
    echo "p1 admission reason: $P1_ADMISSION_REASON"
    echo "artifacts:"
    echo "  summary : $SUMMARY"
    echo "  launch  : $LAUNCH_LOG"
    echo "  topics  : $TOPIC_LOG"
    echo "  metrics : $METRIC_LOG"
    echo
    echo "failures: $FAILURE_COUNT"
    echo "first failure: ${FIRST_FAILURE:-none}"
  } | tee -a "$SUMMARY"
}

# ---------------------------------------------------------------------------
# Startup gate: simulation clock must actually advance
# ---------------------------------------------------------------------------

log "waiting for /clock to advance (timeout ${STARTUP_TIMEOUT_SEC}s)"
CLOCK_OK="no"
CLOCK_T0=""
CLOCK_T1=""
deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
while [ "$SECONDS" -lt "$deadline" ]; do
  refresh_topic_cache
  if topic_exists /clock; then
    CLOCK_T0="$(echo_once /clock | awk '/sec:/ && !/nanosec/ {print $2; exit}')"
    sleep 3
    CLOCK_T1="$(echo_once /clock | awk '/sec:/ && !/nanosec/ {print $2; exit}')"
    if [ -n "$CLOCK_T0" ] && [ -n "$CLOCK_T1" ] && [ "$CLOCK_T1" -gt "$CLOCK_T0" ] 2>/dev/null; then
      CLOCK_OK="yes"
      break
    fi
  fi
  sleep 3
done
metric "sim_clock_advancing" "$CLOCK_OK (${CLOCK_T0:-?} -> ${CLOCK_T1:-?})"
if [ "$CLOCK_OK" != "yes" ]; then
  fail "simulation clock did not advance"
  metric "failure_count" "$FAILURE_COUNT"
  metric "first_failure_reason" "${FIRST_FAILURE:-none}"
  log "simulation startup gate failed; skipping all downstream sampling"
  refresh_topic_cache
  cat "$TOPIC_CACHE" >>"$TOPIC_LOG" 2>/dev/null || true
  ros2 node list --no-daemon >>"$TOPIC_LOG" 2>&1 || true
  cleanup
  trap - EXIT INT TERM
  write_artifact_summary
  exit 1
fi

# ---------------------------------------------------------------------------
# Sensor and localization evidence
# ---------------------------------------------------------------------------

log "sampling sensor and localization stage"
refresh_topic_cache
cat "$TOPIC_CACHE" >"$TOPIC_LOG" 2>/dev/null || true
ros2 node list --no-daemon >>"$TOPIC_LOG" 2>&1 || true

GZ_LIDAR_TOPIC="/${ROBOT_NAME}/livox/lidar"
GZ_IMU_TOPIC="/${ROBOT_NAME}/livox/imu"

# Do not create long-lived Python CLI subscribers for the three PointCloud2
# streams before localization/map health is established.  On the target i5
# host that observation itself causes packet loss and can trip the genuine
# 0.5 s odometry freshness gate.  The required payload proof below remains a
# single bounded BEST_EFFORT sample; rate metrics are deliberately reported as
# deferred instead of being fabricated from topic discovery.
metric "gz_lidar_hz(${GZ_LIDAR_TOPIC})" "deferred_nonintrusive"
metric "gz_imu_hz(${GZ_IMU_TOPIC})" "deferred_nonintrusive"
metric "livox_custom_cloud_hz(/livox/lidar)" "deferred_nonintrusive"
metric "localization_hz(/localization)" "deferred_nonintrusive"
metric "registered_scan_hz(/registered_scan)" "deferred_nonintrusive"

LOC_MSG_OK="$(count_messages /localization)"
SCAN_MSG_OK="$(count_messages /registered_scan)"
metric "localization_has_message" "$LOC_MSG_OK"
metric "registered_scan_has_message" "$SCAN_MSG_OK"
[ "$LOC_MSG_OK" = "1" ] || fail "/localization produced no message"
[ "$SCAN_MSG_OK" = "1" ] || fail "/registered_scan produced no message"

START_POSE="$(echo_once /localization \
  | awk '/position:/{f=1} f&&/x:/{x=$2} f&&/y:/{y=$2} f&&/z:/{print x" "y; exit}')"
metric "start_pose_xy" "${START_POSE:-unverified}"

# ---------------------------------------------------------------------------
# Planning map ownership
# ---------------------------------------------------------------------------

log "sampling planning map stage"
refresh_topic_cache
for t in /rog_map/occ /rog_map/inf_occ /rog_map/unk /rog_map/esdf \
         /rog_map_adapter/ready /rog_map_adapter/status \
         /rog_map_adapter/planning_snapshot /traversability_grid \
         /rc_esdf/planning_grid; do
  if topic_exists "$t"; then
    metric "present:$t" "listed_in_phase_graph"
  else
    metric "present:$t" "absent"
  fi
done

read -r GRID_PUB GRID_SUB <<<"$(topic_counts /rc_esdf/planning_grid)"
# This startup CLI query is only a discovery diagnostic.  A fresh ros2cli
# participant can miss the DDS graph while the C++ health probe already has a
# valid grid payload.  The action-lifetime recorder below owns the definitive
# publisher-count and identity gate once the map health gate has opened.
metric "planning_grid_pub/sub_startup_cli" "${GRID_PUB:-unverified}/${GRID_SUB:-unverified}"

MAP_READY="$(echo_once /rog_map_adapter/ready | awk '/data:/ {print $2; exit}')"
metric "map_ready_heartbeat" "${MAP_READY:-unverified}"

if ros2 service list --no-daemon 2>/dev/null | grep -q /rog_map/get_ground_projection; then
  metric "projection_service" "available"
else
  metric "projection_service" "absent"
fi

ADAPTER_SEQ="$(echo_once /rog_map_adapter/status \
  | awk '/publication_sequence|sequence:/ {print $2; exit}')"
metric "adapter_publication_sequence" "${ADAPTER_SEQ:-unverified}"
ROG_GEN="$(echo_once /rog_map_adapter/status | awk '/generation:/ {print $2; exit}')"
metric "rog_map_source_generation" "${ROG_GEN:-unverified}"

# The Gazebo profile has no physical LiDAR-occlusion switch.  For this one
# isolated fault, the authorized ROGMap fixture preserves healthy cloud/TF
# receipt while withholding numeric occupancy, then the adapter masks only the
# secondary static/terrain inputs before its ordinary fusion truth table.
if [ "$P2_FAULT_CASE" = "all-unknown" ]; then
  ALL_UNKNOWN_BASELINE="$(sample_timeout 12 python3 "$WORKSPACE_ROOT/scripts/query_rog_projection.py" \
    --timeout 6 --mode generation 2>/dev/null | awk '{for (i = 1; i <= NF; ++i) if ($i ~ /^generation=/) {sub(/^generation=/, "", $i); print $i; exit}}')"
  metric "all_unknown_source_generation_baseline" "${ALL_UNKNOWN_BASELINE:-unverified}"
  if [ -z "$ALL_UNKNOWN_BASELINE" ]; then
    fail "cannot capture healthy ROGMap generation before all-unknown injection"
  fi
  log "injecting all-unknown source fixture"
  sample_timeout 10 ros2 param set /ats_rog_map test_reset_to_unknown true \
    >"$RUN_DIR/all_unknown_rog_map_param.log" 2>&1 || \
    fail "cannot enable authorized ROGMap all-unknown fixture"
  if [ -n "$ALL_UNKNOWN_BASELINE" ]; then
    ALL_UNKNOWN_SOURCE="$(sample_timeout 15 python3 "$WORKSPACE_ROOT/scripts/query_rog_projection.py" \
      --timeout 8 --mode all-unknown --baseline-generation "$ALL_UNKNOWN_BASELINE" \
      2>"$RUN_DIR/all_unknown_projection.err")" || \
      fail "ROGMap numeric source did not become strictly all-unknown"
  else
    ALL_UNKNOWN_SOURCE="unverified"
  fi
  metric "all_unknown_numeric_projection" "${ALL_UNKNOWN_SOURCE:-unverified}"
  sample_timeout 10 ros2 param set /ats_rog_map_adapter test_mask_secondary_evidence true \
    >"$RUN_DIR/all_unknown_adapter_param.log" 2>&1 || \
    fail "cannot enable authorized adapter secondary-evidence mask"
fi

# ---------------------------------------------------------------------------
# Goal dispatch
# ---------------------------------------------------------------------------

if [ -z "$GOAL_X" ] || [ -z "$GOAL_Y" ]; then
  # Gazebo's spawn pose is in the world frame, while this map and localization
  # chain use a local map/odom frame.  The old (4.8, 9.5) goal is outside the
  # rmuc_2025 map extent; callers can still override this local free-space point.
  GOAL_X="2.0"
  GOAL_Y="0.0"
fi
metric "goal_xy" "$GOAL_X $GOAL_Y"
metric "goal_frame" "$GOAL_FRAME"

log "waiting for ATS action server /ats_navigate_to_pose"
ACTION_DEADLINE=$((SECONDS + ACTION_SERVER_TIMEOUT_SEC))
ACTION_READY="no"
while [ "$SECONDS" -lt "$ACTION_DEADLINE" ]; do
  if ros2 node info --no-daemon --spin-time "$GRAPH_SPIN_TIME_SEC" \
      /ats_goal_manager 2>/dev/null | grep -q '/ats_navigate_to_pose'; then
    ACTION_READY="yes"
    break
  fi
  sleep 1
done
metric "ats_action_server_ready" "$ACTION_READY"

# Do not send an action while localization/map health is false. The probe has
# the same three consecutive-sample gate as before, but is one C++ DDS reader
# with wall-clock deadlines. Repeated rclpy `ros2 topic echo` processes were
# affecting the point-cloud/odometry cadence of the system being measured.
HEALTH_STABLE_REQUIRED="${HEALTH_STABLE_SAMPLES:-3}"
HEALTH_PROBE_TIMEOUT_SEC="${HEALTH_PROBE_TIMEOUT_SEC:-$STARTUP_TIMEOUT_SEC}"
HEALTH_PROBE_TIMEOUT_FLOAT="$(awk -v value="$HEALTH_PROBE_TIMEOUT_SEC" 'BEGIN {printf "%.6f", value + 0.0}')"
HEALTH_READY="no"
HEALTH_LOCALIZATION_STATE="unverified"
HEALTH_MAP_READY="unverified"
HEALTH_STATUS_READY="unverified"
HEALTH_GRID_PAYLOAD="no"
HEALTH_STABLE_COUNT="0"
log "waiting for localization/map health before action dispatch"
if sample_timeout "$((HEALTH_PROBE_TIMEOUT_SEC + KILL_GRACE_SEC))" \
    ros2 run rmu_gazebo_simulator ats_navigation_health_probe --ros-args \
      -p timeout_sec:="$HEALTH_PROBE_TIMEOUT_FLOAT" \
      -p required_stable_samples:="$HEALTH_STABLE_REQUIRED" \
      -p localization_timeout_sec:=1.0 \
      -p map_timeout_sec:=5.0 \
      >"$HEALTH_PROBE_LOG" 2>&1; then
  HEALTH_READY="yes"
fi
HEALTH_RESULT="$(awk '/^ATS_HEALTH_PROBE_RESULT / {line=$0} END {print line}' "$HEALTH_PROBE_LOG")"
health_result_value() {
  awk -v key="$1=" '
    /^ATS_HEALTH_PROBE_RESULT / {
      for (i = 1; i <= NF; ++i) {
        if (index($i, key) == 1) {
          value = substr($i, length(key) + 1)
        }
      }
    }
    END {print value}
  ' "$HEALTH_PROBE_LOG"
}
HEALTH_LOCALIZATION_STATE="$(health_result_value localization_state)"
HEALTH_MAP_READY="$(health_result_value map_ready)"
HEALTH_STATUS_READY="$(health_result_value map_status_ready)"
HEALTH_GRID_PAYLOAD="$(health_result_value grid_payload)"
HEALTH_STABLE_COUNT="$(health_result_value stable | cut -d/ -f1)"
HEALTH_LOCALIZATION_STATE="${HEALTH_LOCALIZATION_STATE:-unverified}"
HEALTH_MAP_READY="${HEALTH_MAP_READY:-unverified}"
HEALTH_STATUS_READY="${HEALTH_STATUS_READY:-unverified}"
HEALTH_GRID_PAYLOAD="${HEALTH_GRID_PAYLOAD:-no}"
HEALTH_STABLE_COUNT="${HEALTH_STABLE_COUNT:-0}"
metric "health_gate" "$HEALTH_READY localization_state=${HEALTH_LOCALIZATION_STATE} map_ready=${HEALTH_MAP_READY} map_status_ready=${HEALTH_STATUS_READY} grid_payload=${HEALTH_GRID_PAYLOAD} stable=${HEALTH_STABLE_COUNT}/${HEALTH_STABLE_REQUIRED} result=${HEALTH_RESULT:-unverified}"
if [ "$HEALTH_READY" != "yes" ]; then
  if [ "$P2_FAULT_CASE" != "all-unknown" ]; then
    fail "localization/map health did not become stable before action dispatch"
  fi
  # This is an integration result, not a launch failure.  Preserve the three
  # safety outputs that make the blocked action meaningful, then terminate
  # this session before long path/action sampling can mask the first cause or
  # leave Gazebo alive when an outer runner reaches its own deadline.
  HEALTH_ESTOP_DUMP="$(safety_echo_once)"
  HEALTH_CMD_DUMP="$(echo_once /cmd_vel_mpc)"
  HEALTH_ESTOP="$(printf '%s\n' "$HEALTH_ESTOP_DUMP" | awk '/data:/ {print $2; exit}')"
  metric "health_gate_emergency_stop" "${HEALTH_ESTOP:-unverified}"
  metric "health_gate_cmd_vel_mpc" "${HEALTH_CMD_DUMP//$'\n'/ }"
  [ "$HEALTH_ESTOP" = "true" ] || fail "health gate failure did not observe planner emergency stop=true"
  printf '%s\n' "$HEALTH_CMD_DUMP" | twist_is_zero || \
    fail "health gate failure did not observe zero /cmd_vel_mpc"
  if [ "$P2_FAULT_CASE" = "all-unknown" ]; then
    [ "$HEALTH_MAP_READY" = "false" ] || \
      fail "all-unknown fault did not make the planning map unavailable"
    [ "$HEALTH_STATUS_READY" = "false" ] || \
      fail "all-unknown fault did not publish adapter ready=false"
  fi
  metric "goal_dispatch" "skipped health=no"
  metric "goal_action_accepted" "0"
  metric "goal_action_succeeded" "0"
  metric "goal_action_result" "not_dispatched_health_gate"
  metric "failure_count" "$FAILURE_COUNT"
  metric "first_failure_reason" "${FIRST_FAILURE:-none}"
  log "health gate remained closed; skipping action dispatch and collecting shutdown state"
  refresh_topic_cache
  cat "$TOPIC_CACHE" >>"$TOPIC_LOG" 2>/dev/null || true
  ros2 node list --no-daemon >>"$TOPIC_LOG" 2>&1 || true
  cleanup
  trap - EXIT INT TERM
  write_artifact_summary
  if [ "$P2_FAULT_CASE" = "all-unknown" ] && [ "$FAILURE_COUNT" -eq 0 ]; then
    exit 0
  fi
  exit 1
fi

if [ "$ACTION_READY" = "yes" ] && [ "$HEALTH_READY" = "yes" ]; then
  # Start read-only streams before dispatch and give DDS discovery a bounded
  # setup interval. A one-metre nominal goal can complete before a post-dispatch
  # ros2cli process has joined its volatile Path publishers.
  start_active_observers
  capture_launch_process_resources "active_start"
  sleep "${ACTIVE_OBSERVER_SETTLE_SEC:-2}"
  if [ "$P2_FAULT_CASE" = "emergency-stop-recovery" ]; then
    RECOVERY_SOURCE_GENERATION_BEFORE="$(status_value rog_generation)"
    RECOVERY_PUBLICATION_SEQUENCE_BEFORE="$(status_value publication_sequence)"
    metric "recovery_source_generation_before_cancel" "${RECOVERY_SOURCE_GENERATION_BEFORE:-unverified}"
    metric "recovery_publication_sequence_before_cancel" "${RECOVERY_PUBLICATION_SEQUENCE_BEFORE:-unverified}"
    log "dispatching and canceling an owned ATS NavigateToPose action after first MPC command"
    if ! run_recovery_cancel_on_command; then
      fail "owned recovery action was not canceled after its first non-zero /cmd_vel_mpc"
    fi
    RECOVERY_CANCEL_RESULT="$(awk '/^ATS_CANCEL_ON_COMMAND_RESULT / {line=$0} END {print line}' "$RECOVERY_CANCEL_LOG")"
    metric "goal_dispatch" "cancel_on_command_client ${RECOVERY_CANCEL_RESULT:-unverified}"
    metric "goal_cancel_request" "$(recovery_cancel_value cancel_accepted)"
  else
    log "dispatching ATS NavigateToPose action ($GOAL_X, $GOAL_Y)"
    if start_goal_action "$GOAL_X" "$GOAL_Y" "$GOAL_OUTPUT" "$GOAL_ERROR"; then
      metric "goal_dispatch" "action_sent pid=$GOAL_PID session=$GOAL_SESSION_ID"
      capture_active_ownership &
      ACTIVE_OBSERVER_PIDS+=("$!")
      capture_tracking_rviz_screenshot || true
    else
      fail "could not establish an isolated action-client session"
      metric "goal_dispatch" "action_session_unverified"
    fi
  fi
else
  metric "goal_dispatch" "skipped action_server=$ACTION_READY health=$HEALTH_READY"
  if [ "$ACTION_READY" != "yes" ]; then
    fail "ATS NavigateToPose action server unavailable"
    HEALTH_ESTOP_DUMP="$(safety_echo_once)"
    HEALTH_CMD_DUMP="$(echo_once /cmd_vel_mpc)"
    HEALTH_ESTOP="$(printf '%s\n' "$HEALTH_ESTOP_DUMP" | awk '/data:/ {print $2; exit}')"
    metric "action_gate_emergency_stop" "${HEALTH_ESTOP:-unverified}"
    metric "action_gate_cmd_vel_mpc" "${HEALTH_CMD_DUMP//$'\n'/ }"
    [ "$HEALTH_ESTOP" = "true" ] || fail "action gate did not observe planner emergency stop=true"
    printf '%s\n' "$HEALTH_CMD_DUMP" | twist_is_zero || \
      fail "action gate did not observe zero /cmd_vel_mpc"
    metric "failure_count" "$FAILURE_COUNT"
    metric "first_failure_reason" "${FIRST_FAILURE:-none}"
    log "action server gate failed; skipping downstream sampling"
    refresh_topic_cache
    cat "$TOPIC_CACHE" >>"$TOPIC_LOG" 2>/dev/null || true
    ros2 node list --no-daemon >>"$TOPIC_LOG" 2>&1 || true
    cleanup
    trap - EXIT INT TERM
    write_artifact_summary
    exit 1
  fi
fi

# ---------------------------------------------------------------------------
# Mid-run fault injection
# ---------------------------------------------------------------------------

if [ "$P2_FAULT_CASE" = "none" ]; then
  # Nominal evidence comes from the action-lifetime streams above. Do not wait
  # a third of RUN_DURATION before inspecting a short, already-completed goal.
  sleep "${NOMINAL_POST_DISPATCH_OBSERVE_SEC:-2}"
elif [ "$P2_FAULT_CASE" = "emergency-stop-recovery" ]; then
  : # The owned C++ action client above waits for the command-triggered cancel.
else
  sleep "$FAULT_INJECTION_DELAY_SEC"
fi

case "$P2_FAULT_CASE" in
  map-stale|input-stale)
    log "injecting $P2_FAULT_CASE: killing gz_livox_bridge"
    stop_launch_node "gz_livox_bridge_node" || \
      fail "fault injection target gz_livox_bridge_node was not stopped"
    metric "fault_injection_target" "gz_livox_bridge_node pid(s)=${STOPPED_NODE_PIDS:-unverified}"
    ;;
  map-unready|adapter-lease)
    log "injecting $P2_FAULT_CASE: stopping adapter heartbeat owner"
    stop_launch_node "ats_rog_map_adapter_node" || \
      fail "fault injection target ats_rog_map_adapter_node was not stopped"
    metric "fault_injection_target" "ats_rog_map_adapter_node pid(s)=${STOPPED_NODE_PIDS:-unverified}"
    ;;
  projection-timeout)
    log "injecting projection-timeout: stopping ROGMap projection service"
    stop_launch_node "ats_rog_map_node" || \
      fail "fault injection target ats_rog_map_node was not stopped"
    metric "fault_injection_target" "ats_rog_map_node pid(s)=${STOPPED_NODE_PIDS:-unverified}"
    ;;
  emergency-stop-recovery)
    log "verifying the owned action-cancel safety transition"
    [ -n "${RECOVERY_CANCEL_RESULT:-}" ] || \
      fail "recovery cancel client produced no structured result"
    printf '%s\n' "$RECOVERY_CANCEL_RESULT" | grep -q 'goal_accepted=yes' || \
      fail "recovery cancel client goal was not accepted"
    printf '%s\n' "$RECOVERY_CANCEL_RESULT" | grep -q 'cmd_vel_nonzero=yes' || \
      fail "recovery cancel client did not observe a real non-zero /cmd_vel_mpc"
    printf '%s\n' "$RECOVERY_CANCEL_RESULT" | grep -q 'cancel_accepted=yes' || \
      fail "goal manager did not accept the owned action cancellation"
    printf '%s\n' "$RECOVERY_CANCEL_RESULT" | grep -q 'action_result=CANCELED' || \
      fail "owned action did not finish with CANCELED"
    printf '%s\n' "$RECOVERY_CANCEL_RESULT" | grep -q 'action_result_code=1' || \
      fail "owned action returned a non-cancel terminal code"
    RECOVERY_ESTOP_RAW="$(safety_echo_once)"
    RECOVERY_ESTOP="$(printf '%s\n' "$RECOVERY_ESTOP_RAW" | awk '/data:/ {print $2; exit}')"
    metric "planner_emergency_stop_after_cancel" "${RECOVERY_ESTOP:-unverified}"
    [ "$RECOVERY_ESTOP" = "true" ] || \
      fail "action cancellation did not latch planner emergency stop"
    ZERO_WINDOW_SAMPLES="${ZERO_WINDOW_SAMPLES:-3}"
    observe_zero_window "cancelled goal" /cmd_vel_mpc twist_is_zero "$ZERO_WINDOW_SAMPLES" \
      "$RUN_DIR/cancel_cmd_vel_mpc_window.log" || \
      fail "old reference resumed a non-zero /cmd_vel_mpc before a new goal"
    metric "cancel_zero_command_window_samples" "$ZERO_WINDOW_SAMPLES"
    # Keep the post-cancel acceptance segment short.  It must still traverse a
    # fresh JPS/MINCO/MPC reference, but this fault test is not a long-duration
    # Point-LIO stability benchmark.
    RECOVERY_GOAL_X="${RECOVERY_GOAL_X:-0.5}"
    RECOVERY_GOAL_Y="${RECOVERY_GOAL_Y:-0.0}"
    RECOVERY_OUTPUT="$RUN_DIR/recovery_goal_action.log"
    RECOVERY_ERROR="$RUN_DIR/recovery_goal_action.err"
    log "dispatching a new recovery goal ($RECOVERY_GOAL_X, $RECOVERY_GOAL_Y)"
    if start_goal_action "$RECOVERY_GOAL_X" "$RECOVERY_GOAL_Y" \
      "$RECOVERY_OUTPUT" "$RECOVERY_ERROR"; then
      metric "recovery_goal_dispatch" "action_sent pid=$GOAL_PID session=$GOAL_SESSION_ID"
      if ! wait_for_goal_action "recovery" "${RECOVERY_GOAL_RESULT_WAIT_SEC:-$GOAL_RESULT_WAIT_SEC}"; then
        fail "recovery goal did not reach a terminal result before its deadline"
      fi
    else
      fail "could not establish recovery action-client session"
      metric "recovery_goal_dispatch" "action_session_unverified"
    fi
    RECOVERY_SOURCE_GENERATION_AFTER="$(status_value rog_generation)"
    RECOVERY_PUBLICATION_SEQUENCE_AFTER="$(status_value publication_sequence)"
    metric "recovery_source_generation_after_goal" "${RECOVERY_SOURCE_GENERATION_AFTER:-unverified}"
    metric "recovery_publication_sequence_after_goal" "${RECOVERY_PUBLICATION_SEQUENCE_AFTER:-unverified}"
    if ! strictly_increases "$RECOVERY_SOURCE_GENERATION_BEFORE" "$RECOVERY_SOURCE_GENERATION_AFTER"; then
      fail "ROGMap source generation did not continue after emergency-stop recovery"
    fi
    if ! strictly_increases "$RECOVERY_PUBLICATION_SEQUENCE_BEFORE" "$RECOVERY_PUBLICATION_SEQUENCE_AFTER"; then
      fail "adapter publication sequence did not continue after emergency-stop recovery"
    fi
    ;;
esac

# An unreachable goal has no process to stop.  Its evidence is therefore the
# action result itself: the goal manager must terminate the active request
# rather than leave an old reference or client alive until launch shutdown.
if [ "$P2_FAULT_CASE" = "goal-unreachable" ] && [ -n "${GOAL_PID:-}" ]; then
  if ! wait_for_goal_action "unreachable-goal" "$GOAL_RESULT_WAIT_SEC"; then
    fail "unreachable goal did not reach a terminal result before its deadline"
  fi
fi

# ---------------------------------------------------------------------------
# Planning and control evidence
# ---------------------------------------------------------------------------

log "sampling planning and control stage"

# The action is asynchronous. Keep the read-only recorder alive until the
# nominal action reaches a terminal result (or its declared timeout), otherwise
# a short post-dispatch delay can end observation before JPS/MINCO has produced
# its first valid snapshot. The recorder is stopped immediately afterwards so
# final topic samples still represent the deterministic stopped state.
if [ "$P2_FAULT_CASE" = "none" ] && [ -n "${GOAL_PID:-}" ]; then
  log "waiting for nominal action result (timeout ${GOAL_RESULT_WAIT_SEC}s)"
  GOAL_RESULT_DEADLINE=$((SECONDS + GOAL_RESULT_WAIT_SEC))
  while kill -0 "$GOAL_PID" 2>/dev/null && [ "$SECONDS" -lt "$GOAL_RESULT_DEADLINE" ]; do
    sleep 1
  done
  if kill -0 "$GOAL_PID" 2>/dev/null; then
    fail "nominal action did not finish before GOAL_RESULT_WAIT_SEC"
  else
    wait "$GOAL_PID" 2>/dev/null || true
  fi
fi

# Let a short action finish without truncating the configured P1 observation.
wait_for_active_evidence_window
# Freeze the action-lifetime evidence before terminal topic samples. A finished
# action is required to publish zero velocity, so only these read-only logs can
# distinguish a completed trajectory from a goal that never commanded motion.
stop_active_observers
capture_launch_process_resources "active_end"
EVIDENCE_RESULT="$(awk '/^ATS_NAVIGATION_EVIDENCE_RESULT / {line=$0} END {print line}' "$ACTIVE_EVIDENCE_LOG")"
if [ -z "$EVIDENCE_RESULT" ]; then
  fail "active C++ navigation evidence recorder produced no result"
fi
JPS_POINTS="$(evidence_value jps_max_points)"
MINCO_POINTS="$(evidence_value minco_max_points)"
MPC_PRED_POINTS="$(evidence_value mpc_predicted_max_points)"
EXEC_POINTS="$(evidence_value mpc_executed_max_points)"
CMD_NONZERO_OBSERVED="$(evidence_bool_as_int "$(evidence_value cmd_vel_nonzero)")"
metric "active_evidence" "${EVIDENCE_RESULT:-unverified}"
metric "active_evidence_completed" "$(evidence_value completed)"
metric "active_evidence_duration_s" "$(evidence_value duration_s)"
metric "active_evidence_required_duration_s" "$ACTIVE_EVIDENCE_DURATION_SEC"
metric "launch_process_resources" "$PROCESS_RESOURCE_LOG"
metric "gazebo_transport_lidar_observation_enabled" \
  "$(evidence_value gazebo_transport_lidar_observation_enabled)"
metric "gazebo_transport_lidar_subscription_established" \
  "$(evidence_value gazebo_transport_lidar_subscription_established)"
metric "gazebo_transport_lidar_topic" "$(evidence_value gazebo_transport_lidar_topic)"
for stage in clock gazebo_transport_lidar gazebo_lidar livox_input cloud_registered lidar_odometry odometry localization localization_status; do
  metric "${stage}_samples" "$(evidence_value "${stage}_samples")"
  metric "${stage}_p50_wall_interval_s" "$(evidence_value "${stage}_p50_wall_interval_s")"
  metric "${stage}_p95_wall_interval_s" "$(evidence_value "${stage}_p95_wall_interval_s")"
  metric "${stage}_p99_wall_interval_s" "$(evidence_value "${stage}_p99_wall_interval_s")"
  metric "${stage}_max_wall_interval_s" "$(evidence_value "${stage}_max_wall_interval_s")"
  metric "${stage}_p50_stamp_interval_s" "$(evidence_value "${stage}_p50_stamp_interval_s")"
  metric "${stage}_p95_stamp_interval_s" "$(evidence_value "${stage}_p95_stamp_interval_s")"
  metric "${stage}_p99_stamp_interval_s" "$(evidence_value "${stage}_p99_stamp_interval_s")"
  metric "${stage}_max_stamp_interval_s" "$(evidence_value "${stage}_max_stamp_interval_s")"
  metric "${stage}_p50_stamp_age_s" "$(evidence_value "${stage}_p50_stamp_age_s")"
  metric "${stage}_p95_stamp_age_s" "$(evidence_value "${stage}_p95_stamp_age_s")"
  metric "${stage}_p99_stamp_age_s" "$(evidence_value "${stage}_p99_stamp_age_s")"
  metric "${stage}_max_stamp_age_s" "$(evidence_value "${stage}_max_stamp_age_s")"
  metric "${stage}_duplicate_stamp_count" "$(evidence_value "${stage}_duplicate_stamp_count")"
  metric "${stage}_backward_stamp_count" "$(evidence_value "${stage}_backward_stamp_count")"
  metric "${stage}_invalid_stamp_count" "$(evidence_value "${stage}_invalid_stamp_count")"
  metric "${stage}_future_stamp_count" "$(evidence_value "${stage}_future_stamp_count")"
  metric "${stage}_callback_count" "$(evidence_value "${stage}_callback_count")"
  metric "${stage}_p50_callback_duration_s" "$(evidence_value "${stage}_p50_callback_duration_s")"
  metric "${stage}_p95_callback_duration_s" "$(evidence_value "${stage}_p95_callback_duration_s")"
  metric "${stage}_p99_callback_duration_s" "$(evidence_value "${stage}_p99_callback_duration_s")"
  metric "${stage}_max_callback_duration_s" "$(evidence_value "${stage}_max_callback_duration_s")"
done
FRESHNESS_CLASSIFICATION="$(classify_gazebo_freshness "$EVIDENCE_RESULT" \
  "${P1_LOCALIZATION_P99_INTERVAL_LIMIT_SEC:-0.25}" \
  "${P1_LOCALIZATION_MAX_GAP_LIMIT_SEC:-0.5}")"
FRESHNESS_CLASSIFICATION_RC=$?
metric "p1_freshness_contract" "${FRESHNESS_CLASSIFICATION//$'\n'/ }"
metric "p1_first_freshness_violation" "$(freshness_metric_value "$FRESHNESS_CLASSIFICATION" first_violation)"
metric "p1_first_freshness_reason" "$(freshness_metric_value "$FRESHNESS_CLASSIFICATION" reason)"
if [ "$FRESHNESS_CLASSIFICATION_RC" -ne 0 ]; then
  fail "P1 freshness classification is unverified: $FRESHNESS_CLASSIFICATION"
fi
metric "clock_rtf_p50" "$(evidence_value clock_rtf_p50)"
metric "clock_rtf_p95" "$(evidence_value clock_rtf_p95)"
metric "clock_rtf_p99" "$(evidence_value clock_rtf_p99)"
metric "localization_status_tracking_samples" "$(evidence_value localization_status_tracking_samples)"
metric "localization_status_non_tracking_samples" "$(evidence_value localization_status_non_tracking_samples)"
metric "localization_status_last_state" "$(evidence_value localization_status_last_state)"
metric "localization_status_observation_age_p50_s" "$(evidence_value localization_status_observation_age_p50_s)"
metric "localization_status_observation_age_p95_s" "$(evidence_value localization_status_observation_age_p95_s)"
metric "localization_status_observation_age_p99_s" "$(evidence_value localization_status_observation_age_p99_s)"
metric "tf_lookup_attempts" "$(evidence_value tf_lookup_attempts)"
metric "tf_lookup_successes" "$(evidence_value tf_lookup_successes)"
metric "tf_lookup_failures" "$(evidence_value tf_lookup_failures)"
metric "tf_lookup_max_ms" "$(evidence_value tf_lookup_max_ms)"
metric "dds_queue_drop_counter" "$(evidence_value dds_queue_drop_counter)"
metric "adapter_max_wall_interval_s" "$(evidence_value adapter_max_wall_interval_s)"
metric "adapter_status_callback_count" "$(evidence_value adapter_status_callback_count)"
metric "adapter_status_callback_p50_s" "$(evidence_value adapter_status_callback_p50_s)"
metric "adapter_status_callback_p95_s" "$(evidence_value adapter_status_callback_p95_s)"
metric "adapter_status_callback_p99_s" "$(evidence_value adapter_status_callback_p99_s)"
metric "adapter_status_callback_max_s" "$(evidence_value adapter_status_callback_max_s)"
metric "adapter_ready_seen_active" "$(evidence_value adapter_ready_seen)"
metric "adapter_source_generation_active" "$(evidence_value adapter_source_generation_begin)->$(evidence_value adapter_source_generation_end)"
metric "adapter_publication_sequence_active" "$(evidence_value adapter_publication_sequence_begin)->$(evidence_value adapter_publication_sequence_end)"

metric "jps_raw_path_points" "${JPS_POINTS:-unverified}"
metric "minco_preprocessed_guide_points" "$(evidence_value preprocessed_guide_max_points)"
metric "minco_esdf_refined_guide_points" "$(evidence_value esdf_refined_guide_max_points)"
metric "minco_reference_path_points" "${MINCO_POINTS:-unverified}"
metric "mpc_predicted_path_points" "${MPC_PRED_POINTS:-unverified}"
metric "mpc_executed_path_points" "${EXEC_POINTS:-unverified}"
metric "cmd_vel_mpc_nonzero_observed" "${CMD_NONZERO_OBSERVED:-unverified}"

# Ownership is sampled by the C++ recorder throughout the action lifetime.
# The ros2cli snapshots remain in active_ownership.log for diagnostics only:
# their final sample can race teardown and report a transient zero writer.
CMD_ACTIVE_PUB="$(evidence_value cmd_vel_mpc_publisher_max)"
CMD_ACTIVE_SUB="$(evidence_value cmd_vel_mpc_subscriber_max)"
GRID_ACTIVE_PUB="$(evidence_value planning_grid_publisher_max)"
GRID_ACTIVE_SUB="$(evidence_value planning_grid_subscriber_max)"
GRID_ACTIVE_PUBLISHERS="$(evidence_value planning_grid_publisher_names)"
GRID_ADAPTER_SEEN="$(evidence_value planning_grid_adapter_seen)"
GRID_NAMED_NON_ADAPTER_SEEN="$(evidence_value planning_grid_named_non_adapter_seen)"
GRID_ANONYMOUS_ENDPOINT_SEEN="$(evidence_value planning_grid_anonymous_endpoint_seen)"
metric "cmd_vel_mpc_pub/sub_active" "${CMD_ACTIVE_PUB:-unverified}/${CMD_ACTIVE_SUB:-unverified}"
metric "planning_grid_pub/sub_active" "${GRID_ACTIVE_PUB:-unverified}/${GRID_ACTIVE_SUB:-unverified}"
metric "planning_grid_publishers_active" "${GRID_ACTIVE_PUBLISHERS:-unverified}"
metric "planning_grid_adapter_seen_active" "${GRID_ADAPTER_SEEN:-unverified}"
metric "planning_grid_named_non_adapter_seen_active" "${GRID_NAMED_NON_ADAPTER_SEEN:-unverified}"
metric "planning_grid_anonymous_endpoint_seen_active" "${GRID_ANONYMOUS_ENDPOINT_SEEN:-unverified}"
if [ "$P2_FAULT_CASE" = "none" ]; then
  [ "${GRID_ACTIVE_PUB:-0}" = "1" ] || fail "/rc_esdf/planning_grid must have exactly one active publisher, got ${GRID_ACTIVE_PUB:-unverified}"
  [ "${GRID_ADAPTER_SEEN:-no}" = "yes" ] || \
    fail "/rc_esdf/planning_grid never identified /ats_rog_map_adapter as its active publisher"
  [ "${GRID_NAMED_NON_ADAPTER_SEEN:-yes}" = "no" ] || \
    fail "/rc_esdf/planning_grid identified a named non-adapter publisher: ${GRID_ACTIVE_PUBLISHERS:-unverified}"
  [ "${CMD_ACTIVE_PUB:-0}" = "1" ] || fail "/cmd_vel_mpc must have exactly one active publisher, got ${CMD_ACTIVE_PUB:-unverified}"
else
  metric "fault_owner_snapshot" "informational_only; nominal action lifetime enforces unique publishers"
fi

read -r CMD_PUB CMD_SUB <<<"$(topic_counts /cmd_vel_mpc)"
metric "cmd_vel_mpc_pub/sub_terminal" "${CMD_PUB:-?}/${CMD_SUB:-?}"

metric "cmd_vel_mpc_hz" "$(topic_hz /cmd_vel_mpc)"

ESTOP="$(echo_once /planner/emergency_stop | awk '/data:/ {print $2; exit}')"
metric "planner_emergency_stop" "${ESTOP:-unverified}"

twist_is_nonzero() {
  awk '
    /^[[:space:]]+[xyz]:/ {
      value = $2 + 0.0
      if (value < -1e-9 || value > 1e-9) nonzero = 1
      fields++
    }
    END { exit !(fields == 6 && nonzero) }
  '
}

CMD_DUMP="$(echo_once /cmd_vel_mpc)"
metric "cmd_vel_mpc_sample" "${CMD_DUMP//$'\n'/ }"

TERMINAL_LOCALIZATION_POSE="$(echo_once /localization \
  | awk '/position:/{f=1} f&&/x:/{x=$2} f&&/y:/{y=$2} f&&/z:/{print x" "y; exit}')"
metric "terminal_localization_pose_xy" "${TERMINAL_LOCALIZATION_POSE:-unverified}"

if [ -n "$GOAL_X" ] && [ -n "$GOAL_Y" ] && [ -n "$TERMINAL_LOCALIZATION_POSE" ]; then
  TERMINAL_POS_ERR="$(python3 -c "
import math
p='''$TERMINAL_LOCALIZATION_POSE'''.split()
print(round(math.hypot(float(p[0])-$GOAL_X, float(p[1])-$GOAL_Y), 4))
" 2>/dev/null || echo unverified)"
else
  TERMINAL_POS_ERR="unverified"
fi
metric "terminal_localization_goal_error_m" "$TERMINAL_POS_ERR"

if [ -s "$GOAL_OUTPUT" ]; then
  GOAL_ACCEPTED="$(grep -c '^Goal accepted' "$GOAL_OUTPUT" || true)"
  GOAL_SUCCEEDED="$(grep -c 'Goal finished with status: SUCCEEDED' "$GOAL_OUTPUT" || true)"
  GOAL_RESULT="$(grep 'Goal finished with status:' "$GOAL_OUTPUT" | tail -n 1 || true)"
  GOAL_FINAL_DISTANCE="$(awk '/^final_distance:/ {value=$2} END {print value}' "$GOAL_OUTPUT")"
  GOAL_FINAL_POSE="$(awk '
    /^final_pose:/ {in_final_pose=1; next}
    in_final_pose && /^[^[:space:]]/ {in_final_pose=0}
    in_final_pose && /position:/ {in_position=1; next}
    in_position && /^[[:space:]]+x:/ {x=$2}
    in_position && /^[[:space:]]+y:/ {y=$2; print x " " y; exit}
  ' "$GOAL_OUTPUT")"
else
  GOAL_ACCEPTED=0
  GOAL_SUCCEEDED=0
  GOAL_RESULT="unverified"
  GOAL_FINAL_DISTANCE="unverified"
  GOAL_FINAL_POSE="unverified"
fi
metric "goal_action_accepted" "$GOAL_ACCEPTED"
metric "goal_action_succeeded" "$GOAL_SUCCEEDED"
metric "goal_action_result" "${GOAL_RESULT:-unverified}"
metric "goal_action_final_distance_m" "${GOAL_FINAL_DISTANCE:-unverified}"
metric "goal_action_final_pose_xy" "${GOAL_FINAL_POSE:-unverified}"
if [ "$GOAL_FINAL_POSE" != "unverified" ] && [ -n "$TERMINAL_LOCALIZATION_POSE" ]; then
  POST_GOAL_LOCALIZATION_DELTA="$(python3 -c "
import math
action='''$GOAL_FINAL_POSE'''.split()
terminal='''$TERMINAL_LOCALIZATION_POSE'''.split()
print(round(math.hypot(float(terminal[0])-float(action[0]), float(terminal[1])-float(action[1])), 4))
" 2>/dev/null || echo unverified)"
else
  POST_GOAL_LOCALIZATION_DELTA="unverified"
fi
metric "post_goal_localization_delta_m" "$POST_GOAL_LOCALIZATION_DELTA"
if [ "$P2_FAULT_CASE" = "none" ] && [ "$GOAL_ACCEPTED" -ne 1 ]; then
  fail "nominal action was not accepted"
fi
if [ "$P2_FAULT_CASE" = "none" ]; then
  [ "$GOAL_SUCCEEDED" -eq 1 ] || fail "nominal action did not succeed"
  [ "${JPS_POINTS:-0}" -gt 1 ] 2>/dev/null || fail "nominal JPS path is empty"
  [ "${MINCO_POINTS:-0}" -gt 1 ] 2>/dev/null || fail "nominal MINCO reference is empty"
  [ "${MPC_PRED_POINTS:-0}" -gt 1 ] 2>/dev/null || fail "nominal MPC predicted path is empty"
  [ "${EXEC_POINTS:-0}" -gt 1 ] 2>/dev/null || fail "nominal executed path is empty"
  [ "${CMD_NONZERO_OBSERVED:-0}" = "1" ] || fail "nominal /cmd_vel_mpc stayed zero"
fi
if [ "$P2_FAULT_CASE" = "emergency-stop-recovery" ]; then
  RECOVERY_ACCEPTED=0
  RECOVERY_SUCCEEDED=0
  RECOVERY_RESULT="unverified"
  if [ -s "${RECOVERY_OUTPUT:-}" ]; then
    RECOVERY_ACCEPTED="$(grep -c '^Goal accepted' "$RECOVERY_OUTPUT" || true)"
    RECOVERY_SUCCEEDED="$(grep -c 'Goal finished with status: SUCCEEDED' "$RECOVERY_OUTPUT" || true)"
    RECOVERY_RESULT="$(grep 'Goal finished with status:' "$RECOVERY_OUTPUT" | tail -n 1 || true)"
  fi
  metric "recovery_goal_action_accepted" "$RECOVERY_ACCEPTED"
  metric "recovery_goal_action_succeeded" "$RECOVERY_SUCCEEDED"
  metric "recovery_goal_action_result" "${RECOVERY_RESULT:-unverified}"
  [ "$RECOVERY_ACCEPTED" -eq 1 ] || fail "recovery goal was not accepted after action cancellation"
  [ "$RECOVERY_SUCCEEDED" -eq 1 ] || fail "recovery goal did not replan and complete after action cancellation"
fi
if [ "$P2_FAULT_CASE" = "goal-unreachable" ]; then
  [ "$GOAL_ACCEPTED" -eq 1 ] || fail "unreachable goal was not accepted for planning"
  case "$GOAL_RESULT" in
    *"ABORTED"*) ;;
    *) fail "unreachable goal did not finish with ABORTED" ;;
  esac
fi

set_p1_admission_evidence
metric "p1_admission_evidence" "$P1_ADMISSION_EVIDENCE"
metric "p1_admission_reason" "$P1_ADMISSION_REASON"

# There is no standalone Gazebo contact evaluator in this profile, so physical
# contact must stay explicitly unverified. footprint_collisions=0 is a planner
# metric and never evidence of zero physical contact.
metric "minimum_clearance_m" "unverified"
metric "minco_footprint_collisions" "unverified"
metric "gazebo_contact_telemetry" "unverified"
metric "物理接触评估" "未验证"

metric "failure_count" "$FAILURE_COUNT"
metric "recovery_count" "unverified"
metric "first_failure_reason" "${FIRST_FAILURE:-none}"

# ---------------------------------------------------------------------------
# Zero-velocity gate for fault profiles
# ---------------------------------------------------------------------------

if [ "$P2_FAULT_CASE" != "none" ]; then
  log "verifying zero-velocity fallback for $P2_FAULT_CASE"
  FAULT_ESTOP_RAW="$(safety_echo_once)"
  FAULT_ESTOP="$(printf '%s\n' "$FAULT_ESTOP_RAW" | awk '/data:/ {print $2; exit}')"
  metric "fault_planner_emergency_stop" "${FAULT_ESTOP:-unverified}"
  [ "$FAULT_ESTOP" = "true" ] || \
    fail "fault case did not observe planner emergency stop=true"
  CMD_DUMP_RAW="$(echo_once /cmd_vel_mpc)"
  CMD_DUMP="$(printf '%s\n' "$CMD_DUMP_RAW" | tr '\n' ' ')"
  metric "fault_cmd_vel_mpc" "${CMD_DUMP:-unverified}"
  metric "fault_note" "$FAULT_NOTE"
  printf '%s\n' "$CMD_DUMP_RAW" | twist_is_zero || \
    fail "fault case did not observe zero /cmd_vel_mpc"
  case "$P2_FAULT_CASE" in
    input-stale|map-stale|projection-timeout|all-unknown)
      FAULT_MAP_READY="$(echo_once /rog_map_adapter/ready | awk '/data:/ {print $2; exit}')"
      metric "fault_map_ready" "${FAULT_MAP_READY:-unverified}"
      [ "$FAULT_MAP_READY" = "false" ] || \
        fail "fault case did not observe adapter ready=false"
      ;;
    map-unready|adapter-lease)
      read -r FAULT_READY_PUB FAULT_READY_SUB <<<"$(topic_counts /rog_map_adapter/ready)"
      metric "fault_adapter_ready_pub/sub_terminal" "${FAULT_READY_PUB:-?}/${FAULT_READY_SUB:-?}"
      [ "${FAULT_READY_PUB:-unverified}" = "0" ] || \
        fail "adapter lease fault still has a ready-heartbeat publisher"
      ;;
  esac
fi

metric "failure_count_final" "$FAILURE_COUNT"
metric "first_failure_reason_final" "${FIRST_FAILURE:-none}"

log "collecting shutdown state"
refresh_topic_cache
cat "$TOPIC_CACHE" >>"$TOPIC_LOG" 2>/dev/null || true
ros2 node list --no-daemon >>"$TOPIC_LOG" 2>&1 || true

cleanup
trap - EXIT INT TERM
write_artifact_summary

[ "$FAILURE_COUNT" -eq 0 ] || exit 1
exit 0
