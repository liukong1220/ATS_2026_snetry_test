#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_SCRIPT="${SETUP_SCRIPT:-$WORKSPACE_DIR/install/setup.bash}"
if [[ ! -f "$SETUP_SCRIPT" ]]; then
  SETUP_SCRIPT="$WORKSPACE_DIR/install/setup.sh"
fi

WORLD_NAME="${WORLD_NAME:-${1:-rmuc}}"
VISION_CONFIG="${VISION_CONFIG:-$WORKSPACE_DIR/src/sp_vision25/configs/sentry.yaml}"
NAV_PARAMS_FILE="${NAV_PARAMS_FILE:-$WORKSPACE_DIR/src/pb2025_sentry_bringup/params/node_params.yaml}"

USE_RVIZ="${USE_RVIZ:-True}"
RVIZ_FORCE_SOFTWARE="${RVIZ_FORCE_SOFTWARE:-0}"
NAV_LOG_LEVEL="${NAV_LOG_LEVEL:-warn}"
LAUNCH_ROSBAG_RECORDER="${LAUNCH_ROSBAG_RECORDER:-False}"
LAUNCH_TRAJECTORY_OPTIMIZER="${LAUNCH_TRAJECTORY_OPTIMIZER:-False}"
LAUNCH_SMALL_GICP_RELOCALIZATION="${LAUNCH_SMALL_GICP_RELOCALIZATION:-False}"
USE_COMPOSITION="${USE_COMPOSITION:-False}"
USE_RESPAWN="${USE_RESPAWN:-True}"

RESTART_DELAY="${RESTART_DELAY:-2}"
STARTUP_GAP="${STARTUP_GAP:-6}"
WATCH_INTERVAL="${WATCH_INTERVAL:-1}"

TOTAL_CPUS="$(nproc 2>/dev/null || echo 4)"
if (( TOTAL_CPUS >= 16 )); then
  DEFAULT_VISION_CPUSET="0-8"
  DEFAULT_NAV_CPUSET="6-$((TOTAL_CPUS - 1))"
elif (( TOTAL_CPUS >= 12 )); then
  DEFAULT_VISION_CPUSET="0-6"
  DEFAULT_NAV_CPUSET="5-$((TOTAL_CPUS - 1))"
elif (( TOTAL_CPUS >= 8 )); then
  DEFAULT_VISION_CPUSET="0-4"
  DEFAULT_NAV_CPUSET="4-$((TOTAL_CPUS - 1))"
else
  DEFAULT_VISION_CPUSET=""
  DEFAULT_NAV_CPUSET=""
fi

VISION_CPUSET="${VISION_CPUSET:-$DEFAULT_VISION_CPUSET}"
NAV_CPUSET="${NAV_CPUSET:-$DEFAULT_NAV_CPUSET}"
VISION_NICE="${VISION_NICE:-0}"
NAV_NICE="${NAV_NICE:-5}"
VISION_OPENCV_THREADS="${VISION_OPENCV_THREADS:-6}"
VISION_OMP_NUM_THREADS="${VISION_OMP_NUM_THREADS:-6}"
NAV_OMP_NUM_THREADS="${NAV_OMP_NUM_THREADS:-6}"

export ROS_HOME="${ROS_HOME:-$WORKSPACE_DIR/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROS_HOME/log/competition}"
export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-1}"
export RCUTILS_COLORIZED_OUTPUT="${RCUTILS_COLORIZED_OUTPUT:-1}"
mkdir -p "$ROS_LOG_DIR"

WATCHDOG_LOG_FILE="${WATCHDOG_LOG_FILE:-$ROS_LOG_DIR/watchdog.log}"
if [[ -t 1 ]]; then
  exec > >(tee -a "$WATCHDOG_LOG_FILE") 2>&1
else
  exec >>"$WATCHDOG_LOG_FILE" 2>&1
fi

if [[ ! -f "$SETUP_SCRIPT" ]]; then
  echo "[$(date '+%F %T')] ERROR: ROS setup script not found: $SETUP_SCRIPT"
  exit 1
fi

LOCK_FILE="${LOCK_FILE:-$ROS_HOME/competition_watchdog.lock}"
if command -v flock >/dev/null 2>&1; then
  exec 9>"$LOCK_FILE"
  if ! flock -n 9; then
    echo "[$(date '+%F %T')] competition_watchdog is already running, exit this instance."
    exit 0
  fi
fi

NAV_PID=""
VISION_PID=""

is_running() {
  local pid="$1"
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

terminate_process() {
  local name="$1"
  local pid="$2"
  if is_running "$pid"; then
    echo "[$(date '+%H:%M:%S')] 停止 ${name} (pid=${pid})"
    kill "$pid" 2>/dev/null || true
    for _ in {1..20}; do
      is_running "$pid" || return 0
      sleep 0.1
    done
    kill -9 "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  echo
  echo "[$(date '+%H:%M:%S')] 比赛看门狗退出，清理子进程"
  terminate_process "vision" "$VISION_PID"
  terminate_process "nav2" "$NAV_PID"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

command_prefix() {
  local cpuset="$1"
  local nice_value="$2"
  local -n out_ref="$3"
  out_ref=()

  if [[ -n "$cpuset" ]] && command -v taskset >/dev/null 2>&1; then
    out_ref+=(taskset -c "$cpuset")
  fi

  if [[ "$nice_value" != "0" ]] && command -v nice >/dev/null 2>&1; then
    out_ref+=(nice -n "$nice_value")
  fi
}

start_nav2() {
  local -a prefix
  command_prefix "$NAV_CPUSET" "$NAV_NICE" prefix

  echo "[$(date '+%H:%M:%S')] 启动 Nav2: world=${WORLD_NAME}, cpuset=${NAV_CPUSET:-all}, nice=${NAV_NICE}"
  (
    # shellcheck disable=SC1090
    set +u
    source "$SETUP_SCRIPT"
    set -u
    cd "$WORKSPACE_DIR"
    export OMP_NUM_THREADS="$NAV_OMP_NUM_THREADS"
    export OPENBLAS_NUM_THREADS="$NAV_OMP_NUM_THREADS"
    export MKL_NUM_THREADS="$NAV_OMP_NUM_THREADS"
    exec "${prefix[@]}" ros2 launch pb2025_sentry_bringup bringup.launch.py\
      world:="$WORLD_NAME" \
      slam:=False \
      params_file:="$NAV_PARAMS_FILE" \
      use_rviz:="$USE_RVIZ" \
      rviz_force_software:="$RVIZ_FORCE_SOFTWARE" \
      use_composition:="$USE_COMPOSITION" \
      use_respawn:="$USE_RESPAWN" \
      launch_rosbag_recorder:="$LAUNCH_ROSBAG_RECORDER" \
      launch_trajectory_optimizer:="$LAUNCH_TRAJECTORY_OPTIMIZER" \
      launch_small_gicp_relocalization:="$LAUNCH_SMALL_GICP_RELOCALIZATION" \
      log_level:="$NAV_LOG_LEVEL"
  ) >>"$ROS_LOG_DIR/nav2.log" 2>&1 &
  NAV_PID="$!"
  echo "$NAV_PID" >"$ROS_HOME/nav2_watchdog.pid"
}

start_vision() {
  local -a prefix
  command_prefix "$VISION_CPUSET" "$VISION_NICE" prefix

  echo "[$(date '+%H:%M:%S')] 启动视觉: config=${VISION_CONFIG}, cpuset=${VISION_CPUSET:-all}, nice=${VISION_NICE}"
  (
    # shellcheck disable=SC1090
    set +u
    source "$SETUP_SCRIPT"
    set -u
    cd "$WORKSPACE_DIR"
    export SP_VISION_OPENCV_THREADS="$VISION_OPENCV_THREADS"
    export OMP_NUM_THREADS="$VISION_OMP_NUM_THREADS"
    export OPENBLAS_NUM_THREADS="$VISION_OMP_NUM_THREADS"
    export MKL_NUM_THREADS="$VISION_OMP_NUM_THREADS"
    exec "${prefix[@]}" ros2 run sp_vision25 sentry "$VISION_CONFIG"
  ) >>"$ROS_LOG_DIR/vision.log" 2>&1 &
  VISION_PID="$!"
  echo "$VISION_PID" >"$ROS_HOME/vision_watchdog.pid"
}

echo "=========================================="
echo "比赛看门狗已启动，Ctrl+C 退出"
echo "Nav2 world: ${WORLD_NAME}"
echo "Nav2 params: ${NAV_PARAMS_FILE}"
echo "Vision config: ${VISION_CONFIG}"
echo "Logs: ${ROS_LOG_DIR}"
echo "Watchdog log: ${WATCHDOG_LOG_FILE}"
echo "Nav2: cpuset=${NAV_CPUSET:-all}, nice=${NAV_NICE}, OMP=${NAV_OMP_NUM_THREADS}"
echo "Nav2 options: composition=${USE_COMPOSITION}, small_gicp=${LAUNCH_SMALL_GICP_RELOCALIZATION}, trajectory_optimizer=${LAUNCH_TRAJECTORY_OPTIMIZER}, rviz=${USE_RVIZ}"
echo "Vision: cpuset=${VISION_CPUSET:-all}, nice=${VISION_NICE}, OpenCV=${VISION_OPENCV_THREADS}, OMP=${VISION_OMP_NUM_THREADS}"
echo "=========================================="

start_nav2
sleep "$STARTUP_GAP"
start_vision

while true; do
  if ! is_running "$NAV_PID"; then
    echo "[$(date '+%H:%M:%S')] Nav2 已退出，${RESTART_DELAY}s 后重启"
    sleep "$RESTART_DELAY"
    start_nav2
  fi

  if ! is_running "$VISION_PID"; then
    echo "[$(date '+%H:%M:%S')] 视觉已退出，${RESTART_DELAY}s 后重启"
    sleep "$RESTART_DELAY"
    start_vision
  fi

  sleep "$WATCH_INTERVAL"
done
