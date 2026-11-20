#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
SETUP_SCRIPT="${SETUP_SCRIPT:-$WORKSPACE_DIR/install/setup.bash}"

MAP_NAME="${1:-}"
if (( $# > 0 )); then
  shift
fi

USE_RVIZ="${USE_RVIZ:-False}"
AUTO_SAVE="${AUTO_SAVE:-0}"
PID_FILE="${PID_FILE:-$WORKSPACE_DIR/.ros/mapping_sh.pid}"
PCD_SOURCE_DIR="$WORKSPACE_DIR/src/ats_sentry_nav/point_lio/PCD"
SESSION_START_UNIX="$(date +%s)"
STOP_REQUESTED=0
LAUNCH_PID=""

export ROS_HOME="${ROS_HOME:-$WORKSPACE_DIR/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROS_HOME/log}"
mkdir -p "$ROS_LOG_DIR" "$(dirname "$PID_FILE")"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[mapping] ERROR: ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi
if [[ ! -f "$SETUP_SCRIPT" ]]; then
  echo "[mapping] ERROR: Workspace setup not found: $SETUP_SCRIPT (please run ./build.sh first)" >&2
  exit 1
fi

cd "$WORKSPACE_DIR"
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$SETUP_SCRIPT"
set -u

prompt_map_name() {
  if [[ -z "$MAP_NAME" ]]; then
    if [[ -t 0 ]]; then
      read -r -p "请输入地图名 [默认 rmuc_2027]: " MAP_NAME
    fi
    MAP_NAME="${MAP_NAME:-rmuc_2027}"
  fi
  if [[ ! "$MAP_NAME" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "[mapping] ERROR: 地图名仅支持字母、数字、下划线和中划线: $MAP_NAME" >&2
    exit 1
  fi
}

confirm_yes() {
  local prompt="$1"
  if [[ "$AUTO_SAVE" == "1" || "${AUTO_SAVE,,}" == "true" ]]; then
    return 0
  fi
  if [[ ! -t 0 ]]; then
    return 0
  fi
  local reply=""
  read -r -p "$prompt [Y/n]: " reply || true
  [[ -z "$reply" || "$reply" =~ ^[Yy]$ ]]
}

is_launch_running() {
  [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null
}

save_map() {
  echo "[mapping] 正在保存栅格地图与三维 PCD 到 ${MAP_OUTPUT_PREFIX} / ${PCD_OUTPUT_FILE}"
  if ! python3 "$WORKSPACE_DIR/scripts/save_s3_map.py" \
    --prefix "$MAP_OUTPUT_PREFIX" \
    --pcd-output "$PCD_OUTPUT_FILE"; then
    echo "[mapping] WARN: 地图保存失败，请确认建图节点仍在运行并检查日志" >&2
    return 1
  fi
  echo "[mapping] 地图已保存: ${MAP_OUTPUT_PREFIX}.yaml / ${MAP_OUTPUT_PREFIX}.pgm / ${PCD_OUTPUT_FILE}"
}

archive_pcd_segments() {
  if [[ -d "$PCD_SOURCE_DIR" ]]; then
    python3 "$WORKSPACE_DIR/scripts/save_s3_map.py" \
      --archive-source "$PCD_SOURCE_DIR" \
      --archive-output "${MAP_OUTPUT_PREFIX}_raw_segments" \
      --since "$SESSION_START_UNIX"
  fi
}

stop_launch() {
  if ! is_launch_running; then
    rm -f "$PID_FILE"
    return 0
  fi
  echo "[mapping] 正在停止建图进程 PID=${LAUNCH_PID}"
  kill -INT "-${LAUNCH_PID}" 2>/dev/null || kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  for _ in {1..15}; do
    if ! is_launch_running; then
      rm -f "$PID_FILE"
      return 0
    fi
    sleep 1
  done
  kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  sleep 2
  if is_launch_running; then
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
  fi
  rm -f "$PID_FILE"
}

handle_shutdown() {
  if (( STOP_REQUESTED )); then
    return
  fi
  STOP_REQUESTED=1
  echo
  echo "[mapping] 收到退出请求，开始收尾流程"

  if is_launch_running && confirm_yes "是否先保存当前地图"; then
    save_map || true
  fi

  stop_launch || true
  archive_pcd_segments || echo "[mapping] WARN: 原始 PCD 分段归档失败，请检查日志" >&2
  exit 0
}

prompt_map_name

MAP_OUTPUT_PREFIX="$WORKSPACE_DIR/src/ats_sentry_bringup/map/$MAP_NAME"
PCD_OUTPUT_FILE="$WORKSPACE_DIR/src/ats_sentry_bringup/pcd/$MAP_NAME.pcd"

trap handle_shutdown INT TERM

echo "=========================================="
echo "[mapping] 建图模式已配置，地图名: ${MAP_NAME}"
echo "[mapping] 按 Ctrl+C 结束时，通过服务保存 map 系地图与三维 PCD，并归档原始分段。"
echo "[mapping] 地图输出前缀: ${MAP_OUTPUT_PREFIX}"
echo "[mapping] PCD 输出文件: ${PCD_OUTPUT_FILE}"
echo "=========================================="

setsid ros2 launch ats_sentry_bringup bringup.launch.py \
  "world:=${MAP_NAME}" \
  "slam:=True" \
  "use_identified_odometry:=True" \
  "use_rviz:=${USE_RVIZ}" \
  "$@" &
LAUNCH_PID="$!"
echo "$LAUNCH_PID" > "$PID_FILE"

set +e
wait "$LAUNCH_PID"
LAUNCH_STATUS=$?
set -e
rm -f "$PID_FILE"

if (( ! STOP_REQUESTED )); then
  echo "[mapping] 建图进程已退出 (status=${LAUNCH_STATUS})"
  archive_pcd_segments || true
fi
exit "$LAUNCH_STATUS"
