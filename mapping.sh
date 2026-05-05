#!/usr/bin/env bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_SCRIPT="$WORKSPACE_DIR/install/setup.bash"
MAP_NAME="${1:-}"
USE_RVIZ="${USE_RVIZ:-True}"
PID_FILE="$WORKSPACE_DIR/.ros/mapping_sh.pid"
MAP_OUTPUT_PREFIX="$WORKSPACE_DIR/src/pb2025_sentry_bringup/map/$MAP_NAME"
PCD_OUTPUT_FILE="$WORKSPACE_DIR/src/pb2025_sentry_bringup/pcd/$MAP_NAME.pcd"
PCD_GLOB="$WORKSPACE_DIR/src/pb2025_sentry_nav/point_lio/PCD/scans_*.pcd"
PCD_WAIT_TIMEOUT="${PCD_WAIT_TIMEOUT:-5}"
PCD_WAIT_INTERVAL="${PCD_WAIT_INTERVAL:-1}"
STOP_REQUESTED=0

export ROS_HOME="$WORKSPACE_DIR/.ros"
cd "$WORKSPACE_DIR" || exit 1
source "$SETUP_SCRIPT"

prompt_map_name() {
  if [[ -z "$MAP_NAME" ]]; then
    read -r -p "请输入地图名: " MAP_NAME
  fi

  if [[ -z "$MAP_NAME" ]]; then
    echo "地图名不能为空"
    exit 1
  fi

  if [[ ! "$MAP_NAME" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "地图名仅支持字母、数字、下划线和中划线: $MAP_NAME"
    exit 1
  fi
}

print_save_hints() {
  echo "建图模式已配置，地图名: $MAP_NAME"
  echo "按 Ctrl+C 结束时，脚本会先询问是否保存地图，然后自动停止建图并复制最新 PCD。"
  echo "地图输出前缀: $MAP_OUTPUT_PREFIX"
  echo "PCD 输出文件: $PCD_OUTPUT_FILE"
}

start_command() {
  local cmd="$1"
  mkdir -p "$(dirname "$PID_FILE")"
  gnome-terminal -- bash -lc "echo \$\$ > \"$PID_FILE\"; cd \"$WORKSPACE_DIR\"; source \"$SETUP_SCRIPT\"; $cmd; rm -f \"$PID_FILE\"; exec bash"
}

is_running() {
  if [[ ! -f "$PID_FILE" ]]; then
    return 1
  fi

  local pid
  pid="$(<"$PID_FILE")"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    return 0
  fi

  rm -f "$PID_FILE"
  return 1
}

confirm_yes() {
  local prompt="$1"
  local reply
  read -r -p "$prompt [Y/n]: " reply
  [[ -z "$reply" || "$reply" =~ ^[Yy]$ ]]
}

save_map() {
  echo "正在保存栅格地图到 $MAP_OUTPUT_PREFIX"
  if ! ros2 run nav2_map_server map_saver_cli -f "$MAP_OUTPUT_PREFIX"; then
    echo "地图保存失败，请确认建图节点仍在运行并检查终端日志"
    return 1
  fi

  echo "地图已保存: ${MAP_OUTPUT_PREFIX}.yaml / ${MAP_OUTPUT_PREFIX}.pgm"
}

copy_latest_pcd() {
  local latest_pcd
  latest_pcd="$(ls -t $PCD_GLOB 2>/dev/null | head -n 1)"

  if [[ -z "$latest_pcd" ]]; then
    echo "未找到生成的 PCD 文件，跳过复制"
    return 1
  fi

  cp "$latest_pcd" "$PCD_OUTPUT_FILE"
  echo "最新 PCD 已复制: $latest_pcd -> $PCD_OUTPUT_FILE"
}

wait_for_latest_pcd() {
  local waited=0
  local latest_pcd

  while (( waited < PCD_WAIT_TIMEOUT )); do
    latest_pcd="$(ls -t $PCD_GLOB 2>/dev/null | head -n 1)"
    if [[ -n "$latest_pcd" ]]; then
      echo "检测到最新 PCD: $latest_pcd"
      return 0
    fi

    sleep "$PCD_WAIT_INTERVAL"
    waited=$(( waited + PCD_WAIT_INTERVAL ))
  done

  echo "等待 $PCD_WAIT_TIMEOUT 秒后仍未检测到 PCD 文件"
  return 1
}

stop_launch() {
  local pid

  if ! is_running; then
    return 0
  fi

  pid="$(<"$PID_FILE")"
  echo "正在停止建图进程 PID=$pid"
  pkill -TERM -P "$pid" 2>/dev/null || true
  kill -TERM "$pid" 2>/dev/null || true

  for _ in {1..10}; do
    if ! kill -0 "$pid" 2>/dev/null; then
      rm -f "$PID_FILE"
      return 0
    fi
    sleep 1
  done

  echo "建图终端未在预期时间内退出，请手动检查"
  return 1
}

handle_shutdown() {
  if (( STOP_REQUESTED )); then
    return
  fi

  STOP_REQUESTED=1
  echo
  echo "收到退出请求，开始收尾流程"

  if is_running && confirm_yes "是否先保存当前地图"; then
    save_map || true
  fi

  stop_launch || true

  if confirm_yes "是否复制最新 PCD 为地图同名文件"; then
    wait_for_latest_pcd || true
    copy_latest_pcd || true
  fi

  exit 0
}

start_commands() {
  for cmd in "${commands[@]}"; do
    start_command "$cmd"
    sleep 3
  done
}

watch_commands() {
  while true; do
    if (( STOP_REQUESTED )); then
      break
    fi

    for cmd in "${commands[@]}"; do
      if ! is_running; then
        if (( STOP_REQUESTED )); then
          break
        fi
        echo "$cmd 未在运行, 重新启动"
        start_command "$cmd"
        sleep 3
      fi
    done
    sleep 5
  done
}

prompt_map_name

declare -a commands=(
  "ros2 launch pb2025_sentry_bringup bringup.launch.py world:=$MAP_NAME slam:=True use_rviz:=$USE_RVIZ"
)

MAP_OUTPUT_PREFIX="$WORKSPACE_DIR/src/pb2025_sentry_bringup/map/$MAP_NAME"
PCD_OUTPUT_FILE="$WORKSPACE_DIR/src/pb2025_sentry_bringup/pcd/$MAP_NAME.pcd"

trap handle_shutdown INT TERM

print_save_hints
start_commands
watch_commands
