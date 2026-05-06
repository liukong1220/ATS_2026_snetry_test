#!/usr/bin/env bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_SCRIPT="$WORKSPACE_DIR/install/setup.bash"
WORLD_NAME="${1:-rmul}"
USE_RVIZ="${USE_RVIZ:-True}"
RVIZ_FORCE_SOFTWARE="${RVIZ_FORCE_SOFTWARE:-0}"
PID_FILE="$WORKSPACE_DIR/.ros/nav2_sh.pid"

export ROS_HOME="$WORKSPACE_DIR/.ros"
export ROS_LOG_DIR="$ROS_HOME/log"
cd "$WORKSPACE_DIR" || exit 1

declare -a commands=(
  "ros2 launch pb2025_sentry_bringup bringup.launch.py world:=$WORLD_NAME slam:=False use_rviz:=$USE_RVIZ rviz_force_software:=$RVIZ_FORCE_SOFTWARE"
)

start_command() {
  local cmd="$1"
  mkdir -p "$(dirname "$PID_FILE")"
  gnome-terminal -- bash -lc "export ROS_HOME=\"$ROS_HOME\"; export ROS_LOG_DIR=\"$ROS_LOG_DIR\"; echo \$\$ > \"$PID_FILE\"; cd \"$WORKSPACE_DIR\"; source \"$SETUP_SCRIPT\"; $cmd; rm -f \"$PID_FILE\"; exec bash"
}

is_running() {
  if [[ ! -f "$PID_FILE" ]]; then
    return 1
  fi

  local pid
  pid="$(<"$PID_FILE")"
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

start_commands() {
  for cmd in "${commands[@]}"; do
    start_command "$cmd"
    sleep 3
  done
}

watch_commands() {
  while true; do
    for cmd in "${commands[@]}"; do
      if ! is_running; then
        echo "$cmd 未在运行, 重新启动"
        start_command "$cmd"
        sleep 3
      fi
    done
    sleep 5
  done
}

start_commands
watch_commands
