#!/usr/bin/env bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_SCRIPT="$WORKSPACE_DIR/install/setup.bash"
WORLD_NAME="${1:-emul}"
USE_RVIZ="${USE_RVIZ:-True}"

export ROS_HOME="$WORKSPACE_DIR/.ros"
cd "$WORKSPACE_DIR" || exit 1

declare -a commands=(
  "ros2 launch pb2025_sentry_bringup bringup.launch.py 
  world:=$WORLD_NAME 
  slam:=False 
  use_rviz:=$USE_RVIZ"
)

start_command() {
  local cmd="$1"
  gnome-terminal -- bash -lc "cd \"$WORKSPACE_DIR\"; source \"$SETUP_SCRIPT\"; $cmd; exec bash"
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
      if ! pgrep -f "$cmd" > /dev/null; then
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
