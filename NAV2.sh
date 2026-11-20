#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
SETUP_SCRIPT="${SETUP_SCRIPT:-$WORKSPACE_DIR/install/setup.bash}"

WORLD_NAME="${1:-${WORLD_NAME:-rmuc_2027}}"
if (( $# > 0 )); then
  shift
fi

SLAM="${SLAM:-False}"
USE_RVIZ="${USE_RVIZ:-False}"
RVIZ_FORCE_SOFTWARE="${RVIZ_FORCE_SOFTWARE:-0}"
OPEN_TERMINAL="${OPEN_TERMINAL:-0}"
PID_FILE="${PID_FILE:-$WORKSPACE_DIR/.ros/nav_bringup.pid}"

export ROS_HOME="${ROS_HOME:-$WORKSPACE_DIR/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROS_HOME/log}"
mkdir -p "$ROS_LOG_DIR" "$(dirname "$PID_FILE")"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[nav] ERROR: ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi
if [[ ! -f "$SETUP_SCRIPT" ]]; then
  echo "[nav] ERROR: Workspace setup not found: $SETUP_SCRIPT (please run ./build.sh first)" >&2
  exit 1
fi

if [[ "${SLAM,,}" != "true" && "${SLAM}" != "1" ]]; then
  MAP_YAML="$WORKSPACE_DIR/src/ats_sentry_bringup/map/${WORLD_NAME}.yaml"
  PRIOR_PCD="$WORKSPACE_DIR/src/ats_sentry_bringup/pcd/${WORLD_NAME}.pcd"
  if [[ ! -f "$MAP_YAML" ]]; then
    echo "[nav] ERROR: Map YAML not found for world '${WORLD_NAME}': $MAP_YAML" >&2
    exit 1
  fi
  if [[ ! -f "$PRIOR_PCD" ]]; then
    echo "[nav] ERROR: Prior PCD not found for world '${WORLD_NAME}': $PRIOR_PCD" >&2
    exit 1
  fi
fi

LAUNCH_CMD=(
  ros2 launch ats_sentry_bringup bringup.launch.py
  "world:=${WORLD_NAME}"
  "slam:=${SLAM}"
  "use_rviz:=${USE_RVIZ}"
  "rviz_force_software:=${RVIZ_FORCE_SOFTWARE}"
  "$@"
)

run_foreground() {
  cd "$WORKSPACE_DIR"
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  # shellcheck disable=SC1090
  source "$SETUP_SCRIPT"
  set -u
  echo "$$" > "$PID_FILE"
  trap 'rm -f "$PID_FILE"' EXIT
  echo "[nav] launching: ${LAUNCH_CMD[*]}"
  exec "${LAUNCH_CMD[@]}"
}

if [[ "$OPEN_TERMINAL" == "1" || "${OPEN_TERMINAL,,}" == "true" ]]; then
  QUOTED_CMD="$(printf '%q ' "${LAUNCH_CMD[@]}")"
  INNER_SCRIPT="export ROS_HOME=$(printf '%q' "$ROS_HOME") ROS_LOG_DIR=$(printf '%q' "$ROS_LOG_DIR"); echo \$\$ > $(printf '%q' "$PID_FILE"); trap 'rm -f $(printf '%q' "$PID_FILE")' EXIT; cd $(printf '%q' "$WORKSPACE_DIR"); source $(printf '%q' "$ROS_SETUP"); source $(printf '%q' "$SETUP_SCRIPT"); ${QUOTED_CMD}"
  if command -v gnome-terminal >/dev/null 2>&1; then
    exec gnome-terminal --title "ATS Sentry Navigation (${WORLD_NAME})" -- bash -lc "$INNER_SCRIPT"
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    exec xfce4-terminal --title "ATS Sentry Navigation (${WORLD_NAME})" --command "bash -lc $(printf '%q' "$INNER_SCRIPT")"
  elif command -v konsole >/dev/null 2>&1; then
    exec konsole -p "tabtitle=ATS Sentry Navigation (${WORLD_NAME})" -e bash -lc "$INNER_SCRIPT"
  elif command -v xterm >/dev/null 2>&1; then
    exec xterm -T "ATS Sentry Navigation (${WORLD_NAME})" -e bash -lc "$INNER_SCRIPT"
  else
    echo "[nav] WARN: No GUI terminal emulator found, running in current shell." >&2
  fi
fi

run_foreground
