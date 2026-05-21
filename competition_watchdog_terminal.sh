#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${ROS_LOG_DIR:-$WORKSPACE_DIR/.ros/log/competition}"
PID_FILE="${PID_FILE:-$WORKSPACE_DIR/.ros/competition_watchdog_terminal.pid}"
TERMINAL_LOG_FILE="${TERMINAL_LOG_FILE:-$LOG_DIR/watchdog_terminal.log}"
RESPAWN_DELAY="${RESPAWN_DELAY:-3}"
WATCH_INTERVAL="${WATCH_INTERVAL:-5}"
mkdir -p "$LOG_DIR"
mkdir -p "$(dirname "$PID_FILE")"

if [[ "${RUN_IN_TERMINAL:-0}" == "1" ]]; then
  echo "$$" > "$PID_FILE"
  cleanup_terminal_pid() {
    rm -f "$PID_FILE"
  }
  trap cleanup_terminal_pid EXIT

cd "$WORKSPACE_DIR"
echo "=========================================="
echo "Sentry competition watchdog terminal"
echo "Workspace: $WORKSPACE_DIR"
echo "Logs: ${ROS_LOG_DIR:-$WORKSPACE_DIR/.ros/log/competition}"
echo "Terminal pid file: $PID_FILE"
echo "Close this terminal or press Ctrl+C to stop this instance."
echo "The launcher will open a new terminal automatically."
echo "=========================================="
./competition_watchdog.sh
status=$?
echo
echo "competition_watchdog.sh exited with status ${status}."
echo "Logs are under: ${ROS_LOG_DIR:-$WORKSPACE_DIR/.ros/log/competition}"
exit "$status"
fi

is_terminal_running() {
  [[ -f "$PID_FILE" ]] || return 1

  local pid
  pid="$(<"$PID_FILE")"
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

start_terminal() {
  local -a terminal_cmd=()

  if command -v gnome-terminal >/dev/null 2>&1; then
    terminal_cmd=(
      gnome-terminal
      --title "Sentry Competition Watchdog"
      --
      bash -lc "RUN_IN_TERMINAL=1 WORKSPACE_DIR='$WORKSPACE_DIR' PID_FILE='$PID_FILE' exec '$WORKSPACE_DIR/competition_watchdog_terminal.sh'"
    )
  elif command -v x-terminal-emulator >/dev/null 2>&1; then
    terminal_cmd=(
      x-terminal-emulator
      -T "Sentry Competition Watchdog"
      -e bash -lc "RUN_IN_TERMINAL=1 WORKSPACE_DIR='$WORKSPACE_DIR' PID_FILE='$PID_FILE' exec '$WORKSPACE_DIR/competition_watchdog_terminal.sh'"
    )
  elif command -v konsole >/dev/null 2>&1; then
    terminal_cmd=(
      konsole
      --new-tab
      --workdir "$WORKSPACE_DIR"
      -p tabtitle="Sentry Competition Watchdog"
      -e bash -lc "RUN_IN_TERMINAL=1 WORKSPACE_DIR='$WORKSPACE_DIR' PID_FILE='$PID_FILE' exec '$WORKSPACE_DIR/competition_watchdog_terminal.sh'"
    )
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    terminal_cmd=(
      xfce4-terminal
      --title "Sentry Competition Watchdog"
      --working-directory "$WORKSPACE_DIR"
      --command "bash -lc \"RUN_IN_TERMINAL=1 WORKSPACE_DIR='$WORKSPACE_DIR' PID_FILE='$PID_FILE' exec '$WORKSPACE_DIR/competition_watchdog_terminal.sh'\""
    )
  else
    echo "[$(date '+%F %T')] No terminal emulator found, running watchdog in this launcher process." >>"$TERMINAL_LOG_FILE"
    exec "$WORKSPACE_DIR/competition_watchdog.sh"
  fi

  echo "[$(date '+%F %T')] Opening watchdog terminal." >>"$TERMINAL_LOG_FILE"
  "${terminal_cmd[@]}" >>"$TERMINAL_LOG_FILE" 2>&1 &
}

echo "[$(date '+%F %T')] Competition watchdog terminal launcher started." >>"$TERMINAL_LOG_FILE"

start_terminal
sleep "$RESPAWN_DELAY"

while true; do
  if ! is_terminal_running; then
    echo "[$(date '+%F %T')] Watchdog terminal is not running, restarting after ${RESPAWN_DELAY}s." >>"$TERMINAL_LOG_FILE"
    sleep "$RESPAWN_DELAY"
    start_terminal
  fi

  sleep "$WATCH_INTERVAL"
done
