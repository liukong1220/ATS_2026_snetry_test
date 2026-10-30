#!/usr/bin/env bash
# Runner-side regression for the in-process topic capture (scripts/runner_topic_capture.py):
# the helpers are extracted from the real runner and driven against the real capture process on a
# private ROS_DOMAIN_ID.  Needs a sourced ROS 2 environment; prints SKIP without one.
set -u -o pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/scripts/test_mujoco_minco_mpc_chain.sh"
WORK_DIR="$(mktemp -d)"
FAILURES=0
if ! command -v ros2 >/dev/null || ! python3 -c 'import rclpy' 2>/dev/null; then
  echo "SKIP: ROS 2 environment not sourced"
  exit 0
fi
extract_function() {
  awk -v name="$1" '
    $0 ~ "^" name "\\(\\) \\{" {inside = 1}
    inside {print}
    inside && $0 == "}" {exit}
  ' "${RUNNER}"
}
for name in stop_capture_process start_capture_daemon capture_daemon_wait_ack capture_daemon_send \
  capture_daemon_open capture_daemon_close stop_capture_daemon capture_is_alive start_topic_capture \
  ensure_topic_capture_ready wait_for_capture path_pose_count assert_path_has_poses \
  assert_path_visualization_frame; do
  extract_function "${name}" >>"${WORK_DIR}/helpers.sh"
done
fail() { echo "FAIL(helper): $*"; FAILURES=$((FAILURES + 1)); return 1; }
goal_wall_timeout_s() { echo "$1"; }
WORKSPACE_DIR="${ROOT_DIR}"
LOG_DIR="${WORK_DIR}"
GOAL_TIMEOUT=30
RUNNER_CAPTURE=inprocess
CAPTURE_DAEMON_PID="" CAPTURE_DAEMON_FIFO="" CAPTURE_DAEMON_ACK="" CAPTURE_DAEMON_SEQ=0
export ROS_DOMAIN_ID=$((200 + $$ % 30))
# shellcheck source=/dev/null
source "${WORK_DIR}/helpers.sh"
trap 'stop_capture_daemon; [[ -z "${PUB_PID:-}" ]] || kill "${PUB_PID}" 2>/dev/null; rm -rf "${WORK_DIR}"' EXIT

check() {
  if eval "$2"; then echo "ok: $1"; else echo "FAIL: $1"; FAILURES=$((FAILURES + 1)); fi
}

ensure_topic_capture_ready "test path" /runner_capture_test/path nav_msgs/msg/Path "${WORK_DIR}/path.out" path_pid
ensure_topic_capture_ready "test cmd" /runner_capture_test/cmd geometry_msgs/msg/Twist "${WORK_DIR}/cmd.out" cmd_pid auto
check "capture handles carry no ':' (run_navigation_goal splits pid:topic:frame:file)" \
  '[[ "${path_pid}" == capd_c1 && "${cmd_pid}" == capd_c2 ]]'
entry="${path_pid}:/runner_capture_test/path:odom:${WORK_DIR}/path.out"
pid="${entry%%:*}"; rest="${entry#*:}"; topic="${rest%%:*}"
check "the leg capture record parses back to its handle and topic" \
  '[[ "${pid}" == capd_c1 && "${topic}" == /runner_capture_test/path ]]'
check "capture is alive while the daemon runs" 'capture_is_alive "${path_pid}"'

ros2 topic pub --rate 5 /runner_capture_test/path nav_msgs/msg/Path \
  "{header: {frame_id: odom}, poses: [{header: {frame_id: odom}}, {header: {frame_id: odom}}]}" \
  >/dev/null 2>&1 &
PUB_PID=$!
timeout 20 ros2 topic pub --times 5 --rate 5 /runner_capture_test/cmd geometry_msgs/msg/Twist \
  "{linear: {y: 0.3}}" >/dev/null 2>&1
output="$(wait_for_capture "test path" "${path_pid}" "${WORK_DIR}/path.out" odom 2>&1)"
check "wait_for_capture accepts a non-empty odom Path from the in-process capture" \
  'grep -q "Path header frame is odom" <<<"${output}"'
kill "${PUB_PID}" 2>/dev/null; wait "${PUB_PID}" 2>/dev/null; PUB_PID=""
check "closing a capture writes its received/written counts" '[[ -s "${WORK_DIR}/path.out.count" ]]'
stop_capture_process "${cmd_pid}"
check "Twist stream is written in ros2 topic echo layout" 'grep -q "^  y: 0.3$" "${WORK_DIR}/cmd.out"'
daemon_pid="${CAPTURE_DAEMON_PID}"
stop_capture_daemon
check "the capture process exits on quit" '! kill -0 "${daemon_pid}" 2>/dev/null'

if (( FAILURES > 0 )); then
  echo "FAILED: ${FAILURES}"
  exit 1
fi
echo "PASS: runner in-process topic capture"
