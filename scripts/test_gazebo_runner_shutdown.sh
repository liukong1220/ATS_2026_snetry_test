#!/usr/bin/env bash
# Offline lifecycle regression using runner helpers and fake processes only.
set -u -o pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/scripts/test_gazebo_minco_mpc_chain.sh"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT
FAILURES=0
extract_function() {
  awk -v name="$1" '
    $0 ~ "^" name "\\(\\) \\{" {inside = 1}
    inside {print}
    inside && $0 == "}" {exit}
  ' "$RUNNER"
}
for name in gazebo_session_server_pids stop_launch_gazebo_server teardown_launch write_runner_status cleanup runner_exit stop_active_observers; do
  extract_function "$name" >>"$WORK_DIR/helpers.sh"
done
cat >"$WORK_DIR/launch.py" <<'PY'
import signal
import sys
ready, record, code, mode = sys.argv[1:]
def stop(signum, frame):
    if mode == 'ignore':
        return
    print(record, flush=True)
    sys.exit(int(code))
signal.signal(signal.SIGINT, stop)
with open(ready, 'w') as stream:
    stream.write('ready')
if mode == 'early':
    print(record, flush=True)
    sys.exit(int(code))
while True:
    signal.pause()
PY
run_case() {
  local name="$1" record="$2" code="$3" mode="$4" prior="$5" expected="$6" teardown="$7" recorder_exit="${8:-0}"
  local dir="$WORK_DIR/$1" actual=0
  mkdir -p "$dir"
  (
    source "$WORK_DIR/helpers.sh"
    CLEANUP_DONE=0 CLEANUP_STATUS=0 TEARDOWN_STATUS=not_started TEARDOWN_ESCALATION=none
    LAUNCH_WAIT_STATUS=not_started RECORDER_STATUS=passed RECORDER_WAIT_STATUS=0
    ACTION_STATUS=not_started RUNTIME_GATE_STATUS=not_completed
    FAILURE_COUNT=0 FIRST_FAILURE=''
    if [[ "$prior" != 0 ]]; then FAILURE_COUNT=1; FIRST_FAILURE=original_runtime_failure; fi
    P1_ADMISSION_EVIDENCE=false RECORDER_EVIDENCE_COMPLETED=yes
    GOAL_SUCCEEDED=1 P2_FAULT_CASE=none TEST_PROFILE=nominal
    ACTIVE_EVIDENCE_PID='' ACTIVE_EVIDENCE_SESSION_ID='' ACTIVE_OBSERVER_PIDS=()
    SHUTDOWN_GRACE_SEC=1 KILL_GRACE_SEC=1
    LAUNCH_LOG="$dir/launch.log" RUNNER_STATUS_FILE="$dir/runner_status.env"
    log() { printf '%s\n' "$*" >>"$dir/messages"; }
    fail() {
      FAILURE_COUNT=$((FAILURE_COUNT + 1))
      [[ -n "$FIRST_FAILURE" ]] || FIRST_FAILURE="$1"
      log "$1"
    }
    # Reaper invocation is observed without starting an actual Gazebo server.
    reap_launch_gazebo_servers() { printf '%s\n' "$1" >>"$dir/reaper.calls"; }
    setsid env --default-signal=INT python3 "$WORK_DIR/launch.py" "$dir/ready" "$record" "$code" "$mode" >"$LAUNCH_LOG" 2>&1 &
    LAUNCH_PID=$! LAUNCH_SESSION_ID=$!
    trap 'kill -KILL "-$LAUNCH_PID" 2>/dev/null || true' EXIT
    deadline=$((SECONDS + 5))
    while [[ ! -s "$dir/ready" ]] && (( SECONDS < deadline )); do sleep 0.02; done
    [[ -s "$dir/ready" ]] || exit 99
    if [[ "$mode" == early ]]; then
      while kill -0 "$LAUNCH_PID" 2>/dev/null && (( SECONDS < deadline )); do sleep 0.02; done
    fi
    if [[ "$recorder_exit" != 0 ]]; then
      bash -c 'exit "$1"' bash "$recorder_exit" &
      ACTIVE_EVIDENCE_PID=$!
      RECORDER_STATUS=running
      while kill -0 "$ACTIVE_EVIDENCE_PID" 2>/dev/null && (( SECONDS < deadline )); do sleep 0.02; done
    fi
    trap runner_exit EXIT
    result=0
    cleanup "$prior" || result=$?
    printf '%s\n' "$FIRST_FAILURE" >"$dir/first_failure"
    printf '%s\n' "$FAILURE_COUNT" >"$dir/failure_count"
    cleanup 0 || true
    [[ "$(cat "$dir/failure_count")" == "$FAILURE_COUNT" ]] || exit 98
    exit "$result"
  ) || actual=$?
  if [[ "$actual" != "$expected" || ! -s "$dir/runner_status.env" ]]; then
    echo "FAIL: $name exit=$actual expected=$expected or missing artifact" >&2
    FAILURES=$((FAILURES + 1))
    return
  fi
  (
    source "$dir/runner_status.env"
    [[ "$runner_exit" == "$expected" && "$teardown_status" == "$teardown" && "$action_status" == succeeded ]] || exit 1
    [[ "$(wc -l <"$dir/reaper.calls")" == 1 ]] || exit 1
    if [[ "$prior" != 0 ]]; then
      [[ "$(cat "$dir/first_failure")" == original_runtime_failure && "$runtime_gate_status" == failed ]] || exit 1
    else
      [[ "$runtime_gate_status" == passed ]] || exit 1
    fi
    if [[ "$mode" == ignore ]]; then
      [[ "$teardown_escalation" == KILL && "$launch_wait_status" == 137 ]] || exit 1
    else
      [[ "$teardown_escalation" == none && "$launch_wait_status" == "$code" ]] || exit 1
    fi
    if [[ "$recorder_exit" != 0 ]]; then
      [[ "$recorder_status" == failed && "$recorder_wait_status" == "$recorder_exit" ]] || exit 1
    else
      [[ "$recorder_status" == passed ]] || exit 1
    fi
  ) || { echo "FAIL: $name independent status/idempotence contract" >&2; FAILURES=$((FAILURES + 1)); }
}
run_case normal '[INFO] [launch]: user interrupted with ctrl-c (SIGINT)' 0 normal 0 0 passed
run_case child_abort 'process has died [exit code -6]' 0 normal 0 1 failed
run_case child_error 'process has died [exit code 1]' 0 normal 0 1 failed
run_case launch_error '' 7 normal 0 1 failed
run_case already_exited '' 0 early 0 0 passed
run_case already_failed '' 9 early 0 1 failed
run_case preserve_failure 'process has died [exit code -15]' 0 normal 42 42 failed
run_case logged_escalation 'escalating to SIGTERM' 0 normal 0 1 failed
run_case forced_stop '' 0 ignore 0 1 failed
run_case recorder_already_failed '' 0 normal 0 1 passed 8

# Native service tests use a fake transport client and session-process witness;
# they never send a Gazebo request or start a simulator.
run_native_stop_case() {
  local name="$1" response="$2" client_rc="$3" active="$4" clean="$5"
  local owned="$6" expected_rc="$7" expected_status="$8"
  local dir="$WORK_DIR/native_$name"
  mkdir -p "$dir"
  (
    source "$WORK_DIR/helpers.sh"
    LAUNCH_LOG="$dir/launch.log"
    GAZEBO_STOP_LOG="$dir/stop.log"
    IGN_PARTITION="ats_gazebo_fixture_$$"
    export IGN_PARTITION
    [[ "$owned" == yes ]] || IGN_PARTITION=shared_partition
    printf '[INFO] [ign gazebo-1]: process started with pid [123]\n' >"$LAUNCH_LOG"
    gazebo_session_server_pids() { [[ "$active" == yes ]] && printf '123\n'; return 0; }
    timeout() {
      printf '%s\n' "$IGN_PARTITION" >"$dir/client.partition"
      printf '%s\n' "$@" >"$dir/client.args"
      printf '%s\n' "$response"
      if [[ "$clean" == yes ]]; then
        printf '[INFO] [ign gazebo-1]: process has finished cleanly [pid 123]\n' >>"$LAUNCH_LOG"
      elif [[ "$clean" == signal ]]; then
        printf '[ERROR] [ign gazebo-1]: process has died [exit code -15]\n' >>"$LAUNCH_LOG"
      fi
      return "$client_rc"
    }
    # Advance a deterministic clock instead of waiting for real-time timeouts.
    sleep() { SECONDS=$((SECONDS + 1)); }
    SECONDS=100
    actual=0
    stop_launch_gazebo_server fixture_session 102 || actual=$?
    [[ "$actual" == "$expected_rc" && "$GAZEBO_STOP_STATUS" == "$expected_status" ]] || exit 1
    (( SECONDS <= 102 )) || exit 1
    if [[ "$owned" == yes ]]; then
      [[ "$(cat "$dir/client.partition")" == "$IGN_PARTITION" ]] || exit 1
      grep -Fxq /server_control "$dir/client.args" || exit 1
      grep -Fxq ignition.msgs.ServerControl "$dir/client.args" || exit 1
      grep -Fxq ignition.msgs.Boolean "$dir/client.args" || exit 1
      grep -Fxq 'stop: true' "$dir/client.args" || exit 1
      grep -Fxq '2000' "$dir/client.args" || exit 1
    else
      [[ ! -e "$dir/client.args" ]] || exit 1
    fi
  ) || { echo "FAIL: native stop $name" >&2; FAILURES=$((FAILURES + 1)); }
}
run_native_stop_case clean 'data: true' 0 no yes yes 0 passed
run_native_stop_case refused 'data: false' 0 no yes yes 1 request_refused_or_unacknowledged
run_native_stop_case missing_ack '' 0 no yes yes 1 request_refused_or_unacknowledged
run_native_stop_case transport_timeout '' 124 no no yes 1 request_failed_124
run_native_stop_case ack_only 'data: true' 0 yes no yes 1 clean_exit_timeout
run_native_stop_case child_signal 'data: true' 0 no signal yes 1 clean_exit_timeout
run_native_stop_case false_clean_with_residual 'data: true' 0 yes yes yes 1 clean_exit_timeout
run_native_stop_case shared_partition 'data: true' 0 no yes no 1 partition_not_owned

if (( FAILURES )); then
  echo "RESULT: Gazebo runner shutdown FAILED ($FAILURES)"
  exit 1
fi
echo 'RESULT: Gazebo runner shutdown PASSED'
