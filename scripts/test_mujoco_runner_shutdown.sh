#!/usr/bin/env bash
# Offline regression of the actual runner's teardown/evidence helpers; no ROS.
set -u -o pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/scripts/test_mujoco_minco_mpc_chain.sh"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "${WORK_DIR}"' EXIT
FAILURES=0
extract_function() {
  awk -v name="$1" '
    $0 ~ "^" name "\\(\\) \\{" {inside = 1}
    inside {print}
    inside && $0 == "}" {exit}
  ' "${RUNNER}"
}
for name in start_launch teardown_launch update_recorder_evidence_status cleanup runner_exit write_runner_status analyze_nav_tracking_leg stop_nav_tracking_recorder flush_leg_evidence run_navigation_goal fail; do
  extract_function "${name}" >>"${WORK_DIR}/helpers.sh"
done
cat >"${WORK_DIR}/fake_launch.py" <<'PY'
import os
import signal
import sys
import time

ready, record, exit_code, mode = sys.argv[1:]
if signal.getsignal(signal.SIGINT) == signal.SIG_IGN:
    raise RuntimeError('launch inherited ignored SIGINT from background shell')
child_pid = None
if mode == 'fanout':
    child_pid = os.fork()
    if child_pid == 0:
        received = []
        def child_interrupt(signum, frame):
            received.append(signum)
        signal.signal(signal.SIGINT, child_interrupt)
        with open(ready + '.child_ready', 'w') as stream:
            stream.write('ready')
        while not received:
            time.sleep(0.01)
        # Model a child remaining alive while it releases resources. A second
        # interrupt in this window is a lifecycle error, not normal shutdown.
        time.sleep(0.5)
        with open(ready + '.child_count', 'w') as stream:
            stream.write(str(len(received)))
        os._exit(0 if len(received) == 1 else 2)
    while not os.path.exists(ready + '.child_ready'):
        time.sleep(0.01)
def shutdown(signum, frame):
    if mode == 'ignore':
        return
    if child_pid is not None:
        time.sleep(0.1)
        os.kill(child_pid, signal.SIGINT)
        _, child_status = os.waitpid(child_pid, 0)
        if child_status:
            print('process has died [exit code 2]', flush=True)
    print(record, flush=True)
    sys.exit(int(exit_code))
signal.signal(signal.SIGINT, shutdown)
with open(ready, 'w') as stream:
    stream.write(str(os.getpid()))
while True:
    signal.pause()
PY
run_case() {
  local name="$1" record="$2" launch_exit="$3" mode="$4" prior="$5" expected="$6" teardown="$7" recorder="${8:-passed}"
  local analysis="${9:-0}" gate="${10:-1}" skip_action="${11:-0}" evidence="${12:-passed}"
  local dir="${WORK_DIR}/$1" status=0
  mkdir -p "${dir}"
  (
    source "${WORK_DIR}/helpers.sh"
    CAPTURE_PIDS=() STOPPED_PIDS=()
    CLEANUP_DONE=0 CLEANUP_STATUS=0 FLUSHING_EVIDENCE=0
    ACTION_STATUS=succeeded NAVIGATION_SAFETY_STATUS=passed
    RECORDER_STATUS="${recorder}" NAV_TRACKING_ANALYSIS_STATUS="${analysis}"
    RECORDER_EVIDENCE_STATUS=not_started NAV_TRACKING_GATE="${gate}"
    ATS_PROFILE_SKIP_ACTION="${skip_action}" NAV_TRACKING_RECORDER=1
    TEARDOWN_STATUS=not_started TEARDOWN_ESCALATION=none LAUNCH_WAIT_STATUS=not_started
    LAUNCH_LOG="${dir}/launch.log" RUNNER_STATUS_FILE="${dir}/runner_status.env"
    flush_leg_evidence() { echo flushed >>"${dir}/flush.count"; }
    stop_capture_process() { :; }
    start_launch python3 "${WORK_DIR}/fake_launch.py" "${dir}/ready" "${record}" "${launch_exit}" "${mode}"
    trap 'kill -KILL "-${LAUNCH_PID}" 2>/dev/null || true' EXIT
    deadline=$((SECONDS + 5))
    while [[ ! -s "${dir}/ready" ]] && (( SECONDS < deadline )); do sleep 0.02; done
    [[ -s "${dir}/ready" ]] || exit 99
    trap runner_exit EXIT
    cleanup "${prior}" || status=$?
    # EXIT calls cleanup a second time; evidence must not flush again.
    exit "${status}"
  ) || status=$?
  if [[ "${status}" != "${expected}" ]]; then
    echo "FAIL: ${name}: exit ${status}, expected ${expected}" >&2
    FAILURES=$((FAILURES + 1))
  fi
  if [[ ! -s "${dir}/runner_status.env" ]]; then
    echo "FAIL: ${name}: missing status artifact" >&2
    FAILURES=$((FAILURES + 1))
    return
  fi
  (
    source "${dir}/runner_status.env"
    [[ "${runner_exit}" == "${expected}" && "${teardown_status}" == "${teardown}" &&
       "${action_status}" == succeeded && "${navigation_safety_status}" == passed &&
       "${recorder_status}" == "${recorder}" ]] || exit 1
    [[ "${recorder_evidence_status}" == "${evidence}" ]] || exit 1
    [[ "$(wc -l <"${dir}/flush.count")" == 1 ]] || exit 1
    if [[ "${mode}" == fanout ]]; then
      [[ "$(cat "${dir}/ready.child_count")" == 1 ]] || exit 1
    fi
    if [[ "${mode}" == ignore ]]; then
      [[ "${teardown_escalation}" == TERM && "${launch_wait_status}" == 143 ]]
    else
      [[ "${teardown_escalation}" == none && "${launch_wait_status}" == "${launch_exit}" ]]
    fi
  ) || { echo "FAIL: ${name}: status/idempotence contract" >&2; FAILURES=$((FAILURES + 1)); }
}
run_case normal_sigint '[INFO] [launch]: user interrupted with ctrl-c (SIGINT)' 0 normal 0 0 passed
run_case normal_child '[INFO] [node-1]: process has finished cleanly [pid 123]' 0 normal 0 0 passed
run_case single_child_interrupt '[INFO] [node-1]: process has finished cleanly [pid 123]' 0 fanout 0 0 passed
run_case child_abort '[ERROR] [node-1]: process has died [pid 123, exit code -6, cmd node]' 0 normal 0 1 failed
run_case child_error '[ERROR] [node-1]: process has died [pid 123, exit code 1, cmd node]' 0 normal 0 1 failed
run_case nonzero_record '[node-1]: exit code: 1' 0 normal 0 1 failed
run_case launch_error '' 7 normal 0 1 failed
run_case preserve_failure '' 0 normal 42 42 passed
run_case preserve_failure_with_crash 'process has died [exit code -6]' 0 normal 42 42 failed
run_case recorder_error '' 0 normal 0 1 passed failed 0 1 0 failed
run_case launch_escalation 'escalating to SIGTERM' 0 normal 0 1 failed
run_case forced_stop '' 0 ignore 0 1 failed
run_case evidence_collision '' 0 normal 0 1 passed passed 1 1 0 failed
run_case evidence_unknown '' 0 normal 0 1 passed passed 2 1 0 unverified
run_case evidence_missing '' 0 normal 0 1 passed passed not_run 1 0 unverified
run_case explicit_evidence_only '' 0 normal 0 0 passed passed 1 0 0 failed
run_case map_only_evidence '' 0 normal 0 0 passed not_started not_run 1 1 not_applicable
run_case preserve_failure_with_evidence_unknown '' 0 normal 42 42 passed passed 2 1 0 unverified

# Model the analyzer CLI boundary, not its geometry: the runner must combine
# exit codes correctly and must never overwrite a previous leg's verdict.
mkdir -p "${WORK_DIR}/workspace/scripts"
cat >"${WORK_DIR}/workspace/scripts/analyze_nav_tracking.py" <<'PY'
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--input-dir')
parser.add_argument('--output')
args, _ = parser.parse_known_args()
status = int((Path(args.input_dir) / 'samples.jsonl').read_text())
Path(args.output).write_text(json.dumps({'status': status}))
print('analyzer fixture status=%d' % status)
raise SystemExit(status)
PY

run_analysis_case() {
  local name="$1" first="$2" second="$3" expected="$4" evidence="$5"
  local dir="${WORK_DIR}/$1"
  mkdir -p "${dir}/leg1" "${dir}/leg2"
  printf '%s\n' "${first}" >"${dir}/leg1/samples.jsonl"
  if [[ "${second}" != missing ]]; then
    printf '%s\n' "${second}" >"${dir}/leg2/samples.jsonl"
  fi
  (
    source "${WORK_DIR}/helpers.sh"
    WORKSPACE_DIR="${WORK_DIR}/workspace"
    NAV_TRACKING_RECORDER=1 NAV_TRACKING_GATE=1 ATS_PROFILE_SKIP_ACTION=0
    NAV_TRACKING_ANALYSIS_STATUS=not_run RECORDER_STATUS=passed
    ACTION_STATUS=succeeded NAVIGATION_SAFETY_STATUS=passed
    TEARDOWN_STATUS=passed LAUNCH_WAIT_STATUS=0 TEARDOWN_ESCALATION=none
    RUNNER_STATUS_FILE="${dir}/runner_status.env"
    nav_tracking_footprint_param() { printf '0.5'; }
    analyze_nav_tracking_leg first "${dir}/leg1" || exit 1
    [[ "${NAV_TRACKING_ANALYSIS_STATUS}" == "${first}" ]] || exit 1
    analyze_nav_tracking_leg second "${dir}/leg2" || exit 1
    [[ "${NAV_TRACKING_ANALYSIS_STATUS}" == "${expected}" ]] || exit 1
    [[ "$(cat "${dir}/leg1/verdict.json")" == "{\"status\": ${first}}" ]] || exit 1
    if [[ "${second}" == missing ]]; then
      [[ ! -e "${dir}/leg2/verdict.json" ]] || exit 1
    else
      [[ "$(cat "${dir}/leg2/verdict.json")" == "{\"status\": ${second}}" ]] || exit 1
    fi
    update_recorder_evidence_status
    write_runner_status 0 || exit 1
    source "${RUNNER_STATUS_FILE}"
    [[ "${nav_tracking_analysis_status}" == "${expected}" &&
       "${recorder_evidence_status}" == "${evidence}" &&
       "${recorder_status}" == passed && "${action_status}" == succeeded &&
       "${navigation_safety_status}" == passed && "${teardown_status}" == passed ]]
  ) >"${dir}/case.log" 2>&1 || {
    cat "${dir}/case.log"
    echo "FAIL: ${name}: aggregate/per-leg evidence contract" >&2
    FAILURES=$((FAILURES + 1))
  }
}
run_analysis_case incomplete_then_conflict 2 1 1 failed
run_analysis_case conflict_then_incomplete 1 2 1 failed
run_analysis_case complete_then_incomplete 0 2 2 unverified
run_analysis_case incomplete_then_complete 2 0 2 unverified
run_analysis_case conflict_then_complete 1 0 1 failed
run_analysis_case complete_then_complete 0 0 0 passed
run_analysis_case complete_then_missing 0 missing 2 unverified
run_analysis_case conflict_then_missing 1 missing 1 failed

run_prerequisite_case() {
  local prerequisite="$1" dir="${WORK_DIR}/prerequisite_$1" status=0
  mkdir -p "${dir}/prior_leg"
  printf '1\n' >"${dir}/prior_leg/samples.jsonl"
  printf 'prior verdict\n' >"${dir}/prior_leg/verdict.json"
  printf 'prior analysis\n' >"${dir}/prior_leg/analysis.out"
  : >"${dir}/launch.log"
  (
    source "${WORK_DIR}/helpers.sh"
    WORKSPACE_DIR="${WORK_DIR}/workspace"
    TEST_PROFILE=runner_prerequisite_test
    GOAL_NAMES=(first second third) GOAL_XS=(0 1 2) GOAL_YS=(0 0 0)
    NAV_TRACKING_DIR="${dir}" NAV_TRACKING_LEG_DIR="${dir}/prior_leg"
    NAV_TRACKING_PID="" NAV_TRACKING_RECORDER=1 NAV_TRACKING_GATE=1
    NAV_TRACKING_ANALYSIS_STATUS=0 RECORDER_STATUS=passed
    EVIDENCE_FLUSHED=1 LEG_LABEL='second leg'
    CAPTURE_PIDS=() STOPPED_PIDS=()
    CLEANUP_DONE=0 CLEANUP_STATUS=0 FLUSHING_EVIDENCE=0
    ACTION_STATUS=succeeded NAVIGATION_SAFETY_STATUS=passed
    RECORDER_EVIDENCE_STATUS=passed ATS_PROFILE_SKIP_ACTION=0
    TEARDOWN_STATUS=not_started TEARDOWN_ESCALATION=none LAUNCH_WAIT_STATUS=not_started
    LAUNCH_LOG="${dir}/launch.log" RUNNER_STATUS_FILE="${dir}/runner_status.env"
    nav_tracking_footprint_param() { printf '0.5'; }
    capture_pose() { [[ "${prerequisite}" != pose ]]; }
    verify_goal_set_free() { [[ "${prerequisite}" != free_space ]]; }
    capture_contact_telemetry() { [[ "${prerequisite}" != contact ]]; }
    capture_contact_force() { :; }
    contact_count_value() { printf '0'; }
    contact_force_value() { printf '0'; }
    stop_capture_process() { :; }
    teardown_launch() { TEARDOWN_STATUS=passed; LAUNCH_WAIT_STATUS=0; }
    trap runner_exit EXIT
    run_navigation_goal 2
    exit 99
  ) >"${dir}/case.log" 2>&1 || status=$?
  (
    [[ "${status}" == 1 ]] || exit 1
    [[ "$(cat "${dir}/prior_leg/verdict.json")" == 'prior verdict' &&
       "$(cat "${dir}/prior_leg/analysis.out")" == 'prior analysis' ]] || exit 1
    source "${dir}/runner_status.env"
    [[ "${runner_exit}" == 1 && "${nav_tracking_analysis_status}" == 0 &&
       "${recorder_status}" == passed && "${recorder_evidence_status}" == passed &&
       "${navigation_safety_status}" == failed && "${teardown_status}" == passed ]]
  ) || {
    cat "${dir}/case.log"
    echo "FAIL: prerequisite_${prerequisite}: previous leg artifacts reused" >&2
    FAILURES=$((FAILURES + 1))
  }
}
run_prerequisite_case pose
run_prerequisite_case free_space
run_prerequisite_case contact

if (( FAILURES )); then
  echo "RESULT: MuJoCo runner shutdown FAILED (${FAILURES})"
  exit 1
fi
echo 'RESULT: MuJoCo runner shutdown PASSED'
