#!/usr/bin/env bash
# Action-failure evidence flush: analyzer and contact capture still run.
# Missing contact readings must be unverified, never silently zero.

set -u -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="$ROOT_DIR/scripts/test_mujoco_minco_mpc_chain.sh"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

FAILURES=0
note_failure() {
  echo "FAIL: $*" >&2
  FAILURES=$((FAILURES + 1))
}

extract_function() {
  local name="$1"
  awk -v name="$name" '
    $0 ~ "^" name "\\(\\) \\{" {inside = 1}
    inside {print}
    inside && $0 == "}" {exit}
  ' "$RUNNER"
}

HARNESS="$WORK_DIR/harness.sh"
{
  echo 'set -u -o pipefail'
  echo 'fail() { echo "FAIL: $1"; flush_leg_evidence || true; exit 1; }'
  echo 'CAPTURE_PIDS=()'
  echo 'NAV_TRACKING_PID=""'
  echo 'NAV_TRACKING_RECORDER=1'
  echo 'NAV_TRACKING_GATE=0'
  echo 'FLUSHING_EVIDENCE=1'
  echo 'EVIDENCE_FLUSHED=0'
  echo "WORKSPACE_DIR='$ROOT_DIR'"
  echo 'stop_capture_process() { :; }'
  echo 'capture_contact_telemetry() { : >"$1"; return 1; }'
  echo 'capture_contact_force() { : >"$1"; return 1; }'
  echo 'stop_nav_tracking_recorder() { echo "recorder_stopped:$1"; }'
  echo 'analyze_nav_tracking_leg() { echo "analyzer_ran:$1:$2"; printf "{\"ok\":true}\\n" >"$2/verdict.json"; }'
  extract_function contact_count_value
  extract_function contact_force_value
  extract_function flush_leg_evidence
} >"$HARNESS"

for required in contact_count_value contact_force_value flush_leg_evidence; do
  grep -q "^${required}() {" "$HARNESS" ||
    note_failure "could not extract ${required}() from the runner"
done
if [ "$FAILURES" -ne 0 ]; then
  echo "RESULT: MuJoCo failure evidence test FAILED ($FAILURES)"
  exit 1
fi

run_fail_flush() {
  local name="$1"
  local dir="$WORK_DIR/$name"
  mkdir -p "$dir"
  : >"$dir/before.out"
  : >"$dir/after.out"
  : >"$dir/force.out"
  printf '{"mono_s":0}\n' >"$dir/samples.jsonl"
  local output status
  output="$(
    bash -c "
      source '$HARNESS'
      LEG_LABEL='$name'
      LEG_BEFORE_CONTACT='$dir/before.out'
      LEG_AFTER_CONTACT='$dir/after.out'
      LEG_AFTER_CONTACT_FORCE='$dir/force.out'
      NAV_TRACKING_LEG_DIR='$dir'
      fail 'ATS action did not succeed'
    " 2>&1
  )" || status=$?
  status="${status:-0}"
  if [ "$status" -ne 1 ]; then
    note_failure "$name: expected original exit 1, got $status ($output)"
    return 0
  fi
  if ! printf '%s' "$output" | grep -q 'contact_violation_delta=unverified'; then
    note_failure "$name: missing contact must be unverified, got: $output"
    return 0
  fi
  if printf '%s' "$output" | grep -Eq 'contact_violation_delta=0([^.]|$)'; then
    note_failure "$name: missing contact was filled as zero: $output"
    return 0
  fi
  if ! printf '%s' "$output" | grep -q "analyzer_ran:$name:$dir"; then
    note_failure "$name: analyzer did not run on failure: $output"
    return 0
  fi
  if [ ! -s "$dir/verdict.json" ]; then
    note_failure "$name: analyzer produced no verdict.json"
    return 0
  fi
  echo "ok: $name preserved exit 1 and flushed analyzer/contact as unverified"
}

run_fail_flush action_failed

if [ "$FAILURES" -ne 0 ]; then
  echo "RESULT: MuJoCo failure evidence test FAILED ($FAILURES failure(s))"
  exit 1
fi
echo "RESULT: MuJoCo failure evidence test PASSED"
exit 0
