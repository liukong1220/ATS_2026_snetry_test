#!/usr/bin/env bash
# Behavioral regression for the MuJoCo physical-contact gate.
#
# sim_node already computed contact_violation_count and max_contact_force and
# published them on /swerve/telemetry, but no runner subscribed, so no artifact
# or gate ever saw them. This test drives the gate's pure parsing and delta
# logic extracted from the runner, without launching MuJoCo.
#
# The gate is deliberately independent from the planner's discrete
# footprint_collisions check: that one is a geometric self-check of the MINCO
# trajectory at sampled points, this one is what the rigid-body solver actually
# resolved. Neither substitutes for the other.

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
  # The runner aborts the whole run on a gate violation. Here the failure is
  # captured so a rejection can be asserted rather than ending this test.
  echo 'fail() { echo "GATE_FAIL: $*"; exit 42; }'
  extract_function contact_count_value
  extract_function contact_force_value
  extract_function assert_no_physical_contact
} >"$HARNESS"

for required in contact_count_value contact_force_value assert_no_physical_contact; do
  grep -q "^${required}() {" "$HARNESS" ||
    note_failure "could not extract ${required}() from the runner"
done
if [ "$FAILURES" -ne 0 ]; then
  echo "RESULT: MuJoCo contact gate test FAILED ($FAILURES)"
  exit 1
fi

# run_case <name> <before> <after> <force> <expect: pass|reject> <reason pattern>
run_case() {
  local name="$1" before="$2" after="$3" force="$4" expect="$5" pattern="${6:-}"
  local dir="$WORK_DIR/$name"
  mkdir -p "$dir"
  printf '%s\n' "$before" >"$dir/before.out"
  printf '%s\n' "$after" >"$dir/after.out"
  printf '%s\n' "$force" >"$dir/force.out"
  local output status
  output="$(
    bash -c "source '$HARNESS'; assert_no_physical_contact '$name' \
      '$dir/before.out' '$dir/after.out' '$dir/force.out'" 2>&1
  )"
  status=$?
  if [ "$expect" = "pass" ]; then
    if [ "$status" -ne 0 ]; then
      note_failure "$name: expected the gate to accept, got status=$status ($output)"
      return 0
    fi
    if [ -n "$pattern" ] && ! printf '%s' "$output" | grep -Eq -- "$pattern"; then
      note_failure "$name: expected output to match /$pattern/ got: $output"
      return 0
    fi
    echo "ok: $name accepted -> $output"
    return 0
  fi
  if [ "$status" -eq 0 ]; then
    note_failure "$name: expected the gate to reject, but it accepted ($output)"
    return 0
  fi
  if [ -n "$pattern" ] && ! printf '%s' "$output" | grep -Eq -- "$pattern"; then
    note_failure "$name: expected rejection to match /$pattern/ got: $output"
    return 0
  fi
  echo "ok: $name rejected -> $output"
}

# The documented goals 1-4 shape: the count is nonzero from earlier legs but did
# not advance during this one. Judging the absolute value instead of the delta
# would wrongly reject this.
run_case zero_delta_nonzero_baseline 41 41 0.0 pass 'contact_violation_delta=0'

# A completely clean run from sim start.
run_case clean_from_start 0 0 0.0 pass 'contact_violation_delta=0'

# The red_box goal-5 shape: the robot parked into the wall-side contact band, so
# the count advances inside this goal window. contact_violation_count accumulates
# per physics step, so a sustained touch produces a large delta.
run_case contact_during_goal 41 1839 268.4 reject 'GATE_FAIL.*1798 次物理接触违规'

# A single-step graze must still be rejected; there is no tolerance band.
run_case single_step_graze 100 101 3.2 reject 'GATE_FAIL.*1 次物理接触违规'

# Fail-closed: telemetry unreadable means "no evidence", not "no contact".
run_case missing_before '' 41 0.0 reject 'GATE_FAIL.*物理接触证据缺失'
run_case missing_after 41 '' 0.0 reject 'GATE_FAIL.*物理接触证据缺失'
run_case garbage_after 41 'contact_violation_count: n/a' 0.0 reject \
  'GATE_FAIL.*物理接触证据缺失'

# A backward count means the sim restarted mid-run, so the delta is meaningless
# and must not be read as a clean leg.
run_case counter_regressed 500 12 0.0 reject 'GATE_FAIL.*回退'

# The force is diagnostic, not a gate input: an unreadable force must not turn a
# clean leg into a failure, and must be reported as unverified.
run_case unreadable_force_clean_leg 41 41 '' pass 'max_contact_force_n=unverified'

# Force is reported alongside a rejection so the artifact carries the magnitude.
run_case force_reported_on_reject 0 96 512.75 reject 'max_contact_force=512\.75 N'

if [ "$FAILURES" -ne 0 ]; then
  echo "RESULT: MuJoCo contact gate test FAILED ($FAILURES failure(s))"
  exit 1
fi
echo "RESULT: MuJoCo contact gate test PASSED"
exit 0
