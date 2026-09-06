#!/usr/bin/env bash
# Behavioral regression for the P1 dynamic TF freshness gate.
#
# The static contract test only checks that the gate's text is present. This
# one extracts set_p1_admission_evidence() from the runner and drives it with
# synthetic ATS_NAVIGATION_EVIDENCE_RESULT lines, so a gate that is present but
# ineffective still fails here.
#
# The case that matters is "frozen": a broadcaster that stopped publishing while
# the recorder kept polling with TimePointZero. Every pre-existing field in that
# line is identical to a passing run - established=yes, zero post-establishment
# failures, 600/600 TRACKING - and only the source-stamp fields differ.

set -u -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="$ROOT_DIR/scripts/test_gazebo_minco_mpc_chain.sh"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

FAILURES=0
fail() {
  echo "FAIL: $*" >&2
  FAILURES=$((FAILURES + 1))
}

# Extract only the pure evidence/admission helpers. Sourcing the runner itself
# would launch Gazebo.
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
  echo "source '$ROOT_DIR/scripts/gazebo_freshness_classifier.sh'"
  extract_function evidence_value
  extract_function set_p1_admission_evidence
} >"$HARNESS"

for required in evidence_value set_p1_admission_evidence; do
  grep -q "^${required}() {" "$HARNESS" ||
    fail "could not extract ${required}() from the runner"
done
if [ "$FAILURES" -ne 0 ]; then
  echo "RESULT: dynamic TF gate test FAILED ($FAILURES)"
  exit 1
fi

# Field set shared by every case below, matching the shape of the passing
# domain-131 artifact.
BASE_FIELDS="completed=yes duration_s=60.009186 \
localization_status_tracking_samples=600 localization_status_non_tracking_samples=0 \
tf_lookup_attempts=601 tf_lookup_successes=598 tf_lookup_failures=3 \
tf_chain_established=yes tf_lookup_failures_before_establishment=3 \
tf_lookup_failures_after_establishment=0 tf_lookup_max_ms=0.076489"

CLEAN_FRESHNESS="first_violation=none"

# run_case <name> <expected_evidence> <expected_reason_pattern> <extra evidence fields>
run_case() {
  local name="$1" expect_evidence="$2" expect_reason="$3" extra="$4"
  local log="$WORK_DIR/${name}.log"
  printf 'ATS_NAVIGATION_EVIDENCE_RESULT %s %s\n' "$BASE_FIELDS" "$extra" >"$log"
  local output
  output="$(
    ACTIVE_EVIDENCE_LOG="$log" \
    ACTIVE_OBSERVER_WINDOW_SEC=60 \
    ACTIVE_EVIDENCE_DURATION_SEC=60 \
    FRESHNESS_CLASSIFICATION="$CLEAN_FRESHNESS" \
    P2_FAULT_CASE=none \
    GOAL_SUCCEEDED="${CASE_GOAL_SUCCEEDED:-1}" \
    bash -c "source '$HARNESS'; set_p1_admission_evidence; printf '%s|%s' \"\$P1_ADMISSION_EVIDENCE\" \"\$P1_ADMISSION_REASON\""
  )"
  local evidence="${output%%|*}" reason="${output##*|}"
  if [ "$evidence" != "$expect_evidence" ]; then
    fail "$name: expected evidence=$expect_evidence got=$evidence (reason=$reason)"
    return 0
  fi
  if ! printf '%s' "$reason" | grep -Eq -- "$expect_reason"; then
    fail "$name: expected reason to match /$expect_reason/ got=$reason"
    return 0
  fi
  echo "ok: $name -> evidence=$evidence reason=$reason"
}

# A healthy 10 Hz chain polled at 10 Hz for 60 s, with the ages the one passing
# run actually measured: domain 131 reported 0.012-0.032 s p50 and a 0.072 s
# worst case across all six pipeline stages.
run_case healthy true '^all_p1_gates_passed$' \
  "tf_dynamic_samples=598 tf_dynamic_distinct_stamp_updates=596 \
tf_dynamic_duplicate_stamps=2 tf_dynamic_backward_stamps=0 tf_dynamic_invalid_stamps=0 \
tf_dynamic_future_stamps=0 tf_dynamic_age_p50_s=0.022042 tf_dynamic_age_p99_s=0.042043 \
tf_dynamic_age_max_s=0.072011 tf_dynamic_age_floor_s=0.012 tf_dynamic_age_samples=598 \
tf_dynamic_staleness_samples=598 tf_dynamic_stamp_staleness_p99_s=0.2 \
tf_dynamic_update_gap_p99_s=0.100113 tf_dynamic_update_gap_max_s=0.120015"

# The regression. Broadcaster died at ~1 s; the buffer replayed the cached
# transform for the remaining 59 s. Note every legacy field is identical to the
# healthy case: this line passes the pre-existing gate chain in full.
run_case frozen_broadcaster false 'tf_dynamic_stamp_updates_1_below_min_rate' \
  "tf_dynamic_samples=598 tf_dynamic_distinct_stamp_updates=1 \
tf_dynamic_duplicate_stamps=597 tf_dynamic_backward_stamps=0 tf_dynamic_invalid_stamps=0 \
tf_dynamic_future_stamps=0 tf_dynamic_age_p50_s=31.8 tf_dynamic_age_p99_s=61.2 \
tf_dynamic_age_max_s=61.7 tf_dynamic_age_floor_s=2.02 tf_dynamic_age_samples=598 \
tf_dynamic_staleness_samples=598 \
tf_dynamic_stamp_staleness_p99_s=59.18 \
tf_dynamic_update_gap_p99_s=0 tf_dynamic_update_gap_max_s=0"

# A chain that updates but stalls mid-run for 1.4 s. The update count clears the
# rate floor and the run is otherwise fresh - the p99 age and staleness both stay
# healthy because a single 1.4 s outage in 60 s is a ~2% tail - so only the
# steady-clock gap gate catches this.
run_case mid_run_stall false 'tf_dynamic_update_gap_max_1\.42' \
  "tf_dynamic_samples=598 tf_dynamic_distinct_stamp_updates=580 \
tf_dynamic_duplicate_stamps=18 tf_dynamic_backward_stamps=0 tf_dynamic_invalid_stamps=0 \
tf_dynamic_future_stamps=0 tf_dynamic_age_p50_s=0.022 tf_dynamic_age_p99_s=0.31 \
tf_dynamic_age_max_s=1.44 tf_dynamic_age_floor_s=0.012 tf_dynamic_age_samples=598 \
tf_dynamic_staleness_samples=598 tf_dynamic_stamp_staleness_p99_s=0.34 \
tf_dynamic_update_gap_p99_s=0.31 tf_dynamic_update_gap_max_s=1.42"

# Sustained lag: updates keep arriving on time and at full rate, but each one
# carries a stamp far behind the reference clock, so the consumer view trails
# reality. This is the shape the real defect takes, and no cadence gate sees it -
# the update count, both gap percentiles and the staleness are all healthy here.
run_case sustained_age_lag false 'tf_dynamic_age_p99_2\.96' \
  "tf_dynamic_samples=598 tf_dynamic_distinct_stamp_updates=596 \
tf_dynamic_duplicate_stamps=2 tf_dynamic_backward_stamps=0 tf_dynamic_invalid_stamps=0 \
tf_dynamic_future_stamps=0 tf_dynamic_age_p50_s=2.5 tf_dynamic_age_p99_s=2.96 \
tf_dynamic_age_max_s=3.1 tf_dynamic_age_floor_s=2.02 tf_dynamic_age_samples=598 \
tf_dynamic_staleness_samples=598 tf_dynamic_stamp_staleness_p99_s=0.2 \
tf_dynamic_update_gap_p99_s=0.1 tf_dynamic_update_gap_max_s=0.11"

# A source clock that jumped backwards (restarted broadcaster / sim-time reset).
run_case backward_stamp false 'tf_dynamic_stamp_anomalies_backward_1_invalid_0' \
  "tf_dynamic_samples=598 tf_dynamic_distinct_stamp_updates=595 \
tf_dynamic_duplicate_stamps=2 tf_dynamic_backward_stamps=1 tf_dynamic_invalid_stamps=0 \
tf_dynamic_future_stamps=0 tf_dynamic_age_p50_s=0.022 tf_dynamic_age_p99_s=0.042 \
tf_dynamic_age_max_s=0.062 tf_dynamic_age_floor_s=0.012 tf_dynamic_age_samples=598 \
tf_dynamic_staleness_samples=595 tf_dynamic_stamp_staleness_p99_s=0.2 \
tf_dynamic_update_gap_p99_s=0.1008 tf_dynamic_update_gap_max_s=0.1121"

# An older recorder binary that emits none of the new fields must fail closed
# instead of skipping the gate.
run_case missing_evidence false 'tf_dynamic_freshness_evidence_missing' \
  "tf_lookup_max_ms=0.076489"

# The pre-existing gates must keep their precedence: a never-established chain
# is still reported as such, not as a freshness problem. evidence_value() takes
# the last matching field, so appending overrides the base line.
run_case never_established false '^tf_chain_never_established$' \
  "tf_dynamic_samples=0 tf_dynamic_distinct_stamp_updates=0 tf_dynamic_update_gap_max_s=0 \
tf_dynamic_stamp_staleness_p99_s=0 tf_dynamic_staleness_samples=0 \
tf_chain_established=no"

# A chain that came up and then went missing keeps its own dedicated reason
# rather than being reclassified as a freshness failure.
run_case post_establishment_loss false '^tf_lookup_failures_after_establishment_7$' \
  "tf_dynamic_samples=598 tf_dynamic_distinct_stamp_updates=596 \
tf_dynamic_update_gap_max_s=0.1121 tf_dynamic_stamp_staleness_p99_s=0.46 \
 tf_dynamic_staleness_samples=598 \
tf_lookup_failures_after_establishment=7"

# Re-run the never_established case standalone so the ordering claim above is
# exercised against a minimal line rather than only the base-field override.
NEVER_LOG="$WORK_DIR/never_established_override.log"
printf 'ATS_NAVIGATION_EVIDENCE_RESULT %s\n' \
  "completed=yes duration_s=60.009186 localization_status_non_tracking_samples=0 \
tf_chain_established=no tf_lookup_failures_after_establishment=0 \
tf_dynamic_samples=0 tf_dynamic_distinct_stamp_updates=0 tf_dynamic_update_gap_max_s=0" >"$NEVER_LOG"
NEVER_OUT="$(
  ACTIVE_EVIDENCE_LOG="$NEVER_LOG" ACTIVE_OBSERVER_WINDOW_SEC=60 ACTIVE_EVIDENCE_DURATION_SEC=60 \
  FRESHNESS_CLASSIFICATION="$CLEAN_FRESHNESS" P2_FAULT_CASE=none GOAL_SUCCEEDED=1 \
  bash -c "source '$HARNESS'; set_p1_admission_evidence; printf '%s' \"\$P1_ADMISSION_REASON\""
)"
[ "$NEVER_OUT" = "tf_chain_never_established" ] ||
  fail "never-established chain reported '$NEVER_OUT' instead of tf_chain_never_established"

# The two runs that motivate the age gate, with the numbers those artifacts
# actually recorded. There was once a case here asserting that an arbitrarily
# large age must pass, on the theory that the magnitude came from a /clock-to-
# stamp epoch offset rather than from lag. Domain 131 disproved it: the same
# stages read 0.012-0.072 s on the run that succeeded. The premise is gone, so
# the case is gone with it.
#
# What makes these two decisive is that every other gate is clean. 633 and 642
# updates clear the rate floor, both gap percentiles sit at the 0.2-0.3 s poll
# period, there are no stamp anomalies, and the staleness p99 reads a healthy
# 0.200 s - because a lag shared by every sample cancels out of a measure built
# from two reference-clock values. Only the age sees it, and both runs aborted
# with the planning grid never becoming ready.
#
# Both cases run with the default GOAL_SUCCEEDED=1, while both real runs aborted.
# That is deliberate: the action gate outranks every evidence gate, so replaying
# the artifacts verbatim would only re-test the ordering. Reporting success makes
# the age gate the thing under test - a lagged chain must be rejected even when
# the action claims it arrived.
run_case domain139_lagged_chain_rejected false 'tf_dynamic_age_p99_2\.442011' \
  "tf_dynamic_samples=899 tf_dynamic_distinct_stamp_updates=633 \
tf_dynamic_duplicate_stamps=266 tf_dynamic_backward_stamps=0 tf_dynamic_invalid_stamps=0 \
tf_dynamic_future_stamps=0 tf_dynamic_age_p50_s=2.062011 tf_dynamic_age_p99_s=2.442011 \
tf_dynamic_age_max_s=2.642011 tf_dynamic_age_floor_s=0.112011 tf_dynamic_age_samples=899 \
tf_dynamic_staleness_samples=899 tf_dynamic_stamp_staleness_p99_s=0.200000 \
tf_dynamic_stamp_staleness_max_s=0.300000 tf_dynamic_backward_clock_samples=0 \
tf_dynamic_update_gap_p99_s=0.201454 tf_dynamic_update_gap_max_s=0.300052"

run_case domain141_lagged_chain_rejected false 'tf_dynamic_age_p99_2\.352011' \
  "tf_dynamic_samples=899 tf_dynamic_distinct_stamp_updates=642 \
tf_dynamic_duplicate_stamps=257 tf_dynamic_backward_stamps=0 tf_dynamic_invalid_stamps=0 \
tf_dynamic_future_stamps=0 tf_dynamic_age_p50_s=2.152011 tf_dynamic_age_p99_s=2.352011 \
tf_dynamic_age_max_s=2.452011 tf_dynamic_age_floor_s=0.082011 tf_dynamic_age_samples=899 \
tf_dynamic_staleness_samples=899 tf_dynamic_stamp_staleness_p99_s=0.200000 \
tf_dynamic_stamp_staleness_max_s=0.200000 tf_dynamic_backward_clock_samples=0 \
tf_dynamic_update_gap_p99_s=0.200073 tf_dynamic_update_gap_max_s=0.201552"

# An older recorder that emits the age percentiles but not the sample count must
# fail closed rather than gate on a percentile of nothing. Domains 139 and 141
# above are exactly that shape on disk; the field was added with the gate.
run_case no_age_samples false 'tf_dynamic_age_samples_missing' \
  "tf_dynamic_samples=899 tf_dynamic_distinct_stamp_updates=633 \
tf_dynamic_duplicate_stamps=266 tf_dynamic_backward_stamps=0 tf_dynamic_invalid_stamps=0 \
tf_dynamic_future_stamps=0 tf_dynamic_age_p50_s=0.022 tf_dynamic_age_p99_s=0.042 \
tf_dynamic_age_max_s=0.062 tf_dynamic_age_floor_s=0.012 tf_dynamic_age_samples=0 \
tf_dynamic_staleness_samples=899 tf_dynamic_stamp_staleness_p99_s=0.2 \
tf_dynamic_update_gap_p99_s=0.2 tf_dynamic_update_gap_max_s=0.3"

# The domain 137 regression, with that artifact's real numbers. This run has been
# classified three different ways, and the case pins down which one is right.
#
# It was first rejected by an excursion gate (age minus the run minimum): one
# early sample read 0.092 s against a 2.03 s steady floor, so min-of-run re-based
# every later excursion to ~2.24 s. That verdict was right by accident and wrong
# by construction - the minimum is an extreme-value estimator, so the number it
# produced described one outlier, not the chain.
#
# It was then admitted, on the theory that the 2.03 s floor was a clock epoch
# offset. That was wrong outright: the run aborted with 180 adapter not-ready
# samples.
#
# The correct verdict is rejection for the lag itself. What this case asserts is
# the reason string: the run must fail on tf_dynamic_age_p99, never on a
# difference against an estimated floor, and the 0.092 s floor it carries must
# not change the outcome.
run_case domain137_rejected_for_lag_not_floor false 'tf_dynamic_age_p99_2\.332011' \
"tf_chain_established=yes tf_lookup_failures_after_establishment=0 \
tf_dynamic_samples=899 tf_dynamic_distinct_stamp_updates=652 \
tf_dynamic_duplicate_stamps=247 tf_dynamic_backward_stamps=0 \
tf_dynamic_invalid_stamps=0 tf_dynamic_future_stamps=0 \
tf_dynamic_age_p50_s=2.032011 tf_dynamic_age_p99_s=2.332011 \
tf_dynamic_age_max_s=2.532011 tf_dynamic_age_floor_s=0.092011 \
tf_dynamic_age_samples=899 \
tf_dynamic_staleness_samples=899 tf_dynamic_stamp_staleness_p99_s=0.200064 \
tf_dynamic_update_gap_p99_s=0.200064 tf_dynamic_update_gap_max_s=0.300007"

# A broadcaster degraded to ~2 Hz. The update count still clears the 1 Hz floor
# and no single gap reaches the 0.5 s limit, so neither cadence gate fires. On a
# chain that stamps at publish time this shows up as lag: each sample sits up to
# one broadcast period behind, which puts the age p99 over the limit. The
# staleness p99 crosses too, one poll period behind the gap it measures, but the
# age gate is consulted first and owns the reason.
run_case halved_broadcast_rate false 'tf_dynamic_age_p99_0\.53' \
"tf_chain_established=yes tf_lookup_failures_after_establishment=0 \
tf_dynamic_samples=598 tf_dynamic_distinct_stamp_updates=120 \
tf_dynamic_duplicate_stamps=478 tf_dynamic_backward_stamps=0 \
tf_dynamic_invalid_stamps=0 tf_dynamic_future_stamps=0 \
tf_dynamic_age_p50_s=0.26 tf_dynamic_age_p99_s=0.53 \
tf_dynamic_age_max_s=0.58 tf_dynamic_age_floor_s=0.012 tf_dynamic_age_samples=598 \
tf_dynamic_staleness_samples=598 tf_dynamic_stamp_staleness_p99_s=0.55 \
tf_dynamic_update_gap_p99_s=0.47 tf_dynamic_update_gap_max_s=0.49"

# The case the staleness gate exists for, and the mirror image of domains 139 and
# 141: an edge whose stamps are dated ahead of the reference clock while its
# updates have degraded to ~2 Hz. A future offset is normal in this chain -
# localization_fusion stamps map -> odom at now() + 0.05 s, and a composed chain
# can carry more - and it subtracts directly from the age, so a 0.55 s offset
# hides a 0.55 s stall inside a 0.02 s age p99. The update count clears the rate
# floor and the gap max stays under its limit, so staleness is the only gate
# left. Neither quantity subsumes the other: this run and domain 139 are each
# invisible to the gate that catches the other.
run_case future_offset_masks_degraded_updates false 'tf_dynamic_stamp_staleness_p99_0\.55' \
"tf_chain_established=yes tf_lookup_failures_after_establishment=0 \
tf_dynamic_samples=598 tf_dynamic_distinct_stamp_updates=130 \
tf_dynamic_duplicate_stamps=468 tf_dynamic_backward_stamps=0 \
tf_dynamic_invalid_stamps=0 tf_dynamic_future_stamps=598 \
tf_dynamic_age_p50_s=-0.27 tf_dynamic_age_p99_s=0.02 \
tf_dynamic_age_max_s=0.04 tf_dynamic_age_floor_s=-0.55 tf_dynamic_age_samples=598 \
tf_dynamic_staleness_samples=598 tf_dynamic_stamp_staleness_p99_s=0.55 \
tf_dynamic_update_gap_p99_s=0.47 tf_dynamic_update_gap_max_s=0.49"

# A future-stamped composed transform is legitimate (localization_fusion's
# +0.05 s offset can dominate a short chain), so a negative floor must not fail.
run_case future_stamped_chain true '^all_p1_gates_passed$' \
  "tf_dynamic_samples=898 tf_dynamic_distinct_stamp_updates=639 \
tf_dynamic_duplicate_stamps=259 tf_dynamic_backward_stamps=0 tf_dynamic_invalid_stamps=0 \
tf_dynamic_future_stamps=898 tf_dynamic_age_p50_s=-0.05 tf_dynamic_age_p99_s=-0.01 \
tf_dynamic_age_max_s=0.02 tf_dynamic_age_floor_s=-0.06 tf_dynamic_age_samples=898 \
tf_dynamic_staleness_samples=898 \
tf_dynamic_stamp_staleness_p99_s=0.05 \
tf_dynamic_update_gap_p99_s=0.200113 tf_dynamic_update_gap_max_s=0.300015"

# A percentile field absent while its sample count is present. The sample-count
# checks do not cover this: they were added for an older recorder that emitted
# neither field, so dropping the percentile from the fail-closed field list left
# a hole that only shows up on a half-upgraded line. An empty value reaches awk
# as 0.0 and reads as a perfectly fresh chain, so the field list must reject it
# before any threshold is consulted. One case per gated percentile.
run_case age_p99_absent_with_samples false 'tf_dynamic_freshness_evidence_missing' \
  "tf_dynamic_samples=898 tf_dynamic_distinct_stamp_updates=639 \
tf_dynamic_duplicate_stamps=259 tf_dynamic_backward_stamps=0 tf_dynamic_invalid_stamps=0 \
tf_dynamic_future_stamps=0 tf_dynamic_age_p50_s=0.022 tf_dynamic_age_max_s=0.062 \
tf_dynamic_age_floor_s=0.012 tf_dynamic_age_samples=898 \
tf_dynamic_staleness_samples=898 tf_dynamic_stamp_staleness_p99_s=0.2 \
tf_dynamic_update_gap_p99_s=0.2 tf_dynamic_update_gap_max_s=0.3"

run_case staleness_p99_absent_with_samples false 'tf_dynamic_freshness_evidence_missing' \
  "tf_dynamic_samples=898 tf_dynamic_distinct_stamp_updates=639 \
tf_dynamic_duplicate_stamps=259 tf_dynamic_backward_stamps=0 tf_dynamic_invalid_stamps=0 \
tf_dynamic_future_stamps=0 tf_dynamic_age_p50_s=0.022 tf_dynamic_age_p99_s=0.042 \
tf_dynamic_age_max_s=0.062 tf_dynamic_age_floor_s=0.012 tf_dynamic_age_samples=898 \
tf_dynamic_staleness_samples=898 tf_dynamic_stamp_staleness_max_s=0.3 \
tf_dynamic_update_gap_p99_s=0.2 tf_dynamic_update_gap_max_s=0.3"

# No staleness sample could be measured: every percentile reads 0.0, which must
# not be accepted as healthy.
run_case no_staleness_samples false 'tf_dynamic_staleness_samples_missing' \
  "tf_dynamic_samples=898 tf_dynamic_distinct_stamp_updates=639 \
tf_dynamic_backward_stamps=0 tf_dynamic_invalid_stamps=0 tf_dynamic_staleness_samples=0 \
tf_dynamic_stamp_staleness_p99_s=0 \
tf_dynamic_age_p50_s=0.022 tf_dynamic_age_p99_s=0.042 tf_dynamic_age_max_s=0.062 \
tf_dynamic_age_floor_s=0.012 tf_dynamic_age_samples=898 \
tf_dynamic_update_gap_p99_s=0.2 tf_dynamic_update_gap_max_s=0.3"

if [ "$FAILURES" -ne 0 ]; then
  echo "RESULT: dynamic TF gate test FAILED ($FAILURES failure(s))"
  exit 1
fi
# Gate ordering. A run whose action aborted must report that, not whichever
# evidence gate trips first. Domains 135 and 137 both aborted with "map did not
# become ready before deadline" while being reported as dynamic-TF failures,
# which pointed the investigation at the wrong subsystem twice. Here the
# evidence line additionally carries a real freshness problem: the action
# outcome must still win.
CASE_GOAL_SUCCEEDED=0 run_case aborted_action_outranks_evidence_gates false \
  '^straight_action_not_succeeded$' \
"tf_chain_established=yes tf_lookup_failures_after_establishment=0 \
tf_dynamic_samples=899 tf_dynamic_distinct_stamp_updates=633 \
tf_dynamic_duplicate_stamps=266 tf_dynamic_backward_stamps=0 \
tf_dynamic_invalid_stamps=0 tf_dynamic_future_stamps=0 \
tf_dynamic_age_p50_s=2.03 tf_dynamic_age_p99_s=2.33 tf_dynamic_age_max_s=2.53 \
tf_dynamic_age_floor_s=0.09 tf_dynamic_age_samples=899 \
tf_dynamic_staleness_samples=899 \
tf_dynamic_stamp_staleness_p99_s=4.20 \
tf_dynamic_update_gap_p99_s=0.20 tf_dynamic_update_gap_max_s=0.30"
unset CASE_GOAL_SUCCEEDED

echo "RESULT: dynamic TF gate test PASSED"
exit 0
