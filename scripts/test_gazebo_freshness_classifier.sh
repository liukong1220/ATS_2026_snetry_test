#!/usr/bin/env bash
# Deterministic regression for the Gazebo freshness first-violation classifier.

set -u -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/gazebo_freshness_classifier.sh
source "$ROOT_DIR/scripts/gazebo_freshness_classifier.sh"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

assert_contains() {
  local actual="$1"
  local expected="$2"
  [[ "$actual" == *"$expected"* ]] || fail "expected '$expected' in '$actual'"
}

# Emits healthy wall-interval metrics for every stage the recorder always
# reports, so each fixture below only has to state the deviation under test.
healthy_stages() {
  local stage
  for stage in gazebo_lidar livox_input cloud_registered lidar_odometry \
    odometry localization localization_status; do
    printf ' %s_p99_wall_interval_s=0.10 %s_max_wall_interval_s=0.20' \
      "$stage" "$stage"
  done
}

# Replaces one stage's healthy pair with the supplied p99/max values.
stage_timing() {
  local stage="$1"
  local p99="$2"
  local max_gap="$3"
  healthy_stages | sed \
    -e "s/ ${stage}_p99_wall_interval_s=0.10/ ${stage}_p99_wall_interval_s=${p99}/" \
    -e "s/ ${stage}_max_wall_interval_s=0.20/ ${stage}_max_wall_interval_s=${max_gap}/"
}

HEADER="ATS_NAVIGATION_EVIDENCE_RESULT clock_max_wall_interval_s=0.12"

# A downstream-only violation still names the downstream stage.
EVIDENCE="$HEADER$(stage_timing lidar_odometry 1.67 1.67)"
RESULT="$(classify_gazebo_freshness "$EVIDENCE")" || fail "valid evidence was rejected"
assert_contains "$RESULT" "first_violation=lidar_odometry"
assert_contains "$RESULT" "p99_wall_interval_s:1.67"

# Regression for the stage list itself: the LiDAR delivery cadence at the ROS
# boundary violates while every Point-LIO-and-later stage stays within limits.
# A stage list starting at lidar_odometry reports first_violation=none here and
# would attribute a bridge-side gap to whatever downstream stage fails first.
EVIDENCE="$HEADER$(stage_timing gazebo_lidar 0.34 0.45)"
RESULT="$(classify_gazebo_freshness "$EVIDENCE")" || fail "upstream evidence was rejected"
assert_contains "$RESULT" "first_violation=gazebo_lidar"
assert_contains "$RESULT" "p99_wall_interval_s:0.34"

# Same guarantee one hop later, for the in-project gz_livox_bridge output.
EVIDENCE="$HEADER$(stage_timing livox_input 0.10 0.61)"
RESULT="$(classify_gazebo_freshness "$EVIDENCE")" || fail "livox_input evidence was rejected"
assert_contains "$RESULT" "first_violation=livox_input"
assert_contains "$RESULT" "max_wall_interval_s:0.61"

# And for the Point-LIO registered scan ahead of loam_interface.
EVIDENCE="$HEADER$(stage_timing cloud_registered 0.33 0.59)"
RESULT="$(classify_gazebo_freshness "$EVIDENCE")" || fail "cloud_registered evidence was rejected"
assert_contains "$RESULT" "first_violation=cloud_registered"

# When several stages violate together, the earliest producer is reported, not
# the first one that used to be checked.
EVIDENCE="$HEADER$(healthy_stages | sed -e 's/_p99_wall_interval_s=0.10/_p99_wall_interval_s=0.33/g')"
RESULT="$(classify_gazebo_freshness "$EVIDENCE")" || fail "pipeline-wide evidence was rejected"
assert_contains "$RESULT" "first_violation=gazebo_lidar"

# The /clock gap outranks every per-stage check.
EVIDENCE="ATS_NAVIGATION_EVIDENCE_RESULT clock_max_wall_interval_s=0.90$(stage_timing gazebo_lidar 0.34 0.45)"
RESULT="$(classify_gazebo_freshness "$EVIDENCE")" || fail "clock evidence was rejected"
assert_contains "$RESULT" "first_violation=clock"

EVIDENCE="$HEADER$(healthy_stages)"
RESULT="$(classify_gazebo_freshness "$EVIDENCE")" || fail "healthy evidence was rejected"
assert_contains "$RESULT" "first_violation=none"

# gazebo_transport_lidar is measured only behind OBSERVE_GAZEBO_TRANSPORT_LIDAR,
# so its `unverified` metrics are an absent measurement, not a violation.
EVIDENCE="$HEADER gazebo_transport_lidar_p99_wall_interval_s=unverified gazebo_transport_lidar_max_wall_interval_s=unverified$(healthy_stages)"
RESULT="$(classify_gazebo_freshness "$EVIDENCE")" || fail "disabled transport observer was rejected"
assert_contains "$RESULT" "first_violation=none"

# An enabled observer with no samples is incomplete evidence, not a disabled observer.
set +e
RESULT="$(classify_gazebo_freshness "$HEADER gazebo_transport_lidar_observation_enabled=yes gazebo_transport_lidar_p99_wall_interval_s=unverified gazebo_transport_lidar_max_wall_interval_s=unverified$(healthy_stages)")"
RC=$?
set -u
[ "$RC" -eq 2 ] || fail "enabled transport observer with no samples did not return 2"
assert_contains "$RESULT" "reason=gazebo_transport_lidar_timing_missing"

# With the observer on, the Gazebo Transport cadence is classified like any
# other stage and outranks the ROS-boundary stage behind it.
EVIDENCE="$HEADER gazebo_transport_lidar_p99_wall_interval_s=0.41 gazebo_transport_lidar_max_wall_interval_s=0.44$(stage_timing gazebo_lidar 0.34 0.45)"
RESULT="$(classify_gazebo_freshness "$EVIDENCE")" || fail "transport observer evidence was rejected"
assert_contains "$RESULT" "first_violation=gazebo_transport_lidar"

# Truncated evidence for an unconditionally emitted stage stays unverified.
set +e
RESULT="$(classify_gazebo_freshness "ATS_NAVIGATION_EVIDENCE_RESULT clock_max_wall_interval_s=0.12")"
RC=$?
set -u
[ "$RC" -eq 2 ] || fail "missing timing did not return 2"
assert_contains "$RESULT" "first_violation=unverified"
assert_contains "$RESULT" "reason=gazebo_lidar_timing_missing"

# A gap in a middle stage is truncated evidence too, not a silent skip.
set +e
RESULT="$(classify_gazebo_freshness "$HEADER$(healthy_stages | sed -e 's/ cloud_registered_p99_wall_interval_s=0.10//')")"
RC=$?
set -u
[ "$RC" -eq 2 ] || fail "partial stage evidence did not return 2"
assert_contains "$RESULT" "reason=cloud_registered_timing_missing"

echo "PASS: Gazebo freshness first-violation classifier"
