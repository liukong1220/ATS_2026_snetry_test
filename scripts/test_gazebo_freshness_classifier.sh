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

EVIDENCE="ATS_NAVIGATION_EVIDENCE_RESULT clock_max_wall_interval_s=0.12 lidar_odometry_p99_wall_interval_s=1.67 lidar_odometry_max_wall_interval_s=1.67 odometry_p99_wall_interval_s=1.65 odometry_max_wall_interval_s=1.65 localization_p99_wall_interval_s=1.66 localization_max_wall_interval_s=1.66 localization_status_p99_wall_interval_s=0.11 localization_status_max_wall_interval_s=0.11"
RESULT="$(classify_gazebo_freshness "$EVIDENCE")" || fail "valid evidence was rejected"
assert_contains "$RESULT" "first_violation=lidar_odometry"
assert_contains "$RESULT" "p99_wall_interval_s:1.67"

EVIDENCE="ATS_NAVIGATION_EVIDENCE_RESULT clock_max_wall_interval_s=0.12 lidar_odometry_p99_wall_interval_s=0.10 lidar_odometry_max_wall_interval_s=0.20 odometry_p99_wall_interval_s=0.11 odometry_max_wall_interval_s=0.20 localization_p99_wall_interval_s=0.12 localization_max_wall_interval_s=0.20 localization_status_p99_wall_interval_s=0.10 localization_status_max_wall_interval_s=0.20"
RESULT="$(classify_gazebo_freshness "$EVIDENCE")" || fail "healthy evidence was rejected"
assert_contains "$RESULT" "first_violation=none"

set +e
RESULT="$(classify_gazebo_freshness "ATS_NAVIGATION_EVIDENCE_RESULT clock_max_wall_interval_s=0.12")"
RC=$?
set -u
[ "$RC" -eq 2 ] || fail "missing timing did not return 2"
assert_contains "$RESULT" "first_violation=unverified"

echo "PASS: Gazebo freshness first-violation classifier"
