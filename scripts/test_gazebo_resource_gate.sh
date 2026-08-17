#!/usr/bin/env bash
# Focused regression for the Gazebo runner's pre-ROS resource modes.

set -u -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="$ROOT_DIR/scripts/test_gazebo_minco_mpc_chain.sh"
TMP_ROOT="$(mktemp -d /tmp/ats_resource_gate.XXXXXX)"
trap 'rm -rf "$TMP_ROOT"' EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

latest_artifact() {
  local root="$1"
  find "$root" -name preflight_resource.txt -type f -print | sort | tail -n 1
}

run_preflight() {
  local name="$1"
  shift
  local domain="$1"
  shift
  local output="$TMP_ROOT/${name}.log"
  set +e
  env ROS_DOMAIN_ID="$domain" P1_RESOURCE_PREFLIGHT_ONLY=true LOG_ROOT="$TMP_ROOT/$name" \
    "$@" "$RUNNER" >"$output" 2>&1
  TEST_RC=$?
  TEST_OUTPUT="$output"
  TEST_ARTIFACT="$(latest_artifact "$TMP_ROOT/$name")"
  [ -n "$TEST_ARTIFACT" ] || fail "$name did not create a preflight artifact"
}

SWAP_USED_KIB="$(awk '/^SwapTotal:/{total=$2} /^SwapFree:/{free=$2} END {if (total != "" && free != "") print total-free}' /proc/meminfo)"
SWAP_USED_GIB="$(awk -v kib="${SWAP_USED_KIB:-0}" 'BEGIN {printf "%.3f", kib / 1048576.0}')"
if awk -v used="$SWAP_USED_GIB" 'BEGIN {exit !(used > 0.0)}'; then
  STRICT_LIMIT="$(awk -v used="$SWAP_USED_GIB" 'BEGIN {limit=used-0.001; if (limit < 0) limit=0; printf "%.3f", limit}')"
else
  STRICT_LIMIT="0.0"
fi

# Invalid modes must fail before any resource or ROS decision is accepted.
run_preflight invalid_mode 321 P1_RESOURCE_MODE=invalid
[ "$TEST_RC" -eq 3 ] || fail "invalid mode returned $TEST_RC, expected 3"
grep -q '^resource_mode=invalid$' "$TEST_ARTIFACT" || fail "invalid mode was not recorded"
grep -q '^ros_domain=not_allocated$' "$TEST_ARTIFACT" || fail "invalid mode allocated a ROS domain"

# Formal admission remains fail-closed when the current swap usage exceeds the
# caller-provided strict limit. On a host with no swap in use, the same check
# is allowed to pass, which is the correct boundary behavior.
run_preflight admission 322 P1_RESOURCE_MODE=admission P1_MAX_SWAP_USED_GIB="$STRICT_LIMIT"
if [ "$SWAP_USED_GIB" != "0.000" ]; then
  [ "$TEST_RC" -eq 3 ] || fail "admission accepted swap_used=${SWAP_USED_GIB}GiB above ${STRICT_LIMIT}GiB"
  grep -q '^resource_gate=fail$' "$TEST_ARTIFACT" || fail "admission over-limit artifact is not fail-closed"
else
  [ "$TEST_RC" -eq 0 ] || fail "zero-swap admission returned $TEST_RC"
  grep -q '^resource_gate=pass$' "$TEST_ARTIFACT" || fail "zero-swap admission did not pass"
fi
grep -q '^resource_mode=admission$' "$TEST_ARTIFACT" || fail "admission mode was not recorded"
grep -q '^p1_admission_evidence=false$' "$TEST_ARTIFACT" || fail "preflight claimed P1 evidence"
grep -q '^ros_domain=not_allocated$' "$TEST_ARTIFACT" || fail "admission preflight allocated a ROS domain"

# Exploratory mode may continue past swap pressure, but must be explicitly
# marked degraded/non-admissible and still remain preflight-only here.
run_preflight exploratory 323 P1_RESOURCE_MODE=exploratory P1_MAX_SWAP_USED_GIB="$STRICT_LIMIT"
[ "$TEST_RC" -eq 0 ] || {
  echo "--- exploratory output ---" >&2
  cat "$TEST_OUTPUT" >&2
  fail "exploratory preflight returned $TEST_RC"
}
grep -q '^resource_mode=exploratory$' "$TEST_ARTIFACT" || fail "exploratory mode was not recorded"
grep -q '^resource_quality=degraded$' "$TEST_ARTIFACT" || fail "exploratory mode was not degraded"
grep -q '^p1_admission_evidence=false$' "$TEST_ARTIFACT" || fail "exploratory mode claimed P1 evidence"
grep -q '^timing_valid_for_admission=false$' "$TEST_ARTIFACT" || fail "exploratory timing was marked admissible"
grep -q '^ros_domain=not_allocated$' "$TEST_ARTIFACT" || fail "exploratory preflight allocated a ROS domain"
grep -q '^decision=exploratory_degraded_continue$' "$TEST_ARTIFACT" || fail "exploratory decision was not explicit"

echo "PASS: resource gate admission/exploratory/invalid-mode preflight regression"
