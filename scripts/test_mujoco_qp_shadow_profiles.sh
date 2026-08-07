#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_ROOT="${QP_PROFILE_OUTPUT_ROOT:-/tmp/ats_qp_shadow_profiles_$(date -u +%Y%m%dT%H%M%SZ)}"
DOMAIN_START="${QP_PROFILE_DOMAIN_START:-200}"

set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"

domain_is_empty() {
  local candidate="$1"
  local nodes
  nodes="$(ROS_DOMAIN_ID="${candidate}" ros2 node list --no-daemon 2>/dev/null || true)"
  [[ -z "$(sed '/^[[:space:]]*$/d' <<<"${nodes}")" ]]
}

next_domain() {
  local candidate="$1"
  while (( candidate <= 232 )); do
    if domain_is_empty "${candidate}"; then
      printf '%s\n' "${candidate}"
      return 0
    fi
    ((candidate += 1))
  done
  echo "No unused ROS_DOMAIN_ID remains in [${DOMAIN_START}, 232]." >&2
  return 1
}

run_case() {
  local label="$1"
  local solver_mode="$2"
  local log_level="$3"
  local domain="$4"
  local run_directory="${OUTPUT_ROOT}/${label}"
  mkdir -p "${run_directory}"
  echo "RUN: ${label} domain=${domain} solver_mode=${solver_mode} log_level=${log_level}"
  ROS_DOMAIN_ID="${domain}" \
  PLANNING_GRID_OWNER=rog_map \
  SOLVER_MODE="${solver_mode}" \
  LOG_LEVEL="${log_level}" \
  TEST_PROFILE=single \
  P2_FAULT_CASE=none \
  P3_FAULT_CASE=none \
  QP_TELEMETRY_OUTPUT="${run_directory}/raw_telemetry.json" \
  QP_TELEMETRY_MANIFEST="${run_directory}/manifest.json" \
  "${WORKSPACE_DIR}/scripts/test_mujoco_minco_mpc_chain.sh"
}

mkdir -p "${OUTPUT_ROOT}"
domain_a="$(next_domain "${DOMAIN_START}")"
domain_b="$(next_domain "$((domain_a + 1))")"
domain_c="$(next_domain "$((domain_b + 1))")"
run_case A_ilqr_warn ilqr warn "${domain_a}"
run_case B_qp_shadow_warn qp_shadow warn "${domain_b}"
run_case C_qp_shadow_info qp_shadow info "${domain_c}"
python3 "${WORKSPACE_DIR}/scripts/analyze_qp_shadow_telemetry.py" \
  --run "A_ilqr_warn=${OUTPUT_ROOT}/A_ilqr_warn" \
  --run "B_qp_shadow_warn=${OUTPUT_ROOT}/B_qp_shadow_warn" \
  --run "C_qp_shadow_info=${OUTPUT_ROOT}/C_qp_shadow_info" \
  --output "${OUTPUT_ROOT}/summary.json"
echo "RESULT: QP telemetry artifacts=${OUTPUT_ROOT}"
