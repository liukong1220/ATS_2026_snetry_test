#!/usr/bin/env bash
# Acceptance matrix driver for prior-map relocalization.
#
# Axes covered by the built-in cell list:
#   deviation  1 m / 2 m / 3 m
#   direction  +x / -x / +y / -y / +yaw
#   entry      seeded (/initialpose near truth) / autonomous (fusion LOST)
#
# Every repetition uses a fresh ROS_DOMAIN_ID and a fresh Gazebo launch, so one
# cell can never pollute the robot state of the next.
#
# Usage:
#   scripts/test_gazebo_reloc_matrix.sh                     # full list, REPEATS each
#   CELLS="dev1_px_seeded dev1_px_auto" REPEATS=2 scripts/test_gazebo_reloc_matrix.sh
#
# Env:
#   REPEATS          repetitions per cell (default 10, the gate is 9/10)
#   CELLS            space-separated subset of cell ids
#   DOMAIN_BASE      first ROS_DOMAIN_ID (default 60); incremented per run
#   DOMAIN_MAX       last permitted ROS_DOMAIN_ID (default 232); never wraps
#   RECOVER_WAIT_S   recovery budget per repetition (default 90)
set -uo pipefail

WS="${WS:-/home/kong/ATS_2026_snetry_test}"
cd "$WS"

REPEATS="${REPEATS:-10}"
DOMAIN_BASE="${DOMAIN_BASE:-60}"
DOMAIN_MAX="${DOMAIN_MAX:-232}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
OUT_ROOT="$WS/log/gazebo_reloc_matrix/matrix_${RUN_ID}"
mkdir -p "$OUT_ROOT"

if ! [[ "$DOMAIN_BASE" =~ ^[0-9]+$ && "$DOMAIN_MAX" =~ ^[0-9]+$ ]] || \
  ((DOMAIN_BASE > DOMAIN_MAX)); then
  echo "DOMAIN_BASE and DOMAIN_MAX must be non-negative integers with DOMAIN_BASE <= DOMAIN_MAX" >&2
  exit 2
fi
# cell_id|offset_x|offset_y|offset_yaw|entry
# yaw offsets stay well above RECOVER_YAW_RAD=0.35 so the yaw gate is a real
# gate: a 0.35 rad deviation would already sit inside the acceptance band.
ALL_CELLS=(
  "dev1_px_seeded|1.0|0.0|0.0|seeded"
  "dev1_px_auto|1.0|0.0|0.0|autonomous"
  "dev1_nx_seeded|-1.0|0.0|0.0|seeded"
  "dev1_py_seeded|0.0|1.0|0.0|seeded"
  "dev1_ny_seeded|0.0|-1.0|0.0|seeded"
  "dev1_pyaw_seeded|0.0|0.0|0.70|seeded"
  "dev1_nyaw_auto|0.0|0.0|-0.70|autonomous"
  "dev2_px_seeded|2.0|0.0|0.0|seeded"
  "dev2_px_auto|2.0|0.0|0.0|autonomous"
  "dev2_ny_seeded|0.0|-2.0|0.0|seeded"
  "dev2_pyaw_seeded|1.4|1.4|0.70|seeded"
  "dev3_px_seeded|3.0|0.0|0.0|seeded"
  "dev3_px_auto|3.0|0.0|0.0|autonomous"
  "dev3_ny_seeded|0.0|-3.0|0.0|seeded"
  "dev3_pyaw_seeded|2.1|2.1|0.70|seeded"
)

SELECTED=()
if [[ -n "${CELLS:-}" ]]; then
  for want in $CELLS; do
    matched=0
    for row in "${ALL_CELLS[@]}"; do
      if [[ "${row%%|*}" == "$want" ]]; then
        SELECTED+=("$row")
        matched=1
      fi
    done
    # A typo must not silently shrink the matrix into a smaller, passing one.
    if ((matched == 0)); then
      echo "unknown cell '$want'; known: ${ALL_CELLS[*]%%|*}" >&2
      exit 2
    fi
  done
  if [[ ${#SELECTED[@]} -eq 0 ]]; then
    echo "no cell matched CELLS='$CELLS'" >&2
    exit 2
  fi
else
  SELECTED=("${ALL_CELLS[@]}")
fi

SUMMARY="$OUT_ROOT/summary.jsonl"
: >"$SUMMARY"
echo "OUT_ROOT=$OUT_ROOT" | tee "$OUT_ROOT/meta.txt"
echo "REPEATS=$REPEATS CELLS=${#SELECTED[@]}" | tee -a "$OUT_ROOT/meta.txt"

domain="$DOMAIN_BASE"
for row in "${SELECTED[@]}"; do
  IFS='|' read -r cell ox oy oyaw entry <<<"$row"
  for ((rep = 1; rep <= REPEATS; rep++)); do
    attempts=0
    while :; do
      # Reusing a domain can retain DDS discovery state from a previous cell and
      # invalidates the fresh-domain isolation contract. Exhaustion is explicit.
      if ((domain > DOMAIN_MAX)); then
        echo "ROS_DOMAIN_ID range exhausted at $domain (DOMAIN_MAX=$DOMAIN_MAX); refusing to reuse a domain" >&2
        exit 3
      fi
      attempts=$((attempts + 1))
      echo "=== $cell rep=$rep/$REPEATS domain=$domain attempt=$attempts ==="
      # A cell that hangs (stuck bringup, wedged ros2 CLI) must never consume the
      # batch: bound it and record the timeout as a failed repetition.
      CELL_ID="$cell" OFFSET_X="$ox" OFFSET_Y="$oy" OFFSET_YAW="$oyaw" ENTRY="$entry" \
        ROS_DOMAIN_ID="$domain" WAIT_LOCALIZATION_S="${WAIT_LOCALIZATION_S:-90}" \
        timeout -k 20 "${CELL_TIMEOUT:-300}" "$WS/scripts/test_gazebo_reloc_cell.sh" \
        >"$OUT_ROOT/${cell}_rep${rep}.log" 2>&1
      rc=$?
      cell_out="$(grep -m1 '^OUT=' "$OUT_ROOT/${cell}_rep${rep}.log" | cut -d= -f2-)"
      used_domain="$domain"
      domain=$((domain + 1))
      # Two infrastructure faults must not be scored as relocalization results:
      #   * no cell.json  -> wedged DDS discovery or a Gazebo start failure
      #   * sim_starved   -> the GT relay dropped the scans, so the estimator
      #                      never saw an input at all
      # Both retry on a fresh domain and keep the attempt count in the record.
      infra_reason=""
      if [[ -z "$cell_out" || ! -f "$cell_out/cell.json" ]]; then
        infra_reason="no cell.json"
      elif grep -q '"sim_starved": true' "$cell_out/cell.json"; then
        infra_reason="scan pipeline starved"
      fi
      if [[ -z "$infra_reason" ]] || ((attempts >= ${CELL_ATTEMPTS:-4})); then
        break
      fi
      echo "    infra failure ($infra_reason); retrying on a fresh domain"
      sleep 15
    done
    python3 - "$cell" "$rep" "$used_domain" "$rc" "$cell_out" "$attempts" >>"$SUMMARY" <<'PY'
import json, sys
from pathlib import Path
cell, rep, domain, rc, out, attempts = sys.argv[1:7]
record = {
    "cell_id": cell,
    "rep": int(rep),
    "domain": int(domain),
    "rc": int(rc),
    "out": out,
    "infra_attempts": int(attempts),
}
path = Path(out) / "cell.json" if out else None
if path is not None and path.is_file():
    try:
        record.update(json.loads(path.read_text()))
    except Exception as exc:
        record["parse_error"] = str(exc)
else:
    record["parse_error"] = "cell.json missing (launch or bringup failure)"
print(json.dumps(record))
PY
    echo "    rc=$rc out=$cell_out"
    # The GPU lidar sensor fails to start when the previous gzserver has not
    # released the iGPU yet; a cooldown between repetitions cuts that rate.
    sleep "${REP_COOLDOWN_S:-12}"
  done
done

python3 "$WS/scripts/summarize_gazebo_reloc_matrix.py" "$SUMMARY" | tee "$OUT_ROOT/summary.md"
echo "OUT_ROOT=$OUT_ROOT"
