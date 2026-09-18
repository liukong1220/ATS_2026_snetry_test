#!/usr/bin/env bash
# One acceptance-matrix cell of prior-map relocalization (single ROS_DOMAIN_ID).
#
# Scenario: fusion + GICP start from a WRONG map->odom seed. Recovery entry is
# either a near-truth /initialpose (seeded) or a fusion LOST timeout that opens
# the autonomous async lattice search (autonomous).
#
# This harness answers "is the accepted candidate trustworthy", not just
# "did anything get accepted":
#   * correct recovery      base pose error <= RECOVER_XY_M and HELD for HOLD_S
#   * wrong acceptance      any accepted observation whose own pose is further
#                           than WRONG_ACCEPT_XY_M from ground truth
#   * status liveness       max /localization/status receive gap < STATUS_GAP_S
#   * map->odom ownership   GICP publish_tf must be false; fusion owns the TF
#
# Ground truth is used for EVALUATION ONLY; it never enters the online scoring.
set -eo pipefail

WS="${WS:-/home/kong/ATS_2026_snetry_test}"
cd "$WS"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

DOMAIN="${ROS_DOMAIN_ID:-$((40 + RANDOM % 80))}"
if ! [[ "$DOMAIN" =~ ^[0-9]+$ ]] || [ "$DOMAIN" -gt 232 ]; then
  echo "ROS_DOMAIN_ID must be an integer in [0,232] (got '$DOMAIN')" >&2
  exit 2
fi
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

CELL_ID="${CELL_ID:-cell}"
RUN_ID="$(date +%Y%m%d_%H%M%S)_${CELL_ID}_domain${DOMAIN}"
OUT="${OUT_ROOT:-$WS/log/gazebo_reloc_matrix}/${RUN_ID}"
mkdir -p "$OUT"

# Truth map->odom for rmuc_2025 spawn (4.75,9.00) + map origin (-3.58,-9.44).
TRUTH_X="${TRUTH_X:-1.17}"
TRUTH_Y="${TRUTH_Y:--0.44}"
TRUTH_YAW="${TRUTH_YAW:-0.0}"

# Injected deviation of the initial map->odom seed.
OFFSET_X="${OFFSET_X:-1.0}"
OFFSET_Y="${OFFSET_Y:-0.0}"
OFFSET_YAW="${OFFSET_YAW:-0.0}"
WRONG_X="$(python3 -c "print($TRUTH_X + $OFFSET_X)")"
WRONG_Y="$(python3 -c "print($TRUTH_Y + $OFFSET_Y)")"
WRONG_YAW="$(python3 -c "print($TRUTH_YAW + $OFFSET_YAW)")"

# seeded  -> publish /initialpose at truth after the pre-measurement
# autonomous -> never publish /initialpose; fusion LOST opens the async lattice
ENTRY="${ENTRY:-seeded}"
if [[ "$ENTRY" != "seeded" && "$ENTRY" != "autonomous" ]]; then
  echo "ENTRY must be seeded|autonomous (got '$ENTRY')" >&2
  exit 2
fi
# Both entries need fusion to actually reach LOST. A cold start on a wrong seed
# has never had an accepted observation, so it IS lost; leaving the nominal
# 600/3600 s Gazebo timeouts in place keeps fusion in TRACKING and caps every
# correction at the 2.0 m TRACKING budget. The difference between the two
# entries is only whether an operator /initialpose seeds the search.
OBSERVATION_TIMEOUT_S="${OBSERVATION_TIMEOUT_S:-6.0}"
OBSERVATION_LOST_TIMEOUT_S="${OBSERVATION_LOST_TIMEOUT_S:-12.0}"

# Navigation-before-injection is opt-in. The default preserves the original
# spawn-adjacent matrix, while named scenarios move through the real action
# chain and only then permit the relocalization fault.
NAVIGATE_BEFORE_INJECTION="${NAVIGATE_BEFORE_INJECTION:-false}"
NAV_SCENARIO="${NAV_SCENARIO:-spawn}"
NAV_GOAL_SEQUENCE="${NAV_GOAL_SEQUENCE:-}"
NAV_GOAL_TIMEOUT_S="${NAV_GOAL_TIMEOUT_S:-120}"
NAV_GOAL_TOLERANCE_M="${NAV_GOAL_TOLERANCE_M:-0.75}"
NAV_STILL_LINEAR_MPS="${NAV_STILL_LINEAR_MPS:-0.25}"
NAV_STILL_ANGULAR_RPS="${NAV_STILL_ANGULAR_RPS:-0.50}"
NAV_STILL_HOLD_S="${NAV_STILL_HOLD_S:-0.50}"
HEALTH_STABLE_SAMPLES="${HEALTH_STABLE_SAMPLES:-3}"
HEALTH_PROBE_TIMEOUT_S="${HEALTH_PROBE_TIMEOUT_S:-120}"
HEALTH_LOCALIZATION_TIMEOUT_S="${HEALTH_LOCALIZATION_TIMEOUT_S:-1.0}"
HEALTH_MAP_TIMEOUT_S="${HEALTH_MAP_TIMEOUT_S:-5.0}"
PROJECTION_RATE_HZ="${PROJECTION_RATE_HZ:-0.2}"
LIVOX_UPDATE_RATE_HZ="${LIVOX_UPDATE_RATE_HZ:-10.0}"
LAUNCH_PLANNING_EFFECTIVE="${LAUNCH_PLANNING:-false}"

case "$NAVIGATE_BEFORE_INJECTION" in
  false)
    ;;
  true)
    LAUNCH_PLANNING_EFFECTIVE=true
    case "$NAV_SCENARIO" in
      corridor_mouth)
        # Existing red-box route: approach the south entrance, enter the west
        # band, then seat at the documented west-corridor mouth.
        NAV_GOAL_SEQUENCE="${NAV_GOAL_SEQUENCE:-4.20,-4.30,-0.91;4.40,-5.90,-1.45;4.40,-6.35,-1.57;5.10,-6.28,0.10}"
        ;;
      north_pocket)
        # The final pose is the persistent north-side pocket observed by the
        # red-box corridor stitches. It is intentionally distinct from the
        # centerline so repeated geometry is exercised before injection.
        NAV_GOAL_SEQUENCE="${NAV_GOAL_SEQUENCE:-4.20,-4.30,-0.91;4.40,-5.90,-1.45;4.40,-6.35,-1.57;5.10,-6.28,0.10;4.50,-6.30,3.14;3.90,-6.30,3.14;3.27,-5.70,1.00}"
        ;;
      custom)
        if [[ -z "$NAV_GOAL_SEQUENCE" ]]; then
          echo "NAV_SCENARIO=custom requires NAV_GOAL_SEQUENCE=x,y,yaw[;...]" >&2
          exit 2
        fi
        ;;
      *)
        echo "NAV_SCENARIO must be corridor_mouth|north_pocket|custom when NAVIGATE_BEFORE_INJECTION=true" >&2
        exit 2
        ;;
    esac
    if ! python3 - "$NAV_GOAL_SEQUENCE" <<'PY'
import math
import sys

goals = [part.strip() for part in sys.argv[1].split(';') if part.strip()]
if not goals:
    raise SystemExit('NAV_GOAL_SEQUENCE is empty')
for index, item in enumerate(goals, 1):
    values = item.split(',')
    if len(values) != 3 or not all(math.isfinite(float(value)) for value in values):
        raise SystemExit(f'invalid NAV_GOAL_SEQUENCE item {index}: {item!r}')
PY
    then
      exit 2
    fi
    ;;
  *)
    echo "NAVIGATE_BEFORE_INJECTION must be true|false (got '$NAVIGATE_BEFORE_INJECTION')" >&2
    exit 2
    ;;
esac

# A navigation scenario must start from the nominal transform. Its only fault
# is the evaluator's post-arrival /initialpose; using WRONG_* here would turn
# the route itself into an untracked pre-injection recovery experiment.
LAUNCH_INITIAL_X="$WRONG_X"
LAUNCH_INITIAL_Y="$WRONG_Y"
LAUNCH_INITIAL_YAW="$WRONG_YAW"
if [[ "$NAVIGATE_BEFORE_INJECTION" == "true" ]]; then
  LAUNCH_INITIAL_X="$TRUTH_X"
  LAUNCH_INITIAL_Y="$TRUTH_Y"
  LAUNCH_INITIAL_YAW="$TRUTH_YAW"
fi

PRIOR_PCD="${PRIOR_PCD:-$WS/src/ats_sentry_bringup/pcd/rmuc_2025.pcd}"
if [[ ! -e "$PRIOR_PCD" ]]; then
  echo "MISSING prior PCD: $PRIOR_PCD" | tee "$OUT/error.txt"
  exit 2
fi
PRIOR_PCD="$(readlink -f "$PRIOR_PCD")"

CANDIDATE_CSV="$OUT/candidates.csv"

export TRUTH_X TRUTH_Y TRUTH_YAW OFFSET_X OFFSET_Y OFFSET_YAW ENTRY CELL_ID DOMAIN OUT
export NAVIGATE_BEFORE_INJECTION NAV_SCENARIO NAV_GOAL_SEQUENCE NAV_GOAL_TIMEOUT_S
export NAV_GOAL_TOLERANCE_M NAV_STILL_LINEAR_MPS NAV_STILL_ANGULAR_RPS NAV_STILL_HOLD_S
export HEALTH_STABLE_SAMPLES HEALTH_PROBE_TIMEOUT_S HEALTH_LOCALIZATION_TIMEOUT_S HEALTH_MAP_TIMEOUT_S
export PROJECTION_RATE_HZ LIVOX_UPDATE_RATE_HZ
export RECOVER_XY_M="${RECOVER_XY_M:-0.80}"
export RECOVER_YAW_RAD="${RECOVER_YAW_RAD:-0.35}"
export HOLD_S="${HOLD_S:-3.0}"
export RECOVER_WAIT_S="${RECOVER_WAIT_S:-90}"
export WRONG_ACCEPT_XY_M="${WRONG_ACCEPT_XY_M:-1.00}"
export STATUS_GAP_S="${STATUS_GAP_S:-1.0}"
export MIN_OBS_ACCEPTED="${MIN_OBS_ACCEPTED:-1}"
export WAIT_LOCALIZATION_S="${WAIT_LOCALIZATION_S:-180}"
export PRE_MEASURE_S="${PRE_MEASURE_S:-12}"

{
  echo "OUT=$OUT"
  echo "CELL_ID=$CELL_ID"
  echo "DOMAIN=$DOMAIN"
  echo "ENTRY=$ENTRY"
  echo "PRIOR_PCD=$PRIOR_PCD"
  echo "TRUTH=($TRUTH_X,$TRUTH_Y,$TRUTH_YAW)"
  echo "WRONG=($WRONG_X,$WRONG_Y,$WRONG_YAW)"
  echo "OFFSET=($OFFSET_X,$OFFSET_Y,$OFFSET_YAW)"
  echo "OBSERVATION_TIMEOUT_S=$OBSERVATION_TIMEOUT_S"
  echo "OBSERVATION_LOST_TIMEOUT_S=$OBSERVATION_LOST_TIMEOUT_S"
  echo "RECOVER_XY_M=$RECOVER_XY_M HOLD_S=$HOLD_S RECOVER_WAIT_S=$RECOVER_WAIT_S"
  echo "WRONG_ACCEPT_XY_M=$WRONG_ACCEPT_XY_M"
  echo "NAVIGATE_BEFORE_INJECTION=$NAVIGATE_BEFORE_INJECTION NAV_SCENARIO=$NAV_SCENARIO"
  echo "NAV_GOAL_SEQUENCE=$NAV_GOAL_SEQUENCE"
  echo "HEALTH_STABLE_SAMPLES=$HEALTH_STABLE_SAMPLES HEALTH_PROBE_TIMEOUT_S=$HEALTH_PROBE_TIMEOUT_S"
  echo "HEALTH_LOCALIZATION_TIMEOUT_S=$HEALTH_LOCALIZATION_TIMEOUT_S HEALTH_MAP_TIMEOUT_S=$HEALTH_MAP_TIMEOUT_S"
  echo "PROJECTION_RATE_HZ=$PROJECTION_RATE_HZ LIVOX_UPDATE_RATE_HZ=$LIVOX_UPDATE_RATE_HZ"
  echo "LAUNCH_INITIAL=($LAUNCH_INITIAL_X,$LAUNCH_INITIAL_Y,$LAUNCH_INITIAL_YAW)"
  echo "LAUNCH_PLANNING_EFFECTIVE=$LAUNCH_PLANNING_EFFECTIVE"
} | tee "$OUT/meta.txt"

# A polluted machine invalidates the cell: leftover sim/nav processes from a
# previous domain keep publishing and the ros2 daemon keeps their stale
# discovery cache, which makes `ros2 topic list` block until timeout. Sweep
# every stack process, stop the daemon, then wait for the sweep to converge.
#
# `static_transform_publisher` is listed explicitly: ros2 launch leaks the sim's
# static TF nodes out of the process group (they get reparented to
# `systemd --user`), so a group kill never reaches them. 190 of them had piled
# up before this entry existed.
RELOC_STACK_PAT='ats_gazebo_nav|ign gazebo|gz sim|gzserver|gzclient|gazebo_gt_|localization_fusion|small_gicp|point_lio|rog_map|minco_planner|ats_swerve|ats_goal_manager|parameter_bridge|rmu_gazebo|ros2 launch|static_transform_publisher.*(front_mid360|gimbal_yaw|base_footprint|chassis)'

# Fast-DDS leaks its shared-memory segments when participants die by SIGKILL.
# They accumulate per run and eventually break discovery with
# "RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed", which looks exactly
# like a relocalization failure but is a wedged transport. Only segments no live
# process maps are reclaimed.
reclaim_dds_shm() {
  python3 - <<'PY'
import glob
import os

held = set()
for maps in glob.glob('/proc/[0-9]*/maps'):
    try:
        with open(maps, errors='ignore') as stream:
            for line in stream:
                if '/dev/shm/' in line:
                    held.add(line.rsplit('/', 1)[-1].strip())
    except OSError:
        pass

removed = 0
for path in (
    glob.glob('/dev/shm/fastrtps_*')
    + glob.glob('/dev/shm/sem.fastrtps_*')
    + glob.glob('/dev/shm/_port*')
):
    if os.path.basename(path) in held:
        continue
    try:
        os.unlink(path)
        removed += 1
    except OSError:
        pass
print(f'shm_reclaimed={removed}')
PY
}

sweep_stack() {
  pkill -9 -f "$RELOC_STACK_PAT" 2>/dev/null || true
  # `ros2 daemon stop` talks to a socket and has no internal timeout; a wedged
  # daemon otherwise blocks the cell forever.
  timeout 10 ros2 daemon stop >/dev/null 2>&1 || true
  pkill -9 -f '_ros2_daemon' 2>/dev/null || true
  for _ in $(seq 1 15); do
    pgrep -f "$RELOC_STACK_PAT" >/dev/null 2>&1 || break
    sleep 1
    pkill -9 -f "$RELOC_STACK_PAT" 2>/dev/null || true
  done
  reclaim_dds_shm
}

sweep_stack
sleep 3

# The launch pins RCUTILS_LOGGING_BUFFERED_STREAM itself, so the probe polls the
# log instead of relying on flush timing. Python nodes still honour this.
export PYTHONUNBUFFERED=1

setsid ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py \
  use_sim_time:=true \
  headless:=true \
  headless_rendering:=true \
  use_viewer:=false \
  use_rviz:=false \
  enable_camera_sensors:=false \
  use_gazebo_gt_odometry:=true \
  launch_terrain_analysis:=false \
  launch_nav2:=false \
  launch_planning:="$LAUNCH_PLANNING_EFFECTIVE" \
  launch_small_gicp_relocalization:=true \
  projection_rate_hz:="$PROJECTION_RATE_HZ" \
  livox_update_rate_hz:="$LIVOX_UPDATE_RATE_HZ" \
  prior_pcd_file:="$PRIOR_PCD" \
  initial_map_to_odom_x:="$LAUNCH_INITIAL_X" \
  initial_map_to_odom_y:="$LAUNCH_INITIAL_Y" \
  initial_map_to_odom_yaw:="$LAUNCH_INITIAL_YAW" \
  observation_timeout_s:="$OBSERVATION_TIMEOUT_S" \
  observation_lost_timeout_s:="$OBSERVATION_LOST_TIMEOUT_S" \
  gicp_candidate_log_path:="$CANDIDATE_CSV" \
  >"$OUT/launch.log" 2>&1 &
echo $! >"$OUT/launch.pid"
echo "LAUNCH_PID=$(cat "$OUT/launch.pid")"

# `set -e` used to abort the cell on a failing probe, which skipped teardown and
# left the whole sim alive: the next repetition then launched on top of a live
# gzserver and its lidar sensor produced nothing (relay ok=0 drop=0), so a real
# harness fault masqueraded as "no accepted relocalization observation". Teardown
# must therefore run on every exit path.
teardown_stack() {
  local status=$?
  if [[ -f "$OUT/launch.pid" ]]; then
    kill -- -"$(cat "$OUT/launch.pid")" 2>/dev/null || true
  fi
  sweep_stack >/dev/null 2>&1 || true
  return "$status"
}
trap teardown_stack EXIT
trap 'exit 143' INT TERM

python3 - <<'PY2' | tee "$OUT/wait_topics.txt"
import os, time, subprocess, sys
need = ["/clock", "/odometry", "/localization", "/registered_scan"]
deadline = time.time() + float(os.environ.get("WAIT_LOCALIZATION_S", "180"))
seen = {t: False for t in need}
gicp = False
while time.time() < deadline:
    try:
        # --no-daemon: a wedged ros2 daemon makes every `topic list` time out for
        # the rest of the cell, which consumed whole repetitions. Direct
        # discovery is slower per call but has no shared state to wedge.
        cli = ["ros2", "topic", "list", "--no-daemon"]
        topics = set(subprocess.check_output(cli, text=True, timeout=15).splitlines())
        nodes = subprocess.check_output(
            ["ros2", "node", "list", "--no-daemon"], text=True, timeout=15
        )
    except Exception as exc:
        print("list_fail", exc, flush=True)
        time.sleep(2)
        continue
    for t in need:
        if not seen[t] and t in topics:
            seen[t] = True
            print("seen", t, flush=True)
    if (not gicp) and "small_gicp_relocalization" in nodes:
        gicp = True
        print("seen_node small_gicp_relocalization", flush=True)
    if all(seen.values()) and gicp:
        print("ALL_SEEN", flush=True)
        sys.exit(0)
    time.sleep(2)
print("TIMEOUT", seen, "gicp", gicp, file=sys.stderr)
sys.exit(1)
PY2

# Topic existence is not data. The gz->ROS lidar bridge sometimes never delivers
# a cloud (relay ok=0 drop=0, no error logged), which starves registration and
# would be scored as a relocalization failure. Detect it here, in ~30 s instead
# of a full probe, and let the matrix driver retry the repetition.
if ! timeout 35 ros2 topic echo --once --no-daemon /registered_scan >/dev/null 2>&1; then
  echo "NO_SCAN_DATA: /registered_scan delivered no message" | tee "$OUT/no_scan_data.txt"
  exit 90
fi

# Navigation is admitted only after the same read-only health witness used by
# the red-box chain.  A failed witness is a navigation infrastructure failure:
# leave the evaluator fail-closed so it cannot inject /initialpose or count a
# pre-fault observation as recovery evidence.
NAV_HEALTH_GATE_READY="false"
NAV_HEALTH_GATE_RESULT=""
HEALTH_PROBE_LOG="$OUT/navigation_health_probe.log"
HEALTH_TIMEOUT_FLOAT="$(awk -v value="$HEALTH_PROBE_TIMEOUT_S" 'BEGIN {printf "%.6f", value + 0.0}')"
HEALTH_LOCALIZATION_TIMEOUT_FLOAT="$(awk -v value="$HEALTH_LOCALIZATION_TIMEOUT_S" 'BEGIN {printf "%.6f", value + 0.0}')"
HEALTH_MAP_TIMEOUT_FLOAT="$(awk -v value="$HEALTH_MAP_TIMEOUT_S" 'BEGIN {printf "%.6f", value + 0.0}')"
if [[ "$NAVIGATE_BEFORE_INJECTION" == "true" ]]; then
  HEALTH_PROBE_WALL_TIMEOUT_S="$(awk -v value="$HEALTH_PROBE_TIMEOUT_S" 'BEGIN {whole=int(value); printf "%d", whole + (value > whole ? 1 : 0) + 15}')"
  timeout "$HEALTH_PROBE_WALL_TIMEOUT_S" \
    ros2 run rmu_gazebo_simulator ats_navigation_health_probe --ros-args \
      -p timeout_sec:="$HEALTH_TIMEOUT_FLOAT" \
      -p required_stable_samples:="$HEALTH_STABLE_SAMPLES" \
      -p localization_timeout_sec:="$HEALTH_LOCALIZATION_TIMEOUT_FLOAT" \
      -p map_timeout_sec:="$HEALTH_MAP_TIMEOUT_FLOAT" \
      >"$HEALTH_PROBE_LOG" 2>&1 || true
  NAV_HEALTH_GATE_RESULT="$(awk '/^ATS_HEALTH_PROBE_RESULT / {line=$0} END {print line}' "$HEALTH_PROBE_LOG")"
  if grep -q '^ATS_HEALTH_PROBE_RESULT ready=yes ' "$HEALTH_PROBE_LOG"; then
    NAV_HEALTH_GATE_READY="true"
  fi
  echo "NAV_HEALTH_GATE_READY=$NAV_HEALTH_GATE_READY" | tee "$OUT/navigation_health_gate.txt"
  echo "${NAV_HEALTH_GATE_RESULT:-ATS_HEALTH_PROBE_RESULT unavailable}" | tee -a "$OUT/navigation_health_gate.txt"
fi
export NAV_HEALTH_GATE_READY NAV_HEALTH_GATE_RESULT

{
  echo "==== gicp publish_tf (must be False: fusion owns map->odom) ===="
  ros2 param get /small_gicp_relocalization publish_tf 2>&1 || true
  echo "==== /tf publishers ===="
  ros2 topic info -v --no-daemon /tf 2>/dev/null | grep -E "Node name|Publisher count" || true
  echo "==== observation topic ===="
  ros2 topic info -v --no-daemon /relocalization_observation 2>/dev/null | head -25 || true
} | tee "$OUT/graph.txt"

# The probe's exit code is data, not a reason to abandon the cell: a failed
# repetition still owes the report, the log digest and a clean teardown.
set +e
python3 "$WS/scripts/evaluate_gazebo_reloc_cell.py" | tee "$OUT/cell.json"
PROBE_RC=${PIPESTATUS[0]}
set -e

python3 - <<PY2 | tee "$OUT/launch_hits.txt"
from pathlib import Path
text = Path("$OUT/launch.log").read_text(errors="ignore")
keys = (
    "reject gicp", "multi_guess sweep", "multi_guess done", "confirmation restart",
    "awaiting consistent", "accepted relocalization", "blocked acceptance",
    "loaded global map", "localization fusion ready", "left tracking",
)
for i, line in enumerate(text.splitlines(), 1):
    low = line.lower()
    if any(k in low for k in keys):
        print(f"{i}:{line[:260]}")
PY2

# A starved scan pipeline never exercises the estimator: with the GT relay
# dropping clouds (TF older than max_extrapolation_sec) GICP never reaches
# accumulate_frames and no observation can exist. That is a host-performance
# fault, not a relocalization result, so it is recorded explicitly and the matrix
# driver retries the repetition instead of scoring it.
python3 - "$OUT" <<'PY3'
import json
import re
import sys
from pathlib import Path

out = Path(sys.argv[1])
text = (out / "launch.log").read_text(errors="ignore")
matches = re.findall(
    r"registered_scan stats ok=(\d+) drop=(\d+) \(tf=(\d+) stale=(\d+) xform=(\d+)\)", text
)
report_path = out / "cell.json"
try:
    report = json.loads(report_path.read_text())
except (OSError, ValueError):
    sys.exit(0)

if matches:
    ok, drop, tf, stale, xform = (int(v) for v in matches[-1])
else:
    ok = drop = tf = stale = xform = -1
report["relay_scan_ok"] = ok
report["relay_scan_drop"] = drop
report["relay_scan_drop_tf"] = tf
report["relay_scan_drop_stale"] = stale
report["relay_scan_drop_xform"] = xform
# Measured over 40 repetitions on this host: runs that produced a relocalization
# delivered 128..266 clouds (median 168); every starved run delivered 0, 8 or 57
# and accepted nothing. The threshold sits in that gap, so a host that cannot
# feed the estimator is reported as infrastructure instead of being scored as a
# relocalization failure. ok=-1 (no stats line at all) is the same fault.
# Second criterion: a run whose relay dropped more clouds than it delivered ran
# the estimator on a heavily decimated stream (stale TF under host load, e.g.
# ok=187 drop=981), which carries no relocalization information either. Healthy
# repetitions on this host drop 0..9.
report["sim_starved"] = ok < 100 or drop > ok
report_path.write_text(json.dumps(report, indent=2))
print(f"relay_scan_ok={ok} sim_starved={report['sim_starved']}")
PY3

# Teardown is the EXIT trap's job; a second copy here would only run on success.

echo "PROBE_RC=$PROBE_RC"
echo "OUT=$OUT"
exit "$PROBE_RC"
