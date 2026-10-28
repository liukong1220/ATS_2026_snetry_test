#!/usr/bin/env bash
# Offline regression of the runner's simulation-time gates; no ROS, no MuJoCo.
# `ros2` is a stub that prints a scripted /clock, so the slow-simulation cases
# (the lio-mode failure: 30 s wall gate vs 16 s of simulated time) run in seconds.
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
for name in sim_clock_now goal_wall_timeout_s enable_sim_time_gates wait_for_command_sim_time wait_for_command capture_pose pose_axis; do
  body="$(extract_function "${name}")"
  if [[ -z "${body}" ]]; then
    echo "FAIL: ${name} is not defined in ${RUNNER}"
    exit 1
  fi
  printf '%s\n' "${body}" >>"${WORK_DIR}/helpers.sh"
done

mkdir -p "${WORK_DIR}/bin"
cat >"${WORK_DIR}/bin/ros2" <<'STUB'
#!/usr/bin/env bash
# Prints builtin_interfaces/Time like `ros2 topic echo --once --field clock /clock`.
# STUB_CLOCK_STEP simulated seconds are added per call; STUB_NO_CLOCK=1 publishes nothing.
# capture_pose: record which topic was asked for and answer like `--field pose.pose.position`.
if [[ " $* " == *" pose.pose.position "* ]]; then
  echo "$*" >"${STUB_STATE}/pose_args"
  printf 'x: 1.5\ny: 2.5\nz: 0.0\n---\n'
  exit 0
fi
[[ "${STUB_NO_CLOCK:-0}" == "1" ]] && exit 1
calls_file="${STUB_STATE}/clock_calls"
calls=$(( $(cat "${calls_file}" 2>/dev/null || echo 0) + 1 ))
echo "${calls}" >"${calls_file}"
awk -v n="${calls}" -v step="${STUB_CLOCK_STEP}" 'BEGIN {
  t = 100.0 + n * step
  printf "sec: %d\nnanosec: %d\n---\n", int(t), (t - int(t)) * 1e9
}'
STUB
chmod +x "${WORK_DIR}/bin/ros2"

# run_case NAME EXPECTED_RC EXPECTED_TEXT -- ENV... : runs one scenario in a fresh shell.
run_case() {
  local name="$1" expected_rc="$2" expected_text="$3"
  shift 3
  [[ "$1" == "--" ]] && shift
  local state="${WORK_DIR}/state_${name}"
  mkdir -p "${state}"
  local output rc
  output="$(env -i PATH="${WORK_DIR}/bin:/usr/bin:/bin" STUB_STATE="${state}" "$@" \
    bash -c '
      set -u
      fail() { echo "FAIL: $*"; exit 1; }
      LAUNCH_PID=$$
      source "'"${WORK_DIR}"'/helpers.sh"
      SIM_TIME_GATES="${SIM_TIME_GATES:-auto}"
      SIM_GATE_WALL_CAP_S="${SIM_GATE_WALL_CAP_S:-1800}"
      SIM_GATE_STALL_S="${SIM_GATE_STALL_S:-90}"
      GOAL_WALL_PER_SIM="${GOAL_WALL_PER_SIM:-6}"
      SIM_TIME_GATES_ACTIVE="${SIM_TIME_GATES_ACTIVE:-0}"
      '"${CASE_BODY}"'
    ' 2>&1)"
  rc=$?
  if [[ "${rc}" == "${expected_rc}" && "${output}" == *"${expected_text}"* ]]; then
    echo "PASS: ${name} (rc=${rc}; \"${expected_text}\")"
  else
    echo "FAIL: ${name}: rc=${rc} (want ${expected_rc}), want text \"${expected_text}\""
    echo "${output}" | sed 's/^/    | /'
    FAILURES=$((FAILURES + 1))
  fi
}

# A command that succeeds on its third attempt.
cat >"${WORK_DIR}/bin/flaky" <<'STUB'
#!/usr/bin/env bash
n=$(( $(cat "${STUB_STATE}/cmd" 2>/dev/null || echo 0) + 1 ))
echo "${n}" >"${STUB_STATE}/cmd"
[[ "${n}" -ge 3 ]]
STUB
chmod +x "${WORK_DIR}/bin/flaky"

CASE_BODY='wait_for_command "slow start" 30 flaky'
# Active gate, simulation at 5 simulated s per clock read: succeeds inside 30 simulated s
# and reports the simulated time it took.
run_case sim_gate_success 0 "OK: slow start (sim" -- \
  SIM_TIME_GATES_ACTIVE=1 STUB_CLOCK_STEP=5

CASE_BODY='wait_for_command "never ready" 25 false'
# Fails on simulated time (10 s per read -> >=25 s after 3 reads), not on the wall clock.
run_case sim_gate_timeout 1 "timeout waiting for never ready (sim" -- \
  SIM_TIME_GATES_ACTIVE=1 STUB_CLOCK_STEP=10

CASE_BODY='wait_for_command "frozen sim" 100000 false'
# A frozen /clock is a dead simulation, reported as such rather than as a slow one.
run_case sim_gate_clock_stall 1 "simulation clock stalled" -- \
  SIM_TIME_GATES_ACTIVE=1 STUB_CLOCK_STEP=0 SIM_GATE_STALL_S=3

CASE_BODY='wait_for_command "creeping sim" 100000 false'
# /clock advances, but so slowly that the wall-clock cap trips first.
run_case sim_gate_wall_cap 1 "wall-clock cap 3 s hit" -- \
  SIM_TIME_GATES_ACTIVE=1 STUB_CLOCK_STEP=0.001 SIM_GATE_WALL_CAP_S=3

CASE_BODY='SIM_TIME_GATES=auto; EXTRA_LAUNCH_ARGS=""; enable_sim_time_gates; echo "active=${SIM_TIME_GATES_ACTIVE}"'
run_case truth_mode_stays_on_wall_clock 0 "active=0" -- STUB_NO_CLOCK=1 STUB_CLOCK_STEP=1

CASE_BODY='SIM_TIME_GATES=auto; EXTRA_LAUNCH_ARGS="lidar_downsample:=1 localization_source:=lio sim_rate_hz:=400.0"; enable_sim_time_gates; echo "active=${SIM_TIME_GATES_ACTIVE}"'
run_case lio_mode_enables_gates 0 "active=1" -- STUB_CLOCK_STEP=1

CASE_BODY='SIM_TIME_GATES=0; EXTRA_LAUNCH_ARGS="localization_source:=lio"; enable_sim_time_gates; echo "active=${SIM_TIME_GATES_ACTIVE}"'
run_case forced_off_wins_over_lio 0 "active=0" -- STUB_CLOCK_STEP=1

CASE_BODY='echo "inactive=$(goal_wall_timeout_s 300) "; SIM_TIME_GATES_ACTIVE=1; echo "active=$(goal_wall_timeout_s 300)"'
run_case goal_wall_backstop 0 "inactive=300 " -- STUB_CLOCK_STEP=1
run_case goal_wall_backstop_scaled 0 "active=1800" -- STUB_CLOCK_STEP=1

CASE_BODY='wait_for_command "wall legacy" 5 flaky'
# Inactive gate keeps the original wall-clock path (no simulated-time suffix).
run_case inactive_gate_uses_wall_clock 0 "OK: wall legacy" -- STUB_CLOCK_STEP=1

# The pose used for "did the robot reach the goal" and for the planning-grid queries must be in the
# map frame.  /localization is Point-LIO's odom-frame pass-through in lio mode (report 5.1.4): with
# the start at (4.5, 6.25) the old check reported 7.76 m of error for a goal reached to 0.03 m.
POSE_BLOCK="$(awk '/^POSE_TOPIC=\/localization$/ {inside = 1} inside {print} inside && /^fi$/ {exit}' "${RUNNER}")"
if [[ -z "${POSE_BLOCK}" ]]; then
  echo "FAIL: POSE_TOPIC block not found in ${RUNNER}"
  exit 1
fi
CASE_BODY='LIO_MODE=1; '"${POSE_BLOCK}"'
capture_pose "${STUB_STATE}/pose.txt" && cat "${STUB_STATE}/pose_args" && echo "x=$(pose_axis "${STUB_STATE}/pose.txt" x)"'
run_case lio_pose_is_the_map_frame_ground_truth 0 "best_effort /simulation/ground_truth/odometry --field pose.pose.position" -- STUB_CLOCK_STEP=1
CASE_BODY='LIO_MODE=0; '"${POSE_BLOCK}"'
capture_pose "${STUB_STATE}/pose.txt" && cat "${STUB_STATE}/pose_args" && echo "x=$(pose_axis "${STUB_STATE}/pose.txt" x)"'
run_case truth_pose_stays_on_localization 0 "best_effort /localization --field pose.pose.position" -- STUB_CLOCK_STEP=1
run_case pose_axis_reads_x 0 "x=1.5" -- STUB_CLOCK_STEP=1

if (( FAILURES > 0 )); then
  echo "${FAILURES} simulation-time gate case(s) failed"
  exit 1
fi
echo "OK: simulation-time gate regression passed"
