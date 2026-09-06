#!/usr/bin/env bash
# Cross-validate scripts/footprint_evaluator.py against the real C++ checker.
#
# footprint_evaluator.py exists so one geometry can judge BOTH a planner
# reference trajectory and the actually driven pose sequence against the same
# immutable snapshot.  That only means anything if the port agrees with the
# checker the planner itself runs, so this harness links the genuine
# minco_planner/src/safety/footprint_safety_checker.cpp translation unit, dumps
# every number the port is supposed to reproduce for a pseudo-random case set,
# and asserts the Python result matches exactly.
#
# Compared per case: safe verdict, discrete and swept collision counts, the
# three sample/segment counters, and the first collision's trajectory index,
# swept flag, segment fraction and offending sample world position.
#
# Two densities are run because they exercise different paths: dense grids
# reject almost always, sparse grids cover the SAFE verdict and interior hits.
# Neither density says anything about physical contact; this is geometry only.

set -u -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

SEEDS="${PARITY_SEEDS:-1 2 3 5 7 11 13 17 19 23 29 31 37 41 43 47}"
CHECKER_SRC="$ROOT_DIR/src/ats_sentry_nav/minco_planner/src/safety/footprint_safety_checker.cpp"
EVALUATOR="$ROOT_DIR/scripts/footprint_evaluator.py"

for required in "$CHECKER_SRC" "$EVALUATOR"; do
  if [ ! -f "$required" ]; then
    echo "RESULT: footprint evaluator parity FAILED (missing $required)" >&2
    exit 1
  fi
done

ROS_INCLUDES=(-I/opt/ros/humble/include)
for pkg in nav_msgs std_msgs builtin_interfaces geometry_msgs rosidl_runtime_c \
           rosidl_typesupport_interface rcutils rosidl_runtime_cpp; do
  ROS_INCLUDES+=("-I/opt/ros/humble/include/${pkg}")
done

cat >"$WORK_DIR/parity.cpp" <<'CPPEOF'
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <random>
#include <utility>
#include <vector>
#include "minco_planner/safety/footprint_safety_checker.hpp"

using minco_planner::FootprintSafetyChecker;
using minco_planner::FootprintSafetyParams;
using minco_planner::ReferencePoint;
using minco_planner::ReferenceTrajectory;

int main(int argc, char ** argv)
{
  const unsigned seed = (argc > 1) ? static_cast<unsigned>(std::atoi(argv[1])) : 7u;
  // mode 1 keeps obstacles sparse and poses inside a clear interior band so the
  // SAFE path and interior first-hits are covered, not just dense rejections.
  const int mode = (argc > 2) ? std::atoi(argv[2]) : 0;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> pos(mode ? 1.0 : 0.2, mode ? 2.0 : 2.8);
  std::uniform_real_distribution<double> yaw(-M_PI, M_PI);
  std::uniform_int_distribution<int> cell(0, 29);
  std::uniform_int_distribution<int> nblock(0, mode ? 2 : 8);

  std::printf("[\n");
  for (int c = 0; c < 40; ++c) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = 0.10;
    grid.info.width = 30;
    grid.info.height = 30;
    const double oyaw = (c % 3 == 0) ? 0.0 : ((c % 3 == 1) ? 0.7 : -1.3);
    grid.info.origin.position.x = (c % 2) ? -1.0 : 0.0;
    grid.info.origin.position.y = (c % 2) ? -0.5 : 0.0;
    grid.info.origin.orientation.z = std::sin(0.5 * oyaw);
    grid.info.origin.orientation.w = std::cos(0.5 * oyaw);
    grid.data.assign(900, 0);
    std::vector<std::pair<int, int>> blocked;
    const int count = nblock(rng);
    for (int b = 0; b < count; ++b) {
      const int bx = cell(rng), by = cell(rng);
      const int8_t v = (b % 4 == 0) ? static_cast<int8_t>(-1)
        : (b % 4 == 1) ? static_cast<int8_t>(100)
        : (b % 4 == 2) ? static_cast<int8_t>(99)
        : static_cast<int8_t>(60);
      grid.data[static_cast<std::size_t>(by) * 30 + bx] = v;
      blocked.push_back({bx, by});
    }
    ReferenceTrajectory traj;
    traj.header.frame_id = "map";
    const int points = 2 + (c % 4);
    for (int p = 0; p < points; ++p) {
      ReferencePoint rp;
      rp.x = pos(rng);
      rp.y = pos(rng);
      rp.yaw = yaw(rng);
      rp.t = 0.3 * p;
      rp.s = 0.4 * p;
      traj.points.push_back(rp);
    }
    FootprintSafetyParams params;
    params.length = 0.60;
    params.width = 0.50;
    params.safety_margin = 0.02;
    params.obstacle_value_threshold = (c % 2) ? 100 : 50;
    params.unknown_is_obstacle = (c % 5 == 0);
    params.swept_max_corner_step_cells = (c % 7 == 0) ? 1.0 : 0.5;
    FootprintSafetyChecker checker(params);
    const auto result = checker.check(traj, grid);
    std::size_t discrete = 0, swept = 0;
    for (const auto & s : result.collisions) {
      if (s.swept) {++swept;} else {++discrete;}
    }
    std::printf("%s{\"case\":%d,\"origin_yaw\":%.17g,\"origin_x\":%.17g,\"origin_y\":%.17g,",
      c ? ",\n" : "", c, oyaw, grid.info.origin.position.x, grid.info.origin.position.y);
    std::printf("\"threshold\":%d,\"unknown_is_obstacle\":%s,\"step_cells\":%.17g,",
      params.obstacle_value_threshold, params.unknown_is_obstacle ? "true" : "false",
      params.swept_max_corner_step_cells);
    std::printf("\"blocked\":[");
    for (std::size_t b = 0; b < blocked.size(); ++b) {
      std::printf("%s[%d,%d,%d]", b ? "," : "", blocked[b].first, blocked[b].second,
        static_cast<int>(grid.data[static_cast<std::size_t>(blocked[b].second) * 30 +
        blocked[b].first]));
    }
    std::printf("],\"poses\":[");
    for (std::size_t p = 0; p < traj.points.size(); ++p) {
      std::printf("%s[%.17g,%.17g,%.17g]", p ? "," : "",
        traj.points[p].x, traj.points[p].y, traj.points[p].yaw);
    }
    std::printf("],\"safe\":%s,\"discrete\":%zu,\"swept\":%zu,",
      result.safe ? "true" : "false", discrete, swept);
    std::printf("\"discrete_checked\":%zu,\"swept_segments\":%zu,\"swept_samples\":%zu",
      result.discrete_samples_checked, result.swept_segments_checked,
      result.swept_samples_checked);
    if (!result.collisions.empty()) {
      const auto & f = result.collisions.front();
      std::printf(
        ",\"first\":{\"index\":%zu,\"swept\":%s,\"fraction\":%.17g,\"x\":%.17g,\"y\":%.17g}",
        f.trajectory_index, f.swept ? "true" : "false", f.segment_fraction, f.x, f.y);
    }
    std::printf("}");
  }
  std::printf("\n]\n");
  return 0;
}
CPPEOF

if ! g++ -std=c++14 -O1 -o "$WORK_DIR/parity" "$WORK_DIR/parity.cpp" "$CHECKER_SRC" \
    -I"$ROOT_DIR/src/ats_sentry_nav/minco_planner/include" -I/usr/include/eigen3 \
    "${ROS_INCLUDES[@]}" 2>"$WORK_DIR/compile.log"; then
  echo "could not build the C++ parity harness:" >&2
  cat "$WORK_DIR/compile.log" >&2
  echo "RESULT: footprint evaluator parity FAILED (compile)" >&2
  exit 1
fi

cat >"$WORK_DIR/parity_check.py" <<'PYEOF'
"""Assert the Python evaluator reproduces the linked C++ checker exactly."""
import json
import os
import sys

sys.path.insert(0, os.environ["EVALUATOR_DIR"])
import footprint_evaluator as fe  # noqa: E402

CASES = json.load(open(sys.argv[1]))
failures = []
checked = 0


def near(a, b, tol=1e-9):
    if a is None or b is None:
        return a is b or a == b
    return abs(a - b) <= tol


for case in CASES:
    width = height = 30
    data = [0] * (width * height)
    for bx, by, value in case["blocked"]:
        data[by * width + bx] = value
    grid = fe.Grid(
        width=width, height=height, resolution=0.10,
        origin_x=case["origin_x"], origin_y=case["origin_y"],
        origin_yaw=case["origin_yaw"], data=data,
        frame_id="map", identity="parity")
    params = fe.FootprintParams(
        length=0.60, width=0.50, safety_margin=0.02,
        obstacle_value_threshold=case["threshold"],
        unknown_is_obstacle=case["unknown_is_obstacle"],
        swept_max_corner_step_cells=case["step_cells"])
    poses = [(p[0], p[1], p[2]) for p in case["poses"]]
    got = fe.check(grid, params, poses, max_collisions=10 ** 6)
    cid = case["case"]
    checked += 1

    for key, want_key, label in (
            ("discrete_collision_count", "discrete", "discrete"),
            ("swept_collision_count", "swept", "swept"),
            ("discrete_samples_checked", "discrete_checked", "discrete_samples_checked"),
            ("swept_segments_checked", "swept_segments", "swept_segments_checked"),
            ("swept_samples_checked", "swept_samples", "swept_samples_checked")):
        if got[key] != case[want_key]:
            failures.append("case %d: %s %d != C++ %d"
                            % (cid, label, got[key], case[want_key]))
    if got["safe"] != case["safe"]:
        failures.append("case %d: safe %s != C++ %s" % (cid, got["safe"], case["safe"]))

    want_first = case.get("first")
    first = got["first_collision"]
    if want_first is None:
        if first is not None:
            failures.append("case %d: python reported a first collision, C++ none" % cid)
        continue
    if first is None:
        failures.append("case %d: python reported no first collision, C++ index %d"
                        % (cid, want_first["index"]))
        continue
    if first["trajectory_index"] != want_first["index"]:
        failures.append("case %d: first index %d != C++ %d"
                        % (cid, first["trajectory_index"], want_first["index"]))
    if bool(first["swept"]) != bool(want_first["swept"]):
        failures.append("case %d: first swept %s != C++ %s"
                        % (cid, first["swept"], want_first["swept"]))
    if not near(first["segment_fraction"], want_first["fraction"]):
        failures.append("case %d: first fraction %.17g != C++ %.17g"
                        % (cid, first["segment_fraction"], want_first["fraction"]))
    # CollisionSample.x/y is the OFFENDING SAMPLE world position, not the pose
    # centre.  Comparing the centre instead would pass without checking anything.
    if not near(first["sample_x"], want_first["x"]) or \
            not near(first["sample_y"], want_first["y"]):
        failures.append("case %d: first sample (%.17g,%.17g) != C++ (%.17g,%.17g)"
                        % (cid, first["sample_x"], first["sample_y"],
                           want_first["x"], want_first["y"]))

print("checked %d" % checked)
for f in failures[:20]:
    print("FAIL: " + f)
sys.exit(1 if failures else 0)
PYEOF

export EVALUATOR_DIR="$ROOT_DIR/scripts"
TOTAL=0
FAILURES=0
for mode in 0 1; do
  for seed in $SEEDS; do
    if ! "$WORK_DIR/parity" "$seed" "$mode" >"$WORK_DIR/cases.json"; then
      echo "FAIL: C++ harness exited nonzero for seed=$seed mode=$mode" >&2
      FAILURES=$((FAILURES + 1))
      continue
    fi
    output="$(python3 "$WORK_DIR/parity_check.py" "$WORK_DIR/cases.json" 2>&1)"
    status=$?
    count="$(printf '%s' "$output" | sed -n 's/^checked \([0-9]*\)$/\1/p')"
    TOTAL=$((TOTAL + ${count:-0}))
    if [ "$status" -ne 0 ]; then
      FAILURES=$((FAILURES + 1))
      echo "FAIL: seed=$seed mode=$mode diverged from the C++ checker" >&2
      printf '%s\n' "$output" | grep '^FAIL:' >&2
    fi
  done
done

echo "PARITY: compared $TOTAL cases against the linked C++ FootprintSafetyChecker"
if [ "$FAILURES" -ne 0 ]; then
  echo "RESULT: footprint evaluator parity FAILED ($FAILURES seed/mode combination(s))"
  exit 1
fi
if [ "$TOTAL" -lt 100 ]; then
  echo "RESULT: footprint evaluator parity FAILED (only $TOTAL cases compared)"
  exit 1
fi
echo "RESULT: footprint evaluator parity PASSED"
exit 0
