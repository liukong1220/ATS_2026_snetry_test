#!/usr/bin/env bash
# Static contract regression for the P1 Gazebo runner.

set -u -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="$ROOT_DIR/scripts/test_gazebo_minco_mpc_chain.sh"
RECORDER="$ROOT_DIR/src/sim/gazebo_simulator/rmu_gazebo_simulator/src/ats_navigation_evidence_recorder.cpp"
DIRECT_LIDAR_BRIDGE="$ROOT_DIR/src/sim/gazebo_simulator/rmu_gazebo_simulator/src/gz_lidar_ros_bridge.cpp"
NAV_LAUNCH="$ROOT_DIR/src/sim/gazebo_simulator/rmu_gazebo_simulator/launch/ats_gazebo_nav.launch.py"
SPAWN_LAUNCH="$ROOT_DIR/src/sim/gazebo_simulator/rmu_gazebo_simulator/launch/spawn_robots.launch.py"
GAZEBO_LAUNCH="$ROOT_DIR/src/sim/gazebo_simulator/rmu_gazebo_simulator/launch/gazebo.launch.py"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

contains() {
  local pattern="$1" file="$2"
  if command -v rg >/dev/null 2>&1; then
    rg -q -- "$pattern" "$file"
  else
    grep -Eq -- "$pattern" "$file"
  fi
}

REMOVED_GATE_PATTERN=$'\x50\x31\x5f\x52\x45\x53\x4f\x55\x52\x43\x45\x5f\x4d\x4f\x44\x45|\x50\x31\x5f\x52\x45\x53\x4f\x55\x52\x43\x45\x5f\x4d\x41\x58\x5f\x53\x57\x41\x50\x5f\x55\x53\x45\x44\x5f\x47\x49\x42|\x73\x77\x61\x70\x5f\x75\x73\x65\x64\x5f\x67\x69\x62'
if contains "$REMOVED_GATE_PATTERN" "$RUNNER"; then
  fail "runner still contains removed admission gate"
fi
contains 'run_runtime_preflight' "$RUNNER" || fail "runtime preflight is missing"
contains 'runtime_binary_freshness.sh' "$RUNNER" || fail "shared runtime freshness helper is missing"
contains 'libsensor_scan_generation.so' "$RUNNER" || fail "sensor component artifact is not checked"
contains 'libsmall_gicp_relocalization.so' "$RUNNER" || fail "localization component artifact is not checked"
contains 'p1_first_freshness_violation' "$RUNNER" || fail "freshness classifier metric is missing"
contains 'p1_admission_evidence' "$RUNNER" || fail "P1 admission metric is missing"
contains 'wait_for_active_evidence_window' "$RUNNER" || fail "P1 observer window is not enforced"
contains 'active_evidence_completed' "$RUNNER" || fail "P1 observer completion metric is missing"
contains 'observer_duration_shorter_than_requested' "$RUNNER" || fail "P1 admission does not reject a short observer run"
contains 'tf_dynamic_freshness_evidence_missing' "$RUNNER" || fail "P1 admission does not fail closed on missing dynamic TF evidence"
contains 'tf_dynamic_stamp_updates_' "$RUNNER" || fail "P1 admission lacks the dynamic TF update-rate floor"
contains 'tf_dynamic_update_gap_max_' "$RUNNER" || fail "P1 admission lacks the dynamic TF stall gate"
contains 'tf_dynamic_stamp_staleness_p99_' "$RUNNER" || fail "P1 admission lacks the dynamic TF stamp-staleness gate"
contains 'tf_dynamic_staleness_samples_missing' "$RUNNER" || fail "P1 admission does not fail closed on an empty dynamic TF staleness sample set"
contains 'tf_dynamic_age_p99_' "$RUNNER" || fail "P1 admission lacks the dynamic TF absolute stamp-age gate"
contains 'tf_dynamic_age_samples_missing' "$RUNNER" || fail "P1 admission does not fail closed on an empty dynamic TF age sample set"
# Both gated percentiles must sit in the fail-closed field list, not only in
# their thresholds. The sample-count checks do not cover their absence: a
# recorder line carrying the count but not the percentile reaches awk with an
# empty value, which evaluates to 0.0 and reads as a perfectly fresh chain. A
# mutation that dropped the age from this list survived the behavioral suite
# until scripts/test_gazebo_dynamic_tf_gate.sh grew a case for it.
for percentile_field in tf_dynamic_age_p99 tf_dynamic_staleness_p99; do
  awk -v field="$percentile_field" '
    /tf_dynamic_freshness_evidence_missing/ {inside = 0}
    inside && $0 ~ ("\\$" field "\"") {found = 1}
    /Fail closed on a recorder that does not emit these fields/ {inside = 1}
    END {exit !found}
  ' "$RUNNER" ||
    fail "$percentile_field is not in the dynamic TF fail-closed field list"
done
# Both freshness quantities must be gated, because each is blind to a failure the
# other catches, and this gate has already shipped each one alone:
#
#  1. Staleness alone (reference clock elapsed since the stamp last advanced)
#     cancels a lag shared by every sample. Domains 139 and 141 reported a
#     healthy 0.200 s staleness p99 while the edge ran 2.06-2.15 s behind, the
#     ROGMap adapter rejected every projection-stamp lookup, and the action
#     aborted. Admission called both runs freshness-clean.
#  2. The absolute age alone cannot see a frozen broadcaster whose stamps stop
#     advancing while the buffer replays the last transform, if the reference
#     clock is what stopped.
#
# The absolute age was briefly removed on the argument that its ~2.0 s floor was
# a /clock-versus-sensor epoch offset present in clean and degraded runs alike.
# That comparison had no healthy baseline - every run in it was lagged. Domain
# 131 supplies one at 0.012-0.072 s. Do not remove the age gate again.
if contains 'P1_ADMISSION_REASON="tf_dynamic_age_excursion' "$RUNNER"; then
  fail "P1 admission gates a min-of-run age excursion, whose baseline domain 137 showed is unstable"
fi
# Neither maximum is gated: a consumer-side /clock catch-up inflates exactly one
# age and one staleness sample. The peak stall is bounded by the update-gap gate,
# which is measured on the steady clock and no sim-clock jump can perturb.
if contains 'P1_ADMISSION_REASON="tf_dynamic_stamp_staleness_max' "$RUNNER"; then
  fail "P1 admission gates the dynamic TF staleness maximum, which a single clock catch-up can trip"
fi
if contains 'P1_ADMISSION_REASON="tf_dynamic_age_max_' "$RUNNER"; then
  fail "P1 admission gates the dynamic TF age maximum, which a single clock catch-up can trip"
fi
contains 'dynamic_transform_freshness.hpp' "$RECORDER" || fail "recorder does not use the dynamic TF freshness witness"
contains 'tf_dynamic_distinct_stamp_updates=' "$RECORDER" || fail "recorder does not emit dynamic TF stamp-update evidence"
contains 'tf_dynamic_age_p99_s=' "$RECORDER" || fail "recorder does not emit dynamic TF age evidence"
contains 'tf_dynamic_stamp_staleness_p99_s=' "$RECORDER" || fail "recorder does not emit dynamic TF stamp-staleness evidence"
contains 'tf_dynamic_age_floor_s=' "$RECORDER" || fail "recorder does not emit the dynamic TF age floor for diagnosis"
contains 'tf_dynamic_backward_clock_samples=' "$RECORDER" || fail "recorder does not report reference-clock regressions that would distort staleness"
# The witness must read the stamp of the transform the lookup returned. A
# TimePointZero query whose result is discarded proves only that the chain
# resolves, which is the gap this evidence exists to close.
contains 'transform\.header\.stamp' "$RECORDER" || fail "recorder does not read the returned transform source stamp"
contains 'ENABLE_CAMERA_SENSORS' "$RUNNER" || fail "camera sensor A/B entry point is missing"
contains 'HEADLESS_RENDERING' "$RUNNER" || fail "Gazebo headless rendering entry point is missing"
contains 'LIVOX_UPDATE_RATE_HZ' "$RUNNER" || fail "LiDAR timing A/B entry point is missing"
contains 'LIVOX_HORIZONTAL_SAMPLES' "$RUNNER" || fail "LiDAR density A/B entry point is missing"
contains 'OBSERVE_GAZEBO_TRANSPORT_LIDAR' "$RUNNER" || fail "Gazebo Transport LiDAR diagnostic entry point is missing"
contains 'ats_swerve_mpc' "$RUNNER" || fail "MPC runtime freshness preflight is missing"
contains 'USE_DIRECT_GAZEBO_LIDAR_BRIDGE' "$RUNNER" || fail "direct Gazebo LiDAR bridge entry point is missing"
contains 'LIDAR_BRIDGE_PUBLISHER_DEPTH' "$RUNNER" || fail "generic LiDAR publisher depth entry point is missing"
contains 'LIDAR_BRIDGE_PUBLISHER_RELIABILITY' "$RUNNER" || fail "generic LiDAR publisher reliability entry point is missing"
contains 'WORLD_SDF_PATH' "$RUNNER" || fail "explicit world SDF runner entry point is missing"
contains 'EXTRA_LAUNCH_ARGS\+=\("world_sdf_path:=\$WORLD_SDF_PATH"\)' "$RUNNER" || fail "runner does not conditionally forward world SDF path"
contains 'world_sdf_path' "$NAV_LAUNCH" || fail "top-level launch does not expose world SDF path"
contains '"world_sdf_path": LaunchConfiguration\("world_sdf_path"\)' "$NAV_LAUNCH" || fail "top-level launch does not forward world SDF path"
contains 'livox_update_rate_hz' "$NAV_LAUNCH" || fail "top-level launch does not expose LiDAR timing"
contains 'mapping.lidar_time_inte' "$NAV_LAUNCH" || fail "Point-LIO scan period is not tied to LiDAR timing"
contains 'world_sdf_path' "$GAZEBO_LAUNCH" || fail "Gazebo launch does not resolve world SDF path"
contains 'clock gazebo_transport_lidar gazebo_lidar livox_input cloud_registered lidar_odometry' "$RUNNER" || fail "Gazebo Transport and raw LiDAR cadence stages are missing"
contains '"/" \+ robot_name_ \+ "/livox/lidar"' "$RECORDER" || fail "recorder is not subscribed to the raw Gazebo LiDAR topic"
contains 'arrivalStatistics\("gazebo_lidar", gazebo_lidar_arrivals_\)' "$RECORDER" || fail "recorder does not emit raw Gazebo LiDAR statistics"
contains 'observe_gazebo_transport_lidar' "$RECORDER" || fail "recorder lacks Gazebo Transport LiDAR opt-in parameter"
contains 'gazebo_transport_node_\.Subscribe' "$RECORDER" || fail "recorder does not subscribe to Gazebo Transport LiDAR"
contains 'gazebo_transport_lidar_mutex_' "$RECORDER" || fail "transport callback statistics are not synchronized"
contains 'gazebo_transport_lidar' "$RUNNER" || fail "runner does not emit Gazebo Transport LiDAR metrics"

# The recorder polls map -> gimbal_yaw_odom before localization_fusion can
# publish it, so admission must judge post-establishment failures and require
# the chain to actually come up, instead of rejecting warm-up absence.
contains 'tf_chain_established' "$RECORDER" || fail "recorder does not report TF chain establishment"
contains 'tf_lookup_failures_after_establishment' "$RECORDER" || fail "recorder does not separate post-establishment TF failures"
contains 'tf_chain_never_established' "$RUNNER" || fail "admission does not require an established TF chain"
contains 'tf_lookup_failures_after_establishment' "$RUNNER" || fail "admission does not gate on post-establishment TF failures"

contains 'use_direct_gazebo_lidar_bridge' "$NAV_LAUNCH" || fail "top-level launch does not expose direct LiDAR bridge"
contains 'use_direct_gazebo_lidar_bridge' "$SPAWN_LAUNCH" || fail "spawn launch does not route direct LiDAR bridge"
contains 'qos_overrides.' "$SPAWN_LAUNCH" || fail "spawn launch does not configure generic LiDAR QoS overrides"
contains 'lidar_bridge_publisher_depth' "$NAV_LAUNCH" || fail "top-level launch does not expose generic LiDAR publisher depth"
contains 'lidar_bridge_publisher_reliability' "$NAV_LAUNCH" || fail "top-level launch does not expose generic LiDAR publisher reliability"
contains 'BEST_EFFORT' "$DIRECT_LIDAR_BRIDGE" || fail "direct LiDAR bridge does not document BEST_EFFORT QoS"
contains 'sensor_qos.best_effort' "$DIRECT_LIDAR_BRIDGE" || fail "direct LiDAR bridge does not configure BEST_EFFORT QoS"
contains 'convert_gz_to_ros' "$DIRECT_LIDAR_BRIDGE" || fail "direct LiDAR bridge does not retain standard conversion"
contains 'latest_message_' "$DIRECT_LIDAR_BRIDGE" || fail "direct LiDAR bridge does not retain a bounded latest-frame slot"
contains 'worker_thread_' "$DIRECT_LIDAR_BRIDGE" || fail "direct LiDAR bridge does not isolate conversion from Transport callback"

# The LiDAR stamp-age evidence puts the lag between gz-transport publication and
# ROS receipt, so the bridge process must appear in the resource capture or the
# implicated participant is the one that goes unmeasured.
awk '
  /capture_launch_process_resources\(\)/ {inside = 1}
  # Match the ps filter expression, not prose: a comment naming the process
  # would otherwise satisfy this check while the filter had dropped it.
  inside && /\$0 ~ \// && /parameter_bridge/ {found = 1}
  inside && /^}/ {inside = 0}
  END {exit !found}
' "$RUNNER" || fail "parameter_bridge is not in the launch process resource capture filter"

echo "PASS: Gazebo runner runtime contract"
