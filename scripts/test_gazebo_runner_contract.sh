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
contains 'p1_first_freshness_violation' "$RUNNER" || fail "freshness classifier metric is missing"
contains 'p1_admission_evidence' "$RUNNER" || fail "P1 admission metric is missing"
contains 'wait_for_active_evidence_window' "$RUNNER" || fail "P1 observer window is not enforced"
contains 'active_evidence_completed' "$RUNNER" || fail "P1 observer completion metric is missing"
contains 'observer_duration_shorter_than_requested' "$RUNNER" || fail "P1 admission does not reject a short observer run"
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

echo "PASS: Gazebo runner runtime contract"
