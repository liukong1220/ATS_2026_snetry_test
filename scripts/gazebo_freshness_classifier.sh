#!/usr/bin/env bash
# Pure parser for the single-line C++ navigation evidence witness.
#
# The classifier deliberately reports the first observable timing contract
# violation in pipeline order. It never changes a runtime timeout or declares
# admission by itself; the caller owns the resource and action gates.

# Stages follow real message flow, so the reported violation is the earliest
# observable one rather than the earliest one that happens to be checked:
#   Gazebo Transport -> generic/direct LiDAR bridge -> /<robot>/livox/lidar
#   -> gz_livox_bridge -> /livox/lidar -> Point-LIO -> /cloud_registered
#   -> loam_interface -> /lidar_odometry -> sensor_scan_generation -> /odometry
#   -> localization_fusion -> /localization -> /localization/status
# Starting the scan downstream of the ROS boundary would attribute a bridge
# delivery gap to Point-LIO, which the recorder already measures separately.
GAZEBO_FRESHNESS_PIPELINE_STAGES=(
  gazebo_transport_lidar
  gazebo_lidar
  livox_input
  cloud_registered
  lidar_odometry
  odometry
  localization
  localization_status
)

# Stages measured only behind an explicit observation switch. Their metrics read
# `unverified` while the observer is off, which is an absent measurement rather
# than a contract violation, so they are skipped instead of blocking the
# classification. Every other stage is emitted unconditionally, so a missing
# metric there means truncated evidence and still returns 2.
GAZEBO_FRESHNESS_OPTIONAL_STAGES=(
  gazebo_transport_lidar
)

freshness_metric_value() {
  local evidence_line="$1"
  local key="$2"
  awk -v key="${key}=" '
    {
      for (field_index = 1; field_index <= NF; ++field_index) {
        if (substr($field_index, 1, length(key)) == key) {
          print substr($field_index, length(key) + 1)
          exit
        }
      }
    }
  ' <<<"${evidence_line}"
}

freshness_is_number() {
  awk -v value="$1" 'BEGIN {
    exit !(value ~ /^([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$/)
  }'
}

freshness_exceeds() {
  local value="$1"
  local limit="$2"
  freshness_is_number "$value" && awk -v value="$value" -v limit="$limit" \
    'BEGIN { exit !(value > limit) }'
}

freshness_stage_is_optional() {
  local stage="$1"
  local optional_stage
  for optional_stage in "${GAZEBO_FRESHNESS_OPTIONAL_STAGES[@]}"; do
    if [ "$stage" = "$optional_stage" ]; then
      return 0
    fi
  done
  return 1
}

# Output is a compact key/value record suitable for both shell metrics and
# deterministic tests. Return 2 when a required metric is unavailable.
classify_gazebo_freshness() {
  local evidence_line="$1"
  local p99_limit="${2:-0.25}"
  local max_gap_limit="${3:-0.5}"
  local stage p99 max_gap

  if ! freshness_is_number "$p99_limit" || ! freshness_is_number "$max_gap_limit"; then
    printf 'first_violation=unverified reason=invalid_threshold\n'
    return 2
  fi

  max_gap="$(freshness_metric_value "$evidence_line" clock_max_wall_interval_s)"
  if ! freshness_is_number "$max_gap"; then
    printf 'first_violation=unverified reason=clock_max_wall_interval_missing\n'
    return 2
  fi
  if freshness_exceeds "$max_gap" "$max_gap_limit"; then
    printf 'first_violation=clock reason=max_wall_interval_s:%s\n' "$max_gap"
    return 0
  fi

  for stage in "${GAZEBO_FRESHNESS_PIPELINE_STAGES[@]}"; do
    p99="$(freshness_metric_value "$evidence_line" "${stage}_p99_wall_interval_s")"
    max_gap="$(freshness_metric_value "$evidence_line" "${stage}_max_wall_interval_s")"
    if ! freshness_is_number "$p99" || ! freshness_is_number "$max_gap"; then
      if freshness_stage_is_optional "$stage"; then
        observation_enabled="$(freshness_metric_value "$evidence_line" \
          "${stage}_observation_enabled")"
        if [[ -z "$p99" && -z "$max_gap" ]] ||
          { [[ "$p99" == "unverified" && "$max_gap" == "unverified" ]] &&
            [[ "$observation_enabled" != "yes" ]]; }; then
          continue
        fi
      fi
      printf 'first_violation=unverified reason=%s_timing_missing\n' "$stage"
      return 2
    fi
    if freshness_exceeds "$p99" "$p99_limit"; then
      printf 'first_violation=%s reason=p99_wall_interval_s:%s limit:%s\n' \
        "$stage" "$p99" "$p99_limit"
      return 0
    fi
    if freshness_exceeds "$max_gap" "$max_gap_limit"; then
      printf 'first_violation=%s reason=max_wall_interval_s:%s limit:%s\n' \
        "$stage" "$max_gap" "$max_gap_limit"
      return 0
    fi
  done

  printf 'first_violation=none reason=timing_contract_observed\n'
  return 0
}
