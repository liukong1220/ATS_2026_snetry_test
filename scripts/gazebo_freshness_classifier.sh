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


# Attribute a sustained localization/TF lag to the earliest observable layer.
# Labels:
#   upstream_publish  - Gazebo Transport cadence/age already degraded
#   bridge_internal   - Transport healthy, ROS-side Gazebo/Livox lidar age jumps
#   dds_receive       - stamp cadence healthy but wall receive gaps dominate
#   none              - ages and gaps within the configured limit
#   unverified        - required stage metrics absent
# The caller records the label; this function never flips admission gates.
classify_p1_delay_attribution() {
  local evidence_line="$1"
  local age_limit="${2:-0.50}"
  local gap_limit="${3:-0.50}"
  local transport_age gazebo_age livox_age
  local transport_wall gazebo_wall livox_wall
  local transport_stamp_p99 gazebo_stamp_p99 livox_stamp_p99
  local observation_enabled

  if ! freshness_is_number "$age_limit" || ! freshness_is_number "$gap_limit"; then
    printf 'delay_attribution=unverified reason=invalid_threshold\n'
    return 2
  fi

  gazebo_age="$(freshness_metric_value "$evidence_line" gazebo_lidar_p99_stamp_age_s)"
  livox_age="$(freshness_metric_value "$evidence_line" livox_input_p99_stamp_age_s)"
  gazebo_wall="$(freshness_metric_value "$evidence_line" gazebo_lidar_p99_wall_interval_s)"
  livox_wall="$(freshness_metric_value "$evidence_line" livox_input_p99_wall_interval_s)"
  gazebo_stamp_p99="$(freshness_metric_value "$evidence_line" gazebo_lidar_p99_stamp_interval_s)"
  livox_stamp_p99="$(freshness_metric_value "$evidence_line" livox_input_p99_stamp_interval_s)"

  if ! freshness_is_number "$gazebo_age" || ! freshness_is_number "$livox_age" \
    || ! freshness_is_number "$gazebo_wall" || ! freshness_is_number "$livox_wall"; then
    printf 'delay_attribution=unverified reason=ros_lidar_timing_missing\n'
    return 2
  fi

  observation_enabled="$(freshness_metric_value "$evidence_line" gazebo_transport_lidar_observation_enabled)"
  transport_age="$(freshness_metric_value "$evidence_line" gazebo_transport_lidar_p99_stamp_age_s)"
  transport_wall="$(freshness_metric_value "$evidence_line" gazebo_transport_lidar_p99_wall_interval_s)"
  transport_stamp_p99="$(freshness_metric_value "$evidence_line" gazebo_transport_lidar_p99_stamp_interval_s)"

  # Upstream Gazebo Transport publish already late/sparse.
  # Stamp age may be unverified while wall cadence is still measured; prefer
  # wall when age is absent so a failure sample can still be layered.
  if [[ "$observation_enabled" == "yes" ]]; then
    if ! freshness_is_number "$transport_wall"; then
      printf 'delay_attribution=unverified reason=gazebo_transport_lidar_timing_missing\n'
      return 2
    fi
    if freshness_is_number "$transport_age" && freshness_exceeds "$transport_age" "$age_limit"; then
      printf 'delay_attribution=upstream_publish reason=transport_age_p99:%s transport_wall_p99:%s\n' \
        "$transport_age" "$transport_wall"
      return 0
    fi
    if freshness_exceeds "$transport_wall" "$gap_limit"; then
      printf 'delay_attribution=upstream_publish reason=transport_age_p99:%s transport_wall_p99:%s\n' \
        "${transport_age:-unverified}" "$transport_wall"
      return 0
    fi
    # Transport wall healthy, but ROS-side Gazebo lidar age jumps: bridge work.
    if freshness_exceeds "$gazebo_age" "$age_limit"; then
      if ! freshness_is_number "$transport_age" || ! freshness_exceeds "$transport_age" "$age_limit"; then
        printf 'delay_attribution=bridge_internal reason=gazebo_lidar_age_p99:%s transport_age_p99:%s transport_wall_p99:%s\n' \
          "$gazebo_age" "${transport_age:-unverified}" "$transport_wall"
        return 0
      fi
    fi
  fi

  # Stamp cadence stays near nominal while wall receive gaps blow up: DDS/receive.
  if freshness_is_number "$gazebo_stamp_p99" \
    && ! freshness_exceeds "$gazebo_stamp_p99" "$gap_limit" \
    && freshness_exceeds "$gazebo_wall" "$gap_limit"; then
    printf 'delay_attribution=dds_receive reason=gazebo_lidar_wall_p99:%s stamp_p99:%s\n' \
      "$gazebo_wall" "$gazebo_stamp_p99"
    return 0
  fi
  if freshness_is_number "$livox_stamp_p99" \
    && ! freshness_exceeds "$livox_stamp_p99" "$gap_limit" \
    && freshness_exceeds "$livox_wall" "$gap_limit"; then
    printf 'delay_attribution=dds_receive reason=livox_input_wall_p99:%s stamp_p99:%s\n' \
      "$livox_wall" "$livox_stamp_p99"
    return 0
  fi

  # Without transport observation, a ROS-boundary age jump is the bridge symptom.
  if freshness_exceeds "$gazebo_age" "$age_limit" \
    || freshness_exceeds "$livox_age" "$age_limit"; then
    printf 'delay_attribution=bridge_internal reason=gazebo_lidar_age_p99:%s livox_age_p99:%s transport_observer:%s\n' \
      "$gazebo_age" "$livox_age" "${observation_enabled:-no}"
    return 0
  fi

  if freshness_exceeds "$gazebo_wall" "$gap_limit" \
    || freshness_exceeds "$livox_wall" "$gap_limit"; then
    printf 'delay_attribution=dds_receive reason=gazebo_lidar_wall_p99:%s livox_wall_p99:%s\n' \
      "$gazebo_wall" "$livox_wall"
    return 0
  fi

  printf 'delay_attribution=none reason=lidar_timing_within_limits\n'
  return 0
}
