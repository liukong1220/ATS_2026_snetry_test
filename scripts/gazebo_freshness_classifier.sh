#!/usr/bin/env bash
# Pure parser for the single-line C++ navigation evidence witness.
#
# The classifier deliberately reports the first observable timing contract
# violation in pipeline order. It never changes a runtime timeout or declares
# admission by itself; the caller owns the resource and action gates.

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

  for stage in lidar_odometry odometry localization localization_status; do
    p99="$(freshness_metric_value "$evidence_line" "${stage}_p99_wall_interval_s")"
    max_gap="$(freshness_metric_value "$evidence_line" "${stage}_max_wall_interval_s")"
    if ! freshness_is_number "$p99" || ! freshness_is_number "$max_gap"; then
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
