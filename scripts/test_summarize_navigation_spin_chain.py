import json
import math

import pytest

from summarize_navigation_spin_chain import segment_velocity, summarize


def test_horizon_uses_model_dt_and_shortest_yaw_arc_with_repeated_stamps():
    first = [123, 0.0, 0.0, math.pi - 0.075]
    second = [123, 0.02, 0.0, -math.pi + 0.075]
    value = segment_velocity(first, second, 0.05)
    assert value == pytest.approx({'vx': 0.4, 'vy': 0.0, 'wz': 3.0})
    with pytest.raises(ValueError):
        segment_velocity(first, second, 0.0)


def test_missing_streams_remain_explicit_and_bad_runtime_dt_is_rejected(tmp_path):
    trace = tmp_path / 'trace.jsonl'
    trace.write_text(json.dumps({'kind': 'selected', 'ros_time_ns': 10**9,
                                'data': {'linear': {'x': 0.4, 'y': 0.0},
                                         'angular': {'z': 3.0}}}) + '\n')
    config = dict(dt=0.05, max_wz=3.0, max_awz=2.0, max_wheel_speed=1.669,
                  wheel_base_x=0.27, wheel_base_y=0.27)
    params = {'nodes': {'ats_swerve_mpc': {'values': config}}}
    result = summarize(trace, params)
    assert result['missing_streams'] == ['autonomy_raw', 'horizon', 'motion_control', 'telemetry']
    assert result['streams']['selected']['near_target_duration_s'] == 0
    config['dt'] = float('nan')
    with pytest.raises(ValueError):
        summarize(trace, params)
