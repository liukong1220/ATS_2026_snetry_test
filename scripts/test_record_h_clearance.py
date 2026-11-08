from record_h_clearance import launch_command

import json
from pathlib import Path

import pytest


def test_parking_delays_fusion_without_extending_production_health_lease(tmp_path):
    command = launch_command([13.977, 1.08, 0.503, 0.0], tmp_path, roll=0.02499485)
    assert "nav_start_delay_sec:=60.0" in command
    assert "use_identified_odometry:=true" in command
    assert "slam:=true" in command
    assert f"map_yaml_file:={tmp_path}/absent.yaml" in command
    assert f"prior_pcd_file:={tmp_path}/absent.pcd" in command
    assert "start_roll:=0.02499485" in command
    assert not any("timeout" in value or "lease" in value for value in command)


def test_parking_motion_rejects_slope_slide_but_allows_vertical_settling():
    from record_h_clearance import check_parking_displacement
    assert check_parking_displacement([9.3, 2.76, 0.508], [9.3, 2.76, 0.48], 0.05) == 0
    assert check_parking_displacement([0, 0], [0.03, 0.04], 0.05) == pytest.approx(0.05)
    with pytest.raises(RuntimeError, match="parking displacement"):
        check_parking_displacement([9.3, 2.76], [9.3140709, 2.8082019], 0.05)


def test_recorded_ramp_slide_is_detected_before_sampling():
    from record_h_clearance import check_parking_displacement
    path = Path(__file__).resolve().parents[1] / 'log/nav_acceptance/20261109_H/ramp_bridge_d164/truth.json'
    if not path.exists():
        pytest.skip('local recorded experiment is not distributed with source')
    rows = json.loads(path.read_text())
    for row in rows:
        try:
            check_parking_displacement([9.3, 2.76], row['position'], 0.05)
        except RuntimeError:
            assert row['stamp'] == pytest.approx(1.34)
            break
    else:
        pytest.fail('recorded slope drift was not rejected')
