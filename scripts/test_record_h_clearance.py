from record_h_clearance import launch_command

import json
from pathlib import Path

import pytest
from concurrent.futures import Future
from types import SimpleNamespace


def test_parking_sampling_requires_successful_service_acknowledgement():
    from record_h_clearance import parking_acknowledged
    future = Future()
    assert not parking_acknowledged(None)
    assert not parking_acknowledged(future)
    future.set_result(SimpleNamespace(cmd_ack=0))
    assert parking_acknowledged(future)
    rejected = Future()
    rejected.set_result(SimpleNamespace(cmd_ack=1))
    with pytest.raises(RuntimeError, match='rejected'):
        parking_acknowledged(rejected)


def test_fixed_body_may_observe_startup_lost_but_never_a_post_tracking_loss():
    from record_h_clearance import localization_abort_required
    lost = SimpleNamespace(state=4, STATE_LOST=4, odom_identity_latched=False)
    assert localization_abort_required(lost, True, False, False)
    assert not localization_abort_required(lost, True, False, True)
    assert localization_abort_required(lost, True, True, True)
    lost.odom_identity_latched = True
    assert localization_abort_required(lost, False, False, True)


def test_settling_needs_park_and_first_tracking_but_not_uninterrupted_tracking():
    from record_h_clearance import settle_origin
    assert settle_origin(None, 2.0, False, True, True) is None
    assert settle_origin(None, 2.0, True, False, True) is None
    origin = settle_origin(None, 3.0, True, True, True)
    # CONFIRMED holds and brief DEGRADED intervals leave tracking_seen latched.
    # Those clouds are skipped by on_cloud; they do not restart physical settling.
    for stamp in (4.0, 4.025, 10.0, 18.0):
        origin = settle_origin(origin, stamp, True, True, True)
    assert 18.0 - origin == 15.0
    assert settle_origin(origin, 1.0, True, True, True) == 1.0
    assert settle_origin(None, 2.0, True, False, False) == 2.0


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
