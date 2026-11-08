import numpy as np
import pytest

from analyze_h_clearance import candidates, localization_seed


def test_connected_floor_and_ceiling_separate_from_low_hanging_obstacle():
    xyz = np.array([[0.15, 0.15, z] for z in [0.0, 0.001, -0.001, 0.25, 0.251, 0.249]])
    result = candidates(xyz)
    assert len(result) == 1
    assert abs(result[0]["clearance_min_m"] - 0.249) < 1e-9
    xyz[-3:, 2] -= 0.01
    assert abs(candidates(xyz)[0]["clearance_min_m"] - 0.239) < 1e-9


def test_floating_plate_cannot_invent_connected_ground():
    xyz = np.array([[0.15, 0.15, z] for z in [0.24, 0.241, 0.239, 0.49, 0.491, 0.489]])
    assert candidates(xyz) == []


def test_single_ceiling_speckle_not_a_measured_surface():
    xyz = np.array([[0.15, 0.15, z] for z in [0.0, 0.001, -0.001, 0.25]])
    assert candidates(xyz) == []


def test_occluded_ground_plane_is_explicit_diagnostic_not_observed_ground():
    xyz = []
    for x in range(-5, 6):
        for y in range(-5, 6):
            if abs(x) <= 2 and abs(y) <= 2:
                continue
            for dz in (-0.001, 0.0, 0.001):
                xyz.append([(x + 0.5) * 0.1, (y + 0.5) * 0.1, dz])
    xyz.extend([[0.05, 0.05, z] for z in (0.249, 0.25, 0.251)])
    assert candidates(xyz) == []
    result = candidates(xyz, ground_fit_radius=0.65)
    assert len(result) == 1
    assert result[0]["ground_source"] == "supported_plane_diagnostic"
    assert result[0]["floor_points"] == 0
    assert abs(result[0]["clearance_min_m"] - 0.249) < 1e-9


def test_scan_time_seed_preserves_odom_and_subtracts_body_height():
    pose = {"header": {"stamp": {"sec": 15, "nanosec": 300000000}, "frame_id": "odom"},
            "child_frame_id": "gimbal_yaw_odom",
            "pose": {"pose": {"position": {"x": 0.09, "y": 0.015, "z": 0.2}}}}
    seed, delta = localization_seed([pose], 15.35, 0.18)
    assert np.allclose(seed, [0.09, 0.015, 0.02])
    assert delta == pytest.approx(-0.05)
    with pytest.raises(ValueError, match="time tolerance"):
        localization_seed([pose], 16.0, 0.18)
    pose["header"]["frame_id"] = "map"
    with pytest.raises(ValueError, match="frames"):
        localization_seed([pose], 15.35, 0.18)
    with pytest.raises(ValueError, match="recorded poses"):
        localization_seed([], 15.35, 0.18)


def test_pose_seed_reaches_observed_floor_without_expanding_seed_radius():
    xyz = np.array([[0.65, 0.05, z] for z in [-0.001, 0, 0.001, 0.249, 0.25, 0.251]])
    assert candidates(xyz) == []
    result = candidates(xyz, seed=(0.09, 0.015, 0.02))
    assert len(result) == 1
    assert result[0]["ground_source"] == "observed"
    assert result[0]["clearance_min_m"] == pytest.approx(0.249)


@pytest.mark.parametrize('clearance', [0.25, 0.24])
def test_observed_plane_diagnostic_compares_floor_at_actual_ceiling_xy(clearance):
    xyz = []
    for x in range(-2, 3):
        for y in range(-2, 3):
            for dx, dy in [(0.017, 0.017), (0.023, 0.017), (0.017, 0.023)]:
                px, py = x * 0.1 + dx, y * 0.1 + dy
                xyz.append([px, py, 0.3 * px + 0.4 * py])
    for dx, dy in [(0.077, 0.077), (0.083, 0.077), (0.077, 0.083)]:
        px, py = 0.1 + dx, 0.1 + dy
        xyz.append([px, py, 0.3 * px + 0.4 * py + clearance])
    baseline = candidates(xyz)
    assert baseline[0]['clearance_min_m'] > clearance + 0.03
    rows = candidates(xyz, detrend_observed_floor=True)
    assert len(rows) == 1
    assert rows[0]['clearance_min_m'] == pytest.approx(clearance)
    assert rows[0]['ground_source'] == 'observed_local_plane_diagnostic'
    assert rows[0]['floor_points'] == 3


def test_observed_plane_diagnostic_does_not_bridge_unseen_ground():
    xyz = [[0.05, 0.05, z] for z in [0.249, 0.25, 0.251]]
    assert candidates(xyz, detrend_observed_floor=True) == []


def test_observed_plane_diagnostic_refuses_rank_deficient_support():
    xyz = [[0.15, 0.15, z] for z in [-0.001, 0.0, 0.001, 0.239, 0.24, 0.241]]
    assert candidates(xyz, detrend_observed_floor=True) == []


def test_observed_plane_diagnostic_refuses_step_instead_of_smoothing_it():
    xyz = []
    for x in range(-1, 2):
        for y in range(-1, 2):
            for dx, dy in [(0.02, 0.02), (0.03, 0.02), (0.02, 0.03)]:
                xyz.append([x * 0.1 + dx, y * 0.1 + dy, 0.03 if x > 0 else 0.0])
    xyz.extend([[0.05, 0.05, z] for z in [0.249, 0.25, 0.251]])
    assert candidates(xyz)
    assert candidates(xyz, detrend_observed_floor=True) == []


def test_observed_plane_diagnostic_keeps_single_low_hanging_return():
    xyz = []
    for x in range(-1, 2):
        for y in range(-1, 2):
            for dx, dy in [(0.02, 0.02), (0.03, 0.02), (0.02, 0.03)]:
                xyz.append([x * 0.1 + dx, y * 0.1 + dy, 0.0])
    xyz.extend([[0.05, 0.05, z] for z in [0.239, 0.25, 0.251]])
    rows = candidates(xyz, detrend_observed_floor=True)
    assert len(rows) == 1
    assert rows[0]['clearance_min_m'] == pytest.approx(0.239)
