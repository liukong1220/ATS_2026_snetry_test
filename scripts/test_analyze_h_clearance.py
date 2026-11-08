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
