"""Deterministic layer/fit tests, independent of the large official asset."""

import numpy as np
import pytest

from audit_rmuc_2027_overhangs import cluster_layers, fit_plane, lowest_air_layers, probe_surfaces
from generate_rmuc_2027_map import Rules, Z_LEVELS, Z_PER_METRE


def keys_for(surfaces):
    return np.array(sorted((col * Z_LEVELS + round(z * Z_PER_METRE)) * 2 + up
                           for col, z, up in surfaces), dtype=np.int64)


def test_lowest_air_layer_preserves_low_lip_and_ignores_buried_faces():
    # Two overlapping roof solids: the buried 0.30/0.35 faces are not the top.
    keys = keys_for([(0, 0., 0), (0, .129, 1), (0, .220, 0),
                     (0, .300, 0), (0, .350, 1), (0, .500, 1)])
    layers, diagnostics = lowest_air_layers(keys, Rules(merge_gap=.001, interface_gap=.001))
    assert layers == pytest.approx(np.array([[0, .129, .220, .500]]))
    assert diagnostics["unbalanced_columns"] == 0


def test_selects_lowest_bounded_air_not_upper_cavity():
    keys = keys_for([(0, 0., 0), (0, .1, 1), (0, .2, 0), (0, .3, 1),
                     (0, .4, 0), (0, .5, 1)])
    layers, _ = lowest_air_layers(keys, Rules())
    assert layers == pytest.approx(np.array([[0, .1, .2, .3]]))


def test_unclosed_roof_is_reported_not_generated():
    keys = keys_for([(0, 0., 0), (0, .1, 1), (0, .4, 0)])
    layers, diagnostics = lowest_air_layers(keys, Rules())
    assert layers.shape == (0, 4)
    assert diagnostics["unbalanced_columns"] == 1


def test_empty_rays():
    layers, _ = lowest_air_layers(np.empty(0, dtype=np.int64), Rules())
    assert layers.shape == (0, 4)


def test_diagonal_cells_are_eight_connected_but_remote_cell_is_separate():
    rules = Rules(width_cells=4, height_cells=4, supersample=1)
    records = np.array([[0, .1, .4, .45], [5, .1, .4, .45], [15, .1, .4, .45]])
    clusters, labels = cluster_layers(records, rules)
    assert len(clusters) == 2
    assert labels[0] == labels[1] != labels[2]


def test_plane_fit_includes_both_slopes_and_reports_nonplanarity():
    x, y = np.meshgrid(np.linspace(1, 2, 4), np.linspace(3, 4, 4))
    x, y = x.ravel(), y.ravel()
    z = .2 + .03 * x - .04 * y
    fit = fit_plane(x, y, z)
    assert fit["a"] == pytest.approx(.2)
    assert fit["slope_x"] == pytest.approx(.03)
    assert fit["slope_y"] == pytest.approx(-.04)
    assert fit["max_abs_error_m"] < 1e-12
    z[5] += .02
    assert fit_plane(x, y, z)["max_abs_error_m"] > .003


def test_exact_probe_is_independent_of_grid_and_handles_face_orientation():
    triangle = np.array([[0., 0., .2], [1., 0., .23], [0., 1., .16]])
    result = probe_surfaces([np.array([triangle, triangle[::-1]])], [(0.2, 0.3), (2., 2.)])
    surfaces = result[0]["surfaces_z_facing_up"]
    assert [s[0] for s in surfaces] == pytest.approx([.194, .194])
    assert {s[1] for s in surfaces} == {False, True}
    assert result[1]["surfaces_z_facing_up"] == []
