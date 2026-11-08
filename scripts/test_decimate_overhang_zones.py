import json

import pytest

from decimate_stl_by_clustering import zones_from_audit


def test_all_audited_clusters_get_fine_margin(tmp_path):
    path = tmp_path / 'audit.json'
    path.write_text(json.dumps({'purpose': 'offline_evaluation_only', 'clusters': [
        {'bounds_xy': [1., 2., 3., 4.], 'clearance_min_max_m': [.09, .1]},
        {'bounds_xy': [5., 6., 7., 8.], 'clearance_min_max_m': [.3, .4]}]}))
    assert zones_from_audit(path, .3) == [(0.7, 2.3, 2.7, 4.3), (4.7, 6.3, 6.7, 8.3)]
    with pytest.raises(ValueError):
        zones_from_audit(path, .29)
