import math
import numpy as np
from s3_map_quality import cell_centres
from summarize_saved_overhangs import summarize


def test_unknown_and_occupied_are_separate_and_neither_is_free():
    meta = {'resolution': .1, 'origin': [0, 0, 0]}
    centres = cell_centres(3, 1, .1, meta['origin'])
    roof = [{'name': 'roof', 'polygon': [[0, 0], [.3, 0], [.3, .1], [0, .1]]}]
    got = summarize(meta, [0, -1, 100], centres, roof)['roof']
    assert (got['free_cells'], got['unknown_cells'], got['occupied_cells']) == (1, 1, 1)
    assert not got['all_interiors_known_free']


def test_yawed_translated_map_counts_whole_cell_footprints():
    meta = {'resolution': .1, 'origin': [2, 3, math.pi / 2]}
    centres = cell_centres(3, 1, .1, meta['origin'])
    roof = [{'name': 'roof', 'polygon': [[1.9, 3], [2, 3], [2, 3.25], [1.9, 3.25]]}]
    got = summarize(meta, [0, 0, 100], centres, roof)['roof']
    assert got['full_interior_cells'] == 2
    assert got['boundary_cells_excluded'] == 1
    assert got['occupied_cells'] == 0


def test_corridor_excludes_supporting_side_walls_and_empty_roi_is_unverified():
    meta = {'resolution': .1, 'origin': [0, 0, 0]}
    centres = cell_centres(3, 1, .1, meta['origin'])
    roof = {'name': 'roof', 'polygon': [[0, 0], [.3, 0], [.3, .1], [0, .1]],
            'corridor_polygon': [[.1, 0], [.2, 0], [.2, .1], [.1, .1]]}
    got = summarize(meta, [100, 0, 100], centres, [roof])['roof']
    assert got['full_interior_cells'] == 1 and got['all_interiors_known_free']
    roof['corridor_polygon'] = [[.14, 0], [.16, 0], [.16, .1], [.14, .1]]
    got = summarize(meta, [100, 0, 100], centres, [roof])['roof']
    assert got['full_interior_cells'] == 0 and not got['all_interiors_known_free']
