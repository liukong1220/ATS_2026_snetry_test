import pytest

from summarize_h_headroom import load, summarise


def replay_csv(tmp_path, rows):
    path = tmp_path / 'headroom.csv'
    path.write_text('stamp,column_x,column_y,ground_z,state,ceiling_low,'
                    'support_lower,support_upper,headroom,support_estimate\n' + rows)
    return path


def test_missing_headroom_remains_in_window_and_cannot_pass(tmp_path):
    path = replay_csv(tmp_path,
        '1,2,3,0,free,0.25,-0.02,0.02,0.25,0\n'
        '2,2,3,,unknown,,,,,\n'
        '3,2,3,0,free,0.25,-0.02,0.02,0.25,0\n')
    report = summarise(load(path), 3, [0.249])
    column = report['per_column'][0]
    assert report['window_frames'] == 3
    assert column['missing_headroom_frames'] == 1
    assert column['state_transitions'] == 2
    assert report['summary']['gate_passes_production']['0.249'] == 0


def test_production_plane_headroom_has_its_own_fluctuation_statistic(tmp_path):
    path = replay_csv(tmp_path, ''.join(
        f'{i},2,3,0,free,0.27,-0.02,0.02,{h},{0.27-h}\n'
        for i, h in enumerate([0.250, 0.249, 0.254], 1)))
    report = summarise(load(path), 3, [0.249, 0.250])
    assert report['per_column'][0]['headroom_frame_ptp'] == pytest.approx(0.005)
    assert report['summary']['headroom_production_median'] == pytest.approx(0.250)
    assert report['summary']['gate_passes_production'] == {'0.249': 1, '0.250': 0}


def test_all_unknown_columns_are_reported_instead_of_disappearing(tmp_path):
    path = replay_csv(tmp_path, '1,2,3,,unknown,,,,,\n2,2,3,,unknown,,,,,\n')
    report = summarise(load(path), 2, [0.249])
    assert report['columns'] == 1
    assert report['per_column'][0]['frames'] == 0
    assert report['per_column'][0]['missing_headroom_frames'] == 2
    assert report['summary']['gate_passes_production']['0.249'] == 0


def test_nonfinite_headroom_cannot_count_as_measured_clearance(tmp_path):
    path = replay_csv(tmp_path, '1,2,3,0,free,0.25,0,0,nan,0\n')
    with pytest.raises(ValueError, match='non-finite'):
        load(path)
