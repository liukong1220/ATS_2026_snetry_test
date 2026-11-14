import gzip
import json
import pytest
from export_terrain_confirmation_frames import frames


def fixture(tmp_path, received):
    payload = {'info': {'width': 2, 'height': 1}, 'data': [0, 100],
               'stamp_identity': '12.123456789'}
    with gzip.open(tmp_path / 'grid.gz', 'wt') as stream:
        json.dump(payload, stream)
    (tmp_path / 'events.jsonl').write_text(json.dumps({'kind': 'layer_payload', 'payload': {
        'topic': '/traversability_grid', 'path': 'grid.gz'}}) + '\n')
    (tmp_path / 'summary.json').write_text(json.dumps({'layer_payloads_truncated': False,
        'messages_received': {'/traversability_grid': received}}))


def test_exact_nanosecond_identity_and_complete_frame_count(tmp_path):
    fixture(tmp_path, 1)
    assert frames(tmp_path)[0]['stamp_ns'] == 12123456789


def test_content_deduplication_is_not_a_complete_sequence(tmp_path):
    fixture(tmp_path, 2)
    with pytest.raises(ValueError, match='deduplicated'):
        frames(tmp_path)
