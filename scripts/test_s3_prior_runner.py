"""Prior-map dispatch keeps the full route; reload keeps the first-goal contract."""
from contextlib import nullcontext
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import pytest
import s3_mujoco_acceptance as s3


@pytest.mark.parametrize('mode', ['prior', 'reload'])
def test_prior_and_reload_preserve_distinct_route_contracts(tmp_path, mode):
    prefix = tmp_path / 'map'
    prefix.with_suffix('.yaml').write_text('image: map.pgm\n')
    prefix.with_suffix('.pcd').write_text('PCD fixture; ROS launch is mocked\n')
    prefix.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    profile = s3.load_profile(s3.SIM / 'config/s3_mapping_routes.yaml',
                              'rmuc_2027_tunnels', 'six_tunnels')
    probe = Mock()
    probe.graph.return_value = {'/rog_map_adapter/planning_snapshot': {
        'publishers': ['ats_rog_map_adapter']}}
    probe.path, probe.goals = [profile['start'][:2]], [(0, True)]
    probe.endpoint_results, probe.planner_statuses, probe.executed_references = [], [], []
    probe.truth_samples, probe.truth_contact_counts = [], []
    probe.contact_seen, probe.counts = {}, {}
    process = Mock()
    process.poll.return_value = 0
    modules = {'rclpy': Mock(), 'ament_index_python.packages': NS(
        get_package_share_directory=lambda _: str(s3.SIM)),
        's3_tracking_evidence': NS(TrackingLeg=lambda *_: nullcontext())}
    args = NS(profile='rmuc_2027_tunnels', route='six_tunnels', output=str(tmp_path / 'run'),
              prior=str(prefix) if mode == 'prior' else None,
              reload=str(prefix) if mode == 'reload' else None,
              max_goals=0, case='route', post_route_case=None, startup_timeout=1., launch_arg=[])
    with patch.dict('sys.modules', modules), patch.object(s3, 'verify_installed_files'), \
            patch.object(s3, 'prepare_scene', return_value=([], Path('scene.xml'))), \
            patch.object(s3, 'Probe', return_value=probe), \
            patch.object(s3.subprocess, 'Popen', return_value=process) as launch, \
            patch.object(s3, 'route_metrics', return_value={'passed': True}), \
            patch.object(s3, 'required_tunnel_report', return_value={'fixture': {'passed': True}}):
        s3.run(args)
    # Geometry/navigation verdicts above are mocked: this tests entry selection only.
    expected = profile['waypoints'] if mode == 'prior' else profile['waypoints'][:1]
    assert [call.args[0] for call in probe.navigate.call_args_list] == expected
    assert [call.kwargs['final'] for call in probe.navigate.call_args_list] == [
        False] * (len(expected) - 1) + [True]
    command = launch.call_args.args[0]
    assert 'slam:=false' in command
    assert f'map_yaml_file:={prefix}.yaml' in command
    assert f'prior_pcd_file:={prefix}.pcd' in command
    assert probe.expected_grid_frame == 'map'
    probe.save_map.assert_not_called()
    evidence = json.loads((tmp_path / 'run/command.json').read_text())
    assert len(evidence['map_inputs']) == 3
    summary = json.loads((tmp_path / 'run/summary.json').read_text())
    assert summary[mode] == str(prefix)


def test_prior_missing_assets_stops_before_launch(tmp_path):
    args = NS(profile='rmuc_2027_tunnels', route='six_tunnels', output=str(tmp_path / 'run'),
              prior=str(tmp_path / 'missing'), reload=None)
    modules = {'rclpy': Mock(), 'ament_index_python.packages': NS(
        get_package_share_directory=lambda _: str(s3.SIM))}
    with patch.dict('sys.modules', modules), patch.object(s3, 'verify_installed_files'), \
            patch.object(s3.subprocess, 'Popen') as launch:
        with pytest.raises(FileNotFoundError):
            s3.run(args)
    launch.assert_not_called()
