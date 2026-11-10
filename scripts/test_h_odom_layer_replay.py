"""Compile and exercise the active implementation, not a Python classifier copy."""
from pathlib import Path
import subprocess

import pytest


@pytest.fixture(scope='module')
def replay(tmp_path_factory):
    root = Path(__file__).resolve().parents[1]
    package = root / 'src/ats_sentry_nav/ats_rog_map'
    output = tmp_path_factory.mktemp('h2_replay') / 'replay'
    subprocess.run(['g++', '-std=c++17', '-O3', '-Wall', '-Wextra', '-Werror',
                    '-I', str(package / 'include'), str(root / 'scripts/h_odom_layer_replay.cpp'),
                    *[str(package / f'src/{name}.cpp') for name in
                      ['ground_column', 'odom_layer', 'odom_layer_integration']],
                    '-o', str(output)], check=True)
    return output


def run_replay(replay, directory, points, sensor_stamp=1):
    source = directory / 'columns.txt'
    source.write_text('10 10 0.1 0 0 0.1 0.242 0.005 0.6 0 0 0.3 0.3\n'
                      + f'1 {len(points)} 0 0 0\n'
                      + ''.join(' '.join(map(str, p)) + '\n' for p in points))
    sensor = directory / 'origins.txt'
    sensor.write_text(f'{sensor_stamp} 0.01 0.01 0.4\n')
    return subprocess.run([str(replay), str(source), str(sensor), str(directory / 'timing.csv')],
                          text=True, capture_output=True)


@pytest.mark.parametrize('clearance,expected', [(0.239, 'occupied'), (0.257, 'free')])
def test_slope_headroom_reaches_global_layer(replay, tmp_path, clearance, expected):
    points = [(x, y, 0.3 * x) for x in [0.01, 0.11, 0.21] for y in [0.01, 0.11, 0.21]]
    points.append((0.19, 0.15, 0.3 * 0.19 + clearance))
    result = run_replay(replay, tmp_path, points)
    assert result.returncode == 0, result.stderr
    assert f'1,1,1,{expected},{expected}' in result.stdout.splitlines()


def test_missing_floor_and_mismatched_sensor_fail_closed(replay, tmp_path):
    result = run_replay(replay, tmp_path, [(0.15, 0.15, 0.25)])
    assert result.returncode == 0, result.stderr
    assert '1,1,1,unknown,unknown' in result.stdout.splitlines()
    result = run_replay(replay, tmp_path, [(0.15, 0.15, 0.25)], sensor_stamp=2)
    assert result.returncode != 0
    assert 'mismatched' in result.stderr
