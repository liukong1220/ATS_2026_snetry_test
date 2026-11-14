#!/usr/bin/env python3
"""Export the historical Nav2 baseline into a separate, new test workspace.

Preparation only: no build, launch, process signalling, Git mutation or changes
to the production install. This is not an ATS-action substitute for Nav2.
"""
import argparse
import hashlib
import io
import json
import math
from pathlib import Path, PurePosixPath
import shlex
import subprocess
import tarfile

import yaml


NAV_REVISION = '305b985213c655d72101d3d8281efda7fcec2d68'
SIM_REVISION = '71262e7b45e4659189cd35715397ec65a0549aad'
SCENE_FILES = ('maps/rmuc_2026.yaml', 'maps/rmuc_2026.pgm',
               'models/rmuc_2026_swerve.xml', 'models/rmuc_2026_height.png',
               'models/rmuc_2026_wall_boxes.xml', 'models/meshes/mid360.obj',
               'models/meshes/rmuc_2026.stl')
PACKAGES = ('ats_nav2_plugins', 'trajectory_optimizer')
BASELINE_FILES = (
    'ats_nav_bringup/config/simulation/nav2_params.yaml',
    'ats_nav_bringup/behavior_trees',
    'ats_nav_bringup/launch/navigation_launch.py',
)
ROS_RUNTIME_PACKAGES = (
    'nav2_core', 'nav2_msgs', 'nav2_controller', 'nav2_planner',
    'nav2_mppi_controller', 'nav2_smoother', 'nav2_behaviors',
    'nav2_bt_navigator', 'nav2_waypoint_follower', 'nav2_velocity_smoother',
    'nav2_lifecycle_manager', 'behaviortree_cpp_v3', 'sp_msgs',
)


def unpack_regular_archive(data, destination, allowed_roots):
    """Validate the entire archive before creating any exported file."""
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:') as archive:
        members = archive.getmembers()
        seen = set()
        for member in members:
            path = PurePosixPath(member.name)
            if (path.is_absolute() or '..' in path.parts or not path.parts or
                    path.parts[0] not in allowed_roots or
                    not (member.isdir() or member.isfile()) or member.name in seen):
                raise ValueError(f'unsupported archive entry: {member.name}')
            seen.add(member.name)
            target = destination.joinpath(*path.parts)
            if target.exists() or any(p.is_symlink() for p in target.parents):
                raise ValueError(f'export target already exists or traverses a link: {target}')
        hashes = {}
        for member in members:
            target = destination.joinpath(*PurePosixPath(member.name).parts)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            content = archive.extractfile(member).read()
            with target.open('xb') as stream:
                stream.write(content)
            target.chmod(0o755 if member.mode & 0o111 else 0o644)
            hashes[str(target.relative_to(destination))] = hashlib.sha256(content).hexdigest()
        return hashes


def git_bytes(repository, *args):
    return subprocess.check_output(['git', '-C', str(repository), *args])


def runtime_inventory():
    try:
        from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    except ImportError:
        return {'checked': False, 'reason': 'ROS environment is not sourced'}
    present, missing = {}, []
    for package in ROS_RUNTIME_PACKAGES:
        try:
            present[package] = get_package_prefix(package)
        except PackageNotFoundError:
            missing.append(package)
    return {'checked': True, 'present': present, 'missing': missing,
            'scope': 'package index only; compilation/plugin ABI and runtime unverified'}


def tighten_goal_checker(document, tolerance):
    """Tighten only a run-local historical Nav2 copy; never relax acceptance."""
    parameters = document['controller_server']['ros__parameters']
    names = parameters['goal_checker_plugins']
    if len(names) != 1:
        raise ValueError('strict compatibility run requires one explicit goal checker')
    checker = parameters[names[0]]
    original = float(checker['xy_goal_tolerance'])
    if (not math.isfinite(tolerance) or tolerance <= 0 or
            tolerance > min(original, 0.08)):
        raise ValueError('goal checker tolerance must be positive and no larger than 0.08 or original')
    checker['xy_goal_tolerance'] = tolerance
    return {'plugin': names[0], 'original_xy_m': original, 'effective_xy_m': tolerance,
            'scope': 'explicit stricter run variant; historical source and stage truth gate unchanged'}


def prepare_scene(workspace, destination):
    """Restore the byte-identical historical field, never move its test goals."""
    workspace, destination = workspace.resolve(), destination.resolve()
    if destination.exists() or destination == workspace or workspace / 'src' in destination.parents:
        raise ValueError('historical scene needs a new destination outside production src')
    repository = workspace / 'src/sim/ats_mujoco_sim'
    data = git_bytes(repository, 'archive', '--format=tar', SIM_REVISION, '--', *SCENE_FILES)
    hashes = unpack_regular_archive(data, destination, {'maps', 'models'})
    manifest = {'revision': SIM_REVISION, 'files': hashes,
                'archive_sha256': hashlib.sha256(data).hexdigest(),
                'scope': 'original centered RMUC2026 truth/Nav2 baseline, not current 2025/2027 acceptance'}
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def validate_map_points(map_yaml, points):
    """Reject a historical start/goal outside the selected map or in nonfree cells."""
    from PIL import Image
    metadata = yaml.safe_load(map_yaml.read_text())
    raster = Image.open(map_yaml.parent / metadata['image']).convert('L')
    ox, oy, yaw = metadata['origin']
    resolution = float(metadata['resolution'])
    if resolution <= 0:
        raise ValueError('map resolution must be positive')
    result = []
    for x, y in points:
        dx, dy = x - ox, y - oy
        ix = math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / resolution)
        iy = math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / resolution)
        if not (0 <= ix < raster.width and 0 <= iy < raster.height):
            raise ValueError(f'baseline point {(x, y)} is outside selected map')
        shade = raster.getpixel((ix, raster.height - 1 - iy)) / 255.0
        probability = shade if metadata.get('negate', 0) else 1.0 - shade
        if probability >= metadata['free_thresh']:
            raise ValueError(f'baseline point {(x, y)} is not known-free in selected map')
        result.append({'xy': [x, y], 'cell': [ix, iy], 'occupied_probability': probability})
    return result


def prepare(workspace, output):
    workspace, output = workspace.resolve(), output.resolve()
    # Refuse the production tree and every existing destination, including empty
    # directories. A failed preparation is kept for inspection, never replaced.
    if output.exists() or output == workspace or workspace / 'src' in output.parents:
        raise ValueError('choose a new destination outside the production src tree')
    nav = workspace / 'src/ats_sentry_nav'
    revision = git_bytes(nav, 'rev-parse', NAV_REVISION + '^{commit}').decode().strip()
    package_archive = git_bytes(nav, 'archive', '--format=tar', revision, '--', *PACKAGES)
    baseline_archive = git_bytes(nav, 'archive', '--format=tar', revision, '--', *BASELINE_FILES)
    output.mkdir(parents=True)
    package_hashes = unpack_regular_archive(package_archive, output / 'src', set(PACKAGES))
    baseline_hashes = unpack_regular_archive(
        baseline_archive, output / 'historical', {'ats_nav_bringup'})
    scene = prepare_scene(workspace, output / 'historical_scene')
    command = '\n'.join([
        '#!/usr/bin/env bash', 'set -e', 'set +u',
        'source /opt/ros/humble/setup.bash',
        'source ' + shlex.quote(str(workspace / 'install/setup.bash')),
        'source ' + shlex.quote(str(workspace / 'log/nav_acceptance/dds/wlan_env.sh')),
        ': "${ROS_DOMAIN_ID:?allocate and record a fresh domain before building/testing}"',
        'cd ' + shlex.quote(str(output)),
        'export ROS_LOG_DIR="$PWD/ros_log"',
        'JOBS=$(awk -v n="$(nproc)" \'/^MemAvailable:/ '
        '{j=int($2/1024/1536); if (j>n) j=n; if (j<1) j=1; print j}\' /proc/meminfo)',
        'export MAKEFLAGS=-j${JOBS}',
        'flock /tmp/ats27_build.lock colcon build --base-paths src '
        '--packages-select ats_nav2_plugins trajectory_optimizer --parallel-workers 1 '
        '--cmake-args -DCMAKE_BUILD_TYPE=Release',
        '',
    ])
    (output / 'build-compat.sh').write_text(command)
    (output / 'build-compat.sh').chmod(0o755)
    manifest = {
        'status': 'prepared; build and isolated launch not yet run',
        'source_repository': str(nav), 'source_revision': revision,
        'packages': package_hashes, 'historical_baseline': baseline_hashes,
        'archive_sha256': {'packages': hashlib.sha256(package_archive).hexdigest(),
                           'baseline': hashlib.sha256(baseline_archive).hexdigest()},
        'runtime_inventory': runtime_inventory(),
        'historical_scene': scene,
        'integration_gaps': [
            'historical launch duplicates LIO/terrain/TF if combined with production navigation',
            'use current MuJoCo with launch_navigation=false and one current LIO/fusion chain',
            'restore legacy RC-ESDF/elastic-path owners only in this isolated test domain',
            'legacy map/start coordinates must match the selected field assets',
            'keep NavigateToPose result, nonempty elastic path, unique velocity ownership and truth gates',
        ],
    }
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = prepare(args.workspace, args.output)
    print(json.dumps({'output': str(args.output.resolve()), 'revision': manifest['source_revision'],
                      'files': len(manifest['packages']) + len(manifest['historical_baseline']),
                      'status': manifest['status'],
                      'runtime_inventory': manifest['runtime_inventory']}, indent=2))


if __name__ == '__main__':
    main()
