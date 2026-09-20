#!/usr/bin/env bash
# Sequential fresh-launch matrix. The Python driver uses only the standard library.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 - "${ROOT_DIR}" "$@" <<'PY'
import argparse
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time

SCENARIOS = (
    'straight', 'lateral', 'yaw', 'nearby', 'clearance', 'unknown',
    'occupied', 'unreachable', 'map-change', 'localization-recovery', 'estop-replan',
)


def scenario_spec(name):
    # These are existing single/rectangle/red_box waypoints, not new free-space claims.
    env = dict(TEST_PROFILE='single', P2_FAULT_CASE='none', P3_FAULT_CASE='none',
               START_X='-0.18', START_Y='0.06', START_Z='0.42', START_YAW='0.0',
               GOAL_X='1.0', GOAL_Y='0.06', GOAL_YAW='0.0', GOAL_TIMEOUT='60',
               GOAL_TOLERANCE='0.30', MIN_LEG_PROGRESS='0.20',
               GOAL_SET_VERIFY_FREE='1', PLANNING_GRID_OWNER='rog_map',
               ATS_PROFILE_SKIP_ACTION='0', USE_RVIZ='false', NAV_TRACKING_RECORDER='1',
               NAV_TRACKING_GATE='1',
               FORCE_BODY_YAW_FOLLOW='false', YAW_AUTHORITY_EXPECTED='auto')
    runner = 'test_mujoco_minco_mpc_chain.sh'
    expected = 'action success, position/progress/contact and ownership gates'
    limitation = None
    if name == 'lateral':
        env['TEST_PROFILE'] = 'rectangle'
        expected += '; south/north lateral command gates'
    elif name == 'yaw':
        env['GOAL_YAW'] = str(math.pi / 2)
        expected += '; nonzero requested goal yaw (not a pure-rotation test)'
    elif name == 'nearby':
        # Rectangle east waypoint -> stage waypoint: 0.40m, above 0.30m tolerance.
        env.update(START_X='0.90', GOAL_X='0.50')
    elif name == 'clearance':
        env.update(TEST_PROFILE='red_box', GOAL_TIMEOUT='180', GOAL_TOLERANCE='0.15')
        expected += '; existing red_box route, live known-free waypoint checks'
    elif name in ('unknown', 'unreachable'):
        env['P2_FAULT_CASE'] = name
        expected = 'nominal baseline then source-owned P2 safety assertions; runner exit 0'
    elif name in ('localization-recovery', 'estop-replan'):
        runner = 'test_mujoco_localization_fault.sh'
        env = dict(P4_FAULT_CASE='odometry_stale', P4_RELOCALIZATION_MODE='real')
        expected = 'odometry stale -> software estop/zero -> real GICP observation/replan -> action success'
        limitation = 'Real GICP recovery requested; software localization-induced stop, not physical E-stop coverage'
    elif name in ('occupied', 'map-change'):
        runner = 'test_mujoco_unsafe_trajectory.sh'
        env = dict(P4_UNSAFE_FAULT_CASE='occupied' if name == 'occupied' else 'map_after_commit')
        expected = ('live free goal -> source-owned occupied injection -> terminal non-success, '
                    'no executable reference, estop and fresh zero' if name == 'occupied' else
                    'committed moving reference -> source-owned occupied mutation -> runtime unsafe/zero '
                    '-> cancel/remove -> no old execution -> fresh goal execution and success')
    elif name != 'straight':
        raise ValueError(f'unknown scenario: {name}')
    return runner, env, expected, limitation


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Sequential isolated MuJoCo goal scenarios; no navigation tuning.',
        epilog='Every runnable scenario creates a fresh launch. Domains are unique within this invocation; '
               'reserve the chosen range externally. --plan writes planned/NOT_RUN manifests without ROS. '
               'Exit: 0 all passed, 1 failed or NOT_RUN, 2 invalid CLI. '
               'Artifacts: manifest.json, summary.json, per-scenario runner.log, tracking and fault JSON. '
               'Legacy temporary runner outputs are copied before the next scenario. '
               'Example: bash scripts/test_mujoco_goal_set.sh --scenarios straight,lateral,yaw,nearby '
               '--domain-start 180 --output /tmp/goal-set-unique')
    parser.add_argument('--scenarios', default=','.join(SCENARIOS), help=','.join(SCENARIOS))
    parser.add_argument('--domain-start', type=int, default=180)
    parser.add_argument('--output', type=Path, required=True, help='new artifact directory; existing path refused')
    parser.add_argument('--plan', action='store_true', help='only write commands; never report planned runs as pass')
    args = parser.parse_args(argv)
    args.names = args.scenarios.split(',')
    if not args.names or any(name not in SCENARIOS for name in args.names):
        parser.error('unknown or empty scenario name')
    if len(set(args.names)) != len(args.names):
        parser.error('duplicate scenarios are not allowed')
    if args.domain_start < 0 or args.domain_start + len(args.names) - 1 > 232:
        parser.error('entire domain range must be within 0..232')
    if args.output.exists():
        parser.error('output already exists; use a fresh directory')
    return args


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def run(root, args):
    output = args.output.resolve()
    output.mkdir(parents=True)
    manifest = []
    summary = []
    for offset, name in enumerate(args.names):
        domain = args.domain_start + offset
        directory = output / name
        directory.mkdir()
        runner, overrides, expected, limitation = scenario_spec(name)
        overrides.update(ROS_DOMAIN_ID=str(domain), ROS_LOG_DIR=str(directory / 'ros'),
                         NAV_TRACKING_DIR=str(directory / 'tracking'),
                         P4_RESULT_FILE=str(directory / 'fault.json'),
                         P4_LAUNCH_LOG=str(directory / 'launch.log'))
        overrides.update(P4_UNSAFE_RESULT_FILE=str(directory / 'fault.json'),
                         P4_UNSAFE_LAUNCH_LOG=str(directory / 'launch.log'))
        # Prevent inherited fault/profile switches from changing the named scenario.
        environment = dict(os.environ)
        removed = []
        for key in tuple(environment):
            if key.startswith(('P2_', 'P3_', 'P4_', 'GOAL_', 'START_', 'ATS_PROFILE_', 'QP_TELEMETRY_')):
                removed.append(key)
                del environment[key]
        environment.update(overrides)
        command = ['bash', str(root / 'scripts' / runner)] if runner else None
        row = dict(scenario=name, domain=domain, command=command, environment=environment,
                   unset_environment=removed,
                   shell_command=shlex.join(['env', *[part for key in sorted(removed) for part in ('-u', key)],
                                             *[f'{k}={v}' for k, v in sorted(overrides.items())],
                                             *(command or [])]) if command else None,
                   expected=expected, limitation=limitation, artifact=str(directory))
        # Only ROS/test environment is needed for reproduction; never dump unrelated credentials.
        row['environment'] = {k: v for k, v in environment.items() if k in overrides or
                              k.startswith(('ROS_', 'RMW_', 'CYCLONEDDS_', 'FASTDDS_'))}
        manifest.append(row)
        write_json(output / 'manifest.json', manifest)
        result = dict(scenario=name, domain=domain, status='NOT_RUN', exit_code=None,
                      action=None, safety=None, recorder=None, recorder_evidence=None, teardown=None,
                      reason=limitation, artifact=str(directory))
        if runner and not args.plan:
            started = time.time_ns()
            with (directory / 'runner.log').open('w') as log:
                try:
                    completed = subprocess.run(command, env=environment, cwd=root, stdout=log,
                                               stderr=subprocess.STDOUT, check=False)
                    result['exit_code'] = completed.returncode
                    result['status'] = 'passed' if completed.returncode == 0 else 'failed'
                except OSError as exc:
                    result.update(status='failed', reason=str(exc))
            # The legacy runner uses fixed /tmp filenames; retain fresh outputs before reuse.
            legacy = directory / 'legacy'
            legacy.mkdir()
            for pattern in ('ats_minco_mpc_*', 'ats_p2_*', 'ats_p3_*'):
                for path in Path('/tmp').glob(pattern):
                    if path.is_file() and path.stat().st_mtime_ns >= started:
                        shutil.copy2(path, legacy / path.name)
            status_file = Path('/tmp/ats_minco_mpc_test_logs/runner_status.env')
            if runner == 'test_mujoco_minco_mpc_chain.sh' and status_file.is_file() and status_file.stat().st_mtime_ns >= started:
                shutil.copy2(status_file, directory / 'runner_status.env')
                statuses = dict(token.split('=', 1) for token in shlex.split(status_file.read_text()) if '=' in token)
                for field, source in (('action', 'action_status'), ('safety', 'navigation_safety_status'),
                                      ('recorder', 'recorder_status'),
                                      ('recorder_evidence', 'recorder_evidence_status'),
                                      ('teardown', 'teardown_status')):
                    result[field] = statuses.get(source)
                result['runner_status'] = statuses
            result['reason'] = result['reason'] or 'See runner.log and independent artifacts; no inferred safety pass'
        elif runner:
            result['reason'] = 'plan only; simulation not executed'
        summary.append(result)
        write_json(output / 'summary.json', summary)
        print(f"{name}: {result['status']} exit={result['exit_code']} artifacts={directory}", flush=True)
    return 1 if any(row['status'] != 'passed' for row in summary) else 0


if __name__ == '__main__':
    root = Path(sys.argv[1])
    sys.exit(run(root, parse_args(sys.argv[2:])))
PY
