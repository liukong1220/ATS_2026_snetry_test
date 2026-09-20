#!/usr/bin/env python3
"""Offline goal serialization and matrix dispatch contracts; no ROS required."""
import contextlib
import io
import math
from pathlib import Path
import subprocess
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'scripts/test_mujoco_goal_set.sh').read_text()
DRIVER = SOURCE.split("<<'PY'\n", 1)[1].rsplit('\nPY', 1)[0]
MODULE = types.ModuleType('goal_set_driver')
exec(compile(DRIVER, str(ROOT / 'scripts/test_mujoco_goal_set.sh'), 'exec'), MODULE.__dict__)


class GoalQuaternionTest(unittest.TestCase):
    def quaternion(self, yaw):
        runner = (ROOT / 'scripts/test_mujoco_minco_mpc_chain.sh').read_text()
        function = 'goal_quaternion() {' + runner.split('goal_quaternion() {', 1)[1].split('\n}', 1)[0] + '\n}'
        return subprocess.run(['bash', '-c', function + '\ngoal_quaternion "$1"', 'test', yaw],
                              capture_output=True, text=True, check=False)

    def test_default_and_nonzero_yaw_are_unit_quaternions(self):
        for yaw in (0, math.pi / 2, -math.pi, 7.0):
            with self.subTest(yaw=yaw):
                result = self.quaternion(str(yaw))
                self.assertEqual(result.returncode, 0, result.stderr)
                z, w = map(float, result.stdout.split())
                self.assertAlmostEqual(z, math.sin(yaw / 2))
                self.assertAlmostEqual(w, math.cos(yaw / 2))
                self.assertAlmostEqual(z * z + w * w, 1.0)

    def test_nonfinite_and_malformed_yaw_rejected(self):
        for yaw in ('nan', 'inf', '-inf', '1e9999', 'garbage', ''):
            with self.subTest(yaw=yaw):
                self.assertNotEqual(self.quaternion(yaw).returncode, 0)


class GoalSetTest(unittest.TestCase):
    def test_existing_routes_and_fault_owners(self):
        self.assertEqual(MODULE.scenario_spec('lateral')[1]['TEST_PROFILE'], 'rectangle')
        self.assertEqual(MODULE.scenario_spec('clearance')[1]['TEST_PROFILE'], 'red_box')
        nearby = MODULE.scenario_spec('nearby')[1]
        self.assertGreater(abs(float(nearby['START_X']) - float(nearby['GOAL_X'])),
                           float(nearby['GOAL_TOLERANCE']))
        for name in ('unknown', 'unreachable'):
            self.assertEqual(MODULE.scenario_spec(name)[1]['P2_FAULT_CASE'], name)
        for name, fault in (('occupied', 'occupied'), ('map-change', 'map_after_commit')):
            self.assertEqual(MODULE.scenario_spec(name)[0], 'test_mujoco_unsafe_trajectory.sh')
            self.assertEqual(MODULE.scenario_spec(name)[1]['P4_UNSAFE_FAULT_CASE'], fault)
        with self.assertRaises(ValueError):
            MODULE.scenario_spec('typo')

    def test_recovery_routes_require_real_gicp_without_claiming_hardware_estop(self):
        for name in ('localization-recovery', 'estop-replan'):
            runner, environment, expected, limitation = MODULE.scenario_spec(name)
            self.assertEqual(runner, 'test_mujoco_localization_fault.sh')
            self.assertEqual(environment['P4_RELOCALIZATION_MODE'], 'real')
            self.assertEqual(environment['P4_FAULT_CASE'], 'odometry_stale')
            self.assertIn('real GICP', expected)
            self.assertIn('not physical E-stop', limitation)

    def test_cli_rejects_invalid_domain_duplicates_and_names(self):
        with tempfile.TemporaryDirectory() as tmp:
            for extra in (['--domain-start', '-1'], ['--domain-start', '233'],
                          ['--domain-start', '232', '--scenarios', 'straight,yaw'],
                          ['--scenarios', 'straight,straight'], ['--scenarios', 'missing'],
                          ['--scenarios', '']):
                with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as caught:
                        MODULE.parse_args(['--output', str(Path(tmp) / 'new'), *extra])
                    self.assertEqual(caught.exception.code, 2)

    def test_sequential_unique_domains_failure_aggregation(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = MODULE.parse_args(['--output', str(Path(tmp) / 'matrix'), '--domain-start', '230',
                                      '--scenarios', 'straight,yaw,occupied'])
            with patch.object(MODULE.subprocess, 'run', side_effect=[
                    subprocess.CompletedProcess([], 9), subprocess.CompletedProcess([], 0),
                    subprocess.CompletedProcess([], 0)]) as run:
                with patch.object(MODULE.Path, 'glob', return_value=[]):
                    self.assertEqual(MODULE.run(ROOT, args), 1)
            self.assertEqual(run.call_count, 3)
            self.assertEqual([call.kwargs['env']['ROS_DOMAIN_ID'] for call in run.call_args_list],
                             ['230', '231', '232'])
            rows = MODULE.json.loads((args.output / 'summary.json').read_text())
            self.assertEqual([row['status'] for row in rows], ['failed', 'passed', 'passed'])
            self.assertEqual(rows[0]['exit_code'], 9)
            self.assertEqual(rows[2]['exit_code'], 0)
            self.assertIsNone(rows[1]['safety'])

    def test_plan_never_launches_or_claims_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = MODULE.parse_args(['--output', str(Path(tmp) / 'plan'), '--plan',
                                      '--scenarios', 'straight'])
            with patch.object(MODULE.subprocess, 'run') as run:
                self.assertEqual(MODULE.run(ROOT, args), 1)
                run.assert_not_called()
            rows = MODULE.json.loads((args.output / 'summary.json').read_text())
            self.assertEqual(rows[0]['status'], 'NOT_RUN')


if __name__ == '__main__':
    unittest.main()
