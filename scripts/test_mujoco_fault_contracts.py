#!/usr/bin/env python3
"""Focused offline fault-harness contracts; source install/setup.bash before running."""
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import evaluate_mujoco_unsafe_trajectory as unsafe
import set_mujoco_lidar_occlusion as lidar

E = unsafe.UnsafeTrajectoryEvaluator
P = unsafe.PlannerStatus
C = unsafe.ExecutionCommand
L = lidar.LidarOcclusionControl
ROOT = Path(__file__).resolve().parents[1]


def stamp(value):
    return NS(sec=value, nanosec=0)


def command(**changes):
    fields = dict(mode=C.MODE_EXECUTE, goal_id=7, localization_epoch=3,
                  map_generation=10, map_publication_sequence=20,
                  manager_incarnation=9, command_sequence=40,
                  header=NS(stamp=stamp(12)),
                  reference=NS(header=NS(stamp=stamp(11)), poses=[1, 2]))
    fields.update(changes)
    return NS(**fields)


def status(**changes):
    fields = dict(state=P.STATE_REFERENCE_READY, failure_reason=P.FAILURE_NONE,
                  goal_id=7, localization_epoch=3, map_generation=10,
                  map_publication_sequence=20, plan_request_sequence=4,
                  reference_stamp=stamp(11))
    fields.update(changes)
    return NS(**fields)


class MapFaultContractTest(unittest.TestCase):
    def setUp(self):
        self.old = command()
        self.committed = status()
        self.invalidated = status(state=P.STATE_FAILED,
                                  failure_reason=P.FAILURE_SNAPSHOT_CHANGED,
                                  map_generation=11)
        self.node = NS(stamp_ns=E.stamp_ns, planner_statuses=[self.committed, self.invalidated],
                       execution_commands=[], map_snapshots=[])

    def test_commit_pair_requires_full_map_and_reference_identity(self):
        self.assertIs(E.committed_status(self.node, self.old), self.committed)
        for field, value in [('goal_id', 8), ('localization_epoch', 4), ('map_generation', 11),
                             ('map_publication_sequence', 21), ('reference_stamp', stamp(12))]:
            with self.subTest(field=field):
                saved = getattr(self.committed, field)
                setattr(self.committed, field, value)
                self.assertIsNone(E.committed_status(self.node, self.old))
                setattr(self.committed, field, saved)

    def test_snapshot_change_is_valid_only_for_invalidated_committed_request(self):
        self.assertIs(E.map_invalidation(self.node, self.old, self.committed, 1), self.invalidated)
        self.assertIsNone(E.map_invalidation(self.node, self.old, self.committed, 2))
        for field, value in [('goal_id', 8), ('localization_epoch', 4),
                             ('plan_request_sequence', 5), ('map_generation', 10),
                             ('map_publication_sequence', 21), ('reference_stamp', stamp(12)),
                             ('failure_reason', P.FAILURE_START_OR_GOAL_OCCUPIED)]:
            with self.subTest(field=field):
                saved = getattr(self.invalidated, field)
                setattr(self.invalidated, field, value)
                self.assertIsNone(E.map_invalidation(self.node, self.old, self.committed, 1))
                setattr(self.invalidated, field, saved)
        self.invalidated.failure_reason = P.FAILURE_RUNTIME_UNSAFE
        self.invalidated.reference_stamp = stamp(0)
        self.assertIs(E.map_invalidation(self.node, self.old, self.committed, 1), self.invalidated)

    def test_occupied_rejection_requires_exact_new_snapshot_not_latest_map(self):
        rejected = status(state=P.STATE_FAILED, failure_reason=P.FAILURE_START_OR_GOAL_OCCUPIED,
                          map_generation=11, map_publication_sequence=21, plan_request_sequence=5)
        self.node.planner_statuses.append(rejected)
        snapshot = dict(ready=True, occupied=True, epoch=3, publication=21)
        self.node.map_snapshots = [snapshot]
        check = lambda: E.occupied_rejection(self.node, self.old, self.committed,
                                             self.invalidated, 1, 0)
        self.assertIs(check(), rejected)
        for field, value in [('ready', False), ('occupied', False), ('epoch', 4), ('publication', 22)]:
            with self.subTest(field=field):
                saved = snapshot[field]
                snapshot[field] = value
                self.assertIsNone(check())
                snapshot[field] = saved
        rejected.plan_request_sequence = 4
        self.assertIsNone(check())

    def test_renewed_sequence_cannot_revive_old_reference_or_map(self):
        fresh = command(command_sequence=50, map_generation=11, map_publication_sequence=21,
                        reference=NS(header=NS(stamp=stamp(13)), poses=[1, 2]))
        self.node.execution_commands = [self.old, command(mode=C.MODE_STOP), fresh]
        E.assert_no_old_execution(self.node, self.old, 1)
        for field, value in [('command_sequence', 40), ('map_generation', 10),
                             ('map_publication_sequence', 20),
                             ('reference', self.old.reference), ('manager_incarnation', 10)]:
            with self.subTest(field=field):
                saved = getattr(fresh, field)
                setattr(fresh, field, value)
                with self.assertRaisesRegex(RuntimeError, 'authorization revived'):
                    E.assert_no_old_execution(self.node, self.old, 1)
                setattr(fresh, field, saved)

    def test_stop_uses_actual_zero_epoch_contract_and_rejects_stale_identity(self):
        stopped = command(mode=C.MODE_STOP, command_sequence=41, localization_epoch=0,
                          map_generation=0, map_publication_sequence=0,
                          header=NS(stamp=stamp(13)))
        self.node.execution_commands = [stopped]
        self.assertIs(E.fresh_map_stop(self.node, self.old, 0), stopped)
        self.assertIsNone(E.fresh_map_stop(self.node, self.old, 1))
        for field, value in [('goal_id', 8), ('manager_incarnation', 10),
                             ('command_sequence', 40), ('header', NS(stamp=stamp(12))),
                             ('mode', C.MODE_EXECUTE)]:
            with self.subTest(field=field):
                saved = getattr(stopped, field)
                setattr(stopped, field, value)
                self.assertIsNone(E.fresh_map_stop(self.node, self.old, 0))
                setattr(stopped, field, saved)

    def test_cached_or_pre_stop_zero_does_not_prove_fresh_stop(self):
        node = NS(latest_cmd=(0., 0., 0.), latest_cmd_at=10., command_norm=E.command_norm)
        with patch.object(unsafe.time, 'monotonic', return_value=10.2):
            self.assertFalse(E.commands_are_zero(node, 10.1))
            self.assertTrue(E.commands_are_zero(node, 9.9))
        with patch.object(unsafe.time, 'monotonic', return_value=10.6):
            self.assertFalse(E.commands_are_zero(node, 9.9))


class LidarSourceContractTest(unittest.TestCase):
    def test_source_effect_needs_two_distinct_fresh_valid_scans(self):
        node = NS(samples=[dict(stamp=10, points=0, valid=True),
                           dict(stamp=11, points=0, valid=True)])
        self.assertFalse(L.fresh_effect(node, True, 0, 10))
        node.samples.append(dict(stamp=12, points=0, valid=True))
        self.assertTrue(L.fresh_effect(node, True, 0, 10))
        self.assertFalse(L.fresh_effect(node, False, 0, 10))
        node.samples[-1]['stamp'] = 11
        self.assertFalse(L.fresh_effect(node, True, 0, 10))
        node.samples[-1].update(stamp=12, valid=False)
        self.assertFalse(L.fresh_effect(node, True, 0, 10))

    def test_source_owner_is_unique_not_an_arbitrary_scan_publisher(self):
        owner = NS(node_name='swerve_lidar_publisher', node_namespace='/')
        node = NS(TOPIC=L.TOPIC, get_publishers_info_by_topic=Mock(return_value=[owner]))
        self.assertTrue(L.unique_source(node))
        node.get_publishers_info_by_topic.return_value = [owner, owner]
        self.assertFalse(L.unique_source(node))
        node.get_publishers_info_by_topic.return_value = [NS(node_name='fake_map', node_namespace='/')]
        self.assertFalse(L.unique_source(node))

    def test_source_transition_requires_acceptance_readback_and_sensor_effect(self):
        for enabled in (True, False):
            for failure in (None, 'rejected', 'readback', 'no_effect', 'already_set'):
                with self.subTest(enabled=enabled, failure=failure):
                    result = NS(results=[NS(successful=failure != 'rejected')])
                    future = NS(done=lambda: True, result=lambda: result)
                    node = NS(OWNER=L.OWNER, PARAMETER=L.PARAMETER, TOPIC=L.TOPIC,
                              samples=[dict(stamp=10, points=4 if enabled else 0, valid=True)],
                              setter=NS(service_is_ready=lambda: True, call_async=Mock(return_value=future)),
                              getter=NS(service_is_ready=lambda: True), unique_source=lambda: True,
                              get_clock=lambda: NS(now=lambda: NS(nanoseconds=11)))
                    node.readback = Mock(side_effect=[enabled if failure == 'already_set' else not enabled,
                                                      not enabled if failure == 'readback' else enabled])
                    node.fresh_effect = lambda state, first, floor: L.fresh_effect(node, state, first, floor)
                    def wait(predicate, deadline, label):
                        if label.startswith('two fresh') and failure != 'no_effect':
                            node.samples.extend(dict(stamp=value, points=0 if enabled else 4, valid=True)
                                                for value in (12, 13))
                        if not predicate():
                            raise RuntimeError('timeout waiting for ' + label)
                    node.wait_for = wait
                    if failure:
                        with self.assertRaises(RuntimeError):
                            L.run(node, enabled, 6.)
                    else:
                        report = L.run(node, enabled, 6.)
                        self.assertEqual(report['enabled'], enabled)
                        request = node.setter.call_async.call_args.args[0]
                        self.assertEqual(request.parameters[0].name, 'lidar_occlusion_enabled')
                        self.assertEqual(request.parameters[0].value.bool_value, enabled)
                        self.assertEqual(len(report['effect_samples']), 2)
                    if failure == 'already_set':
                        node.setter.call_async.assert_not_called()


class UnreachableRunnerContractTest(unittest.TestCase):
    def test_bounded_timeout_requires_independent_no_path_and_zero_evidence(self):
        text = (ROOT / 'scripts/test_mujoco_minco_mpc_chain.sh').read_text()
        def function(name):
            start = text.index(name + '() {')
            end = text.index('\n}', start) + 2
            return text[start:end]
        functions = '\n'.join(function(name) for name in
                              ('run_p2_fault_injection', 'wait_for_fault_action_result'))
        script = r'''
set -eu -o pipefail
LAUNCH_LOG="$1/launch.log"
FAULT_ACTION_OUTPUT="$1/action.out"
CAUSE="$2" CODE="$3"
: >"${LAUNCH_LOG}"
fail() { echo "FAIL: $*" >&2; exit 97; }
find_unreachable_goal() {
  UNREACHABLE_GOAL_FRAME=map UNREACHABLE_GOAL_X=8.97 UNREACHABLE_GOAL_Y=3.91
}
publish_relative_fault_goal() { :; }
send_fault_goal() {
  [[ "$1" == unreachable && "$5" == 8 ]] || fail 'deadline does not fit observer'
  GOAL_DEADLINE="$5"
  printf 'result_code: %s\nmessage: goal timeout\nGoal finished with status: ABORTED\n' "${CODE}" >"${FAULT_ACTION_OUTPUT}"
  printf '%s\n' "${CAUSE}" >>"${LAUNCH_LOG}"
}
wait_for_command() {
  if [[ "$1" == 'unreachable action result code '* ]]; then
    [[ "$2" == 12 && "${GOAL_DEADLINE}" -lt "$2" ]] || fail 'observer bound changed'
  fi
  shift 2
  "$@" || fail 'required predicate absent'
}
topic_field_equals() { [[ "$1" == /planner/emergency_stop && "$3" == true ]]; }
capture_zero_outputs() { echo 'observed fresh selected and motion zero'; }
'''
        with tempfile.TemporaryDirectory() as directory:
            for cause, code, success in (
                    ('jps failed: no path expanded=24123 clearance=0.341 m', '3', True),
                    ('jps failed: goal is occupied expanded=0 clearance=0.341 m', '3', False),
                    ('jps failed: no path expanded=24123 clearance=0.341 m', '5', False),
                    ('', '3', False)):
                with self.subTest(cause=cause, code=code):
                    result = subprocess.run(['bash', '-c', script + '\n' + functions +
                                             '\nrun_p2_fault_injection unreachable', 'test',
                                             directory, cause, code], capture_output=True, text=True)
                    self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
                    if success:
                        self.assertIn('observed fresh selected and motion zero', result.stdout)
                    else:
                        self.assertNotIn('PASS:', result.stdout)


if __name__ == '__main__':
    unittest.main()
