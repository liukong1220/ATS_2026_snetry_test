#!/usr/bin/env python3
"""Focused recovery/launch contracts; source install/setup.bash before running."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction

import evaluate_mujoco_localization_fault as evaluator

E = evaluator.LocalizationFaultEvaluator
ROOT = Path(__file__).resolve().parents[1]


class LocalizationFaultContractTest(unittest.TestCase):
    def test_real_mode_never_creates_or_maintains_synthetic_observations(self):
        with patch.object(evaluator.Node, '__init__', return_value=None), \
                patch.object(evaluator.Node, 'create_publisher') as publisher, \
                patch.object(evaluator.Node, 'create_subscription'), \
                patch.object(evaluator.Node, 'create_timer'), \
                patch.object(evaluator, 'ActionClient'):
            node = E('odometry_stale', 'real')
            self.assertEqual([call.args[1] for call in publisher.call_args_list], ['/odometry'])
            node.maintenance_sequence = 1
            node.publish_observation = Mock()
            node._publish_maintenance_observation()
            node.publish_observation.assert_not_called()
            with self.assertRaisesRegex(RuntimeError, 'forbidden'):
                E.publish_observation(node, None, 1)
            with self.assertRaisesRegex(ValueError, 'explicit synthetic'):
                E('epoch_jump', 'real')

    def test_relay_fault_preserves_raw_samples_and_restores_only_when_enabled(self):
        node = NS(raw_odometry=[], relay_enabled=True, odom_pub=Mock())
        first, stopped, restored = object(), object(), object()
        E._on_raw_odometry(node, first)
        node.relay_enabled = False
        E._on_raw_odometry(node, stopped)
        node.relay_enabled = True
        E._on_raw_odometry(node, restored)
        self.assertEqual(node.raw_odometry, [first, stopped, restored])
        self.assertEqual([call.args[0] for call in node.odom_pub.publish.call_args_list],
                         [first, restored])

    def test_pre_fault_zero_is_not_stop_evidence(self):
        node = NS(latest_cmd=(0.0, 0.0, 0.0), latest_cmd_received_at=4.0,
                  command_norm=E.command_norm)
        self.assertFalse(E.commands_are_zero(node, 5.0))
        node.latest_cmd_received_at = 5.1
        self.assertTrue(E.commands_are_zero(node, 5.0))
        node.latest_cmd = (0.1, 0.0, 0.0)
        self.assertFalse(E.commands_are_zero(node, 5.0))

    def test_real_recovery_requires_new_scan_and_fusion_acceptance(self):
        observation = NS(accepted=True, status=evaluator.RelocalizationObservation.STATUS_ACCEPTED,
                         sequence=12, header=NS(stamp=NS(sec=20, nanosec=1)))
        status = NS(observation_sequence=12)
        node = NS(observations=[observation], latest_status=lambda: status,
                  recovery_observation_floor=11, recovery_stamp_floor=20_000_000_000,
                  stamp_ns=E.stamp_ns)
        self.assertTrue(E.fresh_real_observation(node))
        for field, value in [('accepted', False), ('sequence', 11)]:
            old = getattr(observation, field)
            setattr(observation, field, value)
            self.assertFalse(E.fresh_real_observation(node))
            setattr(observation, field, old)
        observation.header.stamp.nanosec = 0
        self.assertFalse(E.fresh_real_observation(node))
        observation.header.stamp.nanosec = 1
        status.observation_sequence = 10
        self.assertFalse(E.fresh_real_observation(node))

    def recovery_fixture(self):
        def header(sec, nanosec=0):
            return NS(stamp=NS(sec=sec, nanosec=nanosec))

        digest = 'abcdef123'
        command = NS(mode=evaluator.ExecutionCommand.MODE_EXECUTE, command_sequence=101,
                     manager_incarnation=4, goal_id=7, localization_epoch=9,
                     planner_incarnation=0, planner_candidate_sequence=0,
                     map_generation=11, map_publication_sequence=12,
                     candidate_content_digest=bytes.fromhex('abcdef12') + bytes(28),
                     reference=NS(poses=[object(), object()], header=header(22)))
        stopped = NS(**vars(command))
        stopped.mode = evaluator.ExecutionCommand.MODE_STOP
        stopped.command_sequence = 100
        stopped.header = header(20)
        request = NS(goal_id=7, localization_epoch=9, plan_request_sequence=4,
                     map_publication_sequence=12, header=header(20, 500_000_000))
        status = NS(goal_id=7, localization_epoch=9, plan_request_sequence=4,
                    map_publication_sequence=12, map_generation=11,
                    state=evaluator.PlannerStatus.STATE_REFERENCE_READY,
                    failure_reason=evaluator.PlannerStatus.FAILURE_NONE,
                    reference_stamp=header(21).stamp, content_digest=digest)
        node = NS(execution_commands=[command], stamp_ns=E.stamp_ns,
                  candidate_digest=E.candidate_digest, execution_identity=E.execution_identity,
                  planner_goals=[request], planner_statuses=[status],
                  recovery_planner_offset=0, recovery_status_offset=0, recovery_request_floor=3)
        node.correlated_plan_status = lambda cmd, stop: E.correlated_plan_status(node, cmd, stop)
        return node, command, stopped, request, status

    def test_active_request_status_digest_contract_accepts_reserved_zero_candidate_fields(self):
        node, command, stopped, request, status = self.recovery_fixture()
        self.assertIs(E.fresh_execution(node, 0, stopped, 9), command)
        self.assertIsNone(E.fresh_execution(node, 1, stopped, 9))
        self.assertEqual(E.candidate_digest(status.content_digest), command.candidate_content_digest)
        # Real GICP can advance epoch during recovery; all correlated owners must agree.
        command.localization_epoch = request.localization_epoch = status.localization_epoch = 10
        self.assertIsNone(E.fresh_execution(node, 0, stopped, 9))
        self.assertIs(E.fresh_execution(node, 0, stopped, 10), command)

    def test_recovery_rejects_wrong_execution_and_retained_reference(self):
        node, command, stopped, _, _ = self.recovery_fixture()
        for field, value in [('command_sequence', 100), ('manager_incarnation', 5),
                             ('goal_id', 8), ('localization_epoch', 8),
                             ('map_generation', 10), ('map_publication_sequence', 11),
                             ('candidate_content_digest', bytes(32)),
                             ('mode', evaluator.ExecutionCommand.MODE_STOP)]:
            old = getattr(command, field)
            setattr(command, field, value)
            self.assertIsNone(E.fresh_execution(node, 0, stopped, 9))
            setattr(command, field, old)
        command.reference.header.stamp.sec = 20
        self.assertIsNone(E.fresh_execution(node, 0, stopped, 9))
        reasons = node.recovery_correlation_evidence['rejected_executions'][0]['rejected_by']
        self.assertIn('post_stop_reference_stamp', reasons)

    def test_recovery_requires_observed_fresh_request_and_matching_ready_status(self):
        node, command, stopped, request, status = self.recovery_fixture()
        for target, field, value in (
                (request, 'plan_request_sequence', 3),
                (request, 'goal_id', 8), (request, 'localization_epoch', 8),
                (request, 'map_publication_sequence', 11),
                (status, 'plan_request_sequence', 3),
                (status, 'state', evaluator.PlannerStatus.STATE_ACCEPTED),
                (status, 'failure_reason', evaluator.PlannerStatus.FAILURE_SNAPSHOT_CHANGED),
                (status, 'content_digest', '11223344')):
            old = getattr(target, field)
            setattr(target, field, value)
            self.assertIsNone(E.fresh_execution(node, 0, stopped, 9))
            setattr(target, field, old)
        node.recovery_status_offset = 1
        self.assertIsNone(E.fresh_execution(node, 0, stopped, 9))
        node.recovery_status_offset = 0
        node.recovery_planner_offset = 1
        self.assertIsNone(E.fresh_execution(node, 0, stopped, 9))
        node.recovery_planner_offset = 0
        request.header.stamp.sec = 19
        self.assertIsNone(E.fresh_execution(node, 0, stopped, 9))
        self.assertIn('fresh_request_ready_status_digest',
                      node.recovery_correlation_evidence['rejected_executions'][0]['rejected_by'])

    def test_competing_observation_or_odometry_writer_fails_closed(self):
        own = 'mujoco_localization_fault_evaluator'
        publishers = {'/odometry': [NS(node_name=own)],
                      '/odometry_raw': [NS(node_name='ats_mujoco_sim')],
                      '/relocalization_observation': [NS(node_name='small_gicp_relocalization')]}
        node = NS(relocalization_mode='real', get_name=lambda: own,
                  get_publishers_info_by_topic=lambda topic: publishers[topic],
                  get_subscriptions_info_by_topic=lambda topic: [NS(node_name='localization_fusion')])
        self.assertTrue(E.assert_topic_ownership(node))
        for topic in publishers:
            publishers[topic].append(NS(node_name='unexpected_writer'))
            with self.assertRaisesRegex(RuntimeError, 'observed publishers=.*unexpected_writer'):
                E.assert_topic_ownership(node, allow_missing=True)
            publishers[topic].pop()
            expected = publishers[topic][0]
            publishers[topic][0] = NS(node_name='wrong_owner')
            with self.assertRaisesRegex(RuntimeError, 'observed publishers=.*wrong_owner'):
                E.assert_topic_ownership(node, allow_missing=True)
            publishers[topic][0] = expected

    def test_missing_discovery_waits_until_all_owners_and_fusion_subscription_arrive(self):
        own = 'mujoco_localization_fault_evaluator'
        publishers = {'/odometry': [], '/odometry_raw': [], '/relocalization_observation': []}
        subscribers = []
        node = NS(relocalization_mode='real', get_name=lambda: own,
                  get_publishers_info_by_topic=lambda topic: publishers[topic],
                  get_subscriptions_info_by_topic=lambda topic: subscribers)
        self.assertFalse(E.assert_topic_ownership(node, allow_missing=True))
        for topic, name in (('/odometry', own), ('/odometry_raw', 'ats_mujoco_sim'),
                            ('/relocalization_observation', 'small_gicp_relocalization')):
            publishers[topic].append(NS(node_name=name))
            self.assertFalse(E.assert_topic_ownership(node, allow_missing=True))
        subscribers.append(NS(node_name='localization_fusion'))
        self.assertTrue(E.assert_topic_ownership(node, allow_missing=True))
        publishers['/odometry'] = []
        with self.assertRaisesRegex(RuntimeError, 'missing topic ownership'):
            E.assert_topic_ownership(node)
        # Missing one endpoint never hides a competing authority on another.
        publishers['/relocalization_observation'].append(NS(node_name='unexpected_writer'))
        with self.assertRaisesRegex(RuntimeError, 'observed publishers=.*unexpected_writer'):
            E.assert_topic_ownership(node, allow_missing=True)


class LocalizationLaunchContractTest(unittest.TestCase):
    def test_default_graph_and_explicit_interceptor_wiring(self):
        path = ROOT / 'src/sim/ats_mujoco_sim/launch/rmuc_2025_mujoco.launch.py'
        spec = importlib.util.spec_from_file_location('rmuc_localization_contract', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        def inert_action(*args, **kwargs):
            return OpaqueFunction(function=lambda context: [])

        with patch.object(module, 'Node', side_effect=inert_action) as nodes, \
                patch.object(module, 'IncludeLaunchDescription', side_effect=inert_action) as includes:
            description = module.generate_launch_description()
        context = LaunchContext()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        fusion = next(call.kwargs for call in nodes.call_args_list
                      if call.kwargs.get('name') == 'localization_fusion')
        gicp = next(call.kwargs for call in nodes.call_args_list
                   if call.kwargs.get('name') == 'small_gicp_relocalization')
        simulator = dict(includes.call_args.kwargs['launch_arguments'])
        self.assertEqual(fusion['parameters'][0]['odom_topic'].perform(context), '/odometry')
        self.assertEqual(simulator['odom_topic'].perform(context), '/odometry')
        self.assertTrue(gicp['condition'].evaluate(context))
        self.assertIs(fusion['parameters'][0]['publish_tf'], True)
        self.assertIs(gicp['parameters'][1]['publish_tf'], False)
        self.assertEqual(simulator['publish_map_to_odom_tf'], 'false')
        context.launch_configurations.update(mujoco_odom_topic='/odometry_raw',
                                             fusion_odom_topic='/odometry_intercepted',
                                             launch_small_gicp_relocalization='false')
        self.assertEqual(simulator['odom_topic'].perform(context), '/odometry_raw')
        self.assertEqual(fusion['parameters'][0]['odom_topic'].perform(context), '/odometry_intercepted')
        self.assertFalse(gicp['condition'].evaluate(context))


if __name__ == '__main__':
    unittest.main()
