#!/usr/bin/env python3
"""S3 evidence tools must reject stationary and inaccurate runs."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

spec = importlib.util.spec_from_file_location(
    "s3_acceptance", Path(__file__).with_name("s3_mujoco_acceptance.py"))
s3 = importlib.util.module_from_spec(spec)


class EvidenceTools(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec.loader.exec_module(s3)

    def test_route_requires_measured_motion_and_true_endpoint(self):
        self.assertFalse(s3.route_metrics([(0, 0), (0, 0)], [(0, 0)], 0.1)["passed"])
        self.assertFalse(s3.route_metrics([(0, 0), (1, 0)], [(1.1, 0)], 0.5)["passed"])
        result = s3.route_metrics([(0, 0), (0.5, 0), (1, 0)], [(1.05, 0)], 0.5)
        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["path_length_m"], 1.0)

    def test_final_tolerance_is_a_parameter(self):
        path = [(0, 0), (1, 0)]
        self.assertFalse(s3.route_metrics(path, [(1.1, 0)], 0.5)["passed"])
        self.assertTrue(s3.route_metrics(path, [(1.1, 0)], 0.5, final_tolerance=0.15)["passed"])
        self.assertEqual((s3.WAYPOINT_TOLERANCE_M, s3.FINAL_TOLERANCE_M), (0.15, 0.08))

    def test_only_executed_references_count_as_gate_collisions(self):
        from types import SimpleNamespace as NS

        def status(goal, sequence, sec, count, state=1):
            return NS(state=state, STATE_REFERENCE_READY=1, goal_id=goal,
                      plan_request_sequence=sequence, gate_collision_count=count,
                      reference_stamp=NS(sec=sec, nanosec=0))

        statuses = [status(7, 1, 10, 2), status(7, 2, 11, 3), status(7, 3, 12, 0),
                    status(7, 4, 13, 5, state=0)]
        executed, transient = s3.split_gate_collisions(statuses, {(7, 11.0)})
        self.assertEqual([e["plan_request_sequence"] for e in executed], [2])
        self.assertEqual([t["plan_request_sequence"] for t in transient], [1])

    def test_contact_is_diagnostic_but_post_input_lost_still_aborts(self):
        from types import SimpleNamespace as NS
        probe = s3.Probe.__new__(s3.Probe)
        probe.event = Mock()
        probe.latest = {}
        probe.on_telemetry(NS(contact_violation_count=0, max_contact_force=0.0))
        probe.on_telemetry(NS(contact_violation_count=1, max_contact_force=12.0))
        self.assertEqual(probe.contact_seen, {"max_violation_count": 1, "max_force": 12.0,
            "first_violation_count": 0, "last_violation_count": 1,
            "counter_resets": 0, "violation_delta": 1})
        lost = NS(state=4, STATE_LOST=4, odom_identity_latched=False)
        probe.check_localization_health(lost)
        probe.latest['/localization'] = (object(), 1.0)
        with self.assertRaisesRegex(RuntimeError, 'localization lost'):
            probe.check_localization_health(lost)
        probe.latest.clear()
        lost.odom_identity_latched = True
        with self.assertRaisesRegex(RuntimeError, 'identity latched'):
            probe.check_localization_health(lost)

    def test_runtime_executed_collision_aborts_but_ready_transient_does_not(self):
        from types import SimpleNamespace as NS
        probe = s3.Probe.__new__(s3.Probe)
        probe.planner_statuses = [NS(state=1, STATE_REFERENCE_READY=1, goal_id=7,
                                    plan_request_sequence=2, gate_collision_count=1,
                                    reference_stamp=NS(sec=10, nanosec=0))]
        probe.executed_references = set()
        probe.check_executed_collisions()
        probe.executed_references.add((7, 10.0))
        with self.assertRaisesRegex(RuntimeError, 'executed reference'):
            probe.check_executed_collisions()

    def test_expected_localization_loss_is_bounded_and_ends_at_recovery(self):
        from types import SimpleNamespace as NS
        probe = s3.Probe.__new__(s3.Probe)
        probe.latest = {'/localization': (object(), 1.0)}
        probe.event = Mock()
        lost = NS(state=4, STATE_LOST=4, odom_identity_latched=False)
        with patch.object(s3.time, 'monotonic', return_value=100.0) as clock:
            with probe.expect_localization_loss(5.0) as recovered:
                probe.check_localization_health(lost)
                lost.odom_identity_latched = True
                with self.assertRaisesRegex(RuntimeError, 'identity latched'):
                    probe.check_localization_health(lost)
                lost.odom_identity_latched = False
                clock.return_value = 105.0
                with self.assertRaisesRegex(RuntimeError, 'localization lost'):
                    probe.check_localization_health(lost)
                clock.return_value = 101.0
                recovered()
                with self.assertRaisesRegex(RuntimeError, 'localization lost'):
                    probe.check_localization_health(lost)
            with self.assertRaisesRegex(RuntimeError, 'localization lost'):
                probe.check_localization_health(lost)

    def test_expected_localization_loss_is_cleared_on_failed_experiment(self):
        from types import SimpleNamespace as NS
        probe = s3.Probe.__new__(s3.Probe)
        probe.latest = {'/localization': (object(), 1.0)}
        probe.event = Mock()
        with self.assertRaisesRegex(ValueError, 'fixture failure'):
            with probe.expect_localization_loss(5.0):
                raise ValueError('fixture failure')
        with self.assertRaisesRegex(RuntimeError, 'localization lost'):
            probe.check_localization_health(NS(state=4, STATE_LOST=4, odom_identity_latched=False))
        for timeout in (0.0, -1.0, float('inf'), float('nan')):
            with self.assertRaises(ValueError):
                with probe.expect_localization_loss(timeout):
                    pass

    def test_runtime_collision_checks_both_callback_arrival_orders(self):
        from types import SimpleNamespace as NS
        for execution_first in (False, True):
            with self.subTest(execution_first=execution_first), tempfile.TemporaryDirectory() as tmp:
                probe = s3.Probe.__new__(s3.Probe)
                probe.event = Mock()
                probe.save_grid = Mock()
                probe.directory = Path(tmp)
                probe.latest = {}
                probe.planner_statuses = []
                probe.executed_references = set()
                probe.execution_events = []
                probe.message_dict = lambda _: {}
                status = NS(state=1, STATE_REFERENCE_READY=1, STATE_FAILED=2, goal_id=7,
                            plan_request_sequence=2, gate_collision_count=1,
                            gate_collision_indices=[0], reference_stamp=NS(sec=10, nanosec=0))
                execution = NS(mode=1, MODE_EXECUTE=1, goal_id=7,
                               reference=NS(header=NS(stamp=NS(sec=10, nanosec=0)), poses=[]),
                               manager_incarnation=1, command_sequence=1,
                               localization_epoch=1, map_generation=1)
                first, second = ((lambda: probe.on_execution(execution),
                                  lambda: probe.on_planner_status(status)) if execution_first else
                                 (lambda: probe.on_planner_status(status),
                                  lambda: probe.on_execution(execution)))
                first()
                with self.assertRaisesRegex(RuntimeError, 'executed reference'):
                    second()

    def test_runtime_telemetry_subscription_matches_simulator_default(self):
        source = Path(s3.__file__).read_text()
        self.assertIn('(SwerveTelemetry, "/swerve/telemetry", 10, self.on_telemetry)', source)
        simulator = (s3.SIM / 'ats_mujoco_sim/sim_node.py').read_text()
        self.assertIn('"swerve_telemetry_topic", "/swerve/telemetry"', simulator)

    def test_committed_reference_is_recorded_once_across_lease_renewals(self):
        from types import SimpleNamespace as NS
        from unittest.mock import Mock
        probe = s3.Probe.__new__(s3.Probe)
        probe.event = Mock()
        probe.message_dict = lambda msg: {'poses': msg.poses}
        probe.executed_references = set()
        probe.execution_events = []
        probe.planner_statuses = []
        msg = NS(mode=1, MODE_EXECUTE=1, goal_id=7,
                 reference=NS(header=NS(stamp=NS(sec=10, nanosec=0)), poses=[1, 2]),
                 manager_incarnation=1, command_sequence=1,
                 localization_epoch=1, map_generation=1)
        probe.on_execution(msg)
        msg.command_sequence += 1
        probe.on_execution(msg)
        recorded = [call for call in probe.event.call_args_list
                    if call.args[0] == 'committed_reference']
        self.assertEqual(len(recorded), 1)
        self.assertEqual(recorded[0].args[1]['reference']['poses'], [1, 2])
        self.assertEqual(len(probe.execution_events), 2)

    def test_post_route_exception_revokes_the_overall_pass(self):
        import json
        from types import SimpleNamespace
        import yaml

        profile = yaml.safe_load((s3.SIM / "config/s3_mapping_routes.yaml").read_text())["rmuc_2025"]
        probe = Mock()
        probe.graph.return_value = {"/rog_map_adapter/planning_snapshot": {"publishers": ["ats_rog_map_adapter"]}}
        probe.path = [profile["start"][:2], profile["waypoints"][0][:2]]
        probe.goals = [(0, True)]
        probe.endpoint_results = [{"truth_error_m": 0.01}]
        probe.counts = {}
        probe.planner_statuses = []
        probe.save_map.return_value = {"success": True}
        probe.health_summary.return_value = {"ready": True}
        process = Mock()
        process.poll.return_value = 0
        failure = RuntimeError("physical obstacle latency exceeds 150 ms")
        modules = {
            "rclpy": Mock(),
            "ament_index_python.packages": SimpleNamespace(
                get_package_share_directory=lambda _: str(s3.SIM)),
            "s3_fault_scenarios": SimpleNamespace(run_case=Mock(side_effect=failure)),
            "s3_tracking_evidence": SimpleNamespace(
                TrackingLeg=lambda *_: __import__('contextlib').nullcontext()),
        }
        with tempfile.TemporaryDirectory() as tmp, patch.dict("sys.modules", modules), \
                patch.object(s3, "verify_installed_files"), \
                patch.object(s3, "prepare_scene", return_value=([], Path("scene.xml"))), \
                patch.object(s3, "Probe", return_value=probe), \
                patch.object(s3.subprocess, "Popen", return_value=process):
            args = SimpleNamespace(profile="rmuc_2025", output=tmp, case="route", reload=None,
                                   post_route_case="obstacle_and_dynamic", max_goals=1,
                                   startup_timeout=1.0, launch_arg=[])
            with self.assertRaisesRegex(RuntimeError, "physical obstacle latency"):
                s3.run(args)
            summary = json.loads((Path(tmp) / "summary.json").read_text())
            self.assertFalse(summary["passed"])
            self.assertIn("exceeds 150 ms", summary["error"])
            self.assertEqual(summary["endpoints"], probe.endpoint_results)

    def test_full_field_post_route_experiment_preserves_the_production_scene(self):
        import yaml
        profiles = yaml.safe_load((s3.SIM / "config/s3_mapping_routes.yaml").read_text())
        for name in ("rmuc_2025", "rmuc_2027"):
            with self.subTest(profile=name), tempfile.TemporaryDirectory() as tmp:
                extra, scene = s3.prepare_scene(profiles[name], Path(tmp), fixture=True)
                self.assertIn("enable_test_obstacle_fixture:=true", extra)
                self.assertIn(f"model_file:={scene}", extra)
                provenance = __import__("json").loads((Path(tmp) / "scene-provenance.json").read_text())
                self.assertEqual(provenance["source_model"],
                                 str((s3.SIM / f"models/{name}_swerve.xml").resolve()))

    def test_full_field_no_path_uses_live_grid_without_modifying_geometry(self):
        import yaml
        profiles = yaml.safe_load((s3.SIM / "config/s3_mapping_routes.yaml").read_text())
        for name in ("rmuc_2025", "rmuc_2027"):
            with self.subTest(profile=name), tempfile.TemporaryDirectory() as tmp:
                extra, scene = s3.prepare_scene(profiles[name], Path(tmp), barrier=True)
                self.assertEqual(extra, [])
                self.assertEqual(scene, s3.SIM / f"models/{name}_swerve.xml")
                policy = __import__("json").loads((Path(tmp) / 'no-path-policy.json').read_text())
                self.assertFalse(policy['scene_modified'])

    def test_failure_preserves_source_layers_once_per_goal_and_reason(self):
        from types import SimpleNamespace as NS
        probe = s3.Probe.__new__(s3.Probe)
        probe.event = Mock()
        probe.save_grid = Mock()
        probe.planner_statuses = []
        probe.message_dict = lambda _: {}
        probe.check_executed_collisions = Mock()
        status = NS(state=2, STATE_REFERENCE_READY=1, STATE_FAILED=2,
                    goal_id=7, failure_reason=5)
        for _ in range(400):
            probe.on_planner_status(status)
        probe.save_grid.assert_called_once()
        self.assertIn('/traversability_grid', probe.save_grid.call_args.args[1])
        status.failure_reason = 6
        probe.on_planner_status(status)
        self.assertEqual(probe.save_grid.call_count, 2)

    def test_route_deviation_is_measured_against_segments(self):
        result = s3.route_deviation([(0, 0), (1, 0.2), (2, 0)], [(0, 0), (2, 0)])
        self.assertAlmostEqual(result["maximum_m"], 0.2)
        self.assertAlmostEqual(result["mean_m"], 0.2 / 3)
        self.assertGreater(result["p95_m"], 0.15)

    def test_tunnel_profile_is_explicit_and_keeps_other_profiles_unchanged(self):
        import yaml
        source = s3.SIM / "config/s3_mapping_routes.yaml"
        profiles = yaml.safe_load(source.read_text())
        self.assertNotIn("routes", profiles["rmuc_2027"])
        base = profiles["rmuc_2027_tunnels"]
        self.assertEqual(base["start"], profiles["rmuc_2027"]["start"])
        self.assertEqual(base["odom_layer_origin"], profiles["rmuc_2027"]["odom_layer_origin"])
        with self.assertRaises(ValueError):
            s3.load_profile(source, "rmuc_2027_tunnels", "unknown_route")
        with self.assertRaises(ValueError):
            s3.load_profile(source, "rmuc_2027", "tunnel_b_roundtrip")
        for route in base["routes"]:
            profile = s3.load_profile(source, "rmuc_2027_tunnels", route)
            x0, y0 = profile["odom_layer_origin"]
            sx, sy = profile["start"][:2]
            truth = s3.load_tunnel_truth(s3.SIM / "config", [t["name"] for t in profile["tunnels"]])
            self.assertEqual(len(truth), len(profile["tunnels"]))
            for x, y, _ in profile["waypoints"]:
                self.assertTrue(x0 <= x - sx < x0 + 40.0 and y0 <= y - sy < y0 + 30.0)
                # The official ridge field (STL peak-to-valley 6-7 cm, S2) is excluded.
                self.assertFalse(5.15 <= x <= 7.8 and 0.35 <= y <= 3.2, (route, x, y))

    def test_tunnel_report_counts_complete_traversals_and_axis_error(self):
        tunnel = {"name": "t", "corridor_polygon": [[0, -0.4], [4, -0.4], [4, 0.4], [0, 0.4]],
                  "entry": [0.1, 0.0, 0.0], "exit": [3.9, 0.0, 0.0], "axis_yaw": 0.0,
                  "width": 0.8}
        through = [(x / 10.0, 0.05, 0.02) for x in range(0, 41)]
        report = s3.tunnel_report(through, [tunnel])["t"]
        self.assertEqual((report["forward_passes"], report["reverse_passes"]), (1, 0))
        self.assertAlmostEqual(report["axis_lateral_max_m"], 0.05)
        self.assertTrue(report["within_walls"])
        back = s3.tunnel_report(through + through[::-1], [tunnel])["t"]
        self.assertEqual((back["forward_passes"], back["reverse_passes"]), (1, 1))
        # Reaching the exit by going around (outside the corridor) is not a pass.
        around = [(0.05, 0.0, 0.0), (0.0, 1.0, 0.0), (4.0, 1.0, 0.0), (3.95, 0.0, 0.0)]
        self.assertEqual(s3.tunnel_report(around, [tunnel])["t"]["forward_passes"], 0)
        # Hugging a wall beyond (width - robot) / 2 fails the wall criterion.
        wall = [(x / 10.0, 0.3, 0.0) for x in range(0, 41)]
        self.assertFalse(s3.tunnel_report(wall, [tunnel])["t"]["within_walls"])

    def test_tunnel_report_missing_and_reset_contact_counts_are_not_zero(self):
        tunnel = {"name": "t", "entry": [0., 0.], "exit": [1., 0.], "axis_yaw": 0.,
                  "width": 1., "corridor_polygon": [[0., -.5], [1., -.5], [1., .5], [0., .5]]}
        path = [(0., 0., 0.), (.5, 0., 0.), (1., 0., 0.)]
        self.assertEqual(s3.tunnel_report(path, [tunnel], contact_counts=[8, 9, 11])
                         ["t"]["contact_counter_delta"], 3)
        for counts in (None, [None, 0, 0], [8, 1, 2]):
            self.assertIsNone(s3.tunnel_report(path, [tunnel], contact_counts=counts)
                              ["t"]["contact_counter_delta"])
        with self.assertRaises(ValueError):
            s3.tunnel_report(path, [tunnel], contact_counts=[0])

    def test_contact_counter_reset_keeps_the_run_delta_unknown(self):
        from types import SimpleNamespace as NS
        probe = s3.Probe.__new__(s3.Probe)
        probe.event = Mock()
        for count in (8, 11):
            probe.on_telemetry(NS(contact_violation_count=count, max_contact_force=1.))
        self.assertEqual(probe.contact_seen['violation_delta'], 3)
        probe.on_telemetry(NS(contact_violation_count=1, max_contact_force=0.))
        self.assertIsNone(probe.contact_seen['violation_delta'])
        self.assertEqual(probe.contact_seen['counter_resets'], 1)

    def test_failed_tunnel_route_still_writes_per_tunnel_evidence(self):
        import json
        from types import SimpleNamespace as NS
        probe = Mock()
        probe.graph.return_value = {"/rog_map_adapter/planning_snapshot": {"publishers": ["ats_rog_map_adapter"]}}
        probe.truth_samples, probe.truth_contact_counts, probe.endpoint_results = [], [], []
        probe.health_summary.return_value = {"ready": True}
        probe.navigate.side_effect = RuntimeError("goal is occupied")
        process = Mock()
        process.poll.return_value = 0
        modules = {
            "rclpy": Mock(),
            "ament_index_python.packages": NS(get_package_share_directory=lambda _: str(s3.SIM)),
            "s3_tracking_evidence": NS(TrackingLeg=lambda *_: __import__('contextlib').nullcontext()),
        }
        with tempfile.TemporaryDirectory() as tmp, patch.dict("sys.modules", modules), \
                patch.object(s3, "verify_installed_files"), \
                patch.object(s3, "prepare_scene", return_value=([], Path("scene.xml"))), \
                patch.object(s3, "Probe", return_value=probe), \
                patch.object(s3.subprocess, "Popen", return_value=process):
            args = NS(profile="rmuc_2027_tunnels", route="tunnel_b_roundtrip", output=tmp,
                      case="route", reload=None, max_goals=0, post_route_case=None,
                      startup_timeout=1., launch_arg=[])
            with self.assertRaisesRegex(RuntimeError, "goal is occupied"):
                s3.run(args)
            summary = json.loads((Path(tmp) / "summary.json").read_text())
            self.assertFalse(summary['passed'])
            self.assertTrue(summary['tunnels'])
            for tunnel in summary['tunnels'].values():
                self.assertFalse(tunnel['passed'])
                self.assertIsNone(tunnel['axis_lateral_max_m'])
                self.assertIsNone(tunnel['contact_counter_delta'])
            probe.close.assert_called_once()

    def test_field_profiles_keep_the_fixed_array_around_the_complete_field(self):
        import yaml
        source = Path(__file__).resolve().parents[1] / "src/sim/ats_mujoco_sim/config/s3_mapping_routes.yaml"
        profiles = yaml.safe_load(source.read_text())
        for name in ("rmuc_2025", "rmuc_2027"):
            p = profiles[name]
            x0, y0 = p["odom_layer_origin"]
            sx, sy = p["start"][:2]
            self.assertLessEqual(x0, -sx)
            self.assertGreaterEqual(x0 + 40.0, 28.0 - sx)
            for x, y, _ in p["waypoints"]:
                self.assertTrue(x0 <= x - sx < x0 + 40.0)
                self.assertTrue(y0 <= y - sy < y0 + 30.0)

    def test_registration_target_and_global_layer_cover_the_same_field(self):
        import yaml
        root = Path(__file__).resolve().parents[1]
        profiles = yaml.safe_load((root / "src/sim/ats_mujoco_sim/config/s3_mapping_routes.yaml").read_text())
        for name in ("rmuc_2025", "rmuc_2027"):
            parameters = {key: {"ros__parameters": {}} for key in
                          ("ats_rog_map", "small_gicp_relocalization")}
            s3.configure_mapping_bounds(parameters, profiles[name])
            self.assertEqual(parameters["small_gicp_relocalization"]["ros__parameters"]["mapping.origin"][:2],
                             profiles[name]["odom_layer_origin"])

    def test_field_coverage_routes_respect_scene_corridors(self):
        import mujoco
        import numpy as np
        import yaml
        from PIL import Image
        from scipy.ndimage import distance_transform_edt

        root = Path(__file__).resolve().parents[1]
        simulation = root / "src/sim/ats_mujoco_sim"
        profiles = yaml.safe_load((simulation / "config/s3_mapping_routes.yaml").read_text())
        for field in ("2025", "2027"):
            profile = profiles["rmuc_" + field]
            map_yaml = root / f"src/ats_sentry_bringup/map/rmuc_{field}.yaml"
            metadata = yaml.safe_load(map_yaml.read_text())
            image = np.asarray(Image.open(map_yaml.parent / metadata["image"]))[::-1]
            resolution = metadata["resolution"]
            ox, oy, _ = metadata["origin"]
            ground_support = image > 240
            if field == "2027":
                # This PGM projects raised platforms as occupied. S3's ground
                # columns distinguish their supporting surface from walls. Use
                # the independently simulated terrain for this offline guard;
                # the roof test below separately rejects insufficient headroom.
                model = mujoco.MjModel.from_xml_path(
                    str(simulation / "models/rmuc_2027_swerve.xml"))
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rmuc_2027_field")
                heights = np.array([[5.0 - mujoco.mj_rayHfield(
                    model, data, geom,
                    np.array([ox + (x + 0.5) * resolution, oy + (y + 0.5) * resolution, 5.0]),
                    np.array([0.0, 0.0, -1.0]))
                    for x in range(image.shape[1])] for y in range(image.shape[0])])
                dy, dx = np.gradient(heights, resolution)
                ground_support = np.hypot(dx, dy) < 0.60
            distance = distance_transform_edt(ground_support) * resolution
            previous = np.asarray(profile["start"][:2])
            for i, point in enumerate(profile["waypoints"]):
                target = np.asarray(point[:2])
                segment = np.linspace(previous, target,
                                      max(2, int(np.ceil(np.linalg.norm(target - previous) / 0.04)) + 1))
                ix = np.floor((segment[:, 0] - ox) / resolution).astype(int)
                iy = np.floor((segment[:, 1] - oy) / resolution).astype(int)
                # Offline route preflight uses the same 0.30 m lower bound as
                # the existing RMUC entry. It cannot prove rectangular yaw
                # clearance; the unchanged live search and footprint gate do.
                clearance = float(distance[iy, ix].min())
                self.assertGreaterEqual(clearance, 0.30,
                                        f"{field} route segment {i + 1} crosses a physical wall")
                previous = target

    def test_field_route_stops_have_level_wheel_support(self):
        import mujoco
        import numpy as np
        import yaml

        simulation = Path(__file__).resolve().parents[1] / "src/sim/ats_mujoco_sim"
        profiles = yaml.safe_load((simulation / "config/s3_mapping_routes.yaml").read_text())
        wheels = np.array([[x, y] for x in (-0.270, 0.270) for y in (-0.270, 0.270)])
        for field in ("2025", "2027"):
            model = mujoco.MjModel.from_xml_path(str(simulation / f"models/rmuc_{field}_swerve.xml"))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            terrain = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"rmuc_{field}_field")
            for i, (x, y, heading) in enumerate(profiles["rmuc_" + field]["waypoints"], 1):
                c, s = np.cos(heading), np.sin(heading)
                supports = wheels @ np.array([[c, s], [-s, c]]) + [x, y]
                heights = [5.0 - mujoco.mj_rayHfield(
                    model, data, terrain, np.array([px, py, 5.0]), np.array([0.0, 0.0, -1.0]))
                    for px, py in supports]
                # Route goals are full stops. Cross a ramp in one goal and stop
                # on its platform. This offline route constraint does not replace
                # the unchanged live footprint and 8 cm truth endpoint gates.
                self.assertLessEqual(max(heights) - min(heights), 0.035,
                                     f"{field} stop {i} terminates on a ramp or terrain edge")

    def test_2025_route_clears_the_observed_wall_tip_with_its_full_body(self):
        import numpy as np
        import yaml

        simulation = Path(__file__).resolve().parents[1] / "src/sim/ats_mujoco_sim"
        route = yaml.safe_load((simulation / "config/s3_mapping_routes.yaml").read_text())[
            "rmuc_2025"]["waypoints"]
        # Minimal occupied patch from domain 143, goal 21. Its map origin is
        # the recorded odom origin transformed by the accepted map<-odom TF.
        # Rows run south to north. The PGM center-line test misses this tip: a
        # 0.1 m planning cell is an area, and the rectangle's front corner
        # intersects it even though the robot center remains clear.
        rows = ["...##", "..###", ".####", "#####", "#####",
                "#####", "#####", ".####", "..###", "..###", "...##"]
        obstacles = np.array([[9.62 + (x + 0.5) * 0.1, -2.64 + (y + 0.5) * 0.1]
                              for y, row in enumerate(rows) for x, v in enumerate(row)
                              if v == "#"])
        half_length, half_width = 0.581 / 2 + 0.02, 0.571 / 2 + 0.02
        start, end = np.array(route[19]), np.array(route[20])
        for t in np.linspace(0.0, 1.0, 80):
            center = start[:2] + t * (end[:2] - start[:2])
            delta = obstacles - center
            for heading in (start[2], end[2], 1.524):
                c, s = np.cos(heading), np.sin(heading)
                # Separating-axis rectangle/cell intersection, including the
                # cell's complete square footprint rather than its center.
                overlap = ((np.abs(delta[:, 0]) <= half_length * abs(c) + half_width * abs(s) + 0.05) &
                           (np.abs(delta[:, 1]) <= half_length * abs(s) + half_width * abs(c) + 0.05) &
                           (np.abs(delta @ [c, s]) <= half_length + 0.05 * (abs(c) + abs(s))) &
                           (np.abs(delta @ [-s, c]) <= half_width + 0.05 * (abs(c) + abs(s))))
                self.assertFalse(overlap.any(),
                                 f"2025 wall-tip route clips a planning cell at {center}, yaw={heading}")

    def test_2025_platform_turn_keeps_the_complete_body_on_its_support(self):
        import mujoco
        import numpy as np
        import yaml

        simulation = Path(__file__).resolve().parents[1] / "src/sim/ats_mujoco_sim"
        route = yaml.safe_load((simulation / "config/s3_mapping_routes.yaml").read_text())[
            "rmuc_2025"]["waypoints"]
        index = next(i for i in range(1, len(route)) if route[i - 1][:2] == [19.945, 3.335])
        target, previous = route[index], route[index - 1]
        model = mujoco.MjModel.from_xml_path(str(simulation / "models/rmuc_2025_swerve.xml"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        terrain = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rmuc_2025_field")
        body = np.array([[x, y] for x in np.linspace(-0.3405, 0.3405, 15)
                         for y in np.linspace(-0.3355, 0.3355, 15)])
        heights = []
        delta = np.arctan2(np.sin(target[2] - previous[2]),
                           np.cos(target[2] - previous[2]))
        for heading in np.linspace(previous[2], previous[2] + delta, 20):
            c, s = np.cos(heading), np.sin(heading)
            points = body @ np.array([[c, s], [-s, c]]) + target[:2]
            heights.extend(5.0 - mujoco.mj_rayHfield(
                model, data, terrain, np.array([x, y, 5.0]), np.array([0.0, 0.0, -1.0]))
                for x, y in points)
        # Domain 147 stopped with a footprint escape prefix here. Four wheel
        # heights alone missed the terrain edge inside the rotating body.
        self.assertLessEqual(max(heights) - min(heights), 0.035)

    def test_2027_west_passage_keeps_the_complete_body_off_the_wall_tip(self):
        import numpy as np
        import yaml

        route = yaml.safe_load((s3.SIM / "config/s3_mapping_routes.yaml").read_text())[
            "rmuc_2027"]["waypoints"]
        # Minimal occupied patch from domain 180's final planning grid. Its
        # corner at map (5.015, 2.738) intersects the old goal 7 footprint,
        # although the center line passes the 0.30 m physical-wall preflight.
        obstacles = np.array([[x, y] for x in np.arange(5.05, 5.56, 0.1)
                              for y in (2.55, 2.65, 2.75)])
        half_length, half_width = 0.581 / 2 + 0.02, 0.571 / 2 + 0.02
        for index in range(6, 11):
            start, end = np.array(route[index - 1]), np.array(route[index])
            angle = np.arctan2(np.sin(end[2] - start[2]),
                               np.cos(end[2] - start[2]))
            for fraction in np.linspace(0.0, 1.0, 80):
                center = start[:2] + fraction * (end[:2] - start[:2])
                heading = start[2] + fraction * angle
                c, s = np.cos(heading), np.sin(heading)
                delta = obstacles - center
                overlap = ((np.abs(delta[:, 0]) <= half_length * abs(c) + half_width * abs(s) + 0.05) &
                           (np.abs(delta[:, 1]) <= half_length * abs(s) + half_width * abs(c) + 0.05) &
                           (np.abs(delta @ [c, s]) <= half_length + 0.05 * (abs(c) + abs(s))) &
                           (np.abs(delta @ [-s, c]) <= half_width + 0.05 * (abs(c) + abs(s))))
                self.assertFalse(overlap.any(),
                                 f"2027 segment {index + 1} clips the west-passage wall tip")

    def test_2027_exploration_segments_keep_the_sensor_facing_the_motion(self):
        import math
        import yaml
        profile = yaml.safe_load((s3.SIM / "config/s3_mapping_routes.yaml").read_text())[
            "rmuc_2027"]
        previous = [*profile["start"][:2], profile["start"][3]]
        for index, target in enumerate(profile["waypoints"], 1):
            if math.dist(previous[:2], target[:2]) > .06:
                direction = math.atan2(target[1] - previous[1], target[0] - previous[0])
                difference = math.atan2(math.sin(direction - previous[2]), math.cos(direction - previous[2]))
                # Offline acquisition guard for the forward-tilted lidar.
                # The unchanged live known-free footprint gate is authoritative.
                self.assertLessEqual(abs(difference), 1.18, f"2027 segment {index} explores behind the sensor")
            previous = target

    def test_2027_route_preserves_ground_relative_roof_clearance(self):
        import mujoco
        import numpy as np
        import yaml

        simulation = Path(__file__).resolve().parents[1] / "src/sim/ats_mujoco_sim"
        profile = yaml.safe_load((simulation / "config/s3_mapping_routes.yaml").read_text())[
            "rmuc_2027"]
        roofs = yaml.safe_load((simulation / "config/tunnels_2027.yaml").read_text())
        model = mujoco.MjModel.from_xml_path(str(simulation / "models/rmuc_2027_swerve.xml"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        terrain = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rmuc_2027_field")
        body = np.array([[x, y] for x in np.linspace(-0.3405, 0.3405, 15)
                         for y in np.linspace(-0.3355, 0.3355, 15)])
        previous = np.array([*profile["start"][:2], profile["start"][3]])
        for index, waypoint in enumerate(profile["waypoints"], 1):
            target = np.array(waypoint)
            delta = np.arctan2(np.sin(target[2] - previous[2]),
                               np.cos(target[2] - previous[2]))
            count = max(2, int(np.ceil(np.linalg.norm(target[:2] - previous[:2]) / 0.025)),
                        int(np.ceil(abs(delta) / 0.04)))
            for fraction in np.linspace(0.0, 1.0, count):
                center = previous[:2] + fraction * (target[:2] - previous[:2])
                heading = previous[2] + fraction * delta
                c, s = np.cos(heading), np.sin(heading)
                points = body @ np.array([[c, s], [-s, c]]) + center
                for roof in roofs["tunnels"] + roofs["overhangs"]:
                    polygon = np.array(roof["polygon"])
                    lo, hi = polygon.min(axis=0), polygon.max(axis=0)
                    under = points[((points >= lo) & (points <= hi)).all(axis=1)]
                    for x, y in under:
                        ground = 5.0 - mujoco.mj_rayHfield(
                            model, data, terrain, np.array([x, y, 5.0]),
                            np.array([0.0, 0.0, -1.0]))
                        bottom = roof.get("bottom_z", roof.get("roof_bottom_z"))
                        bottom += roof.get("slope_y", 0.0) * (y - (lo[1] + hi[1]) / 2)
                        self.assertGreaterEqual(
                            # Offline route guard against the physical vehicle height
                            # (0.242 m, user-confirmed). Every route to the east shelf
                            # passes under ramp_bridge (0.302 m), so the 0.305 m runtime gate
                            # cannot be the preflight; the runtime gate is selected separately.
                            bottom - ground, 0.242,
                            f"2027 segment {index} has insufficient clearance under {roof['name']}")
            previous = target

    def test_stale_installed_launch_is_rejected_even_with_newer_mtime(self):
        with tempfile.TemporaryDirectory() as directory:
            source, installed = Path(directory) / "source.py", Path(directory) / "install.py"
            source.write_text("current\n")
            installed.write_text("old\n")
            with self.assertRaisesRegex(RuntimeError, "stale installed"):
                s3.verify_installed_files([(source, installed)])
            installed.write_text(source.read_text())
            s3.verify_installed_files([(source, installed)])

    def test_small_scene_preserves_robot_and_has_independent_wall_truth(self):
        source = Path(__file__).resolve().parents[1] / "src/sim/ats_mujoco_sim/models/rmuc_2027_swerve.xml"
        original = source.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "scene.xml"
            truth = s3.make_test_scene(source, target, 10.0)
            root = ET.parse(target).getroot()
            self.assertIsNotNone(root.find("worldbody/body[@name='base_link']"))
            self.assertFalse(root.findall("asset/hfield"))
            self.assertEqual(len(truth), 6)
            self.assertTrue((target.parent / "meshes/mid360.obj").is_file())
            self.assertEqual(source.read_bytes(), original)
            import mujoco
            model = mujoco.MjModel.from_xml_path(str(target))
            self.assertGreater(model.nq, 0)


if __name__ == "__main__":
    unittest.main()
