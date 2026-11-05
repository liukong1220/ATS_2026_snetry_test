#!/usr/bin/env python3
"""S3 identity wiring must survive every production include boundary."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT / "src/ats_sentry_nav/ats_nav_bringup/launch"


class IdentityRoute(unittest.TestCase):
    def test_2027_include_preserves_explicit_scene_and_saved_map_inputs(self):
        import importlib.util
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
        from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

        source = ROOT / "src/sim/ats_mujoco_sim/launch/rmuc_2027_mujoco.launch.py"
        spec = importlib.util.spec_from_file_location("s3_2027_inputs", source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        inputs = {
            "model_file": "/tmp/s3-explicit-scene.xml",
            "map_yaml_file": "/tmp/s3-saved-map.yaml",
            "prior_pcd_file": "/tmp/s3-saved-map.pcd",
            "scene_navigation_profile": "/tmp/s3-scene-profile.yaml",
        }
        for overrides in ({}, inputs):
            with self.subTest(overrides=bool(overrides)):
                context = LaunchContext()
                context.launch_configurations.update(overrides)
                description = module.generate_launch_description()
                for action in description.entities:
                    if isinstance(action, DeclareLaunchArgument):
                        action.execute(context)
                include = next(action for action in description.entities
                               if isinstance(action, IncludeLaunchDescription))
                effective = {perform_substitutions(context, normalize_to_list_of_substitutions(key)):
                             perform_substitutions(context, normalize_to_list_of_substitutions(value))
                             for key, value in include.launch_arguments}
                if overrides:
                    for name, expected in inputs.items():
                        self.assertEqual(effective[name], expected, name)
                else:
                    self.assertEqual(effective["model_file"], "rmuc_2027_swerve.xml")
                    self.assertTrue(effective["map_yaml_file"].endswith("/map/rmuc_2027.yaml"))
                    self.assertTrue(effective["prior_pcd_file"].endswith("/pcd/rmuc_2027.pcd"))
                    self.assertTrue(effective["scene_navigation_profile"].endswith(
                        "/config/rmuc_2027_navigation.yaml"))

    def test_mapping_terminal_admission_and_control_fit_authoritative_success_region(self):
        import importlib.util
        import yaml
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument, TimerAction
        from launch_ros.actions import Node
        from launch_ros.utilities import evaluate_parameters

        def nodes(entities):
            for entity in entities:
                if isinstance(entity, Node):
                    yield entity
                else:
                    children = entity.actions if isinstance(entity, TimerAction) else (
                        entity.entities if hasattr(entity, "entities") else entity.get_sub_entities())
                    yield from nodes(children)

        for source, mode_key in [(NAV / "navigation_launch.py", "mapping_mode"),
                                 (ROOT / "src/sim/ats_mujoco_sim/launch/rmuc_2025_mujoco.launch.py", "slam")]:
            with self.subTest(source=source):
                spec = importlib.util.spec_from_file_location("s3_terminal_launch", source)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                description = module.generate_launch_description()
                context = LaunchContext()
                context.launch_configurations.update({
                    "params_file": str(ROOT / "src/ats_sentry_bringup/params/node_params.yaml"),
                    mode_key: "true",
                })
                for action in description.entities:
                    if isinstance(action, DeclareLaunchArgument):
                        action.execute(context)
                effective = {}
                for node in nodes(description.entities):
                    package = node.node_package
                    if package not in ("ats_goal_manager", "minco_planner", "ats_swerve_mpc"):
                        continue
                    values = {}
                    for item in evaluate_parameters(context, node._Node__parameters):
                        if isinstance(item, dict):
                            values.update(item)
                        else:
                            content = yaml.safe_load(Path(item).read_text())
                            values.update(content.get(package, {}).get("ros__parameters", {}))
                    effective[package] = values
                manager, planner, controller = (effective[k] for k in
                    ("ats_goal_manager", "minco_planner", "ats_swerve_mpc"))
                # A controller may stop anywhere in its terminal region. Even
                # its far edge must satisfy the original map goal after MINCO
                # admits a shifted endpoint (domain142: 0.045 + 0.15 rad did not).
                position_budget = (planner.get("goal_admission_position_tolerance", 0.08) *
                                   planner.get("goal_admission_position_shrink", 0.75) +
                                   controller["goal_position_tolerance"])
                yaw_budget = (planner.get("goal_admission_yaw_tolerance", 0.15) *
                              planner.get("goal_admission_yaw_shrink", 0.60) +
                              controller["goal_yaw_tolerance"])
                self.assertLess(position_budget, manager["goal_position_tolerance"])
                self.assertLess(yaw_budget, manager["goal_yaw_tolerance"])
                self.assertEqual(manager["goal_position_tolerance"], 0.03)
                self.assertEqual(manager["goal_yaw_tolerance"], 0.15)

    def test_mapping_entry_explicitly_enables_the_verified_identity_route(self):
        command = next(line for line in (ROOT / "mapping.sh").read_text().splitlines()
                       if "ros2 launch ats_sentry_bringup bringup.launch.py" in line)
        self.assertIn("slam:=True", command)
        self.assertIn("use_identified_odometry:=True", command)

    def test_lio_data_window_covers_ground_below_the_initial_pose(self):
        import importlib.util
        import yaml
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument, TimerAction
        from launch_ros.actions import Node
        from launch_ros.utilities import evaluate_parameters

        source = ROOT / "src/sim/ats_mujoco_sim/launch/rmuc_2025_mujoco.launch.py"
        spec = importlib.util.spec_from_file_location("s3_field_launch", source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        def nodes(entities):
            for entity in entities:
                if isinstance(entity, Node):
                    yield entity
                else:
                    if isinstance(entity, TimerAction):
                        children = entity.actions
                    elif hasattr(entity, "entities"):
                        children = entity.entities
                    else:
                        children = entity.get_sub_entities()
                    yield from nodes(children)

        def parameters(mode):
            # ParameterFile caches its evaluated substitution, as in a real
            # launch, so each deployment needs a fresh description.
            description = module.generate_launch_description()
            context = LaunchContext()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            context.launch_configurations["localization_source"] = mode
            rog = next(node for node in nodes(description.entities)
                       if node.node_package == "ats_rog_map")
            result = {}
            for item in evaluate_parameters(context, rog._Node__parameters):
                if isinstance(item, dict):
                    result.update(item)
                else:
                    content = yaml.safe_load(Path(item).read_text())
                    result.update(content.get("ats_rog_map", {}).get("ros__parameters", {}))
            return result

        lio = parameters("lio")
        # The real 2025 descent reaches sensor z=0.09 in odom. Inflation
        # moves the virtual floor inward by ~0.25 m, so raw bounds must
        # also retain floor returns at z=-0.20, not just the sensor origin.
        self.assertLessEqual(lio["core.virtual_ground_height"], -0.50)
        self.assertLessEqual(lio["core.fix_map_origin"][2] - lio["core.map_size"][2] / 2, -0.50)
        self.assertGreaterEqual(lio["core.virtual_ceil_height"], 1.50)
        truth = parameters("truth")
        self.assertEqual(truth["core.virtual_ground_height"], -0.10)
        self.assertEqual(truth["core.fix_map_origin"], [0.0, 0.0, 0.90])

    def test_mapping_enables_bounded_pointlio_pcd_segments(self):
        source = (NAV / "slam_launch.py").read_text()
        self.assertIn('"pcd_save.interval": 200', source)
        simulation = (ROOT / "src/sim/ats_mujoco_sim/ats_mujoco_sim/lio_launch.py").read_text()
        self.assertIn('"pcd_save.interval": 200', simulation)

    def test_fusion_start_is_independent_of_prior_loading(self):
        tree = ast.parse((NAV / "localization_launch.py").read_text())
        assignment = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                          and any(isinstance(t, ast.Name) and t.id == "fusion" for t in n.targets))
        condition = next(k.value for k in assignment.value.keywords if k.arg == "condition")
        self.assertNotIn("launch_small_gicp_relocalization", ast.unparse(condition))

    def test_mapping_mode_is_explicit_at_adapter_and_matcher(self):
        for path in [ROOT / "src/ats_sentry_bringup/launch/bringup.launch.py",
                     ROOT / "src/sim/ats_mujoco_sim/launch/rmuc_2025_mujoco.launch.py"]:
            source = path.read_text()
            self.assertIn('"mapping_mode":', source)
            self.assertIn('"allow_missing_static_map":', source)
        self.assertIn('"mapping_mode":', (NAV / "localization_launch.py").read_text())

    def test_every_include_declares_and_forwards_the_switch(self):
        for path in [
            ROOT / "src/ats_sentry_bringup/launch/bringup.launch.py",
            NAV / "rm_navigation_reality_launch.py",
            NAV / "bringup_launch.py",
        ]:
            with self.subTest(path=path):
                tree = ast.parse(path.read_text())
                declarations = [
                    n
                    for n in ast.walk(tree)
                    if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Name)
                    and n.func.id == "DeclareLaunchArgument"
                    and n.args
                    and isinstance(n.args[0], ast.Constant)
                    and n.args[0].value == "use_identified_odometry"
                ]
                self.assertEqual(len(declarations), 1)
                forwarded = [
                    n
                    for n in ast.walk(tree)
                    if isinstance(n, ast.Dict)
                    and any(
                        isinstance(k, ast.Constant)
                        and k.value == "use_identified_odometry"
                        for k in n.keys
                    )
                ]
                self.assertTrue(forwarded)

    def test_fusion_monitors_raw_frontend_not_converter_output(self):
        source = (NAV / "localization_launch.py").read_text()
        self.assertIn(
            '"identity_monitor_topic": "aft_mapped_to_init_identified"', source
        )
        self.assertIn('"identified_odom_topic": "odometry_identified"', source)

    def test_simulation_switch_is_available_to_both_components(self):
        source = (
            ROOT / "src/sim/ats_mujoco_sim/ats_mujoco_sim/lio_launch.py"
        ).read_text()
        self.assertGreaterEqual(source.count('"use_identified_odometry":'), 2)
        fusion = (
            ROOT / "src/sim/ats_mujoco_sim/launch/rmuc_2025_mujoco.launch.py"
        ).read_text()
        self.assertIn(
            '"identity_monitor_topic": "/aft_mapped_to_init_identified"', fusion
        )
        self.assertIn('"identified_odom_topic": "/odometry_identified"', fusion)


if __name__ == "__main__":
    unittest.main()
