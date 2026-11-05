#!/usr/bin/env python3
"""Explicit S3 no-prior LIO experiment; raw evidence stays outside source trees.

Run after sourcing install/setup.bash. Navigation only receives ROS goals.
Ground truth is consumed exclusively by this evaluator.
"""
import argparse
import csv
import hashlib
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "src/sim/ats_mujoco_sim"


def route_metrics(path, endpoints, minimum_displacement):
    length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    errors = [math.dist(path[-1], endpoint) for endpoint in endpoints[-1:]] if path else []
    return {
        "path_length_m": length,
        "final_error_m": errors[-1] if errors else None,
        "passed": bool(errors) and length >= minimum_displacement and max(errors) <= 0.08,
    }


def route_deviation(path, waypoints):
    """Distance from each independent truth sample to the configured polyline."""
    import numpy as np
    distances = []
    for point in path:
        best = math.inf
        for start, end in zip(waypoints, waypoints[1:]):
            dx, dy = end[0] - start[0], end[1] - start[1]
            norm = dx * dx + dy * dy
            t = min(1.0, max(0.0, ((point[0] - start[0]) * dx +
                                   (point[1] - start[1]) * dy) / norm)) if norm else 0.0
            best = min(best, math.hypot(point[0] - start[0] - t * dx,
                                       point[1] - start[1] - t * dy))
        if math.isfinite(best):
            distances.append(best)
    return {"samples": len(distances),
            "mean_m": float(np.mean(distances)) if distances else None,
            "p95_m": float(np.percentile(distances, 95)) if distances else None,
            "maximum_m": max(distances) if distances else None}


def verify_installed_files(pairs):
    for source, installed in pairs:
        if not installed.is_file() or source.read_bytes() != installed.read_bytes():
            raise RuntimeError(f"stale installed file {installed}; rebuild its owning package")


def configure_mapping_bounds(parameters, profile):
    if "odom_layer_origin" not in profile:
        return
    x, y = map(float, profile["odom_layer_origin"])
    layer = parameters["ats_rog_map"]["ros__parameters"]
    layer.update({"odom_layer.origin_x": x, "odom_layer.origin_y": y})
    # The registration target must retain the same field as the global layer.
    # A centered [-20,20) target clips the far end of the 28 m field when the
    # mapping start is near its western boundary. Bounds use initial odom axes.
    parameters["small_gicp_relocalization"]["ros__parameters"]["mapping.origin"] = [x, y, -10.0]


def make_test_scene(source, target, size, test_obstacle=False, test_barrier=False):
    """Reuse the production robot; replace field geometry with explicit test walls."""
    # MuJoCo accepts comments containing CLI options with '--'; XML's strict
    # parser does not. Comments carry no model data and source stays untouched.
    tree = ET.ElementTree(ET.fromstring(re.sub(r"<!--.*?-->", "", source.read_text(), flags=re.S)))
    root = tree.getroot()
    root.set("model", "s3_mapping_10x10")
    asset = root.find("asset")
    for element in list(asset):
        if element.tag == "hfield" or element.get("name", "").startswith("rmuc_2027"):
            asset.remove(element)
        elif "file" in element.attrib:
            element.set("file", str((source.parent / element.get("file")).resolve()))
    world = root.find("worldbody")
    for element in list(world):
        if element.tag != "light" and not (element.tag == "body" and element.get("name") == "base_link"):
            world.remove(element)
    for include in list(root.findall("include")):
        root.remove(include)
    ET.SubElement(world, "geom", name="s3_floor", type="plane", pos="0 0 0",
                  size=f"{size / 2} {size / 2} 0.05", material="grid", group="2")
    h = size / 2
    boxes = [(-h, 0.0, 0.05, h), (h, 0.0, 0.05, h),
             (0.0, -h, h, 0.05), (0.0, h, h, 0.05),
             (2.0, 2.0, 0.25, 0.60), (-2.0, 2.6, 0.40, 0.20)]
    for index, (x, y, dx, dy) in enumerate(boxes):
        ET.SubElement(world, "geom", name=f"s3_wall_{index}", type="box",
                      pos=f"{x} {y} 0.5", size=f"{dx} {dy} 0.5",
                      rgba="0.6 0.6 0.6 1", group="2")
    if test_obstacle:
        body = ET.SubElement(world, "body", name="s3_test_obstacle", pos="2 0 -10")
        ET.SubElement(body, "freejoint", name="s3_test_obstacle_joint")
        ET.SubElement(body, "geom", name="s3_test_obstacle_geom", type="box",
                      size="0.3 0.3 0.3", mass="1", group="2", rgba="1 0.2 0.1 1")
    if test_barrier:
        ET.SubElement(world, "geom", name="s3_no_path_barrier", type="box", pos="1.5 0 0.125",
                      size="0.15 4.95 0.125", group="2", rgba="0.7 0.2 0.1 1")
    target.parent.mkdir(parents=True, exist_ok=True)
    mesh_link = target.parent / "meshes/mid360.obj"
    mesh_link.parent.mkdir(exist_ok=True)
    if not mesh_link.exists():
        mesh_link.symlink_to((source.parent / "meshes/mid360.obj").resolve())
    tree.write(target, encoding="unicode")
    return boxes


def prepare_scene(profile, directory, fixture=False, barrier=False):
    """Keep full-field geometry intact; only explicit small profiles replace it."""
    extra = []
    source = SIM / "models" / profile["launch"].replace("_mujoco.launch.py", "_swerve.xml")
    scene = source
    if "generated_size" in profile:
        scene = directory / "scene.xml"
        truth = make_test_scene(SIM / "models/rmuc_2027_swerve.xml", scene,
                                profile["generated_size"], test_obstacle=fixture,
                                test_barrier=barrier)
        (directory / "truth_walls.json").write_text(json.dumps(truth) + "\n")
        scene_parameters = {"ats_rog_map": {"ros__parameters": {
            "core.map_size": [12.0, 12.0, 3.0], "core.fix_map_origin": [0.0, 0.0, 0.5],
            "core.virtual_ground_height": -0.5, "core.virtual_ceil_height": 2.0}}}
        scene_profile = directory / "scene_parameters.yaml"
        scene_profile.write_text(yaml.safe_dump(scene_parameters))
        extra.append(f"scene_navigation_profile:={scene_profile}")
    elif barrier:
        raise ValueError("the barrier experiment requires the explicit small_10x10 profile")
    elif fixture:
        from s3_fixture_scene import make_fixture_scene
        scene = directory / "scene.xml"
        provenance = make_fixture_scene(source, scene)
        (directory / "scene-provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    if scene != source:
        extra.append(f"model_file:={scene}")
    if fixture:
        extra.append("enable_test_obstacle_fixture:=true")
    return extra, scene


def stamp(message):
    return message.sec + message.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe:
    def __init__(self, directory, observe_obstacle_clouds=False):
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        from rclpy.action import ActionClient
        from ats_navigation_interfaces.action import NavigateToPose
        from ats_navigation_interfaces.msg import ExecutionCommand, LocalizationStatus, PlanningMapStatus, PlannerGoal, PlannerStatus, RelocalizationObservation
        from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMessage
        from geometry_msgs.msg import Twist
        from manda_can_control.msg import MotionCtrl
        from std_msgs.msg import Bool
        from tf2_ros import Buffer, TransformListener

        self.rclpy = rclpy
        self.node = rclpy.create_node("s3_acceptance_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.directory = directory
        self.events = (directory / "events.jsonl").open("w", buffering=1)
        self.truth_file = (directory / "truth.csv").open("w", buffering=1)
        self.truth_csv = csv.writer(self.truth_file)
        self.truth_csv.writerow(["wall_s", "sim_s", "x", "y", "z", "yaw", "vx", "vy", "wz"])
        self.path, self.latest, self.counts, self.goals, self.planner_statuses = [], {}, {}, [], []
        self.endpoint_results = []
        self.history = {}
        self.execution_events = []
        self.last_execute = None
        self.obstacle_clouds = []
        self.fixture_site = None
        self.scene_model = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.action = ActionClient(self.node, NavigateToPose, "/ats_navigate_to_pose")
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = []
        topics = [
            (Odometry, "/simulation/ground_truth/odometry", qos_profile_sensor_data, self.on_truth),
            (Odometry, "/localization", qos_profile_sensor_data, None),
            (LocalizationStatus, "/localization/status", durable, None),
            (RelocalizationObservation, "/relocalization_observation", 10, None),
            (OccupancyGrid, "/rc_esdf/planning_grid", durable, None),
            (OccupancyGrid, "/rog_map/odom_layer", durable, None),
            (PlanningMapStatus, "/rog_map_adapter/status", durable, None),
            (Bool, "/rog_map_adapter/ready", durable, None),
            (Bool, "/planner/emergency_stop", durable, None),
            (PlannerGoal, "/ats_goal_manager/planner_goal", 10, self.on_goal),
            (PlannerStatus, "/minco/planning_status", 10, self.on_planner_status),
            (ExecutionCommand, "/planner/execution_command", durable, self.on_execution),
            (Twist, "/cmd_vel/autonomy_raw", 10, None),
            (Twist, "/cmd_vel/selected", 10, None),
            (MotionCtrl, "/motion_control", 10, None),
            (PathMessage, "/minco/raw_path", 10, None),
            (PathMessage, "/minco/reference_path", 10, None),
            (PathMessage, "/ats_swerve_mpc/reference_path", 10, None),
            (PathMessage, "/ats_swerve_mpc/predicted_path", 10, None),
        ]
        for kind, topic, qos, callback in topics:
            def receive(msg, topic=topic, callback=callback):
                received = time.monotonic()
                self.latest[topic] = (msg, received)
                self.counts[topic] = self.counts.get(topic, 0) + 1
                if topic in ("/cmd_vel/autonomy_raw", "/cmd_vel/selected", "/motion_control"):
                    values = ([msg.linear_x, msg.linear_y, msg.angular_z] if topic == "/motion_control"
                              else [msg.linear.x, msg.linear.y, msg.angular.z])
                    self.history.setdefault(topic, []).append((received, values))
                    self.event(topic, {"velocity": values})
                elif topic == "/rog_map_adapter/ready":
                    self.history.setdefault(topic, []).append((received, msg.data))
                    self.event(topic, {"ready": msg.data})
                elif topic == "/rc_esdf/planning_grid":
                    self.history.setdefault(topic, []).append((received, msg))
                    # Retain enough transitions for fault evidence without keeping
                    # every full grid throughout a long field coverage route.
                    self.history[topic] = self.history[topic][-40:]
                elif topic == "/localization":
                    self.history.setdefault(topic, []).append((received, msg))
                if callback:
                    callback(msg)
                elif topic in ("/localization/status", "/rog_map_adapter/status", "/planner/emergency_stop",
                               "/relocalization_observation"):
                    self.event(topic, self.message_dict(msg))
            self.subscriptions.append(self.node.create_subscription(kind, topic, receive, qos))
        if observe_obstacle_clouds:
            from sensor_msgs.msg import PointCloud2
            self.subscriptions.append(self.node.create_subscription(
                PointCloud2, "/registered_scan", self.on_obstacle_cloud, qos_profile_sensor_data))

    def on_obstacle_cloud(self, msg):
        import numpy as np
        if self.fixture_site is None:
            return
        received_ns = time.monotonic_ns()
        offsets = {f.name: f.offset for f in msg.fields}
        count = msg.width * msg.height
        xyz = [np.ndarray((count,), dtype=(">f4" if msg.is_bigendian else "<f4"),
                          buffer=msg.data, offset=offsets[name], strides=(msg.point_step,))
               for name in ("x", "y", "z")]
        center = self.fixture_site["odom_position"]
        extent = self.fixture_site["odom_half_extent"]
        mask = ((np.abs(xyz[0] - center[0]) < extent[0] + .15) &
                (np.abs(xyz[1] - center[1]) < extent[1] + .15) &
                (xyz[2] > center[2] - extent[2] + .15) &
                (xyz[2] < center[2] + extent[2] + .15))
        hits = int(mask.sum())
        record = {"received_ns": received_ns, "stamp_ns": msg.header.stamp.sec * 10**9 +
                  msg.header.stamp.nanosec, "obstacle_points": hits}
        if hits:
            record["points"] = np.column_stack(xyz)[mask].tolist()
        self.obstacle_clouds.append(record)
        self.event("registered_scan_obstacle", record)

    @staticmethod
    def message_dict(msg):
        from rosidl_runtime_py.convert import message_to_ordereddict
        return message_to_ordereddict(msg)

    def event(self, kind, data):
        self.events.write(json.dumps({"wall_s": time.monotonic(), "sim_s": self.sim_time(),
                                      "event": kind, "data": data}) + "\n")

    def sim_time(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg):
        p, v = msg.pose.pose, msg.twist.twist
        self.path.append((p.position.x, p.position.y))
        self.truth_csv.writerow([time.monotonic(), stamp(msg.header.stamp), p.position.x,
                                p.position.y, p.position.z, yaw(p.orientation), v.linear.x,
                                v.linear.y, v.angular.z])

    def on_goal(self, msg):
        accepted = self.latest.get("/localization/status")
        accepted = bool(accepted and stamp(accepted[0].last_observation_stamp) > 0)
        self.goals.append((msg, accepted))
        self.event("planner_goal", {"accepted_before_dispatch": accepted, **self.message_dict(msg)})

    def on_planner_status(self, msg):
        self.planner_statuses.append(msg)
        self.event("planner_status", self.message_dict(msg))

    def on_execution(self, msg):
        if msg.mode == msg.MODE_EXECUTE:
            self.last_execute = msg
        record = {"mode": msg.mode, "manager": msg.manager_incarnation,
                  "sequence": msg.command_sequence, "goal": msg.goal_id,
                  "epoch": msg.localization_epoch, "map_generation": msg.map_generation,
                  "reference_stamp": stamp(msg.reference.header.stamp),
                  "reference_points": len(msg.reference.poses)}
        self.execution_events.append((time.monotonic(), record))
        self.event("execution_command", record)

    def spin_until(self, predicate, wall_timeout, label):
        end = time.monotonic() + wall_timeout
        while time.monotonic() < end:
            if predicate():
                return True
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        self.event("timeout", {"label": label, "latest": self.health_summary()})
        return False

    def health_summary(self):
        return {topic: self.message_dict(self.latest[topic][0]) for topic in
                ("/localization/status", "/rog_map_adapter/status", "/planner/emergency_stop")
                if topic in self.latest}

    def ready(self):
        loc = self.latest.get("/localization/status")
        map_status = self.latest.get("/rog_map_adapter/status")
        grid = self.latest.get("/rc_esdf/planning_grid")
        return bool(loc and map_status and grid and stamp(loc[0].last_observation_stamp) > 0 and
                    map_status[0].ready and loc[0].epoch == map_status[0].localization_epoch and
                    grid[0].header.frame_id == "odom" and time.monotonic() - map_status[1] < 5.0)

    def graph(self):
        result = {}
        for topic in ("/rc_esdf/planning_grid", "/cmd_vel/autonomy_raw",
                      "/cmd_vel/selected", "/motion_control", "/simulation/ground_truth/odometry"):
            result[topic] = {
                "publishers": [x.node_name for x in self.node.get_publishers_info_by_topic(topic)],
                "subscribers": [x.node_name for x in self.node.get_subscriptions_info_by_topic(topic)],
            }
        (self.directory / "graph.json").write_text(json.dumps(result, indent=2) + "\n")
        return result

    def navigate(self, waypoint, timeout):
        from ats_navigation_interfaces.action import NavigateToPose
        self.action.wait_for_server(timeout_sec=10.0)
        goal = NavigateToPose.Goal()
        goal.goal_pose.header.frame_id = "map"
        goal.goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y = waypoint[:2]
        goal.goal_pose.pose.orientation.z = math.sin(waypoint[2] / 2)
        goal.goal_pose.pose.orientation.w = math.cos(waypoint[2] / 2)
        goal.timeout.sec = int(timeout)
        self.event("send_goal", {"waypoint": waypoint})
        future = self.action.send_goal_async(goal)
        if not self.spin_until(future.done, 10.0, "goal acknowledgement") or not future.result().accepted:
            raise RuntimeError("goal was not accepted")
        handle = future.result()
        self.finish_goal(handle, waypoint, timeout)

    def finish_goal(self, handle, waypoint, timeout):
        from ats_navigation_interfaces.action import NavigateToPose
        result = handle.get_result_async()
        if not self.spin_until(result.done, timeout * 10.0 + 30.0, "navigation result"):
            handle.cancel_goal_async()
            raise RuntimeError("navigation wall timeout")
        response = result.result().result
        truth = self.latest["/simulation/ground_truth/odometry"][0]
        p = truth.pose.pose.position
        error = math.hypot(p.x - waypoint[0], p.y - waypoint[1])
        record = {"goal": waypoint, "result_code": response.result_code, "message": response.message,
                  "truth": [p.x, p.y, p.z], "truth_error_m": error,
                  "estimated_error_m": response.final_distance, "sim_s": self.sim_time()}
        self.endpoint_results.append(record)
        self.event("goal_result", record)
        print(json.dumps(record), flush=True)
        self.save_grid(f"goal_{len(self.endpoint_results)}")
        if response.result_code != NavigateToPose.Result.RESULT_SUCCEEDED or error > 0.08:
            raise RuntimeError(f"goal failed independent truth gate: {record}")

    def save_grid(self, name):
        import numpy as np
        for topic in ("/rc_esdf/planning_grid", "/rog_map/odom_layer"):
            if topic not in self.latest:
                continue
            grid = self.latest[topic][0]
            prefix = self.directory / f"{name}_{topic.rsplit('/', 1)[1]}"
            np.save(str(prefix) + ".npy", np.asarray(grid.data, dtype=np.int8))
            (Path(str(prefix) + ".json")).write_text(json.dumps({
                "header": self.message_dict(grid.header), "info": self.message_dict(grid.info),
                "sha256": hashlib.sha256(bytes((v % 256 for v in grid.data))).hexdigest()}) + "\n")

    def save_map(self, name="map"):
        from ats_rog_map_interfaces.srv import SaveOdomMap
        from save_s3_map import pcd_points, pgm_occupied_pixels
        client = self.node.create_client(SaveOdomMap, "/rog_map/save_map")
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("save service unavailable")
        prefix = self.directory / name
        future = client.call_async(SaveOdomMap.Request(output_prefix=str(prefix)))
        if not self.spin_until(future.done, 30.0, "explicit map save"):
            client.remove_pending_request(future)
            raise RuntimeError("save service timeout")
        result = future.result()
        self.event("save_map", self.message_dict(result))
        if not result.success or result.persistent_voxels == 0:
            raise RuntimeError(f"save failed: {result.message}")
        assert pcd_points(Path(str(prefix) + ".pcd")) == result.persistent_voxels
        assert pcd_points(Path(str(prefix) + ".cells.pcd")) == result.persistent_cells
        assert pgm_occupied_pixels(Path(str(prefix) + ".pgm")) == result.raster_occupied_cells
        return self.message_dict(result)

    def close(self):
        self.save_grid("final")
        self.events.close()
        self.truth_file.close()
        self.node.destroy_node()


def run(args):
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    sim_share = Path(get_package_share_directory("ats_mujoco_sim"))
    verify_installed_files([
        (SIM / "launch/rmuc_2025_mujoco.launch.py", sim_share / "launch/rmuc_2025_mujoco.launch.py"),
        (SIM / "launch/rmuc_2027_mujoco.launch.py", sim_share / "launch/rmuc_2027_mujoco.launch.py"),
        (SIM / "config/s3_mapping_routes.yaml", sim_share / "config/s3_mapping_routes.yaml"),
    ])
    profile = yaml.safe_load((SIM / "config/s3_mapping_routes.yaml").read_text())[args.profile]
    directory = Path(args.output).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    parameters = yaml.safe_load((ROOT / "src/ats_sentry_bringup/params/node_params.yaml").read_text())
    configure_mapping_bounds(parameters, profile)
    fixture_case = args.case in ("obstacle_latency", "dynamic_save", "unsafe") or bool(args.post_route_case)
    if fixture_case:
        parameters["minco_planner"]["ros__parameters"]["map_commit_trace_enabled"] = True
        parameters["ats_rog_map"]["ros__parameters"]["map_update_trace_enabled"] = True
    extra, scene = prepare_scene(profile, directory, fixture=fixture_case, barrier=args.case == "no_path")
    # service_timeout stalls the in-process projection through a guarded test
    # parameter, so it needs the same startup authorization as unknown.
    if args.case in ("unknown", "service_timeout"):
        extra.append("enable_test_fault_injection:=true")
    parameter_path = directory / "params.yaml"
    parameter_path.write_text(yaml.safe_dump(parameters, sort_keys=False))
    start = profile["start"]
    command = ["ros2", "launch", "ats_mujoco_sim", profile["launch"],
               "localization_source:=lio", f"slam:={'false' if args.reload else 'true'}",
               "use_identified_odometry:=true", "planning_grid_owner:=rog_map",
               "use_viewer:=false", "show_viewer:=false", "use_rviz:=false",
               "launch_mujoco_rviz:=false", "lidar_threads:=8", "lidar_downsample:=1",
               "nav_start_delay_sec:=4.0", "rog_map_start_delay_sec:=8.0",
               f"params_file:={parameter_path}",
               *[f"{key}:={val}" for key, val in zip(("start_x", "start_y", "start_z", "start_yaw"), start)],
               *extra]
    if args.reload:
        prefix = Path(args.reload).resolve()
        command += [f"map_yaml_file:={prefix}.yaml", f"prior_pcd_file:={prefix}.pcd"]
    else:
        command += [f"map_yaml_file:={directory}/absent.yaml", f"prior_pcd_file:={directory}/absent.pcd"]
    command += args.launch_arg
    (directory / "command.json").write_text(json.dumps({"command": command,
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"), "profile": profile}, indent=2) + "\n")
    print("COMMAND", " ".join(command), flush=True)
    log = (directory / "launch.log").open("w")
    process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
    rclpy.init()
    probe = Probe(directory, observe_obstacle_clouds=fixture_case)
    probe.scene_model = scene
    summary = {"profile": args.profile, "reload": args.reload, "passed": False}
    try:
        if args.case.startswith("correction_"):
            from s3_fault_scenarios import run_case
            summary.update(run_case(args.case, probe, process))
            summary["scope"] = args.case
            return
        if not probe.spin_until(probe.ready, args.startup_timeout, "accepted localization and odom planning map"):
            raise RuntimeError("startup failed: " + json.dumps(probe.health_summary()))
        graph = probe.graph()
        assert graph["/rc_esdf/planning_grid"]["publishers"] == ["ats_rog_map_adapter"], graph
        summary["A"] = True
        print("A ready; graph", json.dumps(graph), flush=True)
        if args.case != "route":
            from s3_fault_scenarios import run_case
            summary.update(run_case(args.case, probe, process))
            summary["scope"] = args.case
            summary["topic_counts"] = probe.counts
            print(json.dumps(summary, indent=2), flush=True)
            return
        waypoints = profile["waypoints"][:args.max_goals or None]
        if args.reload:
            waypoints = waypoints[:1]
        for point in waypoints:
            probe.navigate(point, profile["goal_timeout_sim_s"])
        if not args.reload:
            summary["save"] = probe.save_map()
        minimum = 0.2 if args.reload or args.max_goals else profile["minimum_displacement_m"]
        summary.update(route_metrics(probe.path, [waypoints[-1][:2]], minimum))
        summary["J"] = bool(probe.goals) and all(accepted for _, accepted in probe.goals)
        summary["endpoints"] = probe.endpoint_results
        summary["route_deviation"] = route_deviation(probe.path, [profile["start"][:2]] +
                                                     [p[:2] for p in waypoints])
        summary["route_completed_ratio"] = len(probe.endpoint_results) / len(waypoints)
        summary["topic_counts"] = probe.counts
        summary["footprint_gate_collisions"] = sum(s.gate_collision_count for s in probe.planner_statuses
                                                     if s.state == s.STATE_REFERENCE_READY)
        summary["passed"] = bool(summary["passed"] and summary["J"] and
                                  summary["footprint_gate_collisions"] == 0)
        if not summary["passed"]:
            raise RuntimeError("route evidence gates failed")
        if args.post_route_case:
            from s3_fault_scenarios import run_case
            cases = (("obstacle_latency", "dynamic_save") if args.post_route_case == "obstacle_and_dynamic"
                     else (args.post_route_case,))
            for case in cases:
                evidence = run_case(case, probe, process)
                summary[case] = evidence
                if not evidence["passed"]:
                    raise RuntimeError(f"post-route experiment failed: {case}")
    except BaseException as error:
        summary["passed"] = False
        summary["error"] = type(error).__name__ + ": " + str(error)
        summary["health"] = probe.health_summary()
        summary["endpoints"] = probe.endpoint_results
        raise
    finally:
        (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        probe.close()
        if rclpy.ok():
            rclpy.shutdown()
        # This process group was created here, with the selected isolated domain.
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10.0)
        log.close()
    print(json.dumps(summary, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, choices=("small_10x10", "rmuc_2025", "rmuc_2027"))
    parser.add_argument("--output", required=True)
    parser.add_argument("--reload", help="saved map prefix, without suffix")
    parser.add_argument("--max-goals", type=int, default=0)
    parser.add_argument("--post-route-case", choices=("obstacle_latency", "dynamic_save", "obstacle_and_dynamic"),
                        help="run physical fixture evidence after completing the selected route")
    parser.add_argument("--case", choices=("route", "point_lio_restart", "obstacle_latency",
                                          "dynamic_save", "unsafe", "service_timeout", "unknown",
                                          "no_path", "correction_0.05", "correction_0.15",
                                          "correction_0.30"), default="route")
    parser.add_argument("--startup-timeout", type=float, default=180.0)
    parser.add_argument("--launch-arg", action="append", default=[])
    args = parser.parse_args()
    if args.post_route_case and (args.case != "route" or args.reload):
        parser.error("post-route fixtures require a mapping route")
    domain = int(os.environ.get("ROS_DOMAIN_ID", "-1"))
    if domain < 0 or domain > 232 or domain == 77:
        parser.error("choose an isolated ROS_DOMAIN_ID in 0..232, excluding 77")
    run(args)


if __name__ == "__main__":
    main()
