#!/usr/bin/env python3
"""Deterministic truth-mode equivalence probe for ats_mujoco_sim.

The closed-loop MuJoCo runner paces physics on the wall clock, so two runs are
never bit-identical.  This probe removes that: it builds the real
``SwerveMujocoSim`` node, stops its physics thread, then steps the model a fixed
number of times with a scripted command and hashes the state and the truth
messages that ``_publish_truth`` would publish.  Run it once against the old
tree and once against the new tree; identical digests mean the truth path is
unchanged.

  PYTHONPATH=<tree> python3 scripts/truth_equivalence_probe.py \
      --models <tree>/models --out /tmp/probe_<name>.json
"""

import argparse
import hashlib
import json
from pathlib import Path
import time


def digest(array):
    import numpy as np

    return hashlib.sha256(np.ascontiguousarray(array, dtype=np.float64).tobytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--models", required=True, help="models directory of the tree under test")
    parser.add_argument("--out", required=True)
    parser.add_argument("--steps", type=int, default=3000)
    parser.add_argument("--sim-rate", type=float, default=300.0)
    parser.add_argument("--extra-param", action="append", default=[],
                        help="name=value passed to the node as a parameter override")
    args = parser.parse_args()

    import numpy as np
    import rclpy
    from rclpy.parameter import Parameter

    from ats_mujoco_sim import sim_node as sim_module
    from ats_mujoco_sim.kinematics import ChassisCommand

    model_path = str(Path(args.models) / "rmuc_2027_swerve.xml")
    overrides = [
        Parameter("model_path", value=model_path),
        Parameter("scene_file", value=model_path),
        Parameter("enable_static_walls", value=False),
        Parameter("use_viewer", value=False),
        Parameter("show_viewer", value=False),
        Parameter("enable_lidar", value=False),
        Parameter("enable_tof", value=False),
        Parameter("sim_rate_hz", value=args.sim_rate),
        Parameter("start_x", value=-0.18),
        Parameter("start_y", value=0.06),
        Parameter("start_z", value=0.381),
        Parameter("start_yaw", value=0.0),
        Parameter("publish_map_to_odom_tf", value=False),
    ]
    for item in args.extra_param:
        name, raw = item.split("=", 1)
        try:
            value = json.loads(raw)
        except json.JSONDecodeError:
            value = raw
        overrides.append(Parameter(name, value=value))

    rclpy.init()
    # The node starts its physics thread at the end of __init__, and that thread
    # steps an unpredictable number of times before it could be stopped.  Never
    # start it, so every run begins from the exact initial state.
    sim_module.SwerveMujocoSim._start_simulation_thread = lambda self: None
    sim_module.Node.__init__ = _wrap_init(sim_module.Node.__init__, overrides)
    node = sim_module.SwerveMujocoSim()

    captured = {"odom": [], "tf": []}
    if node.localization_pub is not None:
        node.localization_pub.publish = captured["odom"].append
    node.tf_broadcaster.sendTransform = lambda transforms: captured["tf"].append(list(transforms))

    script = [
        (0, ChassisCommand(0.0, 0.0, 0.0)),
        (300, ChassisCommand(1.0, 0.0, 0.0)),
        (1200, ChassisCommand(0.5, 0.5, 1.0)),
        (2100, ChassisCommand(0.0, -0.8, -2.0)),
        (2700, ChassisCommand(0.0, 0.0, 0.0)),
    ]
    script_index = 0
    clock_now = getattr(node, "_clock_now", time.monotonic)
    qpos_hashes = []
    for step in range(args.steps):
        while script_index < len(script) and step >= script[script_index][0]:
            node.motion_cmd = script[script_index][1]
            script_index += 1
        node.last_motion_time = clock_now()
        with node.sim_lock:
            node._step_simulation_locked()
        if step % 30 == 0:
            qpos_hashes.append(digest(node.data.qpos))
            if not getattr(node, "lio_mode", False):
                node._publish_truth()

    def odom_vector(msg):
        pose, twist = msg.pose.pose, msg.twist.twist
        return [
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        ]

    odom = np.array([odom_vector(m) for m in captured["odom"]], dtype=np.float64)
    tf_values = [
        [t.header.frame_id, t.child_frame_id, t.transform.translation.x, t.transform.translation.y,
         t.transform.translation.z, t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w]
        for batch in captured["tf"] for t in batch
    ]
    result = {
        "steps": args.steps,
        "final_qpos": [float(v) for v in node.data.qpos[:7]],
        "qpos_digest_series": digest(np.array([int(h[:12], 16) for h in qpos_hashes], dtype=np.float64)),
        "qpos_series": qpos_hashes,
        "qpos_digest": digest(node.data.qpos),
        "qvel_digest": digest(node.data.qvel),
        "odom_messages": len(captured["odom"]),
        "odom_digest": digest(odom) if odom.size else "",
        "tf_digest": hashlib.sha256(json.dumps(tf_values, sort_keys=True).encode()).hexdigest(),
        "static_lidar_tf": [float(v) for v in node._lidar_static_pose()[0]]
        if hasattr(node, "_lidar_static_pose") else None,
    }
    Path(args.out).write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    node.stop_event.set()
    node.destroy_node()
    rclpy.shutdown()


def _wrap_init(original, overrides):
    def init(self, *args, **kwargs):
        kwargs.setdefault("parameter_overrides", overrides)
        return original(self, *args, **kwargs)

    return init


if __name__ == "__main__":
    main()
