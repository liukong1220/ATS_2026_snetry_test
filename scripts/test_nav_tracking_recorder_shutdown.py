#!/usr/bin/env python3
"""Exercise recorder readiness, runner dispatch and SIGINT with real ROS inputs."""

from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from ats_navigation_interfaces.msg import PlanningMapSnapshot

from nav_tracking_recorder import latched_qos, stream_qos


class RecorderShutdownTest(unittest.TestCase):
    def setUp(self):
        self.environment = {**os.environ, "ROS_DOMAIN_ID": os.environ.get(
            "ATS_RECORDER_TEST_DOMAIN_ID", "198")}
        self.context = rclpy.context.Context()
        self.context.init(domain_id=int(self.environment["ROS_DOMAIN_ID"]))
        self.node = rclpy.create_node("recorder_readiness_fixture", context=self.context)
        self.actual_pub = self.node.create_publisher(Odometry, "/localization", stream_qos())
        self.snapshot_pub = self.node.create_publisher(
            PlanningMapSnapshot, "/rog_map_adapter/planning_snapshot", latched_qos())
        self.tf_pub = self.node.create_publisher(TFMessage, "/tf_static", latched_qos())
        self.command_pub = self.node.create_publisher(Twist, "/cmd_vel/selected", stream_qos())
        self.enabled = set()

    def tearDown(self):
        self.node.destroy_node()
        self.context.try_shutdown()

    def publish(self):
        stamp = self.node.get_clock().now().to_msg()
        if "actual" in self.enabled:
            actual = Odometry()
            actual.header.stamp = stamp
            actual.header.frame_id = "odom"
            actual.child_frame_id = "base_link"
            actual.pose.pose.orientation.w = 1.0
            self.actual_pub.publish(actual)
            self.command_pub.publish(Twist())
        if "snapshot" in self.enabled:
            snapshot = PlanningMapSnapshot()
            snapshot.header.stamp = stamp
            snapshot.source_stamp = stamp
            snapshot.header.frame_id = "map"
            snapshot.ready = True
            snapshot.unknown_is_obstacle = True
            snapshot.occupied_value_threshold = 100
            snapshot.localization_epoch = 1
            snapshot.publication_sequence = 1
            snapshot.source_generation = 1
            snapshot.info.width = snapshot.info.height = 1
            snapshot.info.resolution = 1.0
            snapshot.info.origin.orientation.w = 1.0
            snapshot.occupancy = [0]
            snapshot.signed_distance_m = [1.0]
            self.snapshot_pub.publish(snapshot)
        if "tf" in self.enabled:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "map"
            transform.child_frame_id = "odom"
            transform.transform.rotation.w = 1.0
            self.tf_pub.publish(TFMessage(transforms=[transform]))

    def until(self, condition, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.publish()
            if condition():
                return
            time.sleep(0.03)
        self.fail("timed out waiting for recorder behavior")

    @staticmethod
    def records(path):
        if not path.exists():
            return []
        # A reader may see the final JSONL write before its terminating newline.
        lines = path.read_text().splitlines(keepends=True)
        return [json.loads(line) for line in lines if line.endswith("\n")]

    @contextmanager
    def process(self, output, duration=0, startup=10, runner=False):
        recorder = Path(__file__).with_name("nav_tracking_recorder.py")
        log_path = output / "process.log"
        if runner:
            source = recorder.with_name("test_mujoco_minco_mpc_chain.sh").read_text()
            helpers = []
            for name in ("start_nav_tracking_recorder", "stop_nav_tracking_recorder"):
                start = source.index(name + "() {\n")
                end = source.index("\n}", start) + 2
                helpers.append(source[start:end])
            script = "\n".join(helpers) + '''
set -u
CAPTURE_PIDS=()
NAV_TRACKING_PID=""
NAV_TRACKING_LEG_DIR=""
NAV_TRACKING_RECORDER=1
NAV_TRACKING_RATE_HZ=50
NAV_TRACKING_STOP_TIMEOUT=5
RECORDER_STATUS=not_started
stop_capture_process() { kill -KILL "$1" 2>/dev/null || true; }
finish() {
  stop_nav_tracking_recorder test
  printf '%s\n' "$RECORDER_STATUS" >"$OUTPUT/recorder_status"
}
trap finish EXIT
fail() { echo "FAIL: $*"; exit 1; }
start_nav_tracking_recorder test "$OUTPUT"
printf 'dispatched\n' >"$OUTPUT/dispatched"
while [[ ! -f "$OUTPUT/stop" ]]; do sleep 0.03; done
'''
            command = ["bash", "-c", script]
            environment = {**self.environment, "WORKSPACE_DIR": str(recorder.parent.parent),
                           "OUTPUT": str(output), "NAV_TRACKING_START_TIMEOUT": str(startup)}
        else:
            command = [sys.executable, str(recorder), "--output-dir", str(output),
                       "--duration-sec", str(duration), "--startup-timeout-sec", str(startup)]
            environment = self.environment
        with log_path.open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       env=environment, start_new_session=True)
            try:
                yield process, log_path
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)

    def test_sigint_finalizes_evidence_and_exits_zero(self):
        self.enabled = {"actual", "snapshot", "tf"}
        for duration in (0, 60):
            with self.subTest(duration=duration), tempfile.TemporaryDirectory() as directory:
                output = Path(directory)
                with self.process(output, duration=duration) as (process, log_path):
                    self.until(lambda: "RECORDER_READY " in log_path.read_text())
                    process.send_signal(signal.SIGINT)
                    self.assertEqual(process.wait(timeout=5), 0, log_path.read_text())
                summary = json.loads((output / "summary.json").read_text())
                self.assertIn(summary["reason"], ("shutdown", "interrupt"))
                self.assertTrue(summary["recording_started"])
                self.assertGreater(summary["records"]["samples"], 0)
                self.assertNotIn("Traceback", log_path.read_text())

    def test_runner_dispatch_waits_for_each_real_prerequisite(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            with self.process(output, startup=20, runner=True) as (process, log_path):
                events = output / "events.jsonl"
                samples = output / "samples.jsonl"

                def missing_is(expected):
                    changes = [event for event in self.records(events)
                               if event["kind"] == "warmup_prerequisites"]
                    return bool(changes) and changes[-1]["payload"]["missing"] == expected

                self.until(lambda: missing_is(["actual", "map_from_odom", "ready_snapshot"]))
                self.assertFalse((output / "dispatched").exists())
                self.assertEqual(self.records(samples), [])
                self.enabled.add("actual")
                self.until(lambda: missing_is(["map_from_odom", "ready_snapshot"]))
                self.assertFalse((output / "dispatched").exists())
                self.assertEqual(self.records(samples), [])
                self.enabled.add("snapshot")
                self.until(lambda: missing_is(["map_from_odom"]))
                self.assertFalse((output / "dispatched").exists())
                self.assertEqual(self.records(samples), [])
                self.enabled.add("tf")
                self.until(lambda: (output / "dispatched").exists())
                first = self.records(samples)[0]
                self.assertEqual(first["tick"], 1)
                self.assertIsNotNone(first["actual"])
                self.assertIsNotNone(first["map_from_odom"])
                self.assertTrue(first["snapshot"]["ready"])
                starts = [event for event in self.records(events)
                          if event["kind"] == "recording_started"]
                self.assertEqual(len(starts), 1)
                self.assertGreater(starts[0]["payload"]["warmup_duration_s_mono"], 0)
                (output / "stop").touch()
                self.assertEqual(process.wait(timeout=8), 0, log_path.read_text())
            self.assertEqual((output / "recorder_status").read_text().strip(), "passed")

    def test_absent_prerequisites_exit_nonzero_without_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            with self.process(output, startup=0.5) as (process, log_path):
                self.assertEqual(process.wait(timeout=10), 2, log_path.read_text())
            summary = json.loads((output / "summary.json").read_text())
            self.assertEqual(summary["reason"], "startup_timeout")
            self.assertFalse(summary["recording_started"])
            self.assertEqual(summary["ticks"], 0)
            self.assertEqual(self.records(output / "samples.jsonl"), [])
            self.assertNotIn("RECORDER_READY ", log_path.read_text())

    def test_runner_readiness_timeout_fails_without_dispatch(self):
        self.enabled = {"actual", "snapshot"}  # no TF can ever arrive
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            with self.process(output, startup=3, runner=True) as (process, log_path):
                self.until(lambda: process.poll() is not None, timeout=10)
                self.assertEqual(process.returncode, 1, log_path.read_text())
            self.assertFalse((output / "dispatched").exists())
            self.assertEqual((output / "recorder_status").read_text().strip(), "failed")
            self.assertEqual(self.records(output / "samples.jsonl"), [])


    def test_executor_runtime_error_is_clean_only_after_context_shutdown(self):
        self.enabled = {"actual", "snapshot", "tf"}
        recorder = Path(__file__).with_name("nav_tracking_recorder.py")
        script = '''
import sys
sys.path.insert(0, sys.argv[1])
import nav_tracking_recorder as recorder
original_spin_once = recorder.rclpy.spin_once
def spin_once(node, *args, **kwargs):
    if node._recording_start_mono is not None:
        if sys.argv[2] == "shutdown":
            node.context.try_shutdown()
        raise RuntimeError("injected executor failure after real readiness")
    return original_spin_once(node, *args, **kwargs)
recorder.rclpy.spin_once = spin_once
recorder.rclpy.spin = lambda node: spin_once(node)
raise SystemExit(recorder.main(sys.argv[3:]))
'''
        for duration in (0, 60):
            for state in ("shutdown", "active"):
                with self.subTest(duration=duration, context=state), \
                        tempfile.TemporaryDirectory() as directory:
                    output = Path(directory)
                    log_path = output / "process.log"
                    with log_path.open("w") as log:
                        process = subprocess.Popen(
                            [sys.executable, "-c", script, str(recorder.parent), state,
                             "--output-dir", directory, "--duration-sec", str(duration),
                             "--startup-timeout-sec", "10"],
                            stdout=log, stderr=subprocess.STDOUT, env=self.environment)
                        try:
                            self.until(lambda: process.poll() is not None)
                        finally:
                            if process.poll() is None:
                                process.kill()
                                process.wait(timeout=5)
                    summary = json.loads((output / "summary.json").read_text())
                    self.assertTrue(summary["recording_started"])
                    self.assertGreater(summary["records"]["samples"], 0)
                    if state == "shutdown":
                        self.assertEqual(process.returncode, 0, log_path.read_text())
                        self.assertEqual(summary["reason"], "shutdown")
                        self.assertNotIn("Traceback", log_path.read_text())
                    else:
                        self.assertNotEqual(process.returncode, 0)
                        self.assertEqual(summary["reason"], "runtime_error")
                        self.assertIn("RuntimeError: injected executor failure",
                                      log_path.read_text())


if __name__ == "__main__":
    unittest.main()
