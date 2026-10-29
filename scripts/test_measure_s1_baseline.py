#!/usr/bin/env python3
"""Offline tests for scripts/measure_s1_baseline.py and the scenarios added to scripts/lio_eval.py.

    python3 -I scripts/test_measure_s1_baseline.py

No ROS and no running stack: /proc is faked in a temporary directory, rclpy and the message packages
are stubbed for lio_eval.py.  Modules are loaded by path, so nothing depends on the working directory
or on a local module that could shadow the standard library (hence -I).
"""

import importlib.util
import json
import math
import os
import re
import shutil
import struct
import sys
import tempfile
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

try:
    import numpy  # noqa: F401  (imported before lio_eval is loaded with stubs around it)
except ImportError:
    numpy = None

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


s1 = load_module("measure_s1_baseline_under_test", "measure_s1_baseline.py")


def cdr(sec, nanosec, big=False):
    """Serialized message that starts with a stamp: encapsulation header, sec, nanosec, filler."""
    head = b"\x00\x00\x00\x00" if big else b"\x00\x01\x00\x00"
    return head + struct.pack(">iI" if big else "<iI", sec, nanosec) + b"\x00" * 8


class FakeProc:
    """A /proc look-alike in a directory: <root>/<pid>/{stat,status,environ,cmdline}."""

    def __init__(self, root):
        self.root = Path(root)
        self.params = {}

    def write(self, pid, **changes):
        params = self.params.setdefault(pid, dict(
            comm="proc", utime=0, stime=0, threads=1, starttime=100, rss_kib=1024, hwm_kib=2048,
            environ={}, argv=["/usr/bin/true"]))
        params.update(changes)
        directory = self.root / str(pid)
        directory.mkdir(exist_ok=True)
        fields = ["S", "1", "1", "1", "0", "-1", "0", "0", "0", "0", "0", str(params["utime"]),
                  str(params["stime"]), "0", "0", "20", "0", str(params["threads"]), "0",
                  str(params["starttime"]), "0", "0"]
        (directory / "stat").write_text(f"{pid} ({params['comm']}) " + " ".join(fields) + "\n")
        (directory / "status").write_text(
            f"Name:\tx\nVmHWM:\t{params['hwm_kib']} kB\nVmRSS:\t{params['rss_kib']} kB\n"
            f"Threads:\t{params['threads']}\n")
        (directory / "environ").write_bytes(
            "".join(f"{k}={v}\0" for k, v in params["environ"].items()).encode())
        (directory / "cmdline").write_bytes("".join(a + "\0" for a in params["argv"]).encode())

    def remove(self, pid):
        shutil.rmtree(self.root / str(pid))
        del self.params[pid]


class TempDirTestCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = self._tmp.name
        self.proc = FakeProc(self.root)


class StatisticsTest(unittest.TestCase):
    def test_percentile_linear_interpolation(self):
        values = list(range(1, 101))
        self.assertAlmostEqual(s1.percentile(values, 50), 50.5)
        self.assertAlmostEqual(s1.percentile(values, 95), 95.05)
        self.assertEqual(s1.percentile(values, 0), 1.0)
        self.assertEqual(s1.percentile(values, 100), 100.0)
        self.assertEqual(s1.percentile([7.0], 95), 7.0)
        self.assertIsNone(s1.percentile([], 50))

    def test_rate_per_second(self):
        self.assertAlmostEqual(s1.rate_per_second(120, 60.0), 2.0)
        self.assertAlmostEqual(s1.rate_per_second(120, 1050.0), 120 / 1050.0)
        for span in (0.0, -1.0, None):
            self.assertIsNone(s1.rate_per_second(5, span))

    def test_age_samples_keep_negative_and_zero_stamp_apart(self):
        ages = s1.AgeSamples()
        for age in (0.1, 0.2, 0.3, -0.05):
            ages.add(age)
        ages.zero_stamp += 2
        summary = ages.summary()
        self.assertEqual((summary["n"], summary["negative"], summary["zero_stamp"]), (4, 1, 2))
        self.assertAlmostEqual(summary["p50"], 0.15)
        self.assertAlmostEqual(summary["max"], 0.3)
        self.assertFalse(summary["sampled"])

    def test_age_reservoir_is_bounded_and_unbiased(self):
        ages = s1.AgeSamples(cap=500, seed=1)
        for value in range(20000):
            ages.add(float(value))
        summary = ages.summary()
        self.assertEqual((len(ages.values), ages.count, summary["sampled"]), (500, 20000, True))
        self.assertLess(abs(summary["p50"] - 10000), 2000)     # ~4.5 sigma of a 500-sample median


class TopicStatsTest(unittest.TestCase):
    def test_counts_rates_and_ages(self):
        stats = s1.TopicStats("/odometry", "nav_msgs/msg/Odometry", "header")
        stats.activate(100.0, 1000.0)
        for sec, nanosec in ((100, 0), (100, 500_000_000), (101, 0)):   # ages 1.25, 0.75, 0.25
            stats.add(cdr(sec, nanosec), 101.25)
        stats.add(cdr(0, 0), 101.25)             # stamp 0: a message, but not an age
        stats.add(b"\x00\x01", 101.25)           # truncated: stamp cannot be read
        result = stats.result(sim_end=102.0, wall_end=1008.0)
        self.assertEqual(result["messages"], 5)
        self.assertAlmostEqual(result["hz_per_sim_s"], 2.5)        # 5 messages / 2 sim s
        self.assertAlmostEqual(result["hz_per_wall_s"], 0.625)     # 5 messages / 8 wall s
        age = result["age_s"]
        self.assertEqual((age["n"], age["zero_stamp"], result["unparsed_stamps"]), (3, 1, 1))
        self.assertAlmostEqual(age["p50"], 0.75)
        self.assertAlmostEqual(age["max"], 1.25)

    def test_topic_without_stamp_has_no_ages_and_inactive_topic_no_rate(self):
        stats = s1.TopicStats("/x", "std_msgs/msg/String", None)
        stats.add(b"not a stamp at all", 1.0)
        result = stats.result(sim_end=5.0, wall_end=50.0)
        self.assertEqual((result["messages"], result["age_s"]["n"]), (1, 0))
        self.assertIsNone(result["hz_per_sim_s"])
        self.assertIsNone(result["hz_per_wall_s"])


class CdrTest(unittest.TestCase):
    def test_little_and_big_endian(self):
        self.assertAlmostEqual(s1.parse_cdr_leading_stamp(cdr(12, 500_000_000)), 12.5)
        self.assertAlmostEqual(s1.parse_cdr_leading_stamp(cdr(12, 500_000_000, big=True)), 12.5)

    def test_rejects_short_or_foreign_encapsulation(self):
        self.assertIsNone(s1.parse_cdr_leading_stamp(None))
        self.assertIsNone(s1.parse_cdr_leading_stamp(b""))
        self.assertIsNone(s1.parse_cdr_leading_stamp(b"\x00\x01\x00\x00" + b"\x00" * 7))
        self.assertIsNone(s1.parse_cdr_leading_stamp(b"\x00\x03\x00\x00" + b"\x00" * 8))

    def test_stamp_kind_of(self):
        def message(first_type):
            return SimpleNamespace(get_fields_and_field_types=lambda: {"a": first_type, "b": "double"})
        self.assertEqual(s1.stamp_kind_of(message("std_msgs/Header")), "header")
        self.assertEqual(s1.stamp_kind_of(message("builtin_interfaces/Time")), "time")
        self.assertIsNone(s1.stamp_kind_of(message("double")))
        self.assertIsNone(s1.stamp_kind_of(SimpleNamespace(get_fields_and_field_types=dict)))

    def test_matches_rclpy_serialization_when_importable(self):
        try:
            from nav_msgs.msg import Odometry
            from rclpy.serialization import serialize_message
        except ImportError:
            self.skipTest("rclpy / nav_msgs not importable (python3 -I drops PYTHONPATH)")
        message = Odometry()
        message.header.stamp.sec, message.header.stamp.nanosec = 123, 456_000_000
        self.assertAlmostEqual(s1.parse_cdr_leading_stamp(serialize_message(message)), 123.456)


class ProcReaderTest(TempDirTestCase):
    def test_stat_with_awkward_comm(self):
        self.proc.write(42, comm="my (odd) name", utime=700, stime=50, threads=9, starttime=777)
        stat = s1.read_stat(self.root, 42)
        self.assertEqual((stat["comm"], stat["ticks"], stat["threads"], stat["starttime"]),
                         ("my (odd) name", 750, 9, 777))
        self.assertIsNone(s1.read_stat(self.root, 43))

    def test_status_environ_cmdline(self):
        argv = ["/usr/bin/python3", "/ws/sim_node", "--ros-args"]
        self.proc.write(7, rss_kib=204800, hwm_kib=307200, threads=4, argv=argv,
                        environ={"ROS_DOMAIN_ID": "110", "A": "b=c"})
        self.assertEqual(s1.read_status(self.root, 7),
                         {"rss_kib": 204800, "hwm_kib": 307200, "threads": 4})
        self.assertEqual(s1.read_environ(self.root, 7), {"ROS_DOMAIN_ID": "110", "A": "b=c"})
        self.assertEqual(s1.read_cmdline(self.root, 7), argv)
        self.assertIsNone(s1.read_environ(self.root, 99))
        self.assertEqual(s1.read_cmdline(self.root, 99), [])

    def test_list_pids_ignores_non_numeric_entries(self):
        self.proc.write(5)
        self.proc.write(31)
        (Path(self.root) / "stat").write_text("cpu 1 2 3\n")
        self.assertEqual(s1.list_pids(self.root), [5, 31])
        self.assertEqual(s1.list_pids(self.root + "/missing"), [])

    def test_domain_filter(self):
        match = s1.domain_matches
        self.assertTrue(match({"ROS_DOMAIN_ID": "110"}, 110))
        self.assertTrue(match({"ROS_DOMAIN_ID": " 0110 "}, 110))
        self.assertFalse(match({"ROS_DOMAIN_ID": "11"}, 110))
        self.assertFalse(match({"ROS_DOMAIN_ID": "1100"}, 110))
        self.assertFalse(match({"ROS_DOMAIN_ID": "x"}, 0))
        self.assertTrue(match({}, 0))              # unset is domain 0 in ROS 2
        self.assertFalse(match({}, 110))
        self.assertFalse(match(None, 0))           # environ unreadable


class IdentityTest(unittest.TestCase):
    def test_node_name_prefers_the_remap(self):
        argv = ["/usr/bin/python3", "/ws/lib/ats_mujoco_sim/sim_node", "--ros-args", "-r",
                "__node:=ats_mujoco_sim", "-r", "__ns:=/robot"]
        self.assertEqual(s1.node_name(argv), "/robot/ats_mujoco_sim")
        self.assertEqual(s1.executable_name(argv), "sim_node")
        self.assertEqual(s1.node_name(argv[:-1] + ["__ns:=/"]), "ats_mujoco_sim")
        self.assertEqual(s1.node_name(["/ws/lib/p/a", "-r", "a:__node:=renamed"]), "renamed")

    def test_executable_fallback_and_python_launchers(self):
        self.assertEqual(s1.node_name(["/ws/lib/loam/loam_interface_node", "--ros-args"]),
                         "loam_interface_node")
        self.assertEqual(s1.executable_name(["python3", "-u", "/a/b/run.py"]), "run.py")
        self.assertEqual(s1.executable_name(["/usr/bin/python3.10", "-m", "mujoco.viewer"]),
                         "mujoco.viewer")
        self.assertEqual(s1.executable_name([]), "")

    def test_ros2_cli_is_named_after_its_verb(self):
        launch = ["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch", "pkg", "x.launch.py"]
        self.assertEqual(s1.node_name(launch), "ros2 launch")
        self.assertEqual(s1.node_name(launch[:2] + ["bag", "record"]), "ros2 bag")

    def test_is_ros_process(self):
        self.assertTrue(s1.is_ros_process(["/ws/lib/p/n", "--ros-args", "-p", "a:=1"]))
        self.assertTrue(s1.is_ros_process(["/usr/bin/python3", "/ws/lib/ats_mujoco_sim/sim_node"]))
        self.assertTrue(s1.is_ros_process(["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch"]))
        self.assertTrue(s1.is_ros_process(["/usr/bin/mujoco_viewer"]))
        self.assertFalse(s1.is_ros_process(["bash", "scripts/test_mujoco_minco_mpc_chain.sh"]))
        self.assertFalse(s1.is_ros_process(["tail", "-f", "/tmp/launch.log"]))
        self.assertFalse(s1.is_ros_process([]))
        self.assertTrue(s1.is_ros_process(["tail", "-f", "/tmp/launch.log"], [re.compile("launch")]))


NODE_ARGV = ["/ws/lib/ats_rog_map/ats_rog_map_node", "--ros-args", "-r", "__node:=ats_rog_map"]
SIM_ARGV = ["/usr/bin/python3", "/ws/lib/ats_mujoco_sim/sim_node", "--ros-args"]


def domain(value):
    return {"ROS_DOMAIN_ID": str(value)}


class ProcessSamplerTest(TempDirTestCase):
    def setUp(self):
        super().setUp()
        self.now = 1000.0
        self.sampler = s1.ProcessSampler(110, proc_root=self.root, clock=lambda: self.now,
                                         clk_tck=100, exclude_pids=[500])

    def test_domain_filter_cpu_and_rss(self):
        self.proc.write(10, argv=NODE_ARGV, environ=domain(110), utime=100, rss_kib=100 * 1024)
        self.proc.write(11, argv=NODE_ARGV, environ=domain(99))              # other session
        self.proc.write(12, argv=["bash", "run.sh"], environ=domain(110))    # wrapper, not a node
        self.proc.write(13, argv=SIM_ARGV, environ=domain(110))
        self.proc.write(14, argv=NODE_ARGV, environ={})                      # unset = domain 0
        self.proc.write(500, argv=NODE_ARGV, environ=domain(110))            # excluded (observer)
        self.sampler.rescan()
        self.sampler.sample()
        self.now += 1.0
        self.proc.write(10, utime=250, rss_kib=200 * 1024, hwm_kib=300 * 1024)   # 150 ticks / 1 s
        self.proc.write(13, utime=40)
        self.sampler.sample()
        self.now += 2.0
        self.proc.write(10, utime=350, rss_kib=150 * 1024)                       # 100 ticks / 2 s
        self.sampler.sample()
        rows = {row["name"]: row for row in self.sampler.rows()}
        self.assertEqual(set(rows), {"ats_rog_map", "sim_node"})
        node = rows["ats_rog_map"]
        self.assertEqual((node["pid"], node["exe"], node["samples"]), (10, "ats_rog_map_node", 3))
        self.assertAlmostEqual(node["cpu_mean_pct"], 100.0 * 250 / 100 / 3.0)   # 83.3 %
        self.assertAlmostEqual(node["cpu_peak_pct"], 150.0)
        self.assertAlmostEqual(node["rss_mean_mib"], 150.0)
        self.assertAlmostEqual(node["rss_peak_mib"], 200.0)
        self.assertAlmostEqual(node["rss_hwm_mib"], 300.0)
        sim = rows["sim_node"]
        self.assertAlmostEqual(sim["cpu_mean_pct"], 100.0 * 40 / 100 / 3.0)
        self.assertAlmostEqual(sim["cpu_peak_pct"], 40.0)
        self.assertEqual([r["name"] for r in self.sampler.rows()], ["ats_rog_map", "sim_node"])

    def test_exit_pid_reuse_and_late_start(self):
        self.proc.write(10, argv=NODE_ARGV, environ=domain(110), starttime=5)
        self.sampler.rescan()
        self.sampler.sample()
        self.proc.remove(10)
        self.proc.write(10, argv=NODE_ARGV, environ=domain(110), starttime=9)    # pid reused
        self.proc.write(20, argv=SIM_ARGV, environ=domain(110), starttime=11)    # started later
        self.sampler.sample()
        self.sampler.rescan()
        self.now += 1.0
        self.sampler.sample()
        alive = {(r["pid"], r["alive_at_end"]) for r in self.sampler.rows()}
        self.assertEqual(alive, {(10, False), (10, True), (20, True)})

    def test_track_always_samples_the_observer(self):
        self.proc.write(500, argv=["python3", "measure_s1_baseline.py"], environ={})
        self.sampler.track(500)
        self.sampler.rescan()
        self.sampler.sample()
        self.assertEqual([r["pid"] for r in self.sampler.rows("observer")], [500])
        self.assertEqual(self.sampler.rows("node"), [])


class SystemTest(TempDirTestCase):
    STAT = "cpu  {user} 0 {system} {idle} {iowait} 0 0 0 0 0\ncpu0 1 0 1 1 0 0 0 0 0 0\n" \
           "cpu1 1 0 1 1 0 0 0 0 0 0\ncpu2 1 0 1 1 0 0 0 0 0 0\ncpu3 1 0 1 1 0 0 0 0 0 0\nintr 1\n"

    def test_parse_proc_stat_cpu(self):
        text = self.STAT.format(user=100, system=50, idle=800, iowait=50)
        self.assertEqual(s1.parse_proc_stat_cpu(text), (150, 1000, 4))
        self.assertIsNone(s1.parse_proc_stat_cpu("intr 1\n"))

    def test_machine_busy_between_samples(self):
        stat = Path(self.root) / "stat"
        sampler = s1.SystemSampler(self.root)
        stat.write_text(self.STAT.format(user=100, system=50, idle=800, iowait=50))
        sampler.sample()
        stat.write_text(self.STAT.format(user=160, system=90, idle=1100, iowait=50))   # +100 busy / +400
        sampler.sample()
        summary = sampler.summary()
        self.assertAlmostEqual(summary["busy_mean_pct_of_machine"], 25.0)
        self.assertAlmostEqual(summary["cores_busy_mean"], 1.0)        # 25 % of 4 CPUs
        self.assertEqual((summary["cpus"], summary["samples"]), (4, 1))

    def test_machine_info(self):
        (Path(self.root) / "cpuinfo").write_text("processor\t: 0\nmodel name\t: Fake CPU @ 3 GHz\n")
        (Path(self.root) / "meminfo").write_text("MemTotal:       16777216 kB\nMemFree: 1 kB\n")
        info = s1.machine_info(self.root)
        self.assertEqual(info["cpu_model"], "Fake CPU @ 3 GHz")
        self.assertAlmostEqual(info["mem_total_mib"], 16384.0)
        self.assertIsNone(s1.machine_info(self.root + "/missing")["cpu_model"])


class WindowTest(unittest.TestCase):
    def test_sim_window_counts_clock_seconds(self):
        window = s1.Window(60.0, True, max_wall_s=7200.0, stall_wall_s=120.0)
        window.start(1000.0, 500.0)
        window.update(1500.0, 559.9)
        self.assertFalse(window.ended)
        window.update(1600.0, 560.0)
        self.assertEqual(window.outcome, "complete")
        self.assertAlmostEqual(window.sim_span, 60.0)
        self.assertAlmostEqual(window.wall_span, 600.0)
        self.assertAlmostEqual(window.sim_over_wall, 0.1)

    def test_wall_window_ignores_the_clock(self):
        window = s1.Window(30.0, False, max_wall_s=7200.0, stall_wall_s=120.0)
        window.start(1000.0, None)
        window.update(1029.9, None)
        self.assertFalse(window.ended)
        window.update(1030.0, None)
        self.assertEqual(window.outcome, "complete")
        self.assertIsNone(window.sim_over_wall)

    def test_stall_and_max_wall(self):
        stalled = s1.Window(60.0, True, max_wall_s=100.0, stall_wall_s=50.0)
        stalled.start(0.0, 10.0)
        stalled.update(40.0, 10.0)
        self.assertFalse(stalled.ended)
        stalled.update(51.0, 10.0)
        self.assertEqual(stalled.outcome, "clock_stalled")
        slow = s1.Window(60.0, True, max_wall_s=100.0, stall_wall_s=1e9)
        slow.start(0.0, 10.0)
        slow.update(50.0, 11.0)
        self.assertFalse(slow.ended)
        slow.update(101.0, 12.0)
        self.assertEqual(slow.outcome, "max_wall")

    def test_finish_keeps_the_first_outcome(self):
        window = s1.Window(1.0, False, 100.0, 100.0)
        window.start(0.0, None)
        window.update(2.0, None)
        window.finish("interrupted", 3.0, None)
        self.assertEqual((window.outcome, window.wall_end), ("complete", 2.0))


class GraphTest(TempDirTestCase):
    @staticmethod
    def endpoint(namespace, name, reliability="BEST_EFFORT"):
        return SimpleNamespace(node_name=name, node_namespace=namespace,
                               topic_type="sensor_msgs/msg/PointCloud2",
                               qos_profile=SimpleNamespace(reliability=SimpleNamespace(name=reliability)))

    def test_describe_endpoints_drops_the_observer_and_sorts(self):
        infos = [self.endpoint("/", "terrain_analysis"), self.endpoint("/", "s1_baseline_observer_7"),
                 self.endpoint("/robot", "a", "RELIABLE")]
        entries = s1.describe_endpoints(infos, ("s1_baseline_observer_7", "/"))
        self.assertEqual([e["node"] for e in entries], ["/robot/a", "/terrain_analysis"])
        self.assertEqual(entries[0]["reliability"], "RELIABLE")
        self.assertEqual(s1.describe_endpoints([], ("o", "/")), [])

    def test_ros_python_paths_are_chosen_by_prefix(self):
        prefix = Path(self.root) / "opt" / "ros"
        inside = prefix / "lib" / "python3.10" / "site-packages"
        outside = Path(self.root) / "elsewhere"
        for directory in (inside, outside):
            directory.mkdir(parents=True)
        pythonpath = os.pathsep.join([str(outside), "relative/dir", "", str(inside),
                                      str(prefix / "missing")])
        self.assertEqual(s1.select_ros_python_paths(pythonpath, [str(prefix)]), [str(inside)])
        self.assertEqual(s1.select_ros_python_paths(pythonpath, []), [])

    def test_ensure_ros_python_path_appends_only_when_isolated(self):
        prefix = Path(self.root) / "ws" / "install" / "pkg"
        inside = prefix / "local" / "lib" / "python3.10" / "dist-packages"
        inside.mkdir(parents=True)
        environ = {"PYTHONPATH": str(inside), "AMENT_PREFIX_PATH": str(prefix)}
        with mock.patch.object(sys, "path", ["/first", "/second"]):
            with mock.patch.object(sys, "flags", SimpleNamespace(isolated=0)):
                self.assertEqual(s1.ensure_ros_python_path(environ), [])
                self.assertEqual(sys.path, ["/first", "/second"])
            with mock.patch.object(sys, "flags", SimpleNamespace(isolated=1)):
                self.assertEqual(s1.ensure_ros_python_path(environ), [str(inside)])
                self.assertEqual(sys.path, ["/first", "/second", str(inside)])    # appended


class ResultTest(TempDirTestCase):
    def build(self, snapshot_end=None):
        args = s1.build_parser().parse_args(
            ["--out", "x.json", "--domain", "110", "--mode", "lio", "--label", "unit"])
        args.topic = list(s1.DEFAULT_RATE_TOPICS)
        args.consumer_topic = ["/sensor_scan", "/Laser_map"]
        window = s1.Window(60.0, True, 7200.0, 120.0)
        window.start(0.0, 100.0)
        window.update(1050.0, 160.0)
        stats = s1.TopicStats("/aft_mapped_to_init", "nav_msgs/msg/Odometry", "header")
        stats.activate(100.0, 0.0)
        for index in range(600):
            stats.add(cdr(160, 0), 160.0 + 0.01 * (index % 5))
        topics = {t: {"present": False, "messages": 0} for t in args.topic}
        topics["/aft_mapped_to_init"] = stats.result(window.sim_end, window.wall_end)
        quiet = {"publisher_count": 1, "subscriber_count": 0, "has_consumer": False,
                 "publishers": [{"node": "/sensor_scan_generation", "type": "t", "reliability": "R"}],
                 "subscribers": []}
        used = dict(quiet, subscriber_count=1, has_consumer=True,
                    subscribers=[{"node": "/ats_rog_map", "type": "t", "reliability": "R"}])
        start = {"/sensor_scan": used, "/Laser_map": quiet}
        end = snapshot_end or start
        row = {"pid": 10, "name": "ats_rog_map", "exe": "ats_rog_map_node", "cpu_mean_pct": 83.3,
               "cpu_peak_pct": 150.0, "rss_mean_mib": 150.0, "rss_peak_mib": 200.0, "samples": 3,
               "alive_at_end": True}
        system = {"cpus": 32, "cores_busy_mean": 5.5, "cores_busy_peak": 7.0}
        host = {"hostname": "h", "cpu_model": "Fake", "logical_cpus": 32, "mem_total_mib": 16384.0}
        return s1.assemble_result(args, window, topics, [row], dict(row, name="observer"), system,
                                  (start, end), [0.05, 0.06], ["note"], host)

    def test_result_window_topics_and_warnings(self):
        result = self.build()
        self.assertEqual(result["window"]["time_base"], "sim")
        self.assertAlmostEqual(result["window"]["sim_over_wall"], 60.0 / 1050.0)
        self.assertEqual((result["mode"], result["domain"], result["label"]), ("lio", 110, "unit"))
        entry = result["topics"]["/aft_mapped_to_init"]
        self.assertAlmostEqual(entry["hz_per_sim_s"], 10.0)                # 600 messages / 60 sim s
        self.assertAlmostEqual(entry["age_s"]["max"], 0.04)
        self.assertFalse(result["consumers"]["/sensor_scan"]["changed_during_window"])
        warnings = " | ".join(result["warnings"])
        self.assertIn("/odometry never appeared", warnings)
        self.assertIn("note", warnings)
        self.assertNotIn("/aft_mapped_to_init never", warnings)
        json.dumps(s1.sanitize(result), allow_nan=False)

    def test_consumer_change_is_flagged(self):
        changed = {"/sensor_scan": self.build()["consumers"]["/Laser_map"], "/Laser_map": {
            "publisher_count": 0, "subscriber_count": 0, "has_consumer": False,
            "publishers": [], "subscribers": []}}
        result = self.build(snapshot_end=changed)
        self.assertTrue(result["consumers"]["/sensor_scan"]["changed_during_window"])
        self.assertIn("changed during the window", " ".join(result["warnings"]))

    def test_summary_text(self):
        text = s1.format_summary(self.build())
        for expected in ("label=unit", "mode=lio", "sim/wall 0.0571", "ats_rog_map (ats_rog_map_node)",
                         "/aft_mapped_to_init", "10.00", "not in graph", "NO CONSUMER",
                         "/sensor_scan_generation", "/ats_rog_map", "observer (this script)"):
            self.assertIn(expected, text)

    def test_write_json_replaces_nan_and_inf(self):
        path = s1.write_json(Path(self.root) / "sub" / "out.json", {"a": float("nan"), "b": [1.0, math.inf]})
        self.assertEqual(json.loads(path.read_text()), {"a": None, "b": [1.0, None]})
        self.assertEqual(list(path.parent.iterdir()), [path])               # no .tmp left behind


def load_lio_eval():
    """lio_eval.py with rclpy and the message packages replaced by stubs (no ROS needed)."""
    def module(name, **attrs):
        stub = types.ModuleType(name)
        stub.__dict__.update(attrs)
        return stub
    node = module("rclpy.node", Node=type("Node", (), {}))
    qos = module("rclpy.qos", QoSProfile=lambda **kw: SimpleNamespace(**kw),
                 ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1))
    stubs = {"rclpy": module("rclpy", node=node, qos=qos), "rclpy.node": node, "rclpy.qos": qos}
    for package, names in {"livox_ros_driver2": ["CustomMsg"], "manda_can_control": ["MotionCtrl"],
                           "nav_msgs": ["Odometry"], "rosgraph_msgs": ["Clock"],
                           "sensor_msgs": ["Imu"], "std_msgs": ["Float64MultiArray"]}.items():
        messages = module(package + ".msg", **{n: type(n, (), {}) for n in names})
        stubs[package + ".msg"] = messages
        stubs[package] = module(package, msg=messages)
    with mock.patch.dict(sys.modules, stubs):
        return load_module("lio_eval_under_test", "lio_eval.py")


def original_command(scenario, args, t, yaw):
    """The driver math of static / straight / spin_walk as it was before the new scenarios."""
    vx_w, vy_w, wz = 0.0, 0.0, 0.0
    if scenario in ("straight", "spin_walk"):
        speed = args.speed if args.speed is not None else (1.5 if scenario == "straight" else 1.0)
        leg_time = args.leg / speed
        phase = int(t // leg_time)
        vx_w = speed if phase % 2 == 0 else -speed
        if scenario == "spin_walk":
            wz = args.spin
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * vx_w + s * vy_w, -s * vx_w + c * vy_w, wz)


@unittest.skipIf(numpy is None, "numpy missing")
class LioEvalScenarioTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lio = load_lio_eval()

    @staticmethod
    def args(scenario, **overrides):
        values = dict(scenario=scenario, duration=60.0, settle=0.0, speed=None, leg=4.0, spin=3.0,
                      burst_time=2.0, pause_time=2.0, entry_gap=0.05, target_x=None, ramp_accel=1.0,
                      start_x=None, start_y=None, start_yaw=None)
        values.update(overrides)
        return SimpleNamespace(**values)

    def drive(self, args, t, x=4.0, y=5.5, yaw=0.0, reached=False):
        """One Evaluator._drive tick at scenario time t; returns (commands sent, evaluator)."""
        evaluator = object.__new__(self.lio.Evaluator)
        evaluator.args, evaluator.truth = args, [(t, x, y, 0.5, yaw)]
        evaluator.sim_time, evaluator.scenario_start = t, 0.0
        evaluator.done, evaluator.entry_reached = False, reached
        sent = []
        evaluator._send = lambda vx, vy, wz: sent.append((vx, vy, wz))
        evaluator._drive()
        return sent, evaluator

    def test_existing_scenarios_are_unchanged(self):
        for scenario in ("static", "straight", "spin_walk"):
            for speed in (None, 0.8):
                args = self.args(scenario, speed=speed)
                for yaw in (0.0, 0.7, -2.0):
                    for step in range(0, 1000, 7):
                        t = step * 0.02
                        sent, _ = self.drive(args, t, yaw=yaw)
                        self.assertEqual(sent, [original_command(scenario, args, t, yaw)],
                                         (scenario, speed, yaw, t))
        self.assertEqual(self.drive(self.args("observe"), 1.0)[0], [])


    def test_spin_turns_in_place(self):
        for yaw in (0.0, 1.1, -2.5):
            self.assertEqual(self.drive(self.args("spin"), 5.0, yaw=yaw)[0], [(0.0, 0.0, 3.0)])
            self.assertEqual(self.drive(self.args("spin", spin=2.0), 5.0, yaw=yaw)[0], [(0.0, 0.0, 2.0)])

    def test_accel_brake_profile(self):
        velocity = self.lio.accel_brake_velocity
        expected = [(0.0, 1.5), (1.99, 1.5), (2.0, 0.0), (3.99, 0.0), (4.0, -1.5), (5.99, -1.5),
                    (6.0, 0.0), (7.99, 0.0), (8.0, 1.5)]
        for t, value in expected:
            self.assertEqual(velocity(t, 1.5, 2.0, 2.0), value, t)
        steps = 80000
        travelled = sum(velocity(i * 0.0001, 1.5, 2.0, 2.0) for i in range(steps)) * 0.0001
        self.assertAlmostEqual(travelled, 0.0, delta=0.01)     # +x burst and -x burst cancel

    def test_accel_brake_driver_uses_the_world_to_body_rotation(self):
        sent, _ = self.drive(self.args("accel_brake"), 1.0, yaw=0.5)
        vx, vy, wz = sent[0]
        self.assertAlmostEqual(math.hypot(vx, vy), 1.5)
        self.assertAlmostEqual(vx, 1.5 * math.cos(0.5))
        self.assertAlmostEqual(vy, -1.5 * math.sin(0.5))
        self.assertEqual(wz, 0.0)
        self.assertEqual(self.drive(self.args("accel_brake"), 3.0)[0], [(0.0, 0.0, 0.0)])
        self.assertEqual(self.drive(self.args("accel_brake", speed=1.0), 0.5)[0], [(1.0, 0.0, 0.0)])

    def test_start_pose_defaults(self):
        for scenario, expected in (("static", (4.0, 5.5, 0.0)), ("straight", (4.0, 5.5, 0.0)),
                                   ("tunnel_entry", (12.90, 2.043, 0.0))):
            args = self.args(scenario)
            self.lio.resolve_start_pose(args)
            self.assertEqual((args.start_x, args.start_y, args.start_yaw), expected)
        args = self.args("tunnel_entry", start_x=11.0)
        self.lio.resolve_start_pose(args)
        self.assertEqual((args.start_x, args.start_y), (11.0, 2.043))

    def test_tunnel_entry_target(self):
        self.assertAlmostEqual(self.lio.tunnel_entry_target_x(self.args("tunnel_entry")),
                               13.30 - 0.2905 - 0.05)
        self.assertEqual(self.lio.tunnel_entry_target_x(self.args("tunnel_entry", target_x=14.5)), 14.5)
        self.assertAlmostEqual(
            self.lio.tunnel_entry_target_x(self.args("tunnel_entry", entry_gap=0.2)), 13.30 - 0.2905 - 0.2)

    def test_tunnel_entry_trapezoid_stops_at_the_target_and_latches(self):
        command = self.lio.tunnel_entry_command
        target, x, reached, top = 12.9595, 12.0, False, 0.0
        for step in range(1, 1500):
            vx, vy, wz, reached = command(step * 0.02, x, 2.043, 0.0, target, 0.5, 1.0, reached)
            top = max(top, vx)
            x += vx * 0.02
            if reached:
                break
        self.assertTrue(reached)
        self.assertAlmostEqual(top, 0.5)                          # cruise speed is the cap
        self.assertTrue(target - 0.006 <= x <= target + 0.002, x)
        self.assertEqual(command(30.0, target - 0.02, 2.043, 0.0, target, 0.5, 1.0, True)[0], 0.0)

    def test_tunnel_entry_holds_the_axis(self):
        command = self.lio.tunnel_entry_command
        vx, vy, wz, _ = command(1.0, 12.9, 2.093, 0.1, 13.0, 0.5, 1.0)
        self.assertAlmostEqual(vy, -0.1)
        self.assertAlmostEqual(wz, -0.2)
        vx, vy, wz, _ = command(1.0, 12.9, 3.5, 2.0, 13.0, 0.5, 1.0)
        self.assertEqual((vy, wz), (-0.2, -1.0))                  # clipped

    def test_tunnel_entry_driver(self):
        args = self.args("tunnel_entry")
        sent, evaluator = self.drive(args, 0.5, x=12.90, y=2.043)
        self.assertAlmostEqual(sent[0][0], math.sqrt(2.0 * (12.9595 - 12.90)))   # decel-limited
        self.assertFalse(evaluator.entry_reached)
        sent, evaluator = self.drive(args, 5.0, x=12.9597, y=2.043)
        self.assertEqual(sent, [(0.0, 0.0, 0.0)])
        self.assertTrue(evaluator.entry_reached)


    def test_bag_topics_and_command(self):
        self.assertEqual(self.lio.BAG_TOPICS, ("/clock", "/livox/lidar", "/livox/imu",
                                               "/simulation/ground_truth/odometry"))
        with tempfile.TemporaryDirectory() as tmp:
            recorder = self.lio.BagRecorder(Path(tmp) / "bags" / "static_01")
            command = recorder.command()
            self.assertEqual(command[:4], ["ros2", "bag", "record", "--use-sim-time"])
            self.assertEqual(command[4], "-o")
            self.assertEqual(command[5], str(Path(tmp).resolve() / "bags" / "static_01"))
            self.assertEqual(tuple(command[6:]), self.lio.BAG_TOPICS)
            self.assertEqual(len(self.lio.BAG_TOPICS), len(set(self.lio.BAG_TOPICS)))
            self.assertEqual(recorder.unsubscribed(), list(self.lio.BAG_TOPICS))     # no log yet
            recorder.log_path.parent.mkdir(parents=True)
            recorder.log_path.write_text("[INFO] Subscribed to topic '/clock'\n"
                                         "[INFO] Subscribed to topic '/livox/imu'\n")
            self.assertEqual(recorder.unsubscribed(),
                             ["/livox/lidar", "/simulation/ground_truth/odometry"])
            self.assertIsNone(recorder.stop())                                      # never started

    def test_constants_match_the_tunnel_config(self):
        try:
            import yaml
        except ImportError:
            self.skipTest("PyYAML missing")
        config = yaml.safe_load(
            (REPO / "src/sim/ats_mujoco_sim/config/tunnels_2027.yaml").read_text(encoding="utf-8"))
        tunnel = next(t for t in config["tunnels"] if t["name"] == "tunnel_B")
        self.assertEqual(tuple(tunnel["entry"]), self.lio.TUNNEL_B_ENTRY)
        self.assertEqual(min(x for x, _ in tunnel["corridor_polygon"]), self.lio.TUNNEL_B_MOUTH_X)
        mount = (REPO / "src/sim/ats_mujoco_sim/ats_mujoco_sim/lidar_mount.py").read_text(encoding="utf-8")
        length = float(re.search(r"^BODY_SIZE_X\s*=\s*([0-9.]+)", mount, re.M).group(1))
        self.assertAlmostEqual(length / 2.0, self.lio.CHASSIS_HALF_LENGTH)


if __name__ == "__main__":
    unittest.main(verbosity=2)
