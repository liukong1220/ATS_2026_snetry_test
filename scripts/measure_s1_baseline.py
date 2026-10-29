#!/usr/bin/env python3
"""S1 simulation baseline observer (report 9.2 part 2, item 5).

Attaches read-only to a stack that is already running on one ROS_DOMAIN_ID and, for a fixed window,
records

  * per node process: CPU mean/peak (% of one core, like pidstat -u) and RSS mean/peak (MiB), sampled
    once per wall second from /proc/<pid>/stat and /proc/<pid>/status.  Only processes whose
    /proc/<pid>/environ carries the target ROS_DOMAIN_ID are listed (other ROS sessions on the machine
    are ignored); the node name is the ``__node:=`` remap, else the executable name;
  * per topic (/aft_mapped_to_init, /cloud_registered, /registered_scan, /odometry, /localization by
    default): messages per sim second and per wall second, and the age "/clock at receipt minus
    header.stamp" as p50 / p95 / max;
  * publisher and subscriber nodes of /sensor_scan, /lidar_odometry, /cloud_registered_body and
    /Laser_map (the observer's own node is left out);
  * the sim/wall ratio over the window.

The window is --duration /clock seconds when /clock has a publisher (--time-base auto), wall seconds
otherwise.  Topic subscriptions are raw (no deserialisation), best effort and volatile, so a topic that
publishes thousands of messages per second costs a few microseconds per message here; the header
stamp is read straight from the CDR bytes.  The observer publishes nothing and sets no parameter.
Its own CPU is reported separately ("observer_self"); above 90 % of a core the counts are lower bounds.

Start it after the runner's topic-ownership checks: its subscriptions are real DDS readers, so
``ros2 topic info`` sees one more subscriber on every --topic (test_mujoco_minco_mpc_chain.sh asserts
"Subscription count: 1" on /odometry).  The consumer check does not subscribe and leaves itself out.

    python3 scripts/measure_s1_baseline.py --domain 110 --duration 60 --mode lio
        --label tunnel_2027_t8 --out ~/ats_evidence/s1/lio_tunnel.json

Only the standard library is imported at module level and rclpy inside run(), so the pure parts load
(and scripts/test_measure_s1_baseline.py runs) under ``python3 -I``.  With -I, PYTHONPATH is ignored
by the interpreter, so the entries that lie under an AMENT/COLCON/CMAKE prefix are appended to
sys.path (appended, never prepended, so nothing can shadow the standard library).
"""

import argparse
import array
import json
import math
import os
import platform
import random
import re
import signal
import socket
import struct
import sys
import time
from pathlib import Path
from types import SimpleNamespace

DEFAULT_RATE_TOPICS = ("/aft_mapped_to_init", "/cloud_registered", "/registered_scan",
                       "/odometry", "/localization")
DEFAULT_CONSUMER_TOPICS = ("/sensor_scan", "/lidar_odometry", "/cloud_registered_body", "/Laser_map")
CLOCK_TOPIC = "/clock"
CLOCK_TYPE = "rosgraph_msgs/msg/Clock"
SAMPLE_PERIOD_S = 1.0        # /proc sampling period (wall seconds)
RESCAN_PERIOD_S = 5.0        # how often /proc is scanned for new processes (wall seconds)
CLOCK_WAIT_S = 30.0          # wall seconds to wait for the first /clock message
CLOCK_STALL_WALL_S = 120.0   # the window ends early when /clock stands still for this long
AGE_SAMPLE_CAP = 2_000_000   # per topic; beyond this the ages are a uniform reservoir sample
OBSERVER_BUSY_PCT = 90.0     # observer CPU (% of one core) above which counts are lower bounds
# Queue depth of the raw subscriptions.  /aft_mapped_to_init is published once per point group
# (report 2.1.7), so it gets a deep queue; the others are clouds and stay shallow.
DEFAULT_QUEUE_DEPTH = 100
DEEP_QUEUE_TOPICS = {"/aft_mapped_to_init": 4096}
ROS_EXE_NAMES = frozenset({
    "sim_node", "ros2", "_ros2_daemon", "rviz2", "component_container", "component_container_mt",
    "component_container_isolated"})


# ----------------------------------------------------------------------------- statistics
def percentile(sorted_values, q):
    """Linear-interpolation percentile (numpy's default method) of a sorted sequence, q in 0..100."""
    n = len(sorted_values)
    if n == 0:
        return None
    position = (n - 1) * q / 100.0
    low = int(math.floor(position))
    high = min(low + 1, n - 1)
    return float(sorted_values[low] + (sorted_values[high] - sorted_values[low]) * (position - low))


def finite(value):
    """JSON-safe number: None for None, NaN and infinity."""
    return float(value) if value is not None and math.isfinite(value) else None


def rate_per_second(count, span_s):
    """Messages per second over a span, None when the span is missing or not positive."""
    return count / span_s if span_s is not None and span_s > 0.0 else None


def value_summary(values):
    """n / mean / min / p50 / p95 / max of a numeric sequence (None fields when it is empty)."""
    ordered = sorted(values)
    if not ordered:
        return {"n": 0, "mean": None, "min": None, "p50": None, "p95": None, "max": None}
    return {"n": len(ordered), "mean": sum(ordered) / len(ordered), "min": ordered[0],
            "p50": percentile(ordered, 50.0), "p95": percentile(ordered, 95.0), "max": ordered[-1]}


class AgeSamples:
    """Ages of one topic; keeps at most ``cap`` values (uniform reservoir, algorithm R)."""

    def __init__(self, cap=AGE_SAMPLE_CAP, seed=0):
        self.cap = cap
        self.values = array.array("d")
        self.count = 0           # ages offered (stamp present and not zero)
        self.negative = 0        # age < 0: the stamp is ahead of the /clock the observer had seen
        self.zero_stamp = 0      # messages with header.stamp == 0, not counted as ages
        self._rng = random.Random(seed)

    def add(self, age):
        self.count += 1
        if age < 0.0:
            self.negative += 1
        if len(self.values) < self.cap:
            self.values.append(age)
            return
        slot = self._rng.randrange(self.count)
        if slot < self.cap:
            self.values[slot] = age

    def summary(self):
        result = value_summary(self.values)
        result["negative"] = self.negative
        result["zero_stamp"] = self.zero_stamp
        result["sampled"] = self.count > self.cap
        return result


# ----------------------------------------------------------------------------- raw message stamps
_STAMP_LE = struct.Struct("<iI")
_STAMP_BE = struct.Struct(">iI")


def parse_cdr_leading_stamp(payload):
    """Seconds of the first time stamp of a serialized ROS 2 message, or None.

    Valid for messages that start with std_msgs/Header (stamp first) and for builtin_interfaces/Time
    ones such as rosgraph_msgs/Clock: the 4-byte CDR encapsulation header is followed by the int32
    sec and the uint32 nanosec.
    """
    if payload is None or len(payload) < 12 or payload[0] != 0 or payload[1] not in (0, 1):
        return None
    sec, nanosec = (_STAMP_LE if payload[1] == 1 else _STAMP_BE).unpack_from(payload, 4)
    return sec + nanosec * 1e-9


def stamp_kind_of(message_class):
    """"header" / "time" when the first field of the message is a Header / Time, else None."""
    fields = list(message_class.get_fields_and_field_types().items())
    if not fields:
        return None
    return {"std_msgs/Header": "header", "builtin_interfaces/Time": "time"}.get(fields[0][1])


class TopicStats:
    """Message count and ages of one topic while the window is open."""

    def __init__(self, topic, type_name, stamp_kind, age_cap=AGE_SAMPLE_CAP):
        self.topic = topic
        self.type_name = type_name
        self.stamp_kind = stamp_kind     # "header", "time" or None (no leading stamp, no ages)
        self.messages = 0
        self.unparsed = 0                # stamp could not be read from the bytes
        self.ages = AgeSamples(age_cap)
        self.active_sim = None           # /clock when counting started for this topic
        self.active_wall = None          # wall (monotonic) seconds when counting started

    def activate(self, sim_now, wall_now):
        self.active_sim, self.active_wall = sim_now, wall_now

    def add(self, payload, reference_s):
        """Count one message; reference_s is /clock (or wall time) at receipt."""
        self.messages += 1
        if self.stamp_kind is None:
            return
        stamp = parse_cdr_leading_stamp(payload)
        if stamp is None:
            self.unparsed += 1
        elif stamp == 0.0:
            self.ages.zero_stamp += 1
        else:
            self.ages.add(reference_s - stamp)

    def result(self, sim_end, wall_end):
        sim_span = (sim_end - self.active_sim
                    if sim_end is not None and self.active_sim is not None else None)
        wall_span = wall_end - self.active_wall if self.active_wall is not None else None
        return {
            "present": True, "type": self.type_name, "stamp": self.stamp_kind,
            "messages": self.messages,
            "observed_sim_s": finite(sim_span), "observed_wall_s": finite(wall_span),
            "hz_per_sim_s": finite(rate_per_second(self.messages, sim_span)),
            "hz_per_wall_s": finite(rate_per_second(self.messages, wall_span)),
            "age_s": self.ages.summary(), "unparsed_stamps": self.unparsed,
        }


# ----------------------------------------------------------------------------- /proc readers
def _read_bytes(path):
    try:
        with open(path, "rb") as handle:
            return handle.read()
    except OSError:
        return None


def list_pids(proc_root):
    try:
        return sorted(int(name) for name in os.listdir(proc_root) if name.isdigit())
    except OSError:
        return []


def read_stat(proc_root, pid):
    """The fields of /proc/<pid>/stat used here, or None when the process is gone."""
    raw = _read_bytes(f"{proc_root}/{pid}/stat")
    if raw is None:
        return None
    head, _, tail = raw.decode("utf-8", "replace").rpartition(")")   # comm may contain spaces and ')'
    fields = tail.split()
    try:
        return {"comm": head[head.index("(") + 1:], "state": fields[0], "ppid": int(fields[1]),
                "ticks": int(fields[11]) + int(fields[12]),          # utime + stime
                "threads": int(fields[17]), "starttime": int(fields[19])}
    except (ValueError, IndexError):
        return None


def read_status(proc_root, pid):
    """VmRSS / VmHWM (KiB) and thread count from /proc/<pid>/status; zeros for kernel threads."""
    raw = _read_bytes(f"{proc_root}/{pid}/status")
    if raw is None:
        return None
    values = {"VmRSS": 0, "VmHWM": 0, "Threads": 0}
    for line in raw.decode("utf-8", "replace").splitlines():
        key, _, rest = line.partition(":")
        if key in values:
            try:
                values[key] = int(rest.split()[0])
            except (ValueError, IndexError):
                pass
    return {"rss_kib": values["VmRSS"], "hwm_kib": values["VmHWM"], "threads": values["Threads"]}


def read_cmdline(proc_root, pid):
    raw = _read_bytes(f"{proc_root}/{pid}/cmdline")
    return [part.decode("utf-8", "replace") for part in raw.split(b"\0") if part] if raw else []


def read_environ(proc_root, pid):
    """Environment of a process as a dict, None when it cannot be read (other user, exited)."""
    raw = _read_bytes(f"{proc_root}/{pid}/environ")
    if raw is None:
        return None
    environ = {}
    for item in raw.split(b"\0"):
        key, separator, value = item.partition(b"=")
        if separator:
            environ[key.decode("utf-8", "replace")] = value.decode("utf-8", "replace")
    return environ


def domain_of(environ):
    """ROS_DOMAIN_ID of a process environment as int (unset means 0, as in ROS 2); None if invalid."""
    value = environ.get("ROS_DOMAIN_ID", "").strip()
    if not value:
        return 0
    try:
        return int(value)
    except ValueError:
        return None


def domain_matches(environ, domain):
    return environ is not None and domain_of(environ) == int(domain)


# ----------------------------------------------------------------------------- process identity
_PYTHON_EXE = re.compile(r"^python[0-9.]*$")
_NODE_REMAP = re.compile(r"^(?:[^:=]+:)?__node:=(.+)$")
_NS_REMAP = re.compile(r"^(?:[^:=]+:)?__ns:=(.+)$")


def executable_name(argv):
    """Program a process runs: basename of argv[0], or of the script / -m module for python."""
    if not argv:
        return ""
    exe = os.path.basename(argv[0])
    if not _PYTHON_EXE.match(exe):
        return exe
    rest = argv[1:]
    for index, arg in enumerate(rest):
        if arg == "-m" and index + 1 < len(rest):
            return rest[index + 1]
        if arg == "-c":
            return exe + " -c"
        if not arg.startswith("-"):
            return os.path.basename(arg)
    return exe


def node_name(argv):
    """Node name from the ``__node:=`` remap (with ``__ns:=``), else the executable name."""
    name = namespace = None
    for arg in argv[1:]:
        match = _NODE_REMAP.match(arg)
        if match:
            name = match.group(1)
        match = _NS_REMAP.match(arg)
        if match:
            namespace = match.group(1)
    if name:
        return namespace.rstrip("/") + "/" + name if namespace not in (None, "", "/") else name
    exe = executable_name(argv)
    if exe == "ros2":
        after = [a for i, a in enumerate(argv) if i and os.path.basename(argv[i - 1]) == "ros2"]
        return f"ros2 {after[0]}" if after and not after[0].startswith("-") else exe
    return exe


def is_ros_process(argv, extra_patterns=()):
    """A ROS node (``--ros-args`` / ``__node:=``), a known ROS or MuJoCo executable, or a --process-pattern hit.

    Shell wrappers such as the runner scripts carry the domain in their environment too, but are not
    nodes; they are not matched here (the executable of ``bash script.sh`` is bash).
    """
    if not argv:
        return False
    if any(arg == "--ros-args" or _NODE_REMAP.match(arg) for arg in argv):
        return True
    exe = executable_name(argv)
    if exe in ROS_EXE_NAMES or "mujoco" in exe.lower():
        return True
    line = " ".join(argv)
    return any(pattern.search(line) for pattern in extra_patterns)


# ----------------------------------------------------------------------------- CPU / RSS sampling
class ProcessRecord:
    """CPU and RSS samples of one process (identified by pid and start time)."""

    def __init__(self, pid, starttime, name, exe, cmdline, role="node"):
        self.pid, self.starttime = pid, starttime
        self.name, self.exe, self.cmdline, self.role = name, exe, cmdline, role
        self.alive = True
        self.last_t = None
        self.last_ticks = 0
        self.ticks_total = 0
        self.elapsed_total = 0.0
        self.cpu_samples = []            # % of one core per sampling interval
        self.rss_mib = []
        self.hwm_kib = 0
        self.threads_max = 0

    def add_sample(self, now, ticks, status, clk_tck):
        if self.last_t is not None and now > self.last_t:
            delta = ticks - self.last_ticks
            self.ticks_total += delta
            self.elapsed_total += now - self.last_t
            self.cpu_samples.append(100.0 * delta / clk_tck / (now - self.last_t))
        self.last_t, self.last_ticks = now, ticks
        self.rss_mib.append(status["rss_kib"] / 1024.0)
        self.hwm_kib = max(self.hwm_kib, status["hwm_kib"])
        self.threads_max = max(self.threads_max, status["threads"])

    def summary(self, clk_tck, series=False):
        elapsed = self.elapsed_total
        row = {
            "pid": self.pid, "name": self.name, "exe": self.exe, "cmdline": self.cmdline,
            "samples": len(self.rss_mib), "alive_at_end": self.alive,
            # time-weighted mean (total CPU ticks over total time), peak of the per-interval values
            "cpu_mean_pct": finite(100.0 * self.ticks_total / clk_tck / elapsed) if elapsed > 0 else None,
            "cpu_peak_pct": max(self.cpu_samples) if self.cpu_samples else None,
            "rss_mean_mib": sum(self.rss_mib) / len(self.rss_mib) if self.rss_mib else None,
            "rss_peak_mib": max(self.rss_mib) if self.rss_mib else None,
            "rss_hwm_mib": self.hwm_kib / 1024.0, "threads_max": self.threads_max,
        }
        if series:
            row["cpu_series_pct"] = [round(v, 2) for v in self.cpu_samples]
            row["rss_series_mib"] = [round(v, 2) for v in self.rss_mib]
        return row


class ProcessSampler:
    """Finds the ROS processes of one domain in /proc and samples their CPU ticks and RSS."""

    def __init__(self, domain, proc_root="/proc", clock=time.monotonic, clk_tck=None,
                 extra_patterns=(), exclude_pids=()):
        self.domain, self.proc_root, self.clock = int(domain), proc_root, clock
        self.clk_tck = clk_tck or os.sysconf("SC_CLK_TCK")
        self.patterns = [re.compile(p) for p in extra_patterns]
        self.exclude_pids = set(exclude_pids)
        self.decided = {}            # (pid, starttime) -> True when tracked; caches skip decisions
        self.records = {}            # (pid, starttime) -> ProcessRecord

    def track(self, pid, role="observer"):
        """Always sample this pid (the observer itself), whatever its environment says."""
        stat = read_stat(self.proc_root, pid)
        if stat is None:
            return
        argv = read_cmdline(self.proc_root, pid)
        key = (pid, stat["starttime"])
        self.decided[key] = True
        self.records[key] = ProcessRecord(pid, stat["starttime"], role, executable_name(argv),
                                          " ".join(argv)[:300], role=role)

    def rescan(self):
        seen = set()
        for pid in list_pids(self.proc_root):
            if pid in self.exclude_pids:
                continue
            stat = read_stat(self.proc_root, pid)
            if stat is None:
                continue
            key = (pid, stat["starttime"])
            seen.add(key)
            if key not in self.decided:
                self.decided[key] = self._classify(pid, stat, key)
        self.decided = {k: v for k, v in self.decided.items() if k in seen or k in self.records}

    def _classify(self, pid, stat, key):
        if not domain_matches(read_environ(self.proc_root, pid), self.domain):
            return False
        argv = read_cmdline(self.proc_root, pid)
        if not is_ros_process(argv, self.patterns):
            return False
        self.records[key] = ProcessRecord(pid, stat["starttime"], node_name(argv),
                                          executable_name(argv), " ".join(argv)[:300])
        return True

    def sample(self):
        for (pid, starttime), record in self.records.items():
            if not record.alive:
                continue
            stat = read_stat(self.proc_root, pid)
            now = self.clock()
            status = read_status(self.proc_root, pid) if stat is not None else None
            if stat is None or status is None or stat["starttime"] != starttime:
                record.alive = False
                continue
            record.add_sample(now, stat["ticks"], status, self.clk_tck)

    def rows(self, role="node", series=False):
        rows = [r.summary(self.clk_tck, series) for r in self.records.values() if r.role == role]
        return sorted(rows, key=lambda row: -(row["cpu_mean_pct"] or 0.0))


def parse_proc_stat_cpu(text):
    """(busy, total, cpu_count) jiffies from /proc/stat; idle and iowait count as idle."""
    busy = total = None
    cpus = 0
    for line in text.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "cpu":
            values = [int(v) for v in parts[1:9]]          # guest columns are already inside user
            total = sum(values)
            busy = total - values[3] - (values[4] if len(values) > 4 else 0)
        elif parts[0].startswith("cpu") and parts[0][3:].isdigit():
            cpus += 1
    return None if total is None else (busy, total, cpus)


class SystemSampler:
    """Whole-machine CPU from /proc/stat, sampled together with the processes."""

    def __init__(self, proc_root="/proc"):
        self.proc_root = proc_root
        self.previous = None
        self.cpus = 0
        self.busy_pct = []               # % of the whole machine per interval

    def sample(self):
        raw = _read_bytes(f"{self.proc_root}/stat")
        parsed = parse_proc_stat_cpu(raw.decode("utf-8", "replace")) if raw else None
        if parsed is None:
            return
        busy, total, self.cpus = parsed
        if self.previous is not None and total > self.previous[1]:
            self.busy_pct.append(100.0 * (busy - self.previous[0]) / (total - self.previous[1]))
        self.previous = (busy, total)

    def summary(self):
        mean = sum(self.busy_pct) / len(self.busy_pct) if self.busy_pct else None
        peak = max(self.busy_pct) if self.busy_pct else None
        return {"cpus": self.cpus, "samples": len(self.busy_pct),
                "busy_mean_pct_of_machine": mean, "busy_peak_pct_of_machine": peak,
                "cores_busy_mean": mean * self.cpus / 100.0 if mean is not None else None,
                "cores_busy_peak": peak * self.cpus / 100.0 if peak is not None else None}


def machine_info(proc_root="/proc"):
    """Host description for the baseline tables: CPU model, logical cores, memory, kernel."""
    info = {"hostname": socket.gethostname(), "kernel": platform.release(),
            "logical_cpus": os.cpu_count(), "cpu_model": None, "mem_total_mib": None}
    cpuinfo = _read_bytes(f"{proc_root}/cpuinfo")
    for line in (cpuinfo or b"").decode("utf-8", "replace").splitlines():
        if line.startswith("model name"):
            info["cpu_model"] = line.partition(":")[2].strip()
            break
    meminfo = _read_bytes(f"{proc_root}/meminfo")
    for line in (meminfo or b"").decode("utf-8", "replace").splitlines():
        if line.startswith("MemTotal:"):
            info["mem_total_mib"] = int(line.split()[1]) / 1024.0
            break
    return info


# ----------------------------------------------------------------------------- measurement window
class Window:
    """Start and end of the measurement window.

    ``use_sim``: the duration counts /clock seconds, otherwise wall seconds.  The window also ends
    after ``max_wall_s`` wall seconds or, with /clock, when /clock stood still for ``stall_wall_s``
    wall seconds (sim paused or dead); ``outcome`` says which one happened.
    """

    def __init__(self, duration_s, use_sim, max_wall_s, stall_wall_s):
        self.duration_s, self.use_sim = duration_s, use_sim
        self.max_wall_s, self.stall_wall_s = max_wall_s, stall_wall_s
        self.started = self.ended = False
        self.outcome = None
        self.wall_start = self.sim_start = self.wall_end = self.sim_end = None
        self._last_sim = self._last_change_wall = None

    def start(self, wall_now, sim_now):
        self.started = True
        self.wall_start, self.sim_start = wall_now, sim_now
        self._last_sim, self._last_change_wall = sim_now, wall_now

    def update(self, wall_now, sim_now):
        if not self.started or self.ended:
            return
        if sim_now is not None and sim_now != self._last_sim:
            self._last_sim, self._last_change_wall = sim_now, wall_now
        if self.use_sim:
            elapsed = sim_now - self.sim_start if sim_now is not None else 0.0
        else:
            elapsed = wall_now - self.wall_start
        if elapsed >= self.duration_s:
            self.finish("complete", wall_now, sim_now)
        elif wall_now - self.wall_start >= self.max_wall_s:
            self.finish("max_wall", wall_now, sim_now)
        elif self.use_sim and wall_now - self._last_change_wall >= self.stall_wall_s:
            self.finish("clock_stalled", wall_now, sim_now)

    def finish(self, outcome, wall_now, sim_now):
        if not self.ended:
            self.ended, self.outcome = True, outcome
            self.wall_end, self.sim_end = wall_now, sim_now

    @property
    def wall_span(self):
        if self.wall_start is None or self.wall_end is None:
            return None
        return self.wall_end - self.wall_start

    @property
    def sim_span(self):
        if self.sim_start is None or self.sim_end is None:
            return None
        return self.sim_end - self.sim_start

    @property
    def sim_over_wall(self):
        return finite(rate_per_second(self.sim_span, self.wall_span)) if self.sim_span is not None else None


# ----------------------------------------------------------------------------- graph endpoints
def full_node_name(namespace, name):
    namespace = namespace or "/"
    return "/" + name if namespace == "/" else namespace.rstrip("/") + "/" + name


def describe_endpoints(infos, own):
    """Endpoint infos (rclpy TopicEndpointInfo) as sorted dicts, without the observer's own node.

    ``own`` is (node name, namespace) of the observer.
    """
    own_full = full_node_name(own[1], own[0])
    entries = []
    for info in infos:
        node = full_node_name(info.node_namespace, info.node_name)
        if node == own_full:
            continue
        reliability = getattr(getattr(info, "qos_profile", None), "reliability", None)
        entries.append({"node": node, "type": getattr(info, "topic_type", None),
                        "reliability": getattr(reliability, "name", None)})
    return sorted(entries, key=lambda entry: entry["node"])


# ----------------------------------------------------------------------------- ROS import under -I
def select_ros_python_paths(pythonpath, prefixes):
    """Entries of a PYTHONPATH string that lie inside one of the ROS/colcon install prefixes."""
    roots = [os.path.normpath(p) for p in prefixes if p and os.path.isabs(p)]
    chosen = []
    for entry in pythonpath.split(os.pathsep):
        if not entry or not os.path.isabs(entry):
            continue
        path = os.path.normpath(entry)
        if any(path == root or path.startswith(root + os.sep) for root in roots) and os.path.isdir(path):
            chosen.append(path)
    return chosen


def ensure_ros_python_path(environ=None):
    """Under ``python3 -I`` re-add the ROS entries of PYTHONPATH (appended); returns what was added."""
    if not sys.flags.isolated:
        return []
    environ = os.environ if environ is None else environ
    prefixes = []
    for variable in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
        prefixes += environ.get(variable, "").split(os.pathsep)
    added = [p for p in select_ros_python_paths(environ.get("PYTHONPATH", ""), prefixes)
             if p not in sys.path]
    sys.path.extend(added)
    return added


def import_ros():
    ensure_ros_python_path()
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise SystemExit(f"cannot import rclpy ({error}); source /opt/ros/humble/setup.bash and the "
                         "workspace install/setup.bash first") from None
    return SimpleNamespace(rclpy=rclpy, QoSProfile=QoSProfile, HistoryPolicy=HistoryPolicy,
                           ReliabilityPolicy=ReliabilityPolicy, DurabilityPolicy=DurabilityPolicy,
                           get_message=get_message)


# ----------------------------------------------------------------------------- the rclpy side
class Observer:
    """Raw best-effort subscriptions and graph queries.  Never publishes."""

    def __init__(self, args, ros, age_cap=AGE_SAMPLE_CAP):
        self.args, self.ros, self.age_cap = args, ros, age_cap
        self.node = ros.rclpy.create_node(f"s1_baseline_observer_{os.getpid()}")
        self.sim_now = None              # latest /clock value, seconds
        self.clock_messages = 0
        self.recording = False
        self.use_sim = False
        self.stats = {}                  # topic -> TopicStats
        self.missing = list(args.topic)  # requested topics without a subscription yet
        self.notes = []

    def pump(self, timeout_s=0.02):
        self.ros.rclpy.spin_once(self.node, timeout_sec=timeout_s)

    def close(self):
        self.node.destroy_node()

    def qos(self, depth):
        ros = self.ros
        return ros.QoSProfile(history=ros.HistoryPolicy.KEEP_LAST, depth=depth,
                              reliability=ros.ReliabilityPolicy.BEST_EFFORT,
                              durability=ros.DurabilityPolicy.VOLATILE)

    def reference_s(self):
        """/clock at receipt (sim time), or system wall time without /clock."""
        return self.sim_now if self.use_sim else time.time()

    def subscribe_clock(self):
        self.node.create_subscription(self.ros.get_message(CLOCK_TYPE), CLOCK_TOPIC,
                                      self._on_clock, self.qos(DEFAULT_QUEUE_DEPTH), raw=True)

    def _on_clock(self, payload):
        stamp = parse_cdr_leading_stamp(payload)
        if stamp is not None:
            self.sim_now = stamp
            self.clock_messages += 1

    def has_publisher(self, topic):
        return len(self.node.get_publishers_info_by_topic(topic)) > 0

    def _callback_for(self, stats):
        def callback(payload):
            if self.recording:
                stats.add(payload, self.reference_s())
        return callback

    def subscribe_missing(self, wall_now=None):
        """Subscribe to requested topics that have appeared in the graph since the last call."""
        if not self.missing:
            return
        graph = dict(self.node.get_topic_names_and_types())
        for topic in list(self.missing):
            types = graph.get(topic)
            if not types:
                continue
            self.missing.remove(topic)
            if len(types) > 1:
                self.notes.append(f"{topic} has several types {types}; using {types[0]}")
            try:
                message_class = self.ros.get_message(types[0])
            except Exception as error:               # message package not on this python path
                self.notes.append(f"{topic}: cannot import {types[0]} ({error})")
                continue
            stats = TopicStats(topic, types[0], stamp_kind_of(message_class), self.age_cap)
            if self.recording:
                stats.activate(self.sim_now, wall_now)
            depth = self.args.depth or DEEP_QUEUE_TOPICS.get(topic, DEFAULT_QUEUE_DEPTH)
            self.node.create_subscription(message_class, topic, self._callback_for(stats),
                                          self.qos(depth), raw=True)
            self.stats[topic] = stats

    def begin_recording(self, wall_now):
        for stats in self.stats.values():
            stats.activate(self.sim_now, wall_now)
        self.recording = True

    def consumer_snapshot(self, topics):
        """Publishers and subscribers per topic from the graph, without the observer's node."""
        own = (self.node.get_name(), self.node.get_namespace())
        snapshot = {}
        for topic in topics:
            publishers = describe_endpoints(self.node.get_publishers_info_by_topic(topic), own)
            subscribers = describe_endpoints(self.node.get_subscriptions_info_by_topic(topic), own)
            snapshot[topic] = {
                "publisher_count": len(publishers), "subscriber_count": len(subscribers),
                "has_consumer": len(subscribers) > 0,
                "publishers": publishers, "subscribers": subscribers}
        return snapshot


# ----------------------------------------------------------------------------- measurement flow
def discover(args, obs):
    """Let the graph settle, subscribe, pick the time base.  Returns (use_sim, failure outcome)."""
    ok = obs.ros.rclpy.ok
    if args.time_base != "wall":
        obs.subscribe_clock()
    end = time.monotonic() + args.discover_s
    next_poll = 0.0
    while ok() and time.monotonic() < end:
        obs.pump(0.05)
        if time.monotonic() >= next_poll:
            obs.subscribe_missing()
            next_poll = time.monotonic() + 0.5
    use_sim = {"sim": True, "wall": False}.get(args.time_base)
    if use_sim is None:                                      # auto
        use_sim = obs.clock_messages > 0 or obs.has_publisher(CLOCK_TOPIC)
    obs.use_sim = use_sim
    if use_sim:
        end = time.monotonic() + CLOCK_WAIT_S
        while ok() and obs.sim_now is None and time.monotonic() < end:
            obs.pump(0.05)
        if obs.sim_now is None:
            return True, "no_clock_message"
    return use_sim, None


def measure(args, obs, sampler, system):
    """Run the window; samples /proc once per second.  Returns (window, ratio samples, consumers)."""
    ok = obs.ros.rclpy.ok
    window = Window(args.duration, obs.use_sim, args.max_wall_s, CLOCK_STALL_WALL_S)
    consumers_start = obs.consumer_snapshot(args.consumer_topic)
    wall0 = time.monotonic()
    window.start(wall0, obs.sim_now)
    sampler.rescan()
    sampler.sample()
    system.sample()
    obs.begin_recording(wall0)
    ratio, last = [], (wall0, obs.sim_now)
    next_sample, next_rescan = wall0 + SAMPLE_PERIOD_S, wall0 + RESCAN_PERIOD_S
    try:
        while ok() and not window.ended:
            obs.pump()
            now = time.monotonic()
            window.update(now, obs.sim_now)
            if now < next_sample:
                continue
            if now >= next_rescan:
                sampler.rescan()
                next_rescan = now + RESCAN_PERIOD_S
            sampler.sample()
            system.sample()
            obs.subscribe_missing(now)
            if obs.use_sim and obs.sim_now is not None and now > last[0]:
                ratio.append((obs.sim_now - last[1]) / (now - last[0]))
            last, next_sample = (now, obs.sim_now), now + SAMPLE_PERIOD_S
    except KeyboardInterrupt:
        pass
    obs.recording = False
    window.finish("interrupted", time.monotonic(), obs.sim_now)    # no-op when it ended by itself
    sampler.sample()
    system.sample()
    consumers_end = obs.consumer_snapshot(args.consumer_topic) if ok() else consumers_start
    return window, ratio, (consumers_start, consumers_end)


def collect_warnings(args, window, topics, processes, observer, consumers, notes):
    """Things that change how the numbers must be read."""
    warnings = list(notes)
    if window.outcome != "complete":
        warnings.append(f"window ended early: {window.outcome}; numbers cover a shorter span")
    if not processes:
        warnings.append(f"no ROS process with ROS_DOMAIN_ID={args.domain} found in /proc")
    for topic in args.topic:
        entry = topics.get(topic, {})
        if not entry.get("present"):
            warnings.append(f"{topic} never appeared in the graph")
        elif entry["messages"] == 0:
            warnings.append(f"{topic} is in the graph but delivered no messages")
        elif entry["age_s"]["negative"]:
            warnings.append(f"{topic}: {entry['age_s']['negative']} messages carry a stamp ahead of "
                            "the observer's /clock (age < 0)")
    if observer and (observer.get("cpu_mean_pct") or 0.0) >= OBSERVER_BUSY_PCT:
        warnings.append(f"observer used {observer['cpu_mean_pct']:.0f} % of a core: message counts "
                        "are lower bounds")
    if not window.use_sim:
        warnings.append("no /clock: ages use system time, no sim/wall ratio")
    changed = [t for t in consumers[0] if (consumers[0][t]["publishers"], consumers[0][t]["subscribers"])
               != (consumers[1][t]["publishers"], consumers[1][t]["subscribers"])]
    if changed:
        warnings.append(f"publishers/subscribers changed during the window: {', '.join(changed)}")
    return warnings


def assemble_result(args, window, topics, process_rows, observer, system, consumers, ratio, notes, host):
    """The JSON document written to --out."""
    ratio_summary = value_summary(ratio) if window.use_sim else None
    result = {
        "schema": "s1_baseline/1", "label": args.label, "mode": args.mode, "domain": int(args.domain),
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "host": host,
        "window": {
            "time_base": "sim" if window.use_sim else "wall", "requested_s": args.duration,
            "outcome": window.outcome, "sim_s": finite(window.sim_span),
            "wall_s": finite(window.wall_span), "sim_over_wall": window.sim_over_wall,
            "sim_over_wall_per_second": ratio_summary},
        "processes": process_rows,
        "process_totals": {
            "cpu_mean_pct_sum": sum(r["cpu_mean_pct"] or 0.0 for r in process_rows),
            "rss_mean_mib_sum": sum(r["rss_mean_mib"] or 0.0 for r in process_rows)},
        "observer_self": observer, "system": system,
        "topics": topics,
        "consumers": {topic: dict(consumers[1][topic], changed_during_window=(
            consumers[0][topic]["publishers"], consumers[0][topic]["subscribers"]) != (
            consumers[1][topic]["publishers"], consumers[1][topic]["subscribers"]))
            for topic in consumers[1]},
    }
    result["warnings"] = collect_warnings(args, window, topics, process_rows, observer, consumers, notes)
    return result


def run(args):
    """Attach to the domain, measure, return the result dict."""
    ros = import_ros()
    os.environ["ROS_DOMAIN_ID"] = str(int(args.domain))      # read by rclpy.init()
    ros.rclpy.init()
    obs = Observer(args, ros)
    sampler = ProcessSampler(args.domain, extra_patterns=args.process_pattern,
                             exclude_pids=(os.getpid(),))
    sampler.track(os.getpid())
    system = SystemSampler()
    try:
        use_sim, failure = discover(args, obs)
        if failure:
            window = Window(args.duration, use_sim, args.max_wall_s, CLOCK_STALL_WALL_S)
            window.finish(failure, time.monotonic(), obs.sim_now)
            snapshot = obs.consumer_snapshot(args.consumer_topic)
            ratio, consumers = [], (snapshot, snapshot)
        else:
            window, ratio, consumers = measure(args, obs, sampler, system)
        topics = {}
        for topic in args.topic:
            stats = obs.stats.get(topic)
            topics[topic] = (stats.result(window.sim_end, window.wall_end) if stats
                             else {"present": False, "messages": 0})
        observer_rows = sampler.rows("observer")
        return assemble_result(
            args, window, topics, sampler.rows("node", series=args.series),
            observer_rows[0] if observer_rows else None, system.summary(), consumers, ratio,
            obs.notes, machine_info())
    finally:
        obs.close()
        try:
            ros.rclpy.shutdown()
        except Exception:               # SIGINT already shut the context down
            pass


# ----------------------------------------------------------------------------- summary table
def render_table(headers, rows, left=(0,)):
    cells = [[str(cell) for cell in row] for row in rows]
    widths = [max([len(header)] + [len(row[i]) for row in cells]) for i, header in enumerate(headers)]

    def line(values):
        return "  ".join(v.ljust(w) if i in left else v.rjust(w)
                         for i, (v, w) in enumerate(zip(values, widths))).rstrip()
    return "\n".join([line(headers), line(["-" * w for w in widths])] + [line(row) for row in cells])


def fmt(value, spec="{:.1f}", scale=1.0):
    return "-" if value is None else spec.format(value * scale)


def summary_header(result):
    window, host, system = result["window"], result["host"], result["system"]
    mem = host.get("mem_total_mib")
    lines = [
        f"S1 baseline  label={result['label'] or '-'}  mode={result['mode'] or '-'}  "
        f"domain={result['domain']}  outcome={window['outcome']}",
        f"host: {host.get('hostname')}  {host.get('cpu_model') or '?'}  {host.get('logical_cpus')} "
        f"logical CPUs  {fmt(mem, '{:.1f}', 1 / 1024.0)} GiB",
        f"window ({window['time_base']} time, {window['requested_s']:g} s requested): "
        f"sim {fmt(window['sim_s'], '{:.2f}')} s, wall {fmt(window['wall_s'], '{:.1f}')} s, "
        f"sim/wall {fmt(window['sim_over_wall'], '{:.4f}')}",
    ]
    per_second = window.get("sim_over_wall_per_second")
    if per_second and per_second["n"]:
        lines.append(f"sim/wall per second: min {per_second['min']:.4f}  p50 {per_second['p50']:.4f}  "
                     f"max {per_second['max']:.4f}")
    lines.append(f"machine busy: mean {fmt(system['cores_busy_mean'], '{:.2f}')} cores, peak "
                 f"{fmt(system['cores_busy_peak'], '{:.2f}')} cores of {system['cpus']}")
    return lines


def summary_processes(result):
    rows = [[row["pid"], row["name"] if row["name"] == row["exe"] else f"{row['name']} ({row['exe']})",
             fmt(row["cpu_mean_pct"]), fmt(row["cpu_peak_pct"]), fmt(row["rss_mean_mib"]),
             fmt(row["rss_peak_mib"]), row["samples"], "" if row["alive_at_end"] else "exited"]
            for row in result["processes"]]
    totals = result["process_totals"]
    rows.append(["", "sum (RSS double-counts shared pages)", fmt(totals["cpu_mean_pct_sum"]), "",
                 fmt(totals["rss_mean_mib_sum"]), "", "", ""])
    observer = result["observer_self"]
    if observer:
        rows.append([observer["pid"], "observer (this script)", fmt(observer["cpu_mean_pct"]),
                     fmt(observer["cpu_peak_pct"]), fmt(observer["rss_mean_mib"]),
                     fmt(observer["rss_peak_mib"]), observer["samples"], ""])
    table = render_table(["pid", "process", "CPU mean %", "CPU peak %", "RSS mean MiB",
                          "RSS peak MiB", "samples", ""], rows, left=(1, 7))
    return ["", "Per process (CPU in % of one core, 1 s samples):", table]


def summary_topics(result):
    rows = []
    for topic, entry in result["topics"].items():
        if not entry.get("present"):
            rows.append([topic, "not in graph", "", "", "", "", "", "", ""])
            continue
        age = entry["age_s"]
        rows.append([topic, entry["type"].split("/")[-1], entry["messages"],
                     fmt(entry["hz_per_sim_s"], "{:.2f}"), fmt(entry["hz_per_wall_s"], "{:.2f}"),
                     fmt(age["p50"], "{:.1f}", 1000.0), fmt(age["p95"], "{:.1f}", 1000.0),
                     fmt(age["max"], "{:.1f}", 1000.0), age["negative"]])
    table = render_table(["topic", "type", "msgs", "Hz per sim s", "Hz per wall s", "age p50 ms",
                          "age p95 ms", "age max ms", "age<0"], rows, left=(0, 1))
    return ["", "Per topic (age = /clock at receipt - header.stamp):", table]


def summary_consumers(result):
    rows = []
    for topic, entry in result["consumers"].items():
        pubs = ", ".join(p["node"] for p in entry["publishers"]) or "-"
        subs = ", ".join(s["node"] for s in entry["subscribers"]) or "NO CONSUMER"
        rows.append([topic, entry["publisher_count"], entry["subscriber_count"], pubs, subs])
    table = render_table(["topic", "pubs", "subs", "publisher nodes", "subscriber nodes"], rows,
                         left=(0, 3, 4))
    return ["", "Publishers and subscribers (observer excluded):", table]


def format_summary(result):
    lines = summary_header(result) + summary_processes(result) + summary_topics(result)
    lines += summary_consumers(result)
    if result["warnings"]:
        lines += ["", "Warnings:"] + [f"  - {warning}" for warning in result["warnings"]]
    return "\n".join(lines)


def sanitize(value):
    """Recursively replace NaN / infinity by None so the JSON stays strictly valid."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: sanitize(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [sanitize(item) for item in value]
    return value


def write_json(path, result):
    result = sanitize(result)
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                         encoding="utf-8")
    os.replace(temporary, path)
    return path


# ----------------------------------------------------------------------------- command line
def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--domain", type=int, default=int(os.environ.get("ROS_DOMAIN_ID") or 0),
                        help="ROS_DOMAIN_ID of the running stack (default: the environment's)")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="window length: /clock seconds with /clock, else wall seconds")
    parser.add_argument("--out", required=True, help="JSON output path")
    parser.add_argument("--mode", choices=["truth", "lio"], default=None,
                        help="localization mode of the stack; a label only")
    parser.add_argument("--label", default="", help="free-text label stored in the JSON")
    parser.add_argument("--time-base", choices=["auto", "sim", "wall"], default="auto",
                        help="auto: sim when /clock has a publisher; sim: require /clock; wall: ignore it")
    parser.add_argument("--topic", action="append", default=None,
                        help="topic to measure rate and age on (repeatable; default: the five of 9.2)")
    parser.add_argument("--consumer-topic", action="append", default=None,
                        help="topic to list publishers/subscribers of (repeatable)")
    parser.add_argument("--process-pattern", action="append", default=[],
                        help="regex on the command line to include more processes of the domain "
                             "(for example '.' for all of them)")
    parser.add_argument("--discover-s", type=float, default=5.0,
                        help="wall seconds to let discovery settle before the window starts")
    parser.add_argument("--max-wall-s", type=float, default=7200.0,
                        help="stop the window after this many wall seconds")
    parser.add_argument("--depth", type=int, default=None,
                        help="queue depth of every topic subscription (default: per topic)")
    parser.add_argument("--series", action="store_true",
                        help="store the per-second CPU and RSS series of every process in the JSON")
    return parser


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 0 <= args.domain <= 232:
        parser.error("--domain must be 0..232")
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    for pattern in args.process_pattern:
        try:
            re.compile(pattern)
        except re.error as error:
            parser.error(f"--process-pattern {pattern!r}: {error}")
    args.topic = list(dict.fromkeys(args.topic or DEFAULT_RATE_TOPICS))
    args.consumer_topic = list(dict.fromkeys(args.consumer_topic or DEFAULT_CONSUMER_TOPICS))
    # `python3 scripts/x.py` puts scripts/ first on sys.path; nothing here imports local modules.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path[:] = [p for p in sys.path if not (p and os.path.abspath(p) == script_dir)]
    signal.signal(signal.SIGTERM, _raise_interrupt)
    try:
        result = run(args)
    except KeyboardInterrupt:
        print("interrupted before the window started; nothing written", file=sys.stderr)
        return 130
    path = write_json(args.out, result)
    print(format_summary(result))
    print(f"\nJSON: {path}")
    return 0 if result["window"]["outcome"] == "complete" else 3


if __name__ == "__main__":
    sys.exit(main())
