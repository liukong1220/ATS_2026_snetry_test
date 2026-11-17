#!/usr/bin/env python3
"""Workspace-only ROS domain reuse with owned-process cleanup and an audit trail.

Reuse requires explicit session authorization. No global DDS/daemon configuration
is changed and no process is killed just because it uses the requested domain.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {5, 77, 91, 92}


def validate_domain(value):
    domain = int(value)
    if not 0 <= domain <= 232 or domain in EXCLUDED:
        raise ValueError("domain must be 0..232 excluding 5, 77, 91, 92")
    return domain


def process_identity(pid, proc_root=Path("/proc")):
    try:
        directory = proc_root / str(pid)
        stat = (directory / "stat").read_text().rsplit(")", 1)[1].split()
        if stat[0] == "Z":
            return None
        env = (directory / "environ").read_bytes().split(b"\0")
        domains = [item.split(b"=", 1)[1].decode() for item in env
                   if item.startswith(b"ROS_DOMAIN_ID=")]
        return {"pid": int(pid), "ppid": int(stat[1]), "pgid": int(stat[2]),
                "start_ticks": int(stat[19]), "domain": domains[-1] if domains else "0",
                "uid": directory.stat().st_uid}
    except (OSError, ValueError, IndexError):
        return None


def domain_processes(domain):
    result = []
    for directory in Path("/proc").iterdir():
        if not directory.name.isdecimal() or int(directory.name) == os.getpid():
            continue
        item = process_identity(int(directory.name))
        # Absent ROS_DOMAIN_ID is the DDS default, but non-ROS host processes
        # with no such variable are not evidence of a domain-0 participant.
        if item and item["domain"] == str(domain):
            try:
                environment = (directory / "environ").read_bytes()
                command = (directory / "cmdline").read_bytes().lower()
                if b"ROS_DOMAIN_ID=" not in environment and not any(
                        token in command for token in (b"ros2", b"rclpy", b"_node")):
                    continue
            except OSError:
                continue
            result.append(item)
    return result


def domain_ports(domain, socket_text):
    base = 7400 + 250 * domain
    occupied = set()
    for line in socket_text.splitlines():
        fields = line.split()
        if len(fields) < 5:
            continue
        try:
            port = int(fields[3].rsplit(":", 1)[1])
        except (ValueError, IndexError):
            continue
        if base <= port < base + 250:
            occupied.add(port)
    return sorted(occupied)


def wlan_contract():
    xml_path = ROOT / "log/nav_acceptance/dds/wlan.xml"
    if os.environ.get("ROS_LOCALHOST_ONLY") != "0":
        raise ValueError("source workspace wlan_env.sh: ROS_LOCALHOST_ONLY must be 0")
    if os.environ.get("CYCLONEDDS_URI") != "file://" + str(xml_path):
        raise ValueError("CYCLONEDDS_URI must select the workspace wlan.xml")
    if os.environ.get("RMW_IMPLEMENTATION") != "rmw_cyclonedds_cpp":
        raise ValueError("workspace reuse requires rmw_cyclonedds_cpp")
    tree = ET.parse(xml_path)
    names = [node.attrib.get("name") for node in tree.iter()
             if node.tag.rsplit("}", 1)[-1] == "NetworkInterface"]
    if len(names) != 1 or not names[0]:
        raise ValueError("wlan.xml must select one explicit WLAN interface")
    interface = Path("/sys/class/net") / names[0]
    if not (interface / "wireless").exists() or (interface / "operstate").read_text().strip() != "up":
        raise ValueError("configured WLAN is not online: " + names[0])
    return {"interface": names[0], "xml": str(xml_path), "ROS_LOCALHOST_ONLY": "0"}


def preflight(domain):
    sockets = subprocess.run(["ss", "-H", "-uan"], capture_output=True, text=True, check=True)
    result = {"domain": domain, "wlan": wlan_contract(), "processes": domain_processes(domain),
              "udp_ports": domain_ports(domain, sockets.stdout), "checked_unix_s": time.time()}
    result["clear"] = not result["processes"] and not result["udp_ports"]
    return result


def same_process(expected):
    current = process_identity(expected["pid"])
    return (current is not None and current["uid"] == os.getuid() and
            current["start_ticks"] == expected["start_ticks"] and
            current["domain"] == expected["domain"])


def cleanup_owned(owned):
    """PID plus kernel start time avoids signaling a recycled or foreign PID."""
    for sig, grace in ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
        alive = [item for item in owned if same_process(item)]
        if not alive:
            break
        for item in alive:
            try:
                os.kill(item["pid"], sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline and any(same_process(item) for item in alive):
            time.sleep(0.1)
    return [item for item in owned if same_process(item)]


def write_manifest(path, payload):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("check", "run", "cleanup"))
    parser.add_argument("--domain", type=validate_domain, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--reuse-authorized", action="store_true",
                        help="explicit user authorization for this batch, recorded in manifest")
    args, command = parser.parse_known_args(argv)
    if command[:1] == ["--"]:
        command = command[1:]
    if not args.reuse_authorized:
        parser.error("historical reuse requires explicit --reuse-authorized")
    locks = ROOT / "log/nav_acceptance/dds/domain_locks"
    locks.mkdir(parents=True, exist_ok=True)
    args.run_dir = args.run_dir.resolve()
    try:
        args.run_dir.relative_to(ROOT / "log/nav_acceptance")
    except ValueError:
        parser.error("run-dir must be inside workspace log/nav_acceptance")
    args.run_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = args.run_dir / "domain_guard.json"
    with (locks / ("domain_%d.lock" % args.domain)).open("a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.mode == "cleanup":
            record = json.loads(manifest_path.read_text())
            if record["domain"] != args.domain or not record.get("reuse_authorized"):
                raise ValueError("cleanup manifest domain/authorization mismatch")
            record["cleanup_survivors"] = cleanup_owned(record.get("owned_processes", []))
            record["after_cleanup"] = preflight(args.domain)
            write_manifest(manifest_path, record)
            print(json.dumps(record["after_cleanup"], ensure_ascii=False))
            return 0 if record["after_cleanup"]["clear"] else 2
        before = preflight(args.domain)
        if args.mode == "check":
            print(json.dumps(before, ensure_ascii=False))
            return 0 if before["clear"] else 2
        if manifest_path.exists():
            raise ValueError("run directory already has a domain manifest; use a new batch")
        record = {"domain": args.domain, "reuse_authorized": True, "before": before,
                  "command": command, "owned_processes": []}
        write_manifest(manifest_path, record)
        if not before["clear"]:
            raise ValueError("domain is occupied; no unrelated process was terminated")
        if not command:
            parser.error("run requires a command after --")
        environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))
        child = subprocess.Popen(command, env=environment, cwd=ROOT, start_new_session=True)
        record["child_pid"] = child.pid
        owned = {}

        def forward(sig, _frame):
            try:
                os.killpg(child.pid, sig)
            except ProcessLookupError:
                pass

        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, forward)
        while child.poll() is None:
            processes = domain_processes(args.domain)
            descendants = {child.pid} | set(owned)
            for _ in range(len(processes) + 1):
                expanded = descendants | {p["pid"] for p in processes if p["ppid"] in descendants}
                if expanded == descendants:
                    break
                descendants = expanded
            for item in processes:
                if item["pid"] in descendants:
                    owned[item["pid"]] = item
            record["owned_processes"] = list(owned.values())
            write_manifest(manifest_path, record)
            time.sleep(1.0)
        record["command_exit"] = child.wait()
        record["cleanup_survivors"] = cleanup_owned(list(owned.values()))
        record["after_cleanup"] = preflight(args.domain)
        write_manifest(manifest_path, record)
        print(json.dumps({"command_exit": record["command_exit"],
                          "domain_clear": record["after_cleanup"]["clear"]}))
        return record["command_exit"] if record["command_exit"] else (
            0 if record["after_cleanup"]["clear"] else 2)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, ET.ParseError) as error:
        print("DOMAIN_GUARD: " + str(error), file=sys.stderr)
        sys.exit(2)
