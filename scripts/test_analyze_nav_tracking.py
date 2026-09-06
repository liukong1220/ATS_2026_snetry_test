#!/usr/bin/env python3
"""Offline test for scripts/analyze_nav_tracking.py.

No ROS graph and no simulation.  Synthetic recorder artifacts are built with a
known geometry so each of the three questions the analyzer exists to separate can
be forced to a known answer:

* case ``actual_drifts_into_wall``: the committed reference is collision-free at
  publish and the driven pose leaves it laterally into an occupied band.
  Q1 yes, Q2 yes -- the tracking/execution owner.
* case ``reference_unsafe_at_publish``: the committed reference already conflicts
  against the snapshot in force when it was published.  Q1 no -- the
  revalidation/commit-timing owner.
* case ``map_update_occupies_rest_pose``: the pose never moves into anything, and
  a later snapshot marks the cell it is resting on as occupied.  Q3 yes -- the
  generation/heartbeat owner.
* case ``clean_run``: nothing conflicts, so a clean run must report clean rather
  than finding something.

Every case places the poses in ``odom`` and the grids in ``map`` with a
*non-identity* ``map <- odom`` transform, because an analyzer that quietly assumed
the two frames were the same would still pass a test written in one frame.
"""

import gzip
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analyze_nav_tracking as ana  # noqa: E402

FAILURES = []

# The transform every case uses.  Deliberately not identity and not axis aligned.
MAP_FROM_ODOM = {"stamp_identity": "10.000000000", "x": 1.0, "y": 0.5, "yaw": 0.3}

GRID_WIDTH = 40
GRID_HEIGHT = 40
RESOLUTION = 0.1
WALL_FIRST_CELL = 30  # world x >= 3.0 m in the map frame is occupied
FOOTPRINT = {"length": 0.60, "width": 0.50, "safety_margin": 0.02}
ROBOT_Y = 1.5


def check(condition, message):
    if condition:
        print("ok: " + message)
    else:
        FAILURES.append(message)
        print("FAIL: " + message)


def odom_from_map(transform, pose):
    """Inverse of analyze_nav_tracking.transform_pose.

    The fixtures are designed in the map frame because that is where the geometry
    is meaningful, then pushed back into odom so the analyzer has to apply the
    recorded transform to recover them.  If it skipped the transform, every case
    would land in the wrong cell and the expectations below would fail.
    """
    dx = pose[0] - transform["x"]
    dy = pose[1] - transform["y"]
    cos_yaw = math.cos(transform["yaw"])
    sin_yaw = math.sin(transform["yaw"])
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
        ana.fe.normalize_angle(pose[2] - transform["yaw"]),
    )


def wall_grid(extra_blocked=()):
    """Occupancy with a solid wall plus any extra blocked cells."""
    data = [0] * (GRID_WIDTH * GRID_HEIGHT)
    for iy in range(GRID_HEIGHT):
        for ix in range(GRID_WIDTH):
            if ix >= WALL_FIRST_CELL:
                data[iy * GRID_WIDTH + ix] = 100
    for ix, iy in extra_blocked:
        data[iy * GRID_WIDTH + ix] = 100
    return data


def signed_distance_field(data):
    """Brute-force distance from each cell centre to the nearest occupied centre.

    Only a diagnostic in the analyzer, but it must be present and plausible for
    the clearance and wall-normal reporting to be exercised at all.
    """
    occupied = [(ix, iy)
                for iy in range(GRID_HEIGHT) for ix in range(GRID_WIDTH)
                if data[iy * GRID_WIDTH + ix] >= 50]
    field = []
    for iy in range(GRID_HEIGHT):
        for ix in range(GRID_WIDTH):
            best = None
            for ox, oy in occupied:
                distance = math.hypot(ox - ix, oy - iy) * RESOLUTION
                if best is None or distance < best:
                    best = distance
            if data[iy * GRID_WIDTH + ix] >= 50:
                best = -(best if best else 0.0)
            field.append(best if best is not None else 99.0)
    return field


def grid_info():
    return {"width": GRID_WIDTH, "height": GRID_HEIGHT, "resolution": RESOLUTION,
            "origin_x": 0.0, "origin_y": 0.0, "origin_yaw": 0.0}


def digest_of(data):
    hasher = hashlib.sha256()
    hasher.update(json.dumps(data).encode("utf-8"))
    return hasher.hexdigest()[:32]


class Fixture(object):
    """Builds one synthetic nav_tracking_recorder output directory."""

    def __init__(self, root, name):
        self.dir = os.path.join(root, name)
        os.makedirs(os.path.join(self.dir, "snapshot_payload"))
        os.makedirs(os.path.join(self.dir, "layers"))
        self.snapshots = []
        self.samples = []
        self.references = []
        self.events = []
        self.tick = 0
        self.mono = 0.0

    def add_snapshot(self, data, publication_sequence, source_generation,
                     threshold=50, unknown_is_obstacle=True, frame_id="map"):
        digest = digest_of(data)
        identity = {
            "stamp_identity": "%d.000000000" % (100 + publication_sequence),
            "frame_id": frame_id,
            "source_stamp_identity": "%d.500000000" % (100 + publication_sequence),
            "ready": True,
            "unknown_is_obstacle": unknown_is_obstacle,
            "occupied_value_threshold": threshold,
            "localization_epoch": 1,
            "source_generation": source_generation,
            "publication_sequence": publication_sequence,
            "occupancy_digest": digest,
            "cell_count": len(data),
            "info": grid_info(),
        }
        name = "snapshot_%020d_%s.json.gz" % (publication_sequence, digest)
        path = os.path.join(self.dir, "snapshot_payload", name)
        with gzip.open(path, "wt") as handle:
            json.dump({"identity": identity, "occupancy": data,
                       "signed_distance_m": signed_distance_field(data)}, handle)
        record = dict(identity)
        record["mono_s"] = self.mono
        record["payload"] = os.path.join("snapshot_payload", name)
        self.snapshots.append(record)
        return identity

    def add_layer(self, topic, data, frame_id="map"):
        digest = digest_of([topic, data])
        safe_topic = topic.strip("/").replace("/", "__")
        name = "%s_%s.json.gz" % (safe_topic, digest)
        with gzip.open(os.path.join(self.dir, "layers", name), "wt") as handle:
            json.dump({"topic": topic, "frame_id": frame_id,
                       "stamp_identity": "100.000000000", "mono_s": 0.0,
                       "digest": digest, "info": grid_info(), "data": data}, handle)

    def add_reference(self, map_poses, frame_id="odom",
                      source="/minco/reference_path",
                      stamp_identity="200.000000000"):
        poses = [list(odom_from_map(MAP_FROM_ODOM, pose)) for pose in map_poses]
        digest = digest_of(poses)
        record = {
            "source": source, "frame_id": frame_id,
            "stamp_identity": stamp_identity, "digest": digest,
            "pose_count": len(poses), "poses": poses,
            "mono_s": self.mono, "reference_index": len(self.references),
        }
        self.references.append(record)
        return record

    def add_sample(self, map_pose, snapshot_identity, reference,
                   speed=0.3, selected=None, emergency_stop=False,
                   frame_id="odom", transform=MAP_FROM_ODOM):
        """One tick, with the actual pose given in the map frame for readability."""
        self.tick += 1
        self.mono += 0.02
        pose = odom_from_map(MAP_FROM_ODOM, map_pose) if transform else map_pose
        actual = {
            "frame_id": frame_id, "child_frame_id": "gimbal_yaw_odom",
            "stamp_identity": "300.%09d" % (self.tick * 1000000),
            "x": pose[0], "y": pose[1], "z": 0.0, "yaw": pose[2],
            "vx_body": speed, "vy_body": 0.0, "wz_body": 0.0,
        }
        if selected is None:
            selected = {"vx": speed, "vy": 0.0, "wz": 0.0}
        error = None
        if reference is not None:
            error = {"nearest_index": 0, "lateral_m": 0.0, "longitudinal_m": 0.0,
                     "yaw_error_rad": 0.0, "distance_m": 0.0}
        self.samples.append({
            "tick": self.tick, "mono_s": self.mono, "actual": actual,
            "actual_age_s_mono": 0.004, "selected": selected,
            "selected_age_s_mono": 0.006,
            "autonomy_raw": selected, "autonomy_raw_age_s_mono": 0.007,
            "motion_control": {"linear_x": selected["vx"],
                               "linear_y": selected["vy"],
                               "angular_z": selected["wz"]},
            "motion_control_age_s_mono": 0.008,
            "telemetry": {"stamp_identity": "300.000000000",
                          "contact_violation_count": 0, "max_contact_force": 0.0,
                          "sequence": self.tick, "command_vx": selected["vx"],
                          "command_vy": 0.0, "command_wz": 0.0,
                          "measured_vx": speed, "measured_vy": 0.0,
                          "measured_wz": 0.0,
                          "drive_speed_saturation_count": 0,
                          "drive_acceleration_saturation_count": 0,
                          "steer_rate_saturation_count": 0,
                          "drive_speed_saturated": [False] * 4,
                          "steer_rate_saturated": [False] * 4},
            "telemetry_age_s_mono": 0.005,
            "emergency_stop": emergency_stop, "emergency_stop_age_s_mono": 0.01,
            "map_ready": True, "map_ready_age_s_mono": 0.01,
            "map_status": {"ready": True, "rog_generation": 7,
                           "publication_sequence": 11},
            "snapshot": snapshot_identity, "snapshot_age_s_mono": 0.02,
            "execution": {"mode": 1, "command_sequence": 3, "goal_id": 5,
                          "reference_frame_id": "odom"},
            "execution_age_s_mono": 0.02,
            "planner_status": {"state": 2, "failure_reason": 0, "goal_id": 5},
            "reference_digest": reference["digest"] if reference else None,
            "reference_stamp_identity":
                reference["stamp_identity"] if reference else None,
            "reference_frame": reference["frame_id"] if reference else None,
            "reference_pose_count": reference["pose_count"] if reference else None,
            "reference_age_s_mono": 0.03,
            "tracking_error": error,
            "map_from_odom": dict(transform) if transform else None,
        })

    def write(self, extra_summary=None):
        for name, records in (("samples.jsonl", self.samples),
                              ("references.jsonl", self.references),
                              ("snapshots.jsonl", self.snapshots),
                              ("events.jsonl", self.events)):
            with open(os.path.join(self.dir, name), "w", encoding="utf-8") as handle:
                for record in records:
                    handle.write(json.dumps(record, sort_keys=True) + "\n")
        summary = {
            "reason": "test", "duration_s_mono": self.mono, "ticks": self.tick,
            "rate_hz": 50.0, "grid_frame": "map", "control_frame": "odom",
            "messages_received": {}, "topics_without_messages": [],
            "snapshot_payloads_written": len(self.snapshots),
            "snapshot_payloads_truncated": False,
            "layer_payloads_written": 0, "layer_payloads_truncated": False,
            "motion_control_message_available": True,
        }
        if extra_summary:
            summary.update(extra_summary)
        with open(os.path.join(self.dir, "summary.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
        return self.dir


def run_analyzer(directory, extra_args=()):
    output = os.path.join(directory, "verdict.json")
    argv = ["--input-dir", directory, "--output", output,
            "--length", str(FOOTPRINT["length"]),
            "--width", str(FOOTPRINT["width"]),
            "--safety-margin", str(FOOTPRINT["safety_margin"])]
    argv.extend(extra_args)
    status = ana.main(argv)
    with open(output, "r", encoding="utf-8") as handle:
        return status, json.load(handle)


def case_actual_drifts_into_wall(root):
    """Reference safe at publish, actual driven past it into the occupied band."""
    fixture = Fixture(root, "actual_drifts_into_wall")
    data = wall_grid()
    snapshot = fixture.add_snapshot(data, publication_sequence=1,
                                    source_generation=5)
    # Provenance: the wall comes from the static map layer.  The terrain layer is
    # deliberately published in another frame to prove the analyzer refuses to look
    # a cell up across frames.
    fixture.add_layer("/map", data, frame_id="map")
    fixture.add_layer("/traversability_grid", wall_grid(), frame_id="odom")
    # 2.6 + 0.32 = 2.92 m < 3.0 m, so the reference clears the wall.
    reference = fixture.add_reference(
        [(x, ROBOT_Y, 0.0) for x in (0.8, 1.1, 1.4, 1.7, 2.0, 2.3, 2.6)])
    for x in (0.8, 1.1, 1.4, 1.7, 2.0, 2.3, 2.6):
        fixture.add_sample((x, ROBOT_Y, 0.0), snapshot, reference, speed=0.3)
    # 2.9 + 0.32 = 3.22 m, so the driven footprint is inside the wall.
    fixture.add_sample((2.9, ROBOT_Y, 0.0), snapshot, reference, speed=0.02,
                       selected={"vx": 0.0, "vy": 0.0, "wz": 0.0},
                       emergency_stop=True)
    fixture.add_sample((2.9, ROBOT_Y, 0.0), snapshot, reference, speed=0.0,
                       selected={"vx": 0.0, "vy": 0.0, "wz": 0.0},
                       emergency_stop=True)
    directory = fixture.write()
    status, report = run_analyzer(directory)

    answers = report["answers"]
    check(status == 1, "drift case exits 1 (complete, conflicts found)")
    check(answers["q1_reference_collision_free_at_publish"]["answer"] == "yes",
          "Q1: the committed reference was collision-free at publish")
    check(answers["q1_reference_collision_free_at_publish"]
          ["references_with_collisions"] == 0,
          "Q1 counts zero colliding references")
    check(answers["q2_actual_left_reference_envelope"]["answer"] == "yes",
          "Q2: the driven pose sequence left the verified envelope")
    check(answers["q3_same_pose_flipped_free_to_occupied"]["answer"] == "no",
          "Q3: no map update was responsible (single snapshot)")
    check("MPC tracking error" in answers["q2_actual_left_reference_envelope"]
          ["routing"],
          "Q2 routes to the tracking/execution owner")

    tracking = report["actual_tracking"]
    conflict = tracking["first_discrete_conflict"]
    check(conflict is not None, "a first discrete conflict is reported")
    check(abs(conflict["grid_pose"]["x"] - 2.9) < 1e-9 and
          abs(conflict["grid_pose"]["y"] - ROBOT_Y) < 1e-9,
          "the conflicting pose is recovered in the map frame (%.6f, %.6f), which "
          "only happens if the recorded transform was applied"
          % (conflict["grid_pose"]["x"], conflict["grid_pose"]["y"]))
    cell = conflict["verdict"]["first_collision"]["cell"]
    check(cell["class"] == "occupied" and cell["value"] == 100,
          "the first conflict names an occupied cell with its value")
    check(cell["ix"] >= WALL_FIRST_CELL,
          "the conflicting cell is inside the wall band (ix=%s)" % cell["ix"])
    check(tracking["swept_colliding_segments"] >= 1,
          "the motion between samples is swept, not only sampled")
    clearance = conflict["verdict"]["first_collision"]["clearance"]
    check(clearance["min_clearance_m"] is not None and
          clearance["min_clearance_m"] <= 0.0,
          "footprint clearance at the conflict is non-positive (%s m)"
          % clearance["min_clearance_m"])
    normal = conflict["verdict"]["first_collision"]["obstacle_normal"]
    check(normal is not None and normal["x"] < -0.5,
          "the wall normal is derived from the ESDF and points away from the wall "
          "(%s)" % (normal and round(normal["x"], 3)))

    provenance = conflict["verdict"]["first_collision"]["provenance"]
    check(provenance["/map"].get("class") == "occupied" and
          provenance["/map"].get("value") == 100,
          "provenance attributes the cell to the static map layer")
    check(provenance["/traversability_grid"].get("frame_unpaired") is True,
          "a layer published in another frame is reported unpaired, not sampled")

    stop = report["stop_envelope"]
    check(stop["reached_rest"] is True, "the stop envelope found a rest pose")
    onset = stop["onsets"]["last_moving_sample"]
    check(onset["found"] and abs(onset["displacement_to_rest_m"] - 0.3) < 1e-6,
          "displacement from the last moving sample to rest is 0.3 m (got %s)"
          % onset.get("displacement_to_rest_m"))
    check(stop["onsets"]["first_emergency_stop_true"]["found"] is True,
          "the emergency-stop onset is reported alongside the braking onset")
    check(any("physical contact" in gap for gap in report["evidence_gaps"]),
          "physical contact is reported as not evaluated by this tool")
    source = report["footprint_params_source"]
    check(source["unknown_is_obstacle"] == "snapshot" and
          source["obstacle_value_threshold"] == "snapshot",
          "threshold and unknown handling come from the snapshot, not a guess")


def case_reference_unsafe_at_publish(root):
    """The committed reference already conflicts against the snapshot in force."""
    fixture = Fixture(root, "reference_unsafe_at_publish")
    data = wall_grid()
    snapshot = fixture.add_snapshot(data, publication_sequence=1,
                                    source_generation=5)
    fixture.add_layer("/map", data, frame_id="map")
    reference = fixture.add_reference(
        [(x, ROBOT_Y, 0.0) for x in (0.8, 1.4, 2.0, 2.6, 2.9)])
    for x in (0.8, 1.1, 1.4, 1.7, 2.0):
        fixture.add_sample((x, ROBOT_Y, 0.0), snapshot, reference, speed=0.3)
    fixture.add_sample((2.0, ROBOT_Y, 0.0), snapshot, reference, speed=0.0,
                       selected={"vx": 0.0, "vy": 0.0, "wz": 0.0})
    directory = fixture.write()
    status, report = run_analyzer(directory)

    answers = report["answers"]
    check(status == 1, "unsafe-reference case exits 1")
    check(answers["q1_reference_collision_free_at_publish"]["answer"] == "no",
          "Q1: the reference was not collision-free at publish")
    check("revalidation" in answers["q1_reference_collision_free_at_publish"]
          ["routing"],
          "Q1 routes to the revalidation / commit-timing owner")
    check(answers["q2_actual_left_reference_envelope"]["answer"] == "no",
          "Q2: the driven pose itself stayed clear, so the owner is not tracking")
    entry = [item for item in report["reference_at_publish"]
             if item.get("evaluated")][0]
    check(entry["transform_applied"] is True,
          "the reference was transformed from odom into the snapshot frame")
    first = entry["verdict"]["first_collision"]
    check(abs(first["center_x"] - 2.9) < 1e-9,
          "the reference's first conflicting pose is the 2.9 m one (got %.6f)"
          % first["center_x"])
    check(first["trajectory_index"] == 4,
          "the conflict is attributed to the reference index that owns it (got %s)"
          % first["trajectory_index"])


def case_map_update_occupies_rest_pose(root):
    """A stationary pose is occupied only by a later snapshot."""
    fixture = Fixture(root, "map_update_occupies_rest_pose")
    first_data = wall_grid()
    # One cell under the resting footprint centre: (2.0, 1.5) m -> cell (20, 15).
    second_data = wall_grid(extra_blocked=[(20, 15)])
    first = fixture.add_snapshot(first_data, publication_sequence=1,
                                 source_generation=5)
    fixture.add_layer("/map", first_data, frame_id="map")
    reference = fixture.add_reference(
        [(x, ROBOT_Y, 0.0) for x in (0.8, 1.4, 2.0)])
    for x in (0.8, 1.1, 1.4, 1.7, 2.0):
        fixture.add_sample((x, ROBOT_Y, 0.0), first, reference, speed=0.3)
    fixture.add_sample((2.0, ROBOT_Y, 0.0), first, reference, speed=0.0,
                       selected={"vx": 0.0, "vy": 0.0, "wz": 0.0})
    second = fixture.add_snapshot(second_data, publication_sequence=2,
                                  source_generation=6)
    fixture.add_sample((2.0, ROBOT_Y, 0.0), second, reference, speed=0.0,
                       selected={"vx": 0.0, "vy": 0.0, "wz": 0.0})
    directory = fixture.write()
    status, report = run_analyzer(directory)

    answers = report["answers"]
    check(status == 1, "map-update case exits 1")
    check(answers["q1_reference_collision_free_at_publish"]["answer"] == "yes",
          "Q1: the reference was clean when it was published")
    check(answers["q3_same_pose_flipped_free_to_occupied"]["answer"] == "yes",
          "Q3: the same pose flipped free -> occupied across snapshots")
    check("generation" in answers["q3_same_pose_flipped_free_to_occupied"]
          ["routing"],
          "Q3 routes to the generation / heartbeat owner")
    flips = {flip["label"]: flip for flip in report["snapshot_flip"]}
    check("actual_rest_pose" in flips,
          "the rest pose is probed against the whole snapshot sequence")
    flip = flips["actual_rest_pose"]
    check(flip["flipped_free_to_occupied"] is True,
          "the rest-pose probe records the free -> occupied flip")
    transition = flip["transitions"][0]
    check(transition["from_source_generation"] == 5 and
          transition["to_source_generation"] == 6,
          "the flip names both source generations (%s -> %s)"
          % (transition["from_source_generation"],
             transition["to_source_generation"]))
    check(transition["from_publication_sequence"] == 1 and
          transition["to_publication_sequence"] == 2,
          "the flip names both publication sequences, which are a different "
          "counter from the source generation")
    check(len(flip["timeline"]) == 2 and
          flip["timeline"][0]["occupied"] is False and
          flip["timeline"][1]["occupied"] is True,
          "the probe timeline is ordered by publication sequence")
    # This shape also makes the actual pose conflict, which is correct: the pose
    # did become colliding.  Q3 is what says the map, not the tracking, did it.
    check(answers["q2_actual_left_reference_envelope"]["answer"] == "yes",
          "Q2 also fires, and Q3 is what distinguishes the owner")


def case_clean_run(root):
    """Nothing conflicts: the analyzer must report clean, not find something."""
    fixture = Fixture(root, "clean_run")
    data = wall_grid()
    snapshot = fixture.add_snapshot(data, publication_sequence=1,
                                    source_generation=5)
    fixture.add_layer("/map", data, frame_id="map")
    reference = fixture.add_reference(
        [(x, ROBOT_Y, 0.0) for x in (0.8, 1.4, 2.0)])
    for x in (0.8, 1.1, 1.4, 1.7, 2.0):
        fixture.add_sample((x, ROBOT_Y, 0.0), snapshot, reference, speed=0.3)
    fixture.add_sample((2.0, ROBOT_Y, 0.0), snapshot, reference, speed=0.0,
                       selected={"vx": 0.0, "vy": 0.0, "wz": 0.0})
    directory = fixture.write()
    status, report = run_analyzer(directory)
    answers = report["answers"]
    check(status == 0, "clean case exits 0 (complete, no conflicts)")
    check(answers["q1_reference_collision_free_at_publish"]["answer"] == "yes",
          "clean case: Q1 yes")
    check(answers["q2_actual_left_reference_envelope"]["answer"] == "no",
          "clean case: Q2 no")
    check(answers["q3_same_pose_flipped_free_to_occupied"]["answer"] == "no",
          "clean case: Q3 no")
    check(report["actual_tracking"]["colliding_ticks"] == 0 and
          report["actual_tracking"]["swept_colliding_segments"] == 0,
          "clean case reports no colliding ticks or segments")
    minimum = report["actual_tracking"]["min_clearance"]
    check(minimum is not None and minimum["min_clearance_m"] > 0.0,
          "clean case still reports a positive minimum clearance (%s m)"
          % (minimum and round(minimum["min_clearance_m"], 4)))


def case_same_geometry_republished(root):
    """A repeated geometry digest still pairs by its distinct ROS stamp."""
    fixture = Fixture(root, "same_geometry_republished")
    first_data = wall_grid()
    second_data = wall_grid(extra_blocked=[(20, 15)])
    first = fixture.add_snapshot(first_data, publication_sequence=1,
                                 source_generation=5)
    fixture.add_layer("/map", first_data, frame_id="map")
    poses = [(x, ROBOT_Y, 0.0) for x in (0.8, 1.4, 2.0)]
    first_reference = fixture.add_reference(
        poses, stamp_identity="200.000000000")
    fixture.add_sample((0.8, ROBOT_Y, 0.0), first, first_reference,
                       speed=0.1)
    second = fixture.add_snapshot(second_data, publication_sequence=2,
                                  source_generation=6)
    second_reference = fixture.add_reference(
        poses, stamp_identity="201.000000000")
    fixture.add_sample((0.8, ROBOT_Y, 0.0), second, second_reference,
                       speed=0.1)
    directory = fixture.write()
    status, report = run_analyzer(directory)
    entries = report["reference_at_publish"]
    check(status == 1, "same-geometry replan case finds the later conflict")
    check(len(entries) == 2 and entries[0]["observed_tick"] != entries[1]["observed_tick"],
          "same geometry references pair to distinct ticks by stamp")
    check(entries[0]["verdict"]["safe"] is True,
          "first same-geometry reference uses the first snapshot")
    check(entries[1]["verdict"]["safe"] is False,
          "second same-geometry reference uses the second snapshot")


def case_missing_transform_is_a_gap(root):
    """No map<-odom transform must fail loud, not evaluate in the wrong frame."""
    fixture = Fixture(root, "missing_transform")
    data = wall_grid()
    snapshot = fixture.add_snapshot(data, publication_sequence=1,
                                    source_generation=5)
    reference = fixture.add_reference([(x, ROBOT_Y, 0.0) for x in (0.8, 1.4, 2.0)])
    for x in (0.8, 1.4, 2.0):
        fixture.add_sample((x, ROBOT_Y, 0.0), snapshot, reference, speed=0.3,
                           transform=None)
    directory = fixture.write()
    status, report = run_analyzer(directory)
    check(status == 2, "a run with no usable transform exits 2 (incomplete)")
    check(report["actual_tracking"]["ticks_unpaired_frame"] == 3,
          "every unpaired tick is counted (got %s)"
          % report["actual_tracking"]["ticks_unpaired_frame"])
    check(report["actual_tracking"]["ticks_evaluated"] == 0,
          "no tick is evaluated when the frames cannot be paired")
    check(any("map<-odom" in gap for gap in report["evidence_gaps"]),
          "the missing transform is listed as an evidence gap")
    check(report["answers"]["q2_actual_left_reference_envelope"]["answer"]
          == "unknown",
          "Q2 answers 'unknown' rather than 'no' when nothing could be evaluated")
    entry = report["reference_at_publish"][0]
    check(entry["evaluated"] is False and entry.get("frame_unpaired") is True,
          "the reference is also reported unpaired instead of being evaluated")


def case_truncated_recording_is_a_gap(root):
    """A leg killed mid-write still analyzes, and says so."""
    fixture = Fixture(root, "truncated_recording")
    data = wall_grid()
    snapshot = fixture.add_snapshot(data, publication_sequence=1,
                                    source_generation=5)
    reference = fixture.add_reference([(x, ROBOT_Y, 0.0) for x in (0.8, 1.4, 2.0)])
    for x in (0.8, 1.4, 2.0):
        fixture.add_sample((x, ROBOT_Y, 0.0), snapshot, reference, speed=0.3)
    directory = fixture.write()
    with open(os.path.join(directory, "samples.jsonl"), "a",
              encoding="utf-8") as handle:
        handle.write('{"tick": 4, "actual": {"x": 1.0,')
    status, report = run_analyzer(directory)
    check(status in (0, 1), "a truncated recording is still analyzed (status=%s)"
          % status)
    check(any("unparseable" in gap for gap in report["evidence_gaps"]),
          "the truncated line is reported rather than silently dropped")
    check(report["actual_tracking"]["ticks_evaluated"] == 3,
          "the complete records before the cut are still evaluated")


def main():
    root = tempfile.mkdtemp(prefix="nav_tracking_analysis_")
    try:
        case_actual_drifts_into_wall(root)
        case_reference_unsafe_at_publish(root)
        case_map_update_occupies_rest_pose(root)
        case_clean_run(root)
        case_same_geometry_republished(root)
        case_missing_transform_is_a_gap(root)
        case_truncated_recording_is_a_gap(root)
    finally:
        shutil.rmtree(root, ignore_errors=True)
    if FAILURES:
        print("RESULT: nav tracking analyzer test FAILED (%d)" % len(FAILURES))
        for failure in FAILURES:
            print("  - " + failure)
        return 1
    print("RESULT: nav tracking analyzer test PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
