"""Reference, tracking, snapshot-flip, and stop-envelope checks for nav tracking analysis."""

from nav_tracking_analysis_io import *  # noqa: F401,F403


def analyze_reference_at_publish(samples, references, store, layers, params,
                                 reference_topic, events=(), snapshots=()):
    """Q1: evaluate only the snapshot authorized by a matching EXECUTE lease.

    Tick snapshots are latest callback state, not commit authority. Repeated
    leases with the same reference/map identity are replay, not ambiguity.
    """
    results = []
    commands = execution_commands(events)
    for reference in references:
        if reference.get("source") != reference_topic:
            continue
        digest = reference.get("digest")
        entry = {
            "reference_index": reference.get("reference_index"),
            "reference_digest": digest,
            "reference_stamp_identity": reference.get("stamp_identity"),
            "reference_frame_id": reference.get("frame_id"),
            "pose_count": reference.get("pose_count"),
            "recorded_mono_s": reference.get("mono_s"),
        }
        entry["evaluated"] = False
        results.append(entry)
        candidates = [command for command in commands
                      if command.get("mode") == 1 and
                      reference.get("stamp_identity") is not None and
                      command.get("reference_stamp_identity") ==
                      reference.get("stamp_identity")]
        if not candidates:
            entry["reason"] = "no matching EXECUTE reference authority recorded"
            continue
        if any(command.get("reference_payload_error") for command in candidates):
            entry["reason"] = "EXECUTE reference payload missing, corrupt or ambiguous"
            continue
        if any(command.get("reference_frame_id") != reference.get("frame_id") or
               command.get("reference_poses") != reference.get("poses") or
               command.get("reference_pose_count") != reference.get("pose_count") or
               (reference.get("poses_se3") is not None and
                command.get("reference_poses_se3") != reference.get("poses_se3"))
               for command in candidates):
            entry["reason"] = "EXECUTE reference frame/content mismatch or ambiguity"
            continue
        identities = {(command.get("localization_epoch"),
                       command.get("map_publication_sequence"),
                       command.get("map_generation"),
                       command.get("validation_map_publication_sequence", 0),
                       command.get("validation_map_source_generation", 0),
                       json.dumps(command.get("validation_from_reference"), sort_keys=True))
                      for command in candidates}
        if len(identities) != 1 or None in next(iter(identities))[:3]:
            entry["reason"] = "missing or ambiguous EXECUTE snapshot identity"
            continue
        epoch, planned_publication, generation, validated_publication, validated_source, _ = \
            next(iter(identities))
        bound_validation = bool(validated_publication)
        publication = validated_publication if bound_validation else planned_publication
        sample = first_sample_for_reference(
            samples, digest, reference.get("stamp_identity"), epoch, publication)
        entry["execution_identity"] = {
            "localization_epoch": epoch, "map_publication_sequence": planned_publication,
            "map_generation": generation,
            "validation_map_publication_sequence": validated_publication,
            "validation_map_source_generation": validated_source,
            "command_sequences": sorted({command.get("command_sequence", 0)
                                         for command in candidates}),
        }
        matches = [snapshot for snapshot in snapshots
                   if snapshot.get("localization_epoch") == epoch and
                   snapshot.get("publication_sequence") == publication]
        # Identical repeated records are harmless; conflicting records are not.
        unique = {json.dumps({key: value for key, value in snapshot.items()
                              if key != "mono_s"}, sort_keys=True): snapshot
                  for snapshot in matches}
        if len(unique) != 1:
            entry["reason"] = ("exact EXECUTE snapshot missing" if not unique else
                               "ambiguous exact EXECUTE snapshot records")
            continue
        snapshot = next(iter(unique.values()))
        entry["snapshot_identity"] = snapshot
        if bound_validation and (not validated_source or
                                 validated_source != snapshot.get("source_generation")):
            entry["reason"] = "EXECUTE validation source generation mismatch"
            continue
        if not snapshot.get("ready") or not snapshot.get("cell_count"):
            entry["reason"] = "authorized snapshot not ready or empty"
            continue
        if sample is not None:
            entry["observed_tick"] = sample.get("tick")
            entry["observed_mono_s"] = sample.get("mono_s")
        snapshot_digest = payload_key(snapshot)
        exact_store = PayloadStore(store._input_dir, [snapshot])
        try:
            payload = exact_store.payload(snapshot_digest)
            if payload is None:
                entry["reason"] = "authorized snapshot payload unavailable"
                continue
            identity = payload.get("identity", {})
            if any(identity.get(key) != snapshot.get(key) for key in
                   ("occupancy_digest", "info", "frame_id",
                    "occupied_value_threshold", "unknown_is_obstacle")):
                entry["reason"] = "authorized snapshot payload identity mismatch"
                continue
            grid = exact_store.grid(snapshot_digest)
            # Shared payload bytes do not supply publication authority.
            grid.identity.update({key: value for key, value in snapshot.items()
                                  if key not in ("payload", "mono_s", "info")})
            if (grid.width <= 0 or grid.height <= 0 or grid.resolution <= 0 or
                    len(payload.get("occupancy", [])) != grid.width * grid.height or
                    len(payload["occupancy"]) != snapshot.get("cell_count")):
                entry["reason"] = "authorized snapshot payload empty or malformed"
                continue
        except (OSError, ValueError, KeyError, TypeError, EOFError) as exc:
            entry["reason"] = "authorized snapshot payload unreadable: %s" % exc
            continue
        poses = [tuple(pose) for pose in reference.get("poses", [])]
        if not poses:
            entry["reason"] = "committed reference has no poses"
            continue
        transform = (sample or {}).get("map_from_odom")
        if bound_validation:
            transform = candidates[0].get("validation_from_reference")
            entry["transform_evidence"] = "EXECUTE-bound validation transform"
            if (not transform or not unit_quaternion(transform.get("quaternion")) or
                    transform.get("frame_id") != grid.frame_id or
                    transform.get("child_frame_id") != reference.get("frame_id")):
                entry["frame_unpaired"] = True
                entry["reason"] = "EXECUTE validation transform frame mismatch"
                continue
            if frames_agree(reference.get("frame_id"), grid.frame_id) and (
                    any(abs(transform.get(key, float("inf"))) > 1e-9
                        for key in ("x", "y", "z")) or
                    not unit_quaternion(transform.get("quaternion")) or
                    any(abs(v) > 1e-9 for v in transform["quaternion"][:3])):
                entry["reason"] = "same-frame validation transform is not identity"
                continue
            grid_poses = transform_poses(transform, poses, reference.get("poses_se3"))
            entry["transform_applied"] = not frames_agree(reference.get("frame_id"), grid.frame_id)
            entry["map_from_odom"] = transform
        elif not frames_agree(reference.get("frame_id"), grid.frame_id):
            execution = (sample or {}).get("execution") or {}
            entry["transform_evidence"] = (
                "tick-sampled map<-odom; TF carries no localization epoch; "
                "not an atomic commit-time transform")
            if (execution.get("localization_epoch") != epoch or
                    execution.get("reference_stamp_identity") != reference.get("stamp_identity") or
                    execution.get("mode") != 1):
                entry["frame_unpaired"] = True
                entry["reason"] = "transform tick lacks matching EXECUTE epoch/reference"
                continue
            grid_poses = transform_poses(transform, poses, reference.get("poses_se3"))
            entry["transform_applied"] = True
            entry["map_from_odom"] = transform
        else:
            grid_poses = poses
            entry["transform_applied"] = False
        if grid_poses is None:
            entry["evaluated"] = False
            entry["frame_unpaired"] = True
            entry["reason"] = (
                "reference is in %r, snapshot is in %r, and no %s <- %s transform "
                "was recorded at that tick" % (
                    reference.get("frame_id"), grid.frame_id,
                    grid.frame_id, reference.get("frame_id")))
            continue
        verdict = fe.check(grid, params, grid_poses, max_collisions=64)
        entry["evaluated"] = True
        entry["verdict"] = summarize_verdict(
            verdict, payload, grid, params, layers, reference.get("mono_s"))
    recorded_stamps = {entry.get("reference_stamp_identity") for entry in results}
    for command in commands:
        stamp = command.get("reference_stamp_identity")
        if command.get("mode") == 1 and stamp not in recorded_stamps:
            results.append({"reference_stamp_identity": stamp, "evaluated": False,
                            "reason": "EXECUTE has no recorded committed reference Path"})
            recorded_stamps.add(stamp)
    return results


def actual_pose_series(samples):
    """Ticks that carry a distinct localization pose, in order.

    Consecutive ticks that re-sample the same ``/localization`` message are
    collapsed by stamp identity: keeping them would inflate the discrete sample
    count and invent zero-length swept segments, both of which would make the
    actual sequence look better checked than it was.
    """
    series = []
    previous_stamp = None
    for sample in samples:
        actual = sample.get("actual")
        if not actual:
            continue
        stamp = actual.get("stamp_identity")
        if stamp is not None and stamp == previous_stamp:
            continue
        previous_stamp = stamp
        series.append(sample)
    return series


def tick_context(sample):
    """The same-clock spine for one tick, as the report needs to cite it."""
    actual = sample.get("actual") or {}
    telemetry = sample.get("telemetry") or {}
    return {
        "tick": sample.get("tick"),
        "mono_s": sample.get("mono_s"),
        "actual_x": actual.get("x"),
        "actual_y": actual.get("y"),
        "actual_yaw": actual.get("yaw"),
        "actual_frame": actual.get("frame_id"),
        "actual_stamp_identity": actual.get("stamp_identity"),
        "actual_vx_body": actual.get("vx_body"),
        "actual_vy_body": actual.get("vy_body"),
        "actual_wz_body": actual.get("wz_body"),
        "actual_speed_mps": (
            math.hypot(actual["vx_body"], actual["vy_body"])
            if actual.get("vx_body") is not None else None),
        "selected": sample.get("selected"),
        "selected_age_s_mono": sample.get("selected_age_s_mono"),
        "autonomy_raw": sample.get("autonomy_raw"),
        "motion_control": sample.get("motion_control"),
        "motion_control_age_s_mono": sample.get("motion_control_age_s_mono"),
        "localization_age_s_mono": sample.get("actual_age_s_mono"),
        "reference_age_s_mono": sample.get("reference_age_s_mono"),
        "emergency_stop": sample.get("emergency_stop"),
        "map_ready": sample.get("map_ready"),
        "tracking_error": sample.get("tracking_error"),
        "reference_digest": sample.get("reference_digest"),
        "planner_status": sample.get("planner_status"),
        "execution_mode": (sample.get("execution") or {}).get("mode"),
        "execution_command_sequence":
            (sample.get("execution") or {}).get("command_sequence"),
        "snapshot_identity": sample.get("snapshot"),
        "telemetry_contact_violation_count":
            telemetry.get("contact_violation_count"),
        "telemetry_max_contact_force": telemetry.get("max_contact_force"),
        "drive_speed_saturation_count":
            telemetry.get("drive_speed_saturation_count"),
        "drive_acceleration_saturation_count":
            telemetry.get("drive_acceleration_saturation_count"),
        "steer_rate_saturation_count": telemetry.get("steer_rate_saturation_count"),
    }


PERSIST_MIN_TICKS = 3
PERSIST_MIN_SNAPSHOTS = 2


def analyze_actual_tracking(samples, store, layers, params,
                            persist_min_ticks=PERSIST_MIN_TICKS,
                            persist_min_snapshots=PERSIST_MIN_SNAPSHOTS):
    """Q2: did the driven pose sequence leave the verified envelope?

    A conflict run is a stretch of consecutive evaluated ticks that each conflict
    (discrete or swept).  A run is *persistent* when it lasts at least
    ``persist_min_ticks`` ticks and spans at least ``persist_min_snapshots``
    distinct snapshot publications; a single snapshot that disagrees for a few ticks
    is reported but is not persistent.

    Every tick's pose is judged against the snapshot that was in force *at that
    tick*, and the motion between consecutive ticks is swept, so a conflict that
    exists only between two 50 Hz samples is still caught.  The tracking error the
    recorder computed against the committed reference is carried alongside, which
    is what makes "the actual left the reference's envelope" a measurement rather
    than an inference.
    """
    series = actual_pose_series(samples)
    result = {
        "ticks_with_pose": len(series),
        "ticks_evaluated": 0,
        "ticks_unpaired_frame": 0,
        "ticks_without_snapshot": 0,
        "ticks_without_snapshot_payload": 0,
        "colliding_ticks": 0,
        "swept_colliding_segments": 0,
        "first_discrete_conflict": None,
        "first_swept_conflict": None,
        "min_clearance": None,
        "max_abs_lateral_error_m": None,
        "max_abs_longitudinal_error_m": None,
        "max_abs_yaw_error_rad": None,
        "max_speed_mps": None,
        "persist_min_ticks": persist_min_ticks,
        "persist_min_snapshots": persist_min_snapshots,
        "longest_conflict_run_ticks": 0,
        "longest_conflict_run_snapshots": 0,
        "persistent_conflict_runs": [],
    }
    run = {"ticks": 0, "snapshots": [], "first_tick": None}

    def close_run():
        if run["ticks"]:
            result["longest_conflict_run_ticks"] = max(
                result["longest_conflict_run_ticks"], run["ticks"])
            result["longest_conflict_run_snapshots"] = max(
                result["longest_conflict_run_snapshots"], len(run["snapshots"]))
            if run["ticks"] >= persist_min_ticks and \
                    len(run["snapshots"]) >= persist_min_snapshots:
                result["persistent_conflict_runs"].append({
                    "first_tick": run["first_tick"], "ticks": run["ticks"],
                    "snapshots": list(run["snapshots"])})
        run["ticks"], run["snapshots"], run["first_tick"] = 0, [], None

    previous = None
    for sample in series:
        actual = sample["actual"]
        pose = (actual["x"], actual["y"], actual["yaw"])
        error = sample.get("tracking_error") or {}
        if not error.get("frame_unpaired"):
            for key, field in (("max_abs_lateral_error_m", "lateral_m"),
                               ("max_abs_longitudinal_error_m", "longitudinal_m"),
                               ("max_abs_yaw_error_rad", "yaw_error_rad")):
                value = error.get(field)
                if value is None:
                    continue
                current = result[key]
                if current is None or abs(value) > current:
                    result[key] = abs(value)
        speed = math.hypot(actual.get("vx_body", 0.0), actual.get("vy_body", 0.0))
        if result["max_speed_mps"] is None or speed > result["max_speed_mps"]:
            result["max_speed_mps"] = speed

        snapshot = sample.get("snapshot")
        if not snapshot:
            result["ticks_without_snapshot"] += 1
            previous = None
            close_run()
            continue
        digest = payload_key(snapshot)
        grid = store.grid(digest)
        if grid is None:
            result["ticks_without_snapshot_payload"] += 1
            previous = None
            close_run()
            continue
        grid.identity.update({key: value for key, value in snapshot.items()
                              if key not in ("payload", "mono_s", "info")})
        if frames_agree(actual.get("frame_id"), grid.frame_id):
            grid_pose = pose
        else:
            grid_pose = transform_pose(sample.get("map_from_odom"), pose, actual.get("pose_se3"))
        if grid_pose is None:
            result["ticks_unpaired_frame"] += 1
            previous = None
            close_run()
            continue

        payload = store.payload(digest)
        clearance = footprint_clearance(payload, grid, params, grid_pose)
        if clearance.get("min_clearance_m") is not None:
            current = result["min_clearance"]
            if current is None or \
                    clearance["min_clearance_m"] < current["min_clearance_m"]:
                record = dict(clearance)
                record["tick"] = sample.get("tick")
                record["mono_s"] = sample.get("mono_s")
                record["snapshot_identity"] = grid.identity
                result["min_clearance"] = record

        result["ticks_evaluated"] += 1
        verdict = fe.check(grid, params, [grid_pose], max_collisions=8)
        tick_conflict = verdict["discrete_collision_count"] > 0
        if verdict["discrete_collision_count"] > 0:
            result["colliding_ticks"] += 1
            if result["first_discrete_conflict"] is None:
                conflict = tick_context(sample)
                conflict["grid_pose"] = {"x": grid_pose[0], "y": grid_pose[1],
                                         "yaw": grid_pose[2]}
                conflict["map_from_odom"] = sample.get("map_from_odom")
                conflict["verdict"] = summarize_verdict(
                    verdict, payload, grid, params, layers, sample.get("mono_s"))
                conflict["clearance"] = clearance
                result["first_discrete_conflict"] = conflict

        if previous is not None:
            # Swept over the motion between two consecutive samples, judged
            # against the snapshot in force at the later of the two.
            swept = fe.check(grid, params, [previous, grid_pose], max_collisions=8)
            if swept["swept_collision_count"] > 0:
                tick_conflict = True
                result["swept_colliding_segments"] += 1
                if result["first_swept_conflict"] is None:
                    conflict = tick_context(sample)
                    conflict["segment_start"] = {
                        "x": previous[0], "y": previous[1], "yaw": previous[2]}
                    conflict["segment_end"] = {
                        "x": grid_pose[0], "y": grid_pose[1], "yaw": grid_pose[2]}
                    conflict["verdict"] = summarize_verdict(
                        swept, payload, grid, params, layers, sample.get("mono_s"))
                    result["first_swept_conflict"] = conflict
        previous = grid_pose
        if tick_conflict:
            if run["first_tick"] is None:
                run["first_tick"] = sample.get("tick")
            run["ticks"] += 1
            identity = snapshot.get("publication_sequence")
            identity = digest if identity is None else identity
            if identity not in run["snapshots"]:
                run["snapshots"].append(identity)
        else:
            close_run()
    close_run()
    result["persistent_conflict"] = bool(result["persistent_conflict_runs"])
    return result


def analyze_snapshot_flip(probes, store, params):
    """Q3: does one fixed pose change verdict as the snapshot sequence advances?

    Each probe pose is held constant and replayed against *every* distinct
    snapshot the recorder captured, in publication order.  A pose that is free
    under one snapshot identity and occupied under a later one is a map-update
    effect, not a tracking or planning error, and the transition is reported with
    both snapshot identities so the owner is unambiguous.
    """
    records = store.digests
    ordered = sorted(
        (record for record in (store.record(digest) for digest in records)
         if record is not None),
        key=lambda item: (item.get("publication_sequence") or 0,
                          item.get("mono_s") or 0.0))
    results = []
    for probe in probes:
        pose = probe.get("pose")
        entry = {
            "label": probe["label"],
            "pose": ({"x": pose[0], "y": pose[1], "yaw": pose[2]}
                     if pose is not None else None),
            "frame_id": probe.get("frame_id"),
            "source_tick": probe.get("tick"),
            "timeline": [],
            "flipped_free_to_occupied": False,
            "flipped_occupied_to_free": False,
            "transitions": [],
        }
        probe_reason = probe.get("reason")
        if pose is None:
            probe_reason = probe_reason or "probe pose unavailable"
        previous = None
        for record in ordered:
            digest = record.get("occupancy_digest")
            storage_key = payload_key(record)
            if probe_reason:
                entry["timeline"].append({
                    "occupancy_digest": digest,
                    "publication_sequence": record.get("publication_sequence"),
                    "evaluated": False,
                    "reason": probe_reason,
                })
                continue
            grid = store.grid(storage_key)
            if grid is None:
                entry["timeline"].append({
                    "occupancy_digest": digest,
                    "publication_sequence": record.get("publication_sequence"),
                    "source_generation": record.get("source_generation"),
                    "evaluated": False,
                    "reason": "payload unavailable",
                })
                continue
            if not frames_agree(probe.get("frame_id"), grid.frame_id):
                entry["timeline"].append({
                    "occupancy_digest": digest,
                    "publication_sequence": record.get("publication_sequence"),
                    "evaluated": False,
                    "frame_unpaired": True,
                    "reason": "probe and snapshot frames cannot be paired",
                    "probe_frame": probe.get("frame_id"),
                    "snapshot_frame": grid.frame_id,
                })
                continue
            verdict = fe.check(grid, params, [pose], max_collisions=4)
            grid.identity.update({key: value for key, value in record.items()
                                  if key not in ("payload", "mono_s", "info")})
            payload = store.payload(storage_key)
            step = {
                "occupancy_digest": digest,
                "publication_sequence": record.get("publication_sequence"),
                "source_generation": record.get("source_generation"),
                "localization_epoch": record.get("localization_epoch"),
                "stamp_identity": record.get("stamp_identity"),
                "mono_s": record.get("mono_s"),
                "evaluated": True,
                "occupied": verdict["discrete_collision_count"] > 0,
                "discrete_collisions": verdict["discrete_collision_count"],
                "min_clearance_m":
                    footprint_clearance(payload, grid, params,
                                        pose).get("min_clearance_m"),
            }
            first = verdict.get("first_collision")
            if first is not None:
                step["first_cell"] = first["cell"]
                step["first_sample_x"] = first["sample_x"]
                step["first_sample_y"] = first["sample_y"]
            entry["timeline"].append(step)
            if previous is not None and previous["occupied"] != step["occupied"]:
                transition = {
                    "from_digest": previous["occupancy_digest"],
                    "from_publication_sequence": previous["publication_sequence"],
                    "from_source_generation": previous["source_generation"],
                    "from_occupied": previous["occupied"],
                    "to_digest": step["occupancy_digest"],
                    "to_publication_sequence": step["publication_sequence"],
                    "to_source_generation": step["source_generation"],
                    "to_occupied": step["occupied"],
                }
                entry["transitions"].append(transition)
                if step["occupied"]:
                    entry["flipped_free_to_occupied"] = True
                else:
                    entry["flipped_occupied_to_free"] = True
            previous = step
        entry["evaluated"] = bool(entry["timeline"]) and all(
            step["evaluated"] for step in entry["timeline"])
        if not entry["evaluated"]:
            entry["reason"] = (probe_reason or
                               "snapshot timeline has unavailable evidence")
        results.append(entry)
    return results


def analyze_stop_envelope(samples, events, moving_speed, rest_speed, command_eps):
    """Braking / emergency-stop instant and the displacement to rest.

    Three candidate onsets are reported rather than one, because they answer
    different questions: the command going to zero bounds what the planner asked
    for, the emergency-stop assertion bounds when the safety path engaged, and the
    last moving sample bounds what the chassis actually did.  Picking only one
    would hide a latency between them.
    """
    series = actual_pose_series(samples)
    report = {
        "moving_speed_threshold_mps": moving_speed,
        "rest_speed_threshold_mps": rest_speed,
        "command_zero_epsilon": command_eps,
        "reached_rest": False,
        "onsets": {},
        "emergency_stop_events": [],
    }
    for event in events:
        if event.get("kind") == "emergency_stop":
            report["emergency_stop_events"].append({
                "mono_s": event.get("mono_s"),
                "value": (event.get("payload") or {}).get("value"),
            })
    if not series:
        report["reason"] = "no localization samples"
        return report

    def speed_of(sample):
        actual = sample.get("actual") or {}
        return math.hypot(actual.get("vx_body", 0.0) or 0.0,
                          actual.get("vy_body", 0.0) or 0.0)

    def command_magnitude(sample):
        selected = sample.get("selected")
        if not selected:
            return None
        return (abs(selected.get("vx", 0.0)) + abs(selected.get("vy", 0.0)) +
                abs(selected.get("wz", 0.0)))

    last_moving = None
    for index, sample in enumerate(series):
        if speed_of(sample) > moving_speed:
            last_moving = index
    if last_moving is None:
        report["reason"] = "the robot never exceeded the moving-speed threshold"
        return report

    rest_index = None
    for index in range(last_moving, len(series)):
        if speed_of(series[index]) <= rest_speed:
            rest_index = index
            break
    if rest_index is None:
        report["reason"] = "never came to rest inside the recording"
        report["last_moving"] = tick_context(series[last_moving])
        return report
    report["reached_rest"] = True
    rest = series[rest_index]
    rest_actual = rest["actual"]
    report["rest"] = tick_context(rest)

    def onset_report(label, index):
        if index is None:
            return {"found": False}
        sample = series[index]
        actual = sample["actual"]
        return {
            "found": True,
            "context": tick_context(sample),
            "displacement_to_rest_m": math.hypot(
                rest_actual["x"] - actual["x"], rest_actual["y"] - actual["y"]),
            "yaw_change_to_rest_rad": fe.normalize_angle(
                rest_actual["yaw"] - actual["yaw"]),
            "duration_to_rest_s_mono":
                (rest.get("mono_s") or 0.0) - (sample.get("mono_s") or 0.0),
        }

    # Last sample whose selected command still requested motion.
    command_index = None
    for index in range(0, rest_index + 1):
        magnitude = command_magnitude(series[index])
        if magnitude is not None and magnitude > command_eps:
            command_index = index
    # First emergency-stop assertion at or before rest.
    estop_index = None
    for index in range(0, rest_index + 1):
        if series[index].get("emergency_stop"):
            estop_index = index
            break
    report["onsets"] = {
        "last_nonzero_selected_command": onset_report(
            "last_nonzero_selected_command", command_index),
        "first_emergency_stop_true": onset_report(
            "first_emergency_stop_true", estop_index),
        "last_moving_sample": onset_report("last_moving_sample", last_moving),
    }
    return report


def grid_pose_for_sample(sample, store):
    """One tick's actual pose expressed in its snapshot's frame, or ``None``."""
    actual = sample.get("actual")
    snapshot = sample.get("snapshot")
    if not actual or not snapshot:
        return None
    grid = store.grid(payload_key(snapshot))
    if grid is None:
        return None
    pose = (actual["x"], actual["y"], actual["yaw"])
    if frames_agree(actual.get("frame_id"), grid.frame_id):
        return pose, grid.frame_id
    if actual.get("frame_id") != "odom" or grid.frame_id != "map":
        return None
    transformed = transform_pose(sample.get("map_from_odom"), pose, actual.get("pose_se3"))
    if transformed is None:
        return None
    return transformed, grid.frame_id
