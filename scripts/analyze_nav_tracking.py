#!/usr/bin/env python3
"""Analyze recorded navigation tracking evidence and answer safety/tracking gates."""

from nav_tracking_analysis_io import *  # noqa: F401,F403
from nav_tracking_analysis_checks import *  # noqa: F401,F403


def find_sample(samples, tick):
    for sample in samples:
        if sample.get("tick") == tick:
            return sample
    return None


def build_probes(samples, store, tracking, references_result, stop):
    """Fixed poses worth replaying against the whole snapshot sequence.

    The set is derived from what the run actually did -- where the actual first
    conflicted, where it came to rest, and where the last committed reference
    started and first conflicted -- rather than from any hard-coded location.
    """
    probes = []
    conflict = tracking.get("first_discrete_conflict")
    if conflict:
        pose = conflict.get("grid_pose")
        frame_id = (conflict.get("verdict") or {}).get(
            "snapshot_identity", {}).get("frame_id")
        probe = {
            "label": "actual_pose_at_first_discrete_conflict",
            "pose": ((pose["x"], pose["y"], pose["yaw"])
                     if pose is not None else None),
            "frame_id": frame_id,
            "tick": conflict.get("tick"),
        }
        if not frames_agree(conflict.get("actual_frame"), frame_id) and not (
                conflict.get("actual_frame") == "odom" and frame_id == "map"
                and conflict.get("map_from_odom") is not None):
            probe["reason"] = "first conflict frame or map<-odom transform unavailable"
        probes.append(probe)
    rest = (stop or {}).get("rest")
    if rest and rest.get("tick") is not None:
        sample = find_sample(samples, rest["tick"])
        resolved = grid_pose_for_sample(sample, store) if sample is not None else None
        probe = {
            "label": "actual_rest_pose",
            "pose": resolved[0] if resolved is not None else None,
            "frame_id": resolved[1] if resolved is not None else None,
            "tick": rest.get("tick"),
        }
        if resolved is None:
            probe["reason"] = "rest pose snapshot, frame or map<-odom transform unavailable"
        probes.append(probe)
    for entry in reversed(references_result):
        verdict = entry.get("verdict")
        if not verdict:
            continue
        first = verdict.get("first_collision")
        if first is None:
            continue
        probes.append({
            "label": "reference_pose_at_first_collision",
            "pose": (first["center_x"], first["center_y"], first["center_yaw"]),
            "frame_id": (verdict.get("snapshot_identity") or {}).get("frame_id"),
            "tick": entry.get("observed_tick"),
        })
        break
    # Deduplicate by rounded pose so the timeline is not recomputed for a probe
    # that two sources happen to agree on.
    unique = []
    seen = set()
    for probe in probes:
        if probe.get("pose") is None:
            unique.append(probe)
            continue
        key = (probe["label"], probe.get("frame_id"), round(probe["pose"][0], 6),
               round(probe["pose"][1], 6), round(probe["pose"][2], 6))
        if key in seen:
            continue
        seen.add(key)
        unique.append(probe)
    return unique


def answer_questions(references_result, tracking, flips):
    """The three falsifiable answers, each with the routing they imply."""
    evaluated = [entry for entry in references_result if entry.get("evaluated")]
    unsafe = [entry for entry in evaluated
              if not (entry.get("verdict") or {}).get("safe", True)]
    answers = {
        "q1_reference_collision_free_at_publish": {
            "references_evaluated": len(evaluated),
            "references_not_evaluated":
                len(references_result) - len(evaluated),
            "references_with_collisions": len(unsafe),
            "answer": ("no" if unsafe else
                       "yes" if evaluated and len(evaluated) == len(references_result)
                       else "unknown"),
            "routing": ("audit the final revalidation snapshot, sampling "
                        "continuity and commit timing"
                        if unsafe else
                        "reference evidence incomplete" if not evaluated or
                        len(evaluated) != len(references_result) else
                        "no reference-side violation to route"),
        },
        "q2_actual_left_reference_envelope": {
            "ticks_evaluated": tracking.get("ticks_evaluated"),
            "colliding_ticks": tracking.get("colliding_ticks"),
            "swept_colliding_segments": tracking.get("swept_colliding_segments"),
            "max_abs_lateral_error_m": tracking.get("max_abs_lateral_error_m"),
            "max_abs_longitudinal_error_m":
                tracking.get("max_abs_longitudinal_error_m"),
            "max_abs_yaw_error_rad": tracking.get("max_abs_yaw_error_rad"),
            "answer": ("unknown" if not tracking.get("ticks_evaluated") else
                       ("yes" if (tracking.get("colliding_ticks") or
                                  tracking.get("swept_colliding_segments"))
                        else "no")),
            "routing": ("audit MPC tracking error, execution latency, "
                        "velocity/acceleration limiting and the stopping envelope"
                        if (tracking.get("colliding_ticks") or
                            tracking.get("swept_colliding_segments"))
                        else "no actual-side violation to route"),
        },
    }
    flipped = [flip for flip in flips if flip.get("flipped_free_to_occupied")]
    incomplete = [flip for flip in flips if not flip.get("evaluated")]
    unevaluated_ticks = sum(tracking.get(key, 0) for key in (
        "ticks_unpaired_frame", "ticks_without_snapshot",
        "ticks_without_snapshot_payload"))
    flip_answer = ("yes" if flipped else "unknown"
                   if not flips or incomplete or unevaluated_ticks else "no")
    answers["q3_same_pose_flipped_free_to_occupied"] = {
        "probes": len(flips),
        "probes_not_evaluated": len(incomplete),
        "unevaluated_actual_ticks": unevaluated_ticks,
        "probes_flipped_free_to_occupied": len(flipped),
        "flipped_labels": [flip["label"] for flip in flipped],
        "answer": flip_answer,
        "routing": ("audit source generation / adapter heartbeat and the timing of "
                    "old-reference revocation"
                    if flipped else "map-update evidence incomplete"
                    if flip_answer == "unknown" else "no map-update flip to route"),
    }
    return answers


def collect_evidence_gaps(summary, truncations, tracking, store):
    """Everything the analysis could not establish, named explicitly.

    An unpaired tick or a truncated payload is a gap, not a pass.  They are listed
    so a report cannot present a partial evaluation as a clean one.
    """
    gaps = []
    if summary is None:
        gaps.append("summary.json missing: the recorder did not shut down cleanly")
    else:
        for topic in summary.get("topics_without_messages", []):
            gaps.append("topic delivered no messages: %s" % topic)
        if summary.get("snapshot_payloads_truncated"):
            gaps.append("snapshot payloads truncated: some snapshots cannot be "
                        "re-evaluated")
        if summary.get("layer_payloads_truncated"):
            gaps.append("layer payloads truncated: some provenance is unavailable")
        if not summary.get("motion_control_message_available"):
            gaps.append("manda_can_control/MotionCtrl unavailable: /motion_control "
                        "was recorded as null")
    for name, count in truncations.items():
        if count:
            gaps.append("%s has %d unparseable line(s): the recording was cut off "
                        "mid-write" % (name, count))
    if tracking.get("ticks_unpaired_frame"):
        gaps.append("%d tick(s) had no map<-odom transform and were excluded"
                    % tracking["ticks_unpaired_frame"])
    if tracking.get("ticks_without_snapshot"):
        gaps.append("%d tick(s) had no snapshot in force and were excluded"
                    % tracking["ticks_without_snapshot"])
    if tracking.get("ticks_without_snapshot_payload"):
        gaps.append("%d tick(s) referenced a snapshot whose payload was not saved"
                    % tracking["ticks_without_snapshot_payload"])
    if not store.digests:
        gaps.append("no snapshots were captured: nothing could be evaluated")
    gaps.append("physical contact is not evaluated here; the MuJoCo contact "
                "counter is separate evidence and remains unverified by this tool")
    return gaps


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Join nav_tracking_recorder output to the footprint evaluator "
                    "and answer the reference/actual/map-update questions.")
    parser.add_argument("--input-dir", required=True,
                        help="a nav_tracking_recorder output directory")
    parser.add_argument("--output", help="write the full verdict JSON here")
    parser.add_argument("--reference-topic", default="/minco/reference_path",
                        help="which recorded reference source is the committed one")
    # Footprint geometry must come from the run profile.  There is no default that
    # is right for every robot, and silently using the header defaults would judge
    # the run with a footprint it never used.
    parser.add_argument("--length", type=float, required=True)
    parser.add_argument("--width", type=float, required=True)
    parser.add_argument("--safety-margin", type=float, required=True)
    parser.add_argument("--obstacle-value-threshold", type=int,
                        help="defaults to the threshold carried by the snapshot")
    parser.add_argument("--unknown-is-obstacle", dest="unknown_is_obstacle",
                        action="store_true", default=None,
                        help="defaults to the flag carried by the snapshot")
    parser.add_argument("--known-only", dest="unknown_is_obstacle",
                        action="store_false",
                        help="treat unknown as free, overriding the snapshot flag")
    parser.add_argument("--swept-max-corner-step-cells", type=float, default=0.5)
    parser.add_argument("--moving-speed-mps", type=float, default=0.05)
    parser.add_argument("--rest-speed-mps", type=float, default=0.01)
    parser.add_argument("--command-zero-epsilon", type=float, default=1e-6)
    parser.add_argument("--persist-min-ticks", type=int, default=PERSIST_MIN_TICKS,
                        help="q2 fails only for a conflict run of at least this many ticks")
    parser.add_argument("--persist-min-snapshots", type=int, default=PERSIST_MIN_SNAPSHOTS,
                        help="... that also spans at least this many snapshot publications")
    parser.add_argument("--strict-diagnostics", action="store_true",
                        help="reproduce the old gate: any q2 conflict fails and unknown q2/q3 is "
                             "incomplete")
    return parser.parse_args(argv)


def resolve_params(args, snapshots):
    """Effective footprint parameters, preferring the snapshot's own flags.

    ``unknown_is_obstacle`` and the occupied threshold are published *inside* the
    snapshot, so the run's own values are used unless the caller overrides them.
    Guessing them would change the verdict without changing the run.
    """
    threshold = args.obstacle_value_threshold
    unknown = args.unknown_is_obstacle
    source = {"obstacle_value_threshold": "cli", "unknown_is_obstacle": "cli"}
    if snapshots:
        latest = snapshots[-1]
        if threshold is None:
            threshold = latest.get("occupied_value_threshold")
            source["obstacle_value_threshold"] = "snapshot"
        if unknown is None:
            unknown = latest.get("unknown_is_obstacle")
            source["unknown_is_obstacle"] = "snapshot"
    if threshold is None:
        threshold = fe.DEFAULT_OBSTACLE_VALUE_THRESHOLD
        source["obstacle_value_threshold"] = "evaluator default"
    if unknown is None:
        unknown = fe.DEFAULT_UNKNOWN_IS_OBSTACLE
        source["unknown_is_obstacle"] = "evaluator default"
    params = fe.FootprintParams(
        length=args.length, width=args.width, safety_margin=args.safety_margin,
        obstacle_value_threshold=int(threshold),
        unknown_is_obstacle=bool(unknown),
        swept_max_corner_step_cells=args.swept_max_corner_step_cells)
    return params, source


def main(argv=None):
    args = parse_args(argv)
    input_dir = args.input_dir
    samples, samples_truncated = load_jsonl(os.path.join(input_dir, "samples.jsonl"))
    references, references_truncated = load_jsonl(
        os.path.join(input_dir, "references.jsonl"))
    snapshots, snapshots_truncated = load_jsonl(
        os.path.join(input_dir, "snapshots.jsonl"))
    events, events_truncated = load_jsonl(os.path.join(input_dir, "events.jsonl"))
    summary = load_json(os.path.join(input_dir, "summary.json"))

    if not samples:
        print("RESULT: no samples.jsonl records in %s" % input_dir)
        return 2

    store = PayloadStore(input_dir, snapshots)
    layers = LayerStore(input_dir)
    params, params_source = resolve_params(args, snapshots)

    references_result = analyze_reference_at_publish(
        samples, references, store, layers, params, args.reference_topic,
        events, snapshots)
    tracking = analyze_actual_tracking(
        samples, store, layers, params, args.persist_min_ticks,
        args.persist_min_snapshots)
    stop = analyze_stop_envelope(
        samples, events, args.moving_speed_mps, args.rest_speed_mps,
        args.command_zero_epsilon)
    probes = build_probes(samples, store, tracking, references_result, stop)
    flips = analyze_snapshot_flip(probes, store, params)
    answers = answer_questions(references_result, tracking, flips)
    gaps = collect_evidence_gaps(
        summary,
        {"samples.jsonl": samples_truncated,
         "references.jsonl": references_truncated,
         "snapshots.jsonl": snapshots_truncated,
         "events.jsonl": events_truncated},
        tracking, store)
    gaps.extend("reference %s: %s" % (entry.get("reference_stamp_identity"),
                                    entry.get("reason", "not evaluated"))
                for entry in references_result if not entry.get("evaluated"))
    gaps.extend("snapshot probe %s: %s" % (flip["label"], flip["reason"])
                for flip in flips if not flip["evaluated"])

    report = {
        "input_dir": os.path.abspath(input_dir),
        "recorder_summary": summary,
        "footprint_params": params.as_dict(),
        "footprint_params_source": params_source,
        "reference_topic": args.reference_topic,
        "snapshot_count": len(store.digests),
        "layer_topics": layers.topics,
        "answers": answers,
        "reference_at_publish": references_result,
        "actual_tracking": tracking,
        "snapshot_flip": flips,
        "stop_envelope": stop,
        "evidence_gaps": gaps,
    }
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")

    print("ANALYSIS: %s" % os.path.abspath(input_dir))
    print("  footprint %.3f x %.3f + %.3f m, threshold=%d, unknown_is_obstacle=%s"
          % (params.length, params.width, params.safety_margin,
             params.obstacle_value_threshold, params.unknown_is_obstacle))
    print("  snapshots=%d  references=%d  samples=%d"
          % (len(store.digests), len(references_result), len(samples)))
    for key in ("q1_reference_collision_free_at_publish",
                "q2_actual_left_reference_envelope",
                "q3_same_pose_flipped_free_to_occupied"):
        answer = answers[key]
        print("  %-42s %s" % (key + ":", answer["answer"]))
    conflict = tracking.get("first_discrete_conflict")
    if conflict:
        print("  first actual conflict at tick %s (mono %.3f s): "
              "pose=(%.4f, %.4f, yaw=%.4f)"
              % (conflict.get("tick"), conflict.get("mono_s") or 0.0,
                 conflict["grid_pose"]["x"], conflict["grid_pose"]["y"],
                 conflict["grid_pose"]["yaw"]))
    minimum = tracking.get("min_clearance")
    if minimum and minimum.get("min_clearance_m") is not None:
        print("  minimum footprint clearance %.4f m at tick %s"
              % (minimum["min_clearance_m"], minimum.get("tick")))
    if stop.get("reached_rest"):
        for label, onset in sorted(stop.get("onsets", {}).items()):
            if not onset.get("found"):
                continue
            print("  stop from %-32s %.4f m over %.3f s"
                  % (label + ":", onset["displacement_to_rest_m"],
                     onset["duration_to_rest_s_mono"]))
    for gap in gaps:
        print("  GAP: %s" % gap)
    if args.output:
        print("  wrote %s" % args.output)

    if not tracking.get("ticks_evaluated"):
        print("RESULT: nav tracking analysis INCOMPLETE (no tick could be "
              "evaluated against a snapshot)")
        return 2
    reference_conflicts = sum(
        1 for entry in references_result
        if entry.get("evaluated") and
        not (entry.get("verdict") or {}).get("safe", True))
    actual_conflicts = (tracking.get("colliding_ticks") or 0) + \
        (tracking.get("swept_colliding_segments") or 0)
    print("  q2 conflict runs: longest %d tick(s) over %d snapshot(s); persistent "
          "(>= %d ticks and >= %d snapshots): %d"
          % (tracking.get("longest_conflict_run_ticks", 0),
             tracking.get("longest_conflict_run_snapshots", 0),
             args.persist_min_ticks, args.persist_min_snapshots,
             len(tracking.get("persistent_conflict_runs", []))))
    if args.strict_diagnostics:
        gate_conflicts = reference_conflicts + actual_conflicts
        gated = answers.values()
    else:
        gate_conflicts = reference_conflicts + \
            len(tracking.get("persistent_conflict_runs", []))
        gated = [answers["q1_reference_collision_free_at_publish"]]
        if actual_conflicts and not gate_conflicts:
            print("  NOTE: q2 conflicts recorded as diagnostics only (not persistent)")
    if gate_conflicts:
        print("RESULT: nav tracking analysis COMPLETE, conflicts found")
        return 1
    if any(answer["answer"] == "unknown" for answer in gated):
        print("RESULT: nav tracking analysis INCOMPLETE (question evidence unknown)")
        return 2
    print("RESULT: nav tracking analysis COMPLETE, no conflicts found")
    return 0


if __name__ == "__main__":
    sys.exit(main())
