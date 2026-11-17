"""Content-addressed execution references used only by the evidence tools.

Each lease remains a separate event. Its complete path is stored before first
use and identified by geometry, orientation, frames, and every pose timestamp.
"""

import hashlib
import json


def reference_payload_key(reference):
    encoded = json.dumps(reference, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def valid_reference_payload(reference):
    if not isinstance(reference, dict):
        return False
    count = reference.get("pose_count")
    return (isinstance(count, int) and count >= 0 and
            isinstance(reference.get("frame_id"), str) and
            isinstance(reference.get("stamp_identity"), str) and
            all(isinstance(reference.get(key), list) and len(reference[key]) == count
                for key in ("poses", "poses_se3", "pose_stamp_identities", "pose_frame_ids")))


def execution_commands(events):
    """Resolve compact leases while retaining support for historical inline paths.

    A missing, late, or contradictory definition is an evidence gap. A later bad
    definition also invalidates earlier uses of its claimed key. No path is
    inferred from a timestamp or from another topic.
    """
    payloads = {}
    invalid = set()
    pending = []
    for event in events:
        payload = event.get("payload", {})
        if event.get("kind") == "execution_reference_payload":
            key = payload.get("key")
            reference = payload.get("reference")
            if not isinstance(key, str):
                continue
            if (not valid_reference_payload(reference) or
                    reference_payload_key(reference) != key or
                    (key in payloads and payloads[key] != reference)):
                invalid.add(key)
                continue
            payloads[key] = reference
        elif event.get("kind") == "execution_command":
            key = payload.get("reference_payload_key")
            pending.append((payload, key, key is None or
                            (isinstance(key, str) and key in payloads)))

    result = []
    for command, key, defined_before_use in pending:
        resolved = dict(command)
        if key is not None:
            reference = payloads.get(key) if isinstance(key, str) else None
            if (not defined_before_use or not isinstance(key, str) or
                    key in invalid or reference is None):
                resolved["reference_payload_error"] = "missing, late or corrupt execution payload"
            elif (reference["frame_id"] != command.get("reference_frame_id") or
                  reference["stamp_identity"] != command.get("reference_stamp_identity") or
                  reference["pose_count"] != command.get("reference_pose_count") or
                  ("reference_poses" in command and
                   command["reference_poses"] != reference["poses"]) or
                  ("reference_poses_se3" in command and
                   command["reference_poses_se3"] != reference["poses_se3"])):
                resolved["reference_payload_error"] = "execution payload metadata/content mismatch"
            else:
                resolved["reference_poses"] = reference["poses"]
                resolved["reference_poses_se3"] = reference["poses_se3"]
        result.append(resolved)
    return result
