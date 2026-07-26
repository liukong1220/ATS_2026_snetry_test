# Vision Perception and Tracking

Use this reference for image-based detection, recognition, keypoints, data association,
single- or multi-object tracking, target selection, and recovery from visual loss.

## Contents

- Evidence-first workflow
- Perception contract
- Acquisition and preprocessing
- Detection and recognition
- Association and track management
- Decision workflow
- Metrics and failure analysis
- Validation and reporting

## Evidence-first workflow

1. Restate the requested behavior and the observed failure in measurable terms.
2. Locate the image source, timestamp source, preprocessing path, model artifact, postprocessor,
   tracker, target selector, output interface, parameters, and nearest tests.
3. Trace one frame and one target end to end before reading unrelated modules.
4. Record every boundary contract: shape, dtype, color order, scale, units, frame, timestamp,
   confidence meaning, identifier meaning, valid flag, and timeout.
5. Verify important claims with two independent sources when practical, such as code plus a
   recorded frame, interface plus test, or parameter plus runtime trace.
6. Label findings as confirmed, inferred, hypothesized, or unknown.
7. Establish a reproducible baseline before changing a model, threshold, or tracker parameter.
8. Change the owning layer only; do not hide acquisition or geometry faults with tracker tuning.

## Perception contract

Define the target semantics before optimizing anything:

- Specify whether an observation represents a box, mask, keypoint set, physical feature,
  rigid-body center, class, identity, or predicted future state.
- Keep detector confidence, class confidence, association confidence, and track confidence
  separate unless a documented calibration combines them.
- Keep observation identifiers distinct from persistent track identifiers.
- Define the coordinate convention for pixels, normalized image coordinates, and any 3D output.
- Define whether box coordinates are inclusive, half-open, center-size, or corner-corner.
- Define the timestamp as capture time whenever available; never silently substitute publish time.
- Define validity, freshness, covariance or uncertainty, and loss semantics explicitly.
- Preserve source metadata through preprocessing so results can be mapped back exactly.

Treat these as invariants:

- Map every output to exactly one input frame and timestamp.
- Apply each resize, crop, pad, mirror, and lens transform exactly once.
- Reject NaN, Inf, negative sizes, invalid class indices, and out-of-image geometry deliberately.
- Prevent a stale observation from refreshing a live track.
- Prevent a track ID from changing merely because output ordering changes.
- Make track creation, confirmation, coasting, loss, deletion, and reuse deterministic.

## Acquisition and preprocessing

Inspect the sensor path before judging model quality:

- Verify exposure, gain, focus, motion blur, rolling-shutter effects, dropped frames, and transport
  corruption under representative motion and illumination.
- Verify pixel format and color conversion with known color patches or channel statistics.
- Verify image dimensions, row stride, memory lifetime, and zero-copy ownership.
- Reproduce preprocessing outside the runtime and compare tensors element by element.
- Record the exact resize transform. For letterboxing, preserve scale `s` and padding `(px, py)`;
  recover source coordinates with `x = (x_net - px) / s` and `y = (y_net - py) / s`.
- Apply normalization in the documented order and range; verify mean, standard deviation, dtype,
  quantization scale, zero point, and tensor layout.
- Separate sensor noise mitigation from semantic postprocessing and benchmark both.

## Detection and recognition

Define evaluation units before comparing implementations:

- Match predictions to ground truth with an explicit IoU or distance rule.
- Compute `IoU = area(A intersect B) / area(A union B)` with identical box conventions.
- Report precision `TP / (TP + FP)`, recall `TP / (TP + FN)`, and class-wise support.
- Use precision-recall curves or average precision instead of selecting a threshold from anecdotes.
- Calibrate confidence when downstream logic treats it as probability; inspect reliability curves
  and expected calibration error.
- Report performance by range, target size, occlusion, motion, illumination, and class.
- Inspect false positives and false negatives as datasets, not isolated screenshots.
- Verify non-maximum suppression class handling, coordinate space, threshold, and maximum count.
- For keypoints, verify ordering, visibility semantics, subpixel convention, and geometric sanity.
- For segmentation, report boundary-sensitive metrics when pose depends on contour accuracy.

Avoid invalid shortcuts:

- Do not compare models using different preprocessing, datasets, thresholds, or hardware budgets.
- Do not count augmented variants of the same scene across training and validation splits.
- Do not infer runtime accuracy from exported-model success alone.
- Do not delete difficult samples to improve an aggregate metric without reporting the slice.

## Association and track management

Build association from explicit gates and costs:

- Predict every existing track to the observation timestamp before matching.
- Gate impossible pairs before assignment using class, geometry, motion, appearance, or covariance.
- Normalize heterogeneous costs before combining them.
- Define a cost such as
  `C = w_m*d_motion + w_a*d_appearance + w_g*d_geometry + w_c*d_class`.
- Document each distance range and every weight; reject pairs beyond independent hard gates.
- Use global assignment when choices compete; verify deterministic tie handling.
- Use Mahalanobis distance `d2 = r^T S^-1 r` when a valid innovation covariance exists.
- Choose a chi-square gate from measurement dimension and stated false-rejection probability.
- Never use an unvalidated covariance merely to make gating appear principled.

Manage lifecycle deliberately:

- Require configurable evidence before promoting a tentative track.
- Coast through short observation gaps using prediction without claiming a fresh detection.
- Increase uncertainty during coasting and cap the allowed age or missed-frame count.
- Delete tracks by elapsed capture time when frame rate can vary.
- Delay ID reuse or use monotonic IDs when downstream consumers retain history.
- Preserve the selected target across brief ambiguity with hysteresis and switch penalties.
- Switch targets only on explicit, explainable criteria; record the reason.
- Reinitialize after long loss instead of extending a filter beyond its valid uncertainty model.

## Decision workflow

Use the following branches in order:

1. If raw frames are wrong, fix acquisition, timestamping, calibration, or preprocessing first.
2. If offline predictions differ from runtime predictions, isolate export, precision, backend,
   preprocessing, or postprocessing differences.
3. If detections are correct but tracks fragment, inspect timestamps, prediction, gates,
   lifecycle thresholds, and assignment competition.
4. If identities swap, inspect ambiguity slices, appearance stability, motion covariance,
   global assignment, and target-switch hysteresis.
5. If tracking lags, separate capture age, queue age, inference time, filtering delay, and
   intentional prediction horizon.
6. If close-range or edge cases fail, inspect clipping, distortion, scale, partial visibility,
   and training coverage before relaxing every threshold.
7. If aggregate metrics pass but the robot fails, evaluate the exact operational slices and
   downstream decision cost.

## Metrics and failure analysis

Measure both accuracy and system behavior:

- Report detection precision, recall, AP, confusion matrix, and confidence calibration.
- Report track recall, mostly-tracked fraction, fragmentation, ID switches, and ID metrics.
- Report center, box, keypoint, or mask error in pixels and normalized by target scale.
- Report time to first confirmed track, loss rate, coast duration, and reacquisition time.
- Report target-switch count and incorrect-selection duration.
- Report end-to-end capture-to-output latency at p50, p95, p99, and maximum.
- Report results per scenario slice and include sample counts and uncertainty intervals.
- Preserve seeds, model hashes, dataset revision, configuration, hardware, and software versions.

Map common symptoms to candidate causes, then test rather than assume:

| Symptom | Candidate causes | Discriminating evidence |
| --- | --- | --- |
| Stable box offset | resize inversion, crop, distortion, label convention | overlay each transform stage |
| Oscillating boxes | threshold churn, NMS competition, blur, poor labels | raw candidates and frame slices |
| Track fragmentation | stale time, tight gate, weak recall, early deletion | innovation and lifecycle trace |
| ID switches | ambiguous targets, local matching, unstable embedding | assignment matrix and replay |
| Growing lag | unbounded queue, blocking callback, slow backend | per-stage timestamp trace |
| False confidence | uncalibrated score, domain shift, class imbalance | reliability plot by scenario |

## Validation and reporting

Validate in increasing scope:

1. Test coordinate transforms, clipping, IoU, assignment, and lifecycle transitions as units.
2. Replay a frozen labeled dataset with deterministic configuration and seeds.
3. Compare native and deployed backends on identical input tensors and tolerances.
4. Replay recorded sensor data at original and stress rates while preserving capture timestamps.
5. Exercise darkness, glare, blur, vibration, partial view, crowded scenes, and long loss.
6. Run integrated simulation or hardware-in-the-loop with downstream consumers enabled.
7. Conduct bounded real-robot tests with stop conditions and retained recordings.

Require every proposed change to state:

- The violated contract or measured bottleneck.
- The smallest owning-layer change and compatibility impact.
- The baseline, acceptance threshold, and regression slices.
- The exact data, configuration, command, and artifact needed to reproduce the result.
- The tests actually run, tests not run, residual risks, and confidence level.
