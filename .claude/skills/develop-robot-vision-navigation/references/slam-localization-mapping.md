# SLAM, Localization, and Mapping

Use this reference for repository-agnostic analysis, implementation, review, and validation of robot state estimation and mapping. Treat every pose, timestamp, and map value as a contract that must be traced to its producer and consumers.

## Contents

- [Discover the Running System](#discover-the-running-system)
- [Define the Estimation Contract](#define-the-estimation-contract)
- [Preserve Mathematical Invariants](#preserve-mathematical-invariants)
- [Control Time and Synchronization](#control-time-and-synchronization)
- [Control Frames and Calibration](#control-frames-and-calibration)
- [Audit the Estimation Pipeline](#audit-the-estimation-pipeline)
- [Define Map Semantics](#define-map-semantics)
- [Design Lifecycle and Recovery](#design-lifecycle-and-recovery)
- [Measure Performance](#measure-performance)
- [Diagnose Common Failures](#diagnose-common-failures)
- [Verify in Stages](#verify-in-stages)
- [Report with Evidence](#report-with-evidence)

## Discover the Running System

1. Read repository instructions, build manifests, runtime entry points, active configuration, interface definitions, and tests before reading algorithm internals.
2. Identify whether the system is ROS, another middleware, or a custom process graph; translate concepts instead of assuming APIs.
3. Draw the actual dataflow from sensors through preprocessing, odometry, loop closure, map storage, localization, and downstream planning.
4. Record each process, thread, callback, transport, queue, and persistence boundary that can alter ordering or latency.
5. Find the configuration that is loaded at runtime; distinguish defaults, examples, generated files, deployment overlays, and stale copies.
6. Identify the authority for pose, velocity, covariance, calibration, time, and map publication; reject ambiguous or duplicate authorities.
7. Separate third-party code from project-owned adapters and policies before assigning a defect or proposing a patch.
8. Preserve uncommitted work and generated artifacts; do not infer ownership from directory names alone.
9. Locate representative logs, recorded sensor data, maps, calibration results, benchmarks, and regression tests.
10. State what is observed in code, what is observed at runtime, and what remains an assumption.

## Define the Estimation Contract

Write the contract before tuning the estimator.

- Define the estimated state explicitly: pose, velocity, angular rate, IMU biases, gravity, scale, extrinsics, clock offsets, and any map state.
- Define the perturbation convention, state ordering, covariance ordering, and units for every state component.
- Define whether poses are active or passive transforms and whether `T_A_B` maps coordinates from B into A.
- Define the world, map, odometry, body, sensor, and actuator frames without relying on familiar frame names.
- Define the published pose semantics: global pose, locally continuous pose, relative motion, prediction, or delayed optimized history.
- Define the timestamp semantics: acquisition start, exposure midpoint, packet receipt, scan end, estimator update, or publication time.
- Define whether covariance represents posterior uncertainty, prediction uncertainty, empirical spread, or a placeholder.
- Define initialization prerequisites, validity flags, stale-data limits, and downstream behavior when the estimate is invalid.
- Define continuity guarantees when loop closure, relocalization, or map switching changes a global estimate.
- Verify every consumer interprets the contract identically.

## Preserve Mathematical Invariants

- Write the process model, observation model, noise model, and discretization used by the implementation.
- Keep quaternion or rotation-matrix states normalized and verify determinant, orthogonality, handedness, and multiplication order.
- Keep covariance symmetric and positive semidefinite; use stable factorizations or square-root methods when conditioning demands them.
- Apply perturbations on the intended side of the manifold and use Jacobians consistent with that convention.
- Convert continuous-time noise density to discrete covariance using the actual sample interval and documented units.
- Model IMU bias random walks, gravity direction, and scale factors only when the data makes them observable.
- Identify gauge freedoms such as global position, yaw, scale, or map origin; do not interpret gauge motion as estimation error.
- Check local and trajectory-level observability under stationary, constant-velocity, planar, low-texture, and low-geometry motion.
- Gate measurements with a statistically meaningful residual test; log accepted and rejected residual distributions.
- Use robust losses to limit outliers, not to hide systematic calibration or synchronization error.
- Prevent a single sensor dropout from silently changing the state dimension, coordinate convention, or noise interpretation.
- Verify marginalization preserves the intended prior and does not double-count measurements.
- Verify interpolation and extrapolation operate on the correct manifold and obey maximum time-gap limits.
- Compare analytic Jacobians against automatic differentiation or finite differences at representative and boundary states.
- Test invariance under rigid changes of the chosen world frame when the problem should be frame invariant.

## Control Time and Synchronization

- Trace sensor time from hardware acquisition through drivers, transport, buffering, estimator use, and publication.
- Distinguish hardware clocks, monotonic host clocks, wall clocks, simulation clocks, and synchronized network clocks.
- Estimate or calibrate clock offset and drift when sensors do not share a clock; do not treat receive time as acquisition time.
- Preserve per-point or per-row timing for scanning lidars and rolling-shutter cameras when motion distortion is material.
- Deskew measurements using a trajectory spanning their acquisition interval, not a single pose with a relabeled timestamp.
- Bound interpolation, extrapolation, reordering, and waiting; expose counters for every dropped or late measurement.
- Make queue policy explicit: block, retain latest, retain all, or discard by deadline.
- Measure acquisition-to-estimate and acquisition-to-consumer latency at p50, p95, and p99.
- Include compute and transport delay in motion compensation and downstream control interfaces.
- Test clock jumps, simulation resets, recording playback rates, wraparound, and long-duration drift.

## Control Frames and Calibration

- Build a directed frame graph and annotate each transform with authority, update rate, timestamp, and uncertainty.
- Reject cycles with competing transform authorities and reject disconnected trees masked by identity fallbacks.
- Distinguish static mechanical extrinsics from online calibration estimates and deployment-specific corrections.
- Verify translation units, axis directions, rotation order, quaternion component order, and degree/radian conversions.
- Test extrinsics by forward projection into raw measurements, not only by inspecting numeric plausibility.
- Measure calibration repeatability across datasets and temperature or mechanical reassembly when relevant.
- Treat time offset and spatial extrinsics as coupled calibration variables when platform motion makes them correlated.
- Version calibration with sensor identity, resolution, lens mode, mounting state, and validity conditions.
- Fail visibly on missing calibration; do not silently substitute an identity transform.

## Audit the Estimation Pipeline

1. Validate sensor health, saturation, rate, noise, missing packets, and timestamp monotonicity.
2. Validate preprocessing such as filtering, feature extraction, segmentation, deskewing, and motion compensation independently.
3. Validate frontend association with residuals, inlier geometry, degeneracy indicators, and repeatability.
4. Validate prediction against raw inertial or wheel data before adding global corrections.
5. Validate backend optimization with cost reduction, conditioning, convergence status, and iteration budget.
6. Validate loop candidates before optimization using appearance and geometric consistency checks.
7. Validate map-to-local correction separately from the continuous local odometry output.
8. Validate relocalization hypotheses with multi-frame consistency before accepting a discontinuous global correction.
9. Validate published state, covariance, frames, timestamps, and validity metadata at the external boundary.
10. Reproduce failures on the smallest recorded interval that preserves the causal chain.

## Define Map Semantics

- Name the map representation: landmarks, surfels, point cloud, voxels, occupancy probability, signed distance, elevation, traversability, or hybrid layers.
- Define resolution, origin, axis orientation, bounds, indexing, interpolation, and out-of-bounds behavior.
- Distinguish unknown, observed-free, occupied, unobserved, invalid, and outside-map cells.
- Define occupancy thresholds and whether values are probabilities, log odds, costs, labels, or confidence.
- Define signed-distance sign, truncation distance, gradient convention, and behavior in unknown space.
- Keep geometric occupancy separate from safety inflation and planner-specific costs.
- Define ray clearing, hit integration, decay, persistence, and dynamic-object filtering policies.
- Prevent stale dynamic observations from becoming permanent structure without an explicit policy.
- Specify how map updates are synchronized with queries and whether readers see snapshots or partial mutations.
- Check memory growth, tile eviction, serialization precision, compression, and schema compatibility.
- Preserve provenance: sensor set, calibration, software version, parameters, environment, and creation time.
- Validate saved and reloaded maps for pose convention, resolution, origin, labels, and numeric loss.
- Treat localization maps and planning maps as separate products unless their semantic contract is demonstrably identical.

## Design Lifecycle and Recovery

Implement explicit states such as uninitialized, initializing, tracking, degraded, lost, relocalizing, and shutdown.

- Define entry, exit, timeout, and reset conditions for every state.
- Require enough excitation and measurement diversity before declaring initialization complete.
- Publish health separately from pose so consumers can reject stale but numerically finite output.
- Detect divergence using innovations, covariance, registration fitness, consistency checks, and motion plausibility.
- Degrade gracefully when one sensor fails; state which guarantees remain and which are withdrawn.
- Bound dead reckoning duration and accumulated uncertainty.
- Require relocalization acceptance thresholds, ambiguity rejection, and temporal confirmation.
- Preserve local continuity while applying global corrections through a clearly owned transform or equivalent abstraction.
- Make reset scope explicit: frontend, estimator state, map, calibration, or complete process.
- Ensure shutdown drains or cancels work without publishing partially updated state.

## Measure Performance

Establish a reproducible baseline before changing algorithms or parameters.

- Measure absolute trajectory error only after declaring the alignment degrees of freedom.
- Measure relative pose error across multiple distances and time intervals.
- Report translation and rotation separately with median, percentiles, maximum, and failure count.
- Measure drift per distance and per time, relocalization precision/recall, recovery time, and false relocalization rate.
- Evaluate consistency with innovation statistics and NEES/NIS when calibrated ground truth permits it.
- Measure map completeness, accuracy, consistency, change latency, memory, and query throughput.
- Measure frontend, backend, loop closure, map update, and end-to-end latency separately.
- Report CPU, accelerator, memory, bandwidth, queue depth, dropped data, and deadline misses.
- Stratify results by motion, speed, lighting, texture, geometry, dynamic content, and sensor degradation.
- Keep datasets, configuration, seeds, software revision, hardware, and evaluation scripts fixed and recorded.

## Diagnose Common Failures

| Symptom | Check first | Do not assume |
| --- | --- | --- |
| Oscillating pose | time offset, frame direction, delayed corrections | filter gains are the root cause |
| Slow drift | observability, bias model, extrinsics, scale | loop closure alone will fix it |
| Sudden jump | relocalization or loop acceptance, transform authority | the pose publisher is merely noisy |
| Curved walls | deskewing, per-point timing, angular motion | map resolution is too low |
| Double surfaces | time sync, extrinsics, pose interpolation | voxel filtering is insufficient |
| False loops | perceptual aliasing and geometric verification | more loop candidates are better |
| Map memory growth | bounds, eviction, duplicate insertion | the environment is simply large |
| Good offline, bad live | clock domains, queues, compute deadlines | dataset quality explains the gap |

## Verify in Stages

1. Run formatting, schema, unit, convention, and static interface checks.
2. Test transform composition, interpolation, Jacobians, covariance propagation, and map indexing with synthetic cases.
3. Replay deterministic recorded data and compare trajectory, residuals, map checksums, timing, and lifecycle transitions.
4. Inject packet loss, reordering, clock offsets, bad calibration, sensor saturation, dynamic obstacles, and compute overload.
5. Run closed-loop simulation with ground truth hidden from the estimator and used only for evaluation.
6. Run hardware-in-the-loop with production clocks, transport, compute, and sensor rates.
7. Run constrained real-robot tests with speed limits, geofencing, an independent stop path, and named observers.
8. Expand to representative environments only after earlier acceptance thresholds pass.
9. Preserve failing datasets and add regression tests for every repaired contract violation.

## Report with Evidence

- Lead with the operational consequence and the owning layer.
- Cite implementation, active configuration, interfaces, tests, and runtime evidence independently when available.
- State frame, timestamp, unit, state, and map semantics for every important numeric claim.
- Separate confirmed facts, supported inferences, hypotheses, and unknowns.
- Provide the smallest corrective change that restores the violated contract.
- Give measurable acceptance criteria and exact verification scope.
- Report tests not run, unavailable ground truth, environmental limits, and residual safety risk.
- Avoid framework-specific prescriptions unless the repository proves that framework is active.
