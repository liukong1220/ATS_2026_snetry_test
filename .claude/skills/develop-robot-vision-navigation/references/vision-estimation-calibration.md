# Vision Estimation and Calibration

Use this reference for camera calibration, time alignment, pose solving, hand-eye calibration,
multi-sensor fusion, target or vehicle state estimation, prediction, and estimator tuning.

## Contents

- Evidence-first workflow
- Frames, time, and units
- Camera and extrinsic calibration
- Pose estimation
- Dynamic state estimation
- Consistency and observability
- Decision workflow and failure modes
- Metrics and validation

## Evidence-first workflow

1. Write the physical quantity that must be estimated and why downstream logic needs it.
2. Locate sensor acquisition, calibration loading, timestamp handling, transforms, measurement
   construction, estimator state, prediction, update, reset, output, parameters, and tests.
3. Draw the frame and time chain before inspecting tuning constants.
4. Write the implemented model from code, including state order, units, noise, constraints, and
   linearization point; do not rely on comments alone.
5. Recompute one representative measurement or update independently.
6. Compare code, configuration, calibration artifact, and runtime trace for critical claims.
7. Establish a recorded-data baseline with immutable inputs before changing calibration or noise.
8. Attribute failures to sensing, geometry, timing, modeling, numerics, or observability before tuning.

## Frames, time, and units

Define every transform as `T_A_B`, mapping coordinates from frame B into frame A.

- Record handedness, axis directions, rotation representation, angle units, and composition order.
- Verify `T_A_C = T_A_B * T_B_C` with a known point and an inverse round trip.
- Keep active and passive rotation conventions distinct.
- Normalize quaternions and handle the `q` versus `-q` equivalence in comparisons.
- Wrap angle innovations consistently; do not subtract periodic angles as ordinary scalars.
- Use SI units internally unless an interface explicitly states otherwise.
- Convert degrees, millimeters, and device ticks exactly once at the boundary.
- Associate each measurement with acquisition time, not processing or publication time.
- Name clock domains and estimate offsets or drift before fusing asynchronous devices.
- Interpolate poses only within supported intervals; reject or mark extrapolation explicitly.
- Apply latency compensation from measured latency distributions, not a single hopeful constant.

Treat these as invariants:

- Express a residual and its covariance in the same coordinates.
- Evaluate a transform at the observation time.
- Rotate covariance whenever rotating the corresponding random variable.
- Preserve positive semidefinite covariance within numerical tolerance.
- Never update twice from the same measurement unless correlation is modeled.
- Never fuse two estimates as independent when they share raw observations or priors.

## Camera and extrinsic calibration

Calibrate intrinsics with representative focus, aperture, resolution, and image pipeline settings.

- Select a camera model supported by observed distortion and field of view.
- Cover the image plane, depth range, orientation range, and expected focus range with samples.
- Reject blurred or poorly localized calibration features using stated criteria.
- Inspect per-image and spatial reprojection residuals, not only global RMS.
- Reserve independent images for validation; do not validate solely on calibration samples.
- Store image size, model name, coefficient order, units, date, and device identity with parameters.
- Fail loudly when runtime resolution or crop invalidates the stored intrinsics.
- Verify whether image rectification has already occurred before applying distortion correction.

For a projected point, use the full declared model. For the pinhole core,
`u = fx*x/z + cx` and `v = fy*y/z + cy`; never omit distortion silently.

Calibrate extrinsics from informative motion and geometry:

- Define which transform the solver returns and invert only with evidence.
- Excite rotation around multiple axes and translation along multiple directions.
- Avoid nearly static, planar-only, or single-axis datasets when they leave parameters weakly observed.
- Verify target rigidity, synchronization, feature localization, and sensor mounting stability.
- Validate on held-out motion by predicting one sensor observation from the other.
- Report translation error, rotation error, residual distribution, and repeatability across runs.
- Treat online extrinsic adaptation as a separate estimator with priors and bounds.

Estimate temporal offset when motion exists:

- Cross-correlate physically comparable angular or translational signals for an initial estimate.
- Refine offset jointly only when the motion makes it observable.
- Inspect residual versus speed; timing error often grows with angular or linear velocity.
- Model clock drift for long runs when devices do not share a disciplined clock.

## Pose estimation

Match the solver to the geometry:

- Use a solver designed for planar, non-planar, minimal, or redundant point sets as applicable.
- Use robust sampling or loss functions when correspondences can contain outliers.
- Generate and score all plausible pose solutions for ambiguous configurations.
- Enforce cheirality, physical dimensions, workspace bounds, and temporal continuity explicitly.
- Refine pose with all accepted observations and a stated pixel-noise model.
- Compare reprojection in the original measurement space.

Define reprojection cost as
`J(T) = sum_i rho((z_i - project(T*P_i))^T W_i (z_i - project(T*P_i)))`.

- Derive `W_i` from measurement uncertainty when available.
- Use a robust loss `rho` only to limit outliers; do not use it to excuse wrong correspondences.
- Inspect residual vectors for spatial patterns indicating distortion or ordering errors.
- Propagate pose uncertainty from pixel noise or estimate it empirically by repeated observations.
- Reject a numerically converged solution that violates geometry or uncertainty limits.

## Dynamic state estimation

Define the estimator mathematically before tuning:

- State the state vector `x`, control `u`, process model `x_k = f(x_{k-1}, u_k, dt) + w_k`,
  measurement model `z_k = h(x_k) + v_k`, and output mapping.
- State the frame and units of every state component.
- State assumptions behind process noise `Q`, measurement noise `R`, and initial covariance `P0`.
- Scale discrete process noise with `dt` according to the continuous-time model.
- Use actual timestamp differences and bound invalid or extreme `dt` deliberately.
- Predict to each measurement time, update once, then predict to the requested output time.
- Maintain cross-covariances; do not filter coupled dimensions independently without justification.
- Use the Joseph covariance update when numerical robustness matters:
  `P = (I-KH) P (I-KH)^T + K R K^T`.
- Use stable factorizations or linear solves instead of explicit matrix inversion.
- Symmetrize only as a numerical cleanup; investigate material asymmetry.

Gate innovations using `r = z - h(x_pred)` and `S = H P_pred H^T + R`.

- Compute normalized innovation squared `NIS = r^T S^-1 r`.
- Select chi-square bounds from measurement dimension and a stated probability.
- Log accepted and rejected innovations with reason, age, and source.
- Avoid permanent rejection after divergence; define reset, covariance inflation, or reacquisition.
- Separate filter confidence from target-existence confidence.
- Bound predictions by valid dynamics and stop extrapolating after a stated horizon.

## Consistency and observability

Test whether the estimator is credible, not merely smooth:

- Compute normalized estimation error squared when ground truth exists:
  `NEES = e^T P^-1 e`, where `e = x_est - x_true` in a valid error representation.
- Compare NIS and NEES distributions with confidence intervals across many runs.
- Interpret persistent high values as inconsistency, modeling error, underestimated noise, or outliers.
- Interpret persistent low values as overestimated uncertainty or correlated samples.
- Report RMSE and covariance consistency together; low RMSE alone does not validate uncertainty.
- Inspect residual whiteness and autocorrelation; correlated innovations expose missing dynamics.
- Analyze local observability or rank of the linearized observability matrix where practical.
- Identify motions and geometries that make depth, scale, bias, yaw, or extrinsics unobservable.
- Freeze, regularize, or report weak states instead of pretending they are well estimated.

## Decision workflow and failure modes

Use the following branches:

1. If residuals have a constant spatial bias, inspect intrinsics, extrinsics, feature definitions,
   and coordinate conventions before changing noise.
2. If error grows with speed, inspect timestamps, latency, rolling shutter, and motion model.
3. If estimates jump between valid poses, inspect geometric ambiguity, point ordering, solution
   selection, and temporal priors.
4. If estimates are smooth but delayed, measure queueing, filter horizon, smoothing window, and
   output-time prediction separately.
5. If innovations are frequently rejected, verify residual frame, covariance, association, and
   model validity before widening the gate.
6. If covariance collapses while error grows, inspect duplicated measurements, unmodeled
   correlation, underestimated noise, and incorrect Jacobians.
7. If covariance explodes, inspect observability, missing updates, unit errors, extreme `dt`, and
   unstable dynamics before adding arbitrary clamps.
8. If resets oscillate, add explicit state-machine hysteresis and validate the recovery contract.

| Symptom | Candidate cause | Discriminating evidence |
| --- | --- | --- |
| Mirrored or rotated pose | frame handedness, point order, transform direction | known-point transform test |
| Range-dependent bias | focal scale, distortion, target dimensions | residual by image radius and depth |
| Speed-dependent bias | time offset, latency, rolling shutter | residual by velocity and offset sweep |
| Filter divergence | wrong model, Jacobian, units, association | innovation and covariance replay |
| Overconfident output | low Q/R, shared data, double update | NIS/NEES and update provenance |
| Periodic angle jump | missing wrap, representation singularity | boundary-focused unit test |

## Metrics and validation

Report calibration and estimator metrics by operational slice:

- Report reprojection median, RMS, p95, spatial residual map, and held-out error.
- Report extrinsic translation and rotation error plus repeatability across independent datasets.
- Report estimated time offset, uncertainty, drift, and residual versus motion.
- Report pose translation error, rotation geodesic error, failure rate, ambiguity rate, and runtime.
- Report state RMSE, bias, p95 error, NIS, NEES, rejection rate, reset rate, and recovery time.
- Report prediction error by horizon and end-to-end capture-to-estimate latency.
- Preserve calibration artifacts, raw data, solver settings, seeds, code revision, and hardware IDs.

Validate in increasing scope:

1. Unit-test transforms, inverses, angle wrapping, projection, Jacobians, and covariance rotation.
2. Compare analytical Jacobians against finite differences or automatic differentiation.
3. Use synthetic noise-free cases with known truth, then add controlled noise and outliers.
4. Replay immutable recorded data and compare residual, state, and covariance traces.
5. Validate calibration on held-out scenes and repeat the full calibration independently.
6. Stress timing, dropped measurements, variable rates, long occlusion, and abrupt reacquisition.
7. Run integrated simulation, hardware-in-the-loop, and bounded real-robot tests.

Require every result to state the data source, ground-truth quality, model assumptions,
acceptance thresholds, tests run, tests omitted, residual risks, and confidence level.
