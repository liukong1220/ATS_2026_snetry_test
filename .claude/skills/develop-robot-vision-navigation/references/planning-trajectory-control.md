# Planning, Trajectory Optimization, and Control

Use this reference for repository-agnostic analysis, implementation, review, and validation of robot navigation from intent through actuator command. Keep behavior, path planning, trajectory generation, feedback control, and safety supervision as distinct contracts even when one component implements several of them.

## Contents

- [Discover the Executed Stack](#discover-the-executed-stack)
- [Separate Responsibilities](#separate-responsibilities)
- [Define the Motion Contract](#define-the-motion-contract)
- [Audit Maps and Collision Semantics](#audit-maps-and-collision-semantics)
- [Analyze Planning](#analyze-planning)
- [Analyze Trajectory Optimization](#analyze-trajectory-optimization)
- [Analyze Feedback Control](#analyze-feedback-control)
- [Control Timing and Concurrency](#control-timing-and-concurrency)
- [Design Safety and Recovery](#design-safety-and-recovery)
- [Measure Performance](#measure-performance)
- [Diagnose Common Failures](#diagnose-common-failures)
- [Verify in Stages](#verify-in-stages)
- [Report with Evidence](#report-with-evidence)

## Discover the Executed Stack

1. Read repository instructions, build manifests, runtime entry points, active parameters, interfaces, tests, and deployment overlays.
2. Identify whether the stack uses ROS, another middleware, or direct function and process calls; reason from contracts rather than names.
3. Trace the actual chain from mission intent to behavior selection, goal generation, route, path, trajectory, control reference, command arbitration, and actuator.
4. Identify every replanning trigger, timer, callback, queue, worker, transport, and watchdog on that chain.
5. Locate the state estimate, map, obstacle, prediction, footprint, dynamics, and actuator-limit sources used at runtime.
6. Find the active configuration and distinguish defaults, examples, generated values, tuning overlays, and dead options.
7. Identify one authority for each goal, map layer, command topic or channel, mode, stop request, and hardware output.
8. Separate third-party algorithms from project-owned adapters, objectives, constraints, and fallback policies.
9. Preserve uncommitted changes and avoid reading generated or vendored trees unless evidence points there.
10. Reconstruct a minimal reproducible scenario before tuning parameters.

## Separate Responsibilities

Keep these products explicit and do not treat their names as interchangeable.

- Let mission logic choose objectives, priorities, and acceptable risk.
- Let behavior logic select modes, goals, recovery actions, and cancellation policy.
- Let global planning choose a collision-free route through large-scale topology or configuration space.
- Let local planning react to nearby geometry, moving agents, kinodynamic limits, and short-horizon changes.
- Let trajectory generation attach time and dynamically feasible derivatives to a geometric path.
- Let feedback control track the reference under state error, delay, disturbance, and actuator limits.
- Let command arbitration enforce ownership and priority across autonomy, teleoperation, safety, and calibration.
- Let safety supervision independently stop or constrain motion when upstream assumptions fail.
- Document intentional responsibility overlaps and define which output wins.
- Fix a defect in the layer that owns its invariant; do not hide invalid goals with controller gains or bad state with planner costs.

## Define the Motion Contract

- Define state `x`, control `u`, disturbance `w`, dynamics `x_dot = f(x,u,w)` or `x[k+1] = f_d(x[k],u[k],w[k])`, and output variables.
- State whether the model is holonomic, differential, Ackermann, legged, aerial, marine, articulated, or a measured black-box approximation.
- Define every pose, twist, acceleration, curvature, force, and command frame with units and sign conventions.
- Distinguish body-frame velocity from world-frame velocity and steering angle from yaw rate.
- Define reference timestamps, validity intervals, update rates, sequence identifiers, and stale-data behavior.
- Define whether a path is geometric, arc-length parameterized, time parameterized, or already a control rollout.
- Define footprint geometry, reference point, swept volume, safety margin, and configuration dependence.
- Define velocity, acceleration, jerk, curvature, steering, force, power, and rate constraints from physical evidence.
- Define goal tolerances, terminal velocity, orientation requirements, dwell time, and success ownership.
- Define cancellation, preemption, replacement, and reset semantics across every boundary.
- Define behavior when state, map, obstacle prediction, solver output, or actuator feedback is missing or stale.
- Verify producers and consumers agree on all semantics before modifying objective weights.

## Audit Maps and Collision Semantics

- Identify whether planners consume occupancy, cost, signed distance, elevation, traversability, topology, semantic zones, or combinations.
- Define resolution, origin, frame, update timestamp, interpolation, bounds, and behavior outside the map.
- Distinguish unknown, observed-free, occupied, inflated, lethal, invalid, and outside-map states.
- Confirm whether unknown is traversable, penalized, or forbidden in each operating mode.
- Keep physical occupancy, uncertainty margin, footprint inflation, and preference costs conceptually separate.
- Verify signed-distance sign, truncation, gradient direction, and treatment of unknown voxels before using gradients.
- Evaluate collision over the full footprint and swept motion, not only the robot reference point.
- Match collision-check sampling to speed, curvature, obstacle scale, map resolution, and control period.
- Account for localization uncertainty, tracking error, braking distance, perception latency, and obstacle motion in clearance.
- Check map snapshot consistency while planning; reject mixed-time layers when their inconsistency can invalidate safety.
- Expire or predict dynamic obstacles explicitly; do not let stale tracks become silently static or disappear without policy.
- Test narrow passages, map edges, unknown boundaries, rotating footprints, and height or terrain transitions.

## Analyze Planning

1. State the planning problem: state space, start, goal set, obstacles, dynamics, constraints, objective, and compute deadline.
2. Validate start and goal states before invoking the planner; distinguish invalid, unreachable, timed out, and internally failed.
3. Verify the search or sampling resolution can represent required passages and motions.
4. Verify heuristics are admissible or document intentional suboptimality and its operational benefit.
5. Bound search, sampling, optimization, and memory; expose timeout and partial-solution semantics.
6. Preserve motion-model feasibility in successor generation or add an explicit, verified feasibility restoration stage.
7. Check cost composition dimensions; normalize terms before comparing or weighting unrelated quantities.
8. Prevent negative cycles, NaN costs, integer overflow, stale caches, and inconsistent obstacle versions.
9. Verify goal-selection logic does not produce oscillation between equivalent candidates.
10. Add hysteresis or commitment only after measuring the cause of switching.
11. Distinguish no-path evidence from insufficient planning time or an overly conservative representation.
12. Validate shortcutting and smoothing because post-processing can reintroduce collision or dynamic infeasibility.
13. Return a structured status with reason, best candidate, age, and safety validity.
14. Test deterministic seeds where repeatability matters and multiple seeds where stochastic robustness matters.

## Analyze Trajectory Optimization

- State the trajectory representation: waypoints, polynomial segments, splines, control points, direct collocation, or sampled controls.
- Define the independent variable and verify all derivatives use the same time or arc-length parameterization.
- Write the complete objective with units: progress, duration, path error, smoothness, clearance, control effort, terminal error, and risk.
- Write hard constraints separately from soft penalties and state which may be violated.
- Normalize cost terms using physical scales; do not infer correctness from a decreasing scalar cost alone.
- Enforce continuity at the required derivative order across segments.
- Enforce position, velocity, acceleration, jerk, curvature, steering, and actuator constraints over continuous segments or a justified discretization.
- Check collision between samples with conservative bounds, adaptive subdivision, or swept-volume tests.
- Account for moving obstacles at the trajectory time, not only at the current map time.
- Validate distance-field gradients near truncation, discontinuities, unknown cells, and map boundaries.
- Prevent zero or negative segment durations and ill-conditioned polynomial bases.
- Scale decision variables and constraints to improve conditioning; log residuals and solver termination reasons.
- Supply a feasible or clearly classified warm start; distinguish infeasibility from numerical failure.
- Bound iterations and wall time, and define whether the last iterate is safe to execute.
- Revalidate the final trajectory independently of the optimizer's internal constraints.
- Preserve a known-safe previous trajectory only while its state, map, and time validity remain true.
- Compare optimization against a simple baseline to justify added complexity and compute cost.

## Analyze Feedback Control

- Derive the controller from the same state, input, frame, reference point, and discrete interval used by the implementation.
- Verify controllability and operating-region assumptions for the selected model.
- Include steering, actuator, drivetrain, contact, or flight dynamics when omitted dynamics dominate tracking error.
- Align estimator state time with the control reference using bounded prediction or delay compensation.
- Define horizon duration as well as sample count; changing the control period changes both model and lookahead.
- For MPC, state stage cost, terminal cost, constraints, terminal assumptions, warm start, and infeasibility behavior.
- For sampling control, state proposal distribution, temperature, rollout count, noise covariance, constraints, and deterministic test mode.
- Penalize quantities in consistent units and inspect each cost component rather than only total cost.
- Enforce input magnitude and slew-rate constraints at the command actually sent to hardware.
- Handle actuator saturation and integrator windup explicitly.
- Verify angle wrapping, quaternion errors, shortest-turn logic, and reverse-motion conventions.
- Define low-speed and standstill behavior where curvature, heading, or division-by-speed models become singular.
- Define the final-stop controller and verify terminal velocity instead of declaring success from position alone.
- Reject NaN, infinite, stale, frame-inconsistent, or dynamically impossible references before actuation.
- Publish solver health, tracking error, saturation, and fallback state for supervision.
- Test recovery after overruns and infeasibility; a controller must not replay an indefinitely stale command.

## Control Timing and Concurrency

- Build a latency budget from sensing and estimation through planning, control, arbitration, transport, and actuator response.
- Use monotonic time for deadlines and durations; keep acquisition time for state alignment.
- Measure state age, map age, obstacle age, reference age, compute time, queue delay, and command age independently.
- Match prediction to the state and command execution time rather than the callback invocation time.
- Use bounded queues and make replacement, dropping, and backpressure policies explicit.
- Prevent overlapping planners or solvers from publishing out of order; attach generation IDs and cancel obsolete work.
- Protect shared maps, trajectories, warm starts, and mode state with snapshots or explicit synchronization.
- Keep blocking I/O and unbounded allocation out of hard or soft real-time control paths.
- Detect deadline misses and transition to a bounded fallback instead of silently reducing update rate.
- Test scheduler load, accelerator contention, network delay, clock resets, and long-duration resource growth.

## Design Safety and Recovery

- Implement an independent stop path that does not depend on the nominal planner or optimizer succeeding.
- Define command priority and arbitration for safety, manual control, autonomy, testing, and calibration.
- Require heartbeat, freshness, validity, and mode agreement before forwarding motion commands.
- Compute a conservative stopping envelope from speed, latency, braking capability, slope, and uncertainty.
- Distinguish pause, controlled stop, emergency stop, recovery, and shutdown.
- Define bounded recovery attempts and prevent cycles among clear, rotate, reverse, replan, and relocalize actions.
- Validate recovery motions with the same collision and actuator contracts as nominal motion.
- Prevent automatic recovery when localization, map validity, or obstacle sensing is insufficient.
- Record the trigger, active command, state, map version, and reason for every safety transition.
- Restore autonomy only after explicit health criteria and command ownership are re-established.

## Measure Performance

Establish fixed scenarios and a reproducible baseline before optimization.

- Measure planning success, valid-solution latency, timeout rate, path length, clearance, curvature, and cost components.
- Measure trajectory feasibility, duration, smoothness, minimum clearance, constraint residuals, and solver convergence.
- Measure tracking position, heading, velocity, and control error with median, p95, maximum, and time outside tolerance.
- Measure end-to-end goal success, completion time, replans, recoveries, collisions, near misses, and false stops.
- Measure control-loop jitter, p50/p95/p99 compute latency, deadline misses, command age, CPU, accelerator, memory, and bandwidth.
- Stratify results by speed, payload, surface, slope, obstacle density, dynamic motion, map quality, and localization uncertainty.
- Evaluate disturbance rejection, model mismatch, sensor dropout, actuator saturation, and communication loss.
- Record software revision, active configuration, map, scenario seed, hardware, initial state, and evaluator version.
- Compare against a simple stable baseline and report confidence intervals over repeated trials.
- Optimize one causally related parameter group at a time and retain failed trials.

## Diagnose Common Failures

| Symptom | Check first | Do not assume |
| --- | --- | --- |
| Path exists but robot stops | command authority, freshness, safety gate, actuator feedback | planner output reaches hardware |
| Replanning oscillation | goal switching, map churn, stale generations, hysteresis | higher planner frequency fixes it |
| Corner cutting | footprint, swept collision, tracking lag, sample spacing | controller gain is the only cause |
| Narrow passage failure | inflation, unknown policy, resolution, uncertainty margin | the search algorithm is defective |
| Control oscillation | delay, frame mismatch, model error, saturation | more damping alone is sufficient |
| Optimizer reports success but collides | discretization, map semantics, final revalidation | solver status proves safety |
| Good simulation, poor robot | latency, actuator dynamics, friction, clocks | random noise explains the gap |
| Stale command after failure | watchdog, fallback, queue ordering, ownership | zero will be sent automatically |

## Verify in Stages

1. Run formatting, schema, interface, unit, frame, and configuration validation.
2. Test motion models, integration, angle handling, collision checks, costs, constraints, and command arbitration with synthetic cases.
3. Test planners and optimizers on deterministic maps with known feasible, infeasible, boundary, and narrow-passage cases.
4. Replay recorded state and map streams while checking timing, generation ordering, output validity, and resource bounds.
5. Inject stale state, delayed maps, dropped messages, moving obstacles, solver overruns, infeasibility, and actuator saturation.
6. Run closed-loop simulation across repeated seeds and adversarial initial conditions.
7. Run hardware-in-the-loop with production compute, clocks, network, controller rate, and actuator interface.
8. Run constrained real-robot tests with speed and workspace limits, an independent stop operator, and explicit abort thresholds.
9. Expand speed and environment complexity only after every earlier acceptance threshold passes.
10. Preserve failures as regression scenarios and verify the fallback path as rigorously as nominal behavior.

## Report with Evidence

- Lead with the observed behavior, violated contract, owning layer, and operational risk.
- Cite active configuration, implementation, interfaces, tests, and runtime traces when available.
- Separate confirmed facts, supported inferences, hypotheses, and unknowns.
- State frames, units, timestamps, map semantics, dynamics, constraints, and deadlines for numeric conclusions.
- Propose the smallest change that fixes the owning invariant and identify compatibility consequences.
- Give measurable acceptance thresholds and the exact scenario matrix.
- Report tests not run, unavailable hardware or ground truth, residual risk, and rollback or stop conditions.
- Avoid prescribing a named framework unless repository evidence proves it is active and relevant.
