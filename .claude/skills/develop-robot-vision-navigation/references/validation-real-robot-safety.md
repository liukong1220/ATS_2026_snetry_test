# Validation And Real-Robot Safety

Use this guide before hardware-in-the-loop, powered hardware, field, autonomous, high-speed, or
otherwise hazardous validation. Increase physical risk only after lower-risk evidence meets explicit
gates. Keep a human-controlled stop path independent of the algorithm under test.

## Contents

- [Define The Safety Envelope](#define-the-safety-envelope)
- [Classify Risk](#classify-risk)
- [Prepare Evidence And Ownership](#prepare-evidence-and-ownership)
- [Gate 0: Static And Mathematical Checks](#gate-0-static-and-mathematical-checks)
- [Gate 1: Offline And Replay](#gate-1-offline-and-replay)
- [Gate 2: Simulation](#gate-2-simulation)
- [Gate 3: Hardware-In-The-Loop](#gate-3-hardware-in-the-loop)
- [Gate 4: Constrained Real Robot](#gate-4-constrained-real-robot)
- [Gate 5: Representative Operation](#gate-5-representative-operation)
- [Stop, Recover, And Roll Back](#stop-recover-and-roll-back)
- [Report Evidence](#report-evidence)

## Define The Safety Envelope

Define the test before powering hardware:

| Item | Required Definition |
|---|---|
| Objective | State one behavior and one decision the test will validate. |
| Environment | Define boundaries, surface, lighting, visibility, traffic, and exclusion zone. |
| Robot state | Define payload, battery, calibration, firmware, and mechanical condition. |
| Energy limits | Set speed, acceleration, torque, power, range, and actuator limits. |
| Human roles | Name operator, safety observer, test lead, and data recorder. |
| Stop path | Specify physical e-stop, remote stop, software stop, and power isolation. |
| Pass criteria | Set metric thresholds and required duration or repetitions. |
| Stop criteria | Set immediate triggers before the run. |
| Rollback | Identify the last known-good revision and restoration procedure. |

Remove people, fragile equipment, public traffic, and uncontrolled robots from the reachable area.
Use barriers, restraints, stands, or open space appropriate to stored energy and stopping distance.
Treat rotating, flying, projectile, cutting, high-voltage, and combustion systems as special hazards
requiring domain-specific controls beyond this guide.

## Classify Risk

Assign the highest applicable risk tier:

| Tier | Example Exposure | Maximum Validation Stage Without Additional Controls |
|---|---|---|
| R0 | Static analysis or logged data | Offline |
| R1 | Simulation or unpowered interfaces | Simulation |
| R2 | Powered sensors or restrained actuators | HIL/bench |
| R3 | Low-speed motion in a controlled area | Constrained real robot |
| R4 | High speed, heavy payload, close obstacles, or autonomous field use | Representative operation with formal review |
| R5 | Risk to public, safety-critical service, or irreversible action | Stop and require qualified authority and applicable process |

Increase the tier for unknown braking distance, unverified command ownership, intermittent communications,
localization uncertainty, untested firmware, damaged hardware, or absent independent stop control. Do not
lower risk based only on confidence in the algorithm.

## Prepare Evidence And Ownership

1. Freeze the source revision, model, calibration, map, parameters, firmware, and test data.
2. Record all uncommitted changes and generated artifacts.
3. Identify the unique writer for motion commands and every override path.
4. Verify actuator limits at hardware or firmware level where possible.
5. Verify command timeouts force zero, hold, or another documented safe state.
6. Verify localization, perception, planning, control, and communications health indicators.
7. Verify clocks, frames, units, transforms, footprint, and actuator signs.
8. Calculate stopping distance with latency, braking performance, slope, and safety margin.
9. Rehearse e-stop, remote stop, software stop, and power isolation while stationary.
10. Confirm that logs and telemetry cannot block the control loop.

Do not proceed when command arbitration, stop authority, sign conventions, timeouts, or physical limits are unknown.

## Gate 0: Static And Mathematical Checks

Complete checks with no powered motion:

- Validate schemas, enum defaults, parameter ranges, units, and frame conventions.
- Check state dimensions, matrix shapes, finite values, covariance symmetry, and positive semidefiniteness where required.
- Check controller constraints, saturation, rate limits, and zero-command behavior.
- Check queue bounds, cancellation, shutdown, watchdogs, and reconnect behavior.
- Run format, type, build, unit, property, and contract tests relevant to the change.
- Review failure branches for stale data, missing transforms, invalid estimates, no path, solver timeout, communication loss, and actuator rejection.
- Verify deterministic safe defaults after startup, reset, and partial initialization.

Pass only when all safety-relevant invariants hold or deviations have approved compensating controls.
Preserve test commands and results.

## Gate 1: Offline And Replay

Use recorded or generated inputs without physical actuation:

1. Replay nominal, boundary, rare, and known-failure sequences.
2. Preserve original timing, then repeat with accelerated and stressed timing.
3. Inject dropouts, stale data, reordering, clock jumps, invalid values, and overload.
4. Compare outputs against ground truth, invariants, and the last known-good revision.
5. Measure accuracy, p50/p95/p99 latency, deadline misses, queue occupancy, memory, and recovery.
6. Inspect state transitions and final commands around every injected fault.
7. Verify deterministic or statistically bounded results across repeated seeds.

Pass only when primary metrics meet thresholds, safety guardrails have no violations, every critical fault
reaches a safe state within its deadline, and unexplained regressions are absent.

## Gate 2: Simulation

Close the perception-to-control loop in simulation:

- Match sensor rates, delay, noise, field of view, dynamics, actuator limits, and collision shape.
- Randomize conditions within the intended operating envelope.
- Test startup, stop, reset, goal change, target loss, relocalization, replanning, and recovery.
- Test moving obstacles, occlusion, degraded texture, poor lighting, slip, and saturation.
- Inject communication loss, compute overload, transform loss, and process restart.
- Observe collisions, minimum clearance, command discontinuity, oscillation, tracking error,
  localization error, solver failures, and recovery time.
- Repeat across seeds and preserve failing scenarios for regression.

Do not treat a simulator as proof of unmodeled hardware behavior. Pass only when no safety invariant is
violated and uncertainty between simulation and hardware is explicitly bounded.

## Gate 3: Hardware-In-The-Loop

Connect production sensors, compute, networks, controllers, or actuators while constraining physical energy.

1. Begin with actuators disabled, disconnected, restrained, lifted, or replaced by loads.
2. Verify real sensor timestamps, calibration, throughput, thermal behavior, and power state.
3. Verify command signs and magnitudes one axis at a time.
4. Verify hardware watchdogs and command leases by intentionally stopping producers.
5. Inject network loss, process crash, sensor disconnect, and compute saturation.
6. Exercise e-stop and recovery without assuming the main compute remains responsive.
7. Measure end-to-end sensor-to-command and command-to-actuator latency.
8. Inspect temperatures, current, voltage, packet errors, and actuator faults.

Pass only when every command path, timeout, stop path, and recovery state behaves as documented under real
timing. Stop on unexpected motion, sign, magnitude, heat, current, sound, or delay.

## Gate 4: Constrained Real Robot

Use the lowest practical energy in a controlled, isolated area:

- Start with one actuator or degree of freedom where practical.
- Limit speed, acceleration, torque, workspace, route length, and run duration.
- Place the safety operator within reliable stop range but outside the reachable hazard area.
- Maintain line of sight and a clear escape path.
- Test straight motion and stopping before turns, tracking, replanning, or autonomy.
- Add one complexity at a time: sensing, estimation, planning, dynamics, then disturbances.
- Measure actual stopping distance and update the exclusion zone.
- Stop between runs to review metrics, faults, and hardware condition.
- Repeat fault and recovery cases only when their physical consequence is controlled.

Pass only after repeated runs meet acceptance thresholds with no unexplained state, command, contact,
instability, or operator intervention.

## Gate 5: Representative Operation

Approach full operation incrementally:

1. Expand speed, area, duration, obstacle density, and autonomy one dimension at a time.
2. Preserve independent stop authority and real-time health telemetry.
3. Run nominal conditions before boundary and degraded conditions.
4. Use predefined routes, scenarios, and repetitions for comparison.
5. Monitor localization confidence, perception freshness, plan validity, controller status, command arbitration, communications, power, temperature, and clearance.
6. Enforce automatic speed reduction or safe stop when health leaves the validated envelope.
7. Require a fresh review after hardware, firmware, calibration, model, map, or safety-critical parameter changes.

Do not claim field readiness from a single successful run. Require repeatability, tail metrics, fault
recovery, and evidence across the intended envelope.

## Stop, Recover, And Roll Back

Trigger an immediate stop for any predefined condition, including:

- loss of independent stop control or safety observer communication;
- unexpected motion, command source, direction, magnitude, oscillation, or acceleration;
- stale or invalid state accepted as valid;
- localization jump, transform discontinuity, unsafe path, or clearance breach;
- watchdog, solver, actuator, battery, thermal, network, or compute-limit violation;
- person or unplanned object entering the exclusion zone;
- missing telemetry needed to assess safety;
- any outcome outside the approved envelope.

After stopping, keep the system de-energized or inhibited until the state is understood. Preserve logs and
volatile diagnostics before restart when safe. Record the exact trigger, robot state, revision, configuration,
and operator action. Reproduce at a lower-risk gate. Roll back when the candidate cannot meet its gate, the failure is not understood, or safe recovery is uncertain.
Do not bypass a stop criterion to finish a run.

## Report Evidence

Create a validation record containing:

| Category | Evidence |
|---|---|
| Identity | revision, model, configuration, calibration, map, firmware, hardware |
| Scope | objective, risk tier, environment, limits, roles, and exclusions |
| Gates | commands, scenarios, repetitions, artifacts, and pass/fail results |
| Metrics | primary metrics, tail latency, faults, interventions, and uncertainty |
| Safety | stop checks, stopping distance, exclusion zone, watchdogs, and rollbacks |
| Deviations | skipped tests, changed conditions, anomalies, and unresolved unknowns |

Distinguish observed results from expected behavior. Report unexecuted gates explicitly. State the validated
envelope, residual risks, confidence, and exact conditions required before the next increase in physical risk.
