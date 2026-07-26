# Vision-Navigation Integration

Use this guide whenever data, state, or behavior crosses a component, process, middleware,
computer, repository, or vision-navigation boundary. Treat vision and navigation as peer
systems connected by explicit contracts. Diagnose the first broken invariant before tuning
either domain.

## Contents

- [Define The Boundary](#define-the-boundary)
- [Trace The Closed Loop](#trace-the-closed-loop)
- [Build The Contract Table](#build-the-contract-table)
- [Verify Geometry And Units](#verify-geometry-and-units)
- [Verify Time And Freshness](#verify-time-and-freshness)
- [Verify Transport And QoS](#verify-transport-and-qos)
- [Assign Ownership And Authority](#assign-ownership-and-authority)
- [Model Validity And Degradation](#model-validity-and-degradation)
- [Test The Integration](#test-the-integration)
- [Report The Evidence](#report-the-evidence)

## Define The Boundary

1. Name the producer, consumer, and behavior enabled by the interface.
2. Identify the concrete runtime artifact: message, function call, shared memory block,
   service, action, file, transform, bus packet, or hardware signal.
3. Locate the schema, producer assignment, serialization, transport configuration, consumer
   validation, and downstream use.
4. Distinguish declared configuration from effective runtime configuration.
5. Distinguish source ownership from deployment ownership when copies or generated bindings
   exist.
6. Mark every unverified link as unknown rather than completing the chain by intuition.

Record the minimum system map:

```text
sensor -> acquisition -> synchronization -> perception -> state estimation
       -> behavior/goal selection -> planning -> trajectory -> control
       -> actuator -> physical state -> sensor feedback
```

Shorten this map to the loop relevant to the observed failure. Preserve side channels that
can override behavior, including safety stops, manual control, watchdogs, and stale-data gates.

## Trace The Closed Loop

Trace one representative datum from physical observation to actuator consequence.

1. Start from the physical quantity, not only the software field name.
2. Record each transformation, filter, queue, gate, and coordinate conversion.
3. Record where identity, covariance, confidence, validity, and timestamps change.
4. Record the decision that converts an estimate into a goal or control reference.
5. Record actuator saturation, rate limiting, and command arbitration.
6. Follow feedback to the estimator or controller that closes the loop.
7. Identify the earliest point where expected and observed semantics diverge.

Use this boundary trace table:

| Hop | Producer | Artifact | Transformation | Consumer | Invariant | Evidence |
|---|---|---|---|---|---|---|
| 1 | component | topic/API | frame or unit conversion | component | stated condition | source/test/trace |

Do not accept matching type names as proof of compatible semantics. Verify every field that
can influence gating, identity, geometry, timing, uncertainty, or fallback behavior.

## Build The Contract Table

Create one row per field or transform with behavioral significance.

| Item | Required Contract | Producer Evidence | Consumer Evidence | Runtime Evidence | Status |
|---|---|---|---|---|---|
| Semantic meaning | Physical quantity and reference object | assignment/calculation | interpretation/gate | trace or bag | confirmed/unknown/broken |
| Type and range | scalar/vector/enum and valid domain | schema and checks | checks and branches | observed extrema | confirmed/unknown/broken |
| Unit | SI or explicitly named unit | conversion/calibration | expected scale | measured values | confirmed/unknown/broken |
| Frame | source, target, axes, handedness | frame assignment | transform lookup | transform snapshot | confirmed/unknown/broken |
| Time | observation time and clock | stamp creation | age calculation | timestamp trace | confirmed/unknown/broken |
| Freshness | maximum accepted age | publish cadence | timeout gate | age distribution | confirmed/unknown/broken |
| Uncertainty | covariance/confidence semantics | estimator output | threshold/weight | calibration curve | confirmed/unknown/broken |
| Validity | valid, stale, lost, degraded | state machine | fallback branch | fault injection | confirmed/unknown/broken |
| Transport | reliability, order, queue, drops | publisher/socket | subscriber/receiver | counters/trace | confirmed/unknown/broken |
| Ownership | unique authority and override order | writer inventory | arbiter/use | runtime graph | confirmed/unknown/broken |

Require an explicit statement for optional fields and default enum values. Treat zero-initialized
or absent fields as unsafe until the consumer semantics prove otherwise.

## Verify Geometry And Units

1. Write each transform as `p_target = T_target_source * p_source`.
2. State whether poses are active or passive and whether quaternions rotate vectors or frames.
3. State axes, handedness, angle sign, angular wrap interval, and Euler rotation order.
4. Verify transform direction at both publication and lookup sites.
5. Verify the reference point: feature, object center, vehicle center, sensor origin, footprint,
   goal point, or control point.
6. Verify dimensions independently: meters versus millimeters, seconds versus milliseconds,
   radians versus degrees, body velocity versus world velocity.
7. Check covariance ordering and transform covariance with the appropriate Jacobian or adjoint.
8. Test identity, one-axis translation, one-axis rotation, and round-trip transforms.
9. Reject NaN, infinity, non-normalized rotations, impossible ranges, and singular projections.

Do not repair a frame or unit defect by retuning a filter, planner, or controller.

## Verify Time And Freshness

Identify every clock domain before comparing timestamps:

| Time Value | Clock Domain | Creation Point | Meaning | Conversion | Consumer |
|---|---|---|---|---|---|
| observation | sensor/device/system/sim/monotonic | acquisition | physical sample time | documented mapping | estimator |
| publication | middleware/system | publisher | send time | none or offset | diagnostics |
| receipt | monotonic/system | receiver | arrival time | none | queue metrics |
| execution | control/sim/monotonic | controller | actuation time | prediction horizon | actuator |

1. Preserve observation time through perception and estimation.
2. Query transforms at observation time unless the algorithm explicitly compensates otherwise.
3. Measure sensor-to-actuator age, not only callback or inference duration.
4. Separate queue delay, compute delay, transport delay, transform wait, and control delay.
5. Define maximum age, maximum out-of-order span, and future-stamp tolerance.
6. Define behavior for clock jumps, simulation resets, reconnects, and device rollover.
7. Use monotonic clocks for durations and a synchronized domain for cross-device event time.
8. Predict state to execution time only with a stated motion model and bounded horizon.

## Verify Transport And QoS

Record transport semantics independently at producer and consumer:

- Set reliability, durability, ordering, history, queue depth, and deadline deliberately.
- Bound every real-time queue and state the overflow policy.
- Prefer latest-state semantics for replaceable sensor state; preserve ordered delivery for
  non-replaceable events and state transitions.
- Match latching or transient behavior only where late joiners require the last valid state.
- Measure publication rate, receipt rate, drops, duplicates, reordering, and queue occupancy.
- Verify serialization compatibility, enum stability, alignment, endianness, and versioning.
- Define reconnect, resubscription, replay, and partial-message behavior.
- Avoid assuming middleware discovery proves payload delivery.
- Test congestion and asymmetric producer-consumer rates.

## Assign Ownership And Authority

1. Assign exactly one authority for each transform edge, fused state, motion command, mode,
   and safety stop.
2. Inventory all writers, including test tools, launch-time bridges, watchdogs, and manual inputs.
3. Define arbitration priority, lease duration, takeover condition, and release condition.
4. Prevent stale authorities from resuming control after reconnect or process restart.
5. Keep source-of-truth ownership separate from cached or integration copies.
6. Version shared schemas and make incompatible changes fail visibly.
7. Route fixes to the component that owns the violated invariant.

Treat multiple plausible writers as a correctness defect until an explicit arbiter is proven.

## Model Validity And Degradation

Define a shared state machine such as `UNAVAILABLE`, `INITIALIZING`, `VALID`, `DEGRADED`,
`STALE`, and `LOST`. Specify entry, exit, hysteresis, timeout, and emitted data for every state.

- Propagate confidence and covariance without conflating them.
- Distinguish no detection, rejected detection, stale estimate, extrapolated estimate, and
  intentionally suppressed output.
- Define whether identity may change while geometry remains valid.
- Require hysteresis for target switches, mode switches, and health recovery.
- Define safe behavior for missing transforms, stale goals, localization loss, planner failure,
  controller timeout, and actuator rejection.
- Prevent the last valid command from persisting beyond its lease.
- Make degraded behavior observable through structured status and counters.

## Test The Integration

Advance from deterministic checks to closed-loop tests:

1. Validate schemas, enum defaults, units, ranges, and frame names statically.
2. Test producer serialization and consumer deserialization with golden contract fixtures.
3. Replay controlled samples with known transforms and timestamps.
4. Inject stale, future, reordered, duplicate, missing, saturated, and invalid values.
5. Inject transport loss, queue overload, process restart, and clock discontinuity.
6. Run a loopback test that observes the final behavior, not only message receipt.
7. Run simulation with motion, occlusion, relocalization, replanning, and command arbitration.
8. Compare runtime traces against the contract table and latency budget.
9. Verify recovery returns through the intended state transitions without unsafe command jumps.

Keep interface tests on both sides of a contract. Add a regression test at the first violated
invariant rather than only at the final symptom.

## Report The Evidence

Report the shortest complete causal chain. Include:

- the physical expectation and observed outcome;
- the first violated invariant and owning component;
- field-level contract rows and source locations;
- runtime measurements, commands, artifacts, and environment;
- confirmed facts, inferences, hypotheses, and unknowns;
- proposed fix, compatibility impact, and migration needs;
- regression coverage, unexecuted tests, residual risk, and confidence.

Do not claim integration success from compilation, discovery, or topic presence alone. Require
field-correct data, bounded timing, intended downstream behavior, and safe recovery.
