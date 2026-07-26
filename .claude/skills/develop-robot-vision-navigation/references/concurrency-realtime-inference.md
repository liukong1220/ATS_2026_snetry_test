# Concurrent and Real-Time Inference

Use this reference for camera-to-inference pipelines, asynchronous accelerators, worker pools,
bounded queues, callback executors, resource ownership, shutdown, and latency optimization.

## Contents

- Evidence-first workflow
- Timing and capacity model
- Ownership and synchronization
- Queues, ordering, and backpressure
- Inference runtime and accelerators
- Lifecycle and failure handling
- Decision workflow
- Metrics, benchmarking, and validation

## Evidence-first workflow

1. State the deadline, input rates, required freshness, acceptable loss, and ordering contract.
2. Locate producers, queues, workers, callbacks, model instances, device contexts, memory pools,
   postprocessors, publishers, shutdown paths, parameters, and tests.
3. Draw the runtime topology with threads, processes, devices, synchronization edges, and owners.
4. Trace one item using capture, enqueue, dequeue, inference-start, inference-end, publish, and
   consume timestamps from named clock domains.
5. Measure before changing thread counts, queue depths, batch sizes, or precision.
6. Confirm suspected races or stalls with traces, sanitizers, stress tests, or minimal reproducers.
7. Preserve functional accuracy while optimizing throughput or latency.
8. Change one causal factor per experiment and retain the configuration and trace.

## Timing and capacity model

Define the budget explicitly:

- Decompose end-to-end latency as
  `L_e2e = L_capture + L_queue + L_pre + L_infer + L_post + L_transport + L_consume`.
- Report distributions, not only means; include p50, p95, p99, maximum, and deadline-miss ratio.
- Separate service time from queueing time and data age from compute latency.
- Define throughput over a steady interval and report warmup separately.
- Use a monotonic clock for durations and preserve acquisition timestamps for freshness.
- Correlate traces with stable item IDs without changing processing order.

Check capacity before tuning:

- Estimate stage utilization as `rho = arrival_rate / service_rate` for a single-server stage.
- Expect unbounded queue growth when sustained `rho >= 1`.
- Use Little's Law `N = lambda * W` only for a stable system and clearly defined boundary.
- Include burst rate, jitter, batch formation time, device transfer, and downstream blocking.
- Reserve headroom for worst-case input and co-resident workloads.
- Set deadlines from control or decision needs, not from the average benchmark result.

Treat these as invariants:

- Bound every queue in a real-time path.
- Define ownership and lifetime for every buffer, request, future, stream, and model instance.
- Never publish an output as fresh when its input exceeded the age limit.
- Never access mutable shared state without a documented synchronization rule.
- Never block shutdown indefinitely on I/O, a full queue, or an accelerator request.
- Keep error paths subject to the same ownership and cleanup rules as success paths.

## Ownership and synchronization

Choose the simplest ownership model that meets the concurrency requirement:

- Prefer immutable messages or move-only ownership across stages.
- Use reference counting only when shared lifetime is intentional and bounded.
- Prevent producers from reusing image buffers while consumers or devices still read them.
- Document whether callbacks may run concurrently and which state each callback mutates.
- Protect compound invariants with one coherent lock or a single owning thread.
- Use atomics only for independent values with explicitly sufficient memory ordering.
- Keep lock scope small, but never split a required invariant merely to reduce contention.
- Establish one lock acquisition order and test error paths for inversions.
- Do not hold application locks across blocking I/O, device synchronization, or user callbacks.
- Avoid detached threads; retain a join or cancellation handle under explicit ownership.
- Verify third-party handles and model objects are thread-safe before sharing them.

Detect unsafe assumptions:

- Run thread or address sanitizers on supported CPU paths.
- Add generation numbers when resources can be reconfigured while requests are in flight.
- Validate double-buffer swaps and publication with happens-before reasoning.
- Treat callbacks from external runtimes as concurrent unless documentation guarantees otherwise.
- Copy or retain callback payloads when the provider owns memory after return.

## Queues, ordering, and backpressure

Select queue policy from product semantics:

- Use latest-only replacement when decisions need the freshest independent frame.
- Use bounded FIFO when every item must be processed and upstream can slow safely.
- Use drop-newest when preserving queued history matters more than new arrivals.
- Use keyed replacement when freshness is required independently per sensor or target.
- Use batching only when throughput gain exceeds batch-formation delay and deadline risk.
- Document whether dropping changes estimator, tracker, logger, or synchronizer correctness.

For each queue, define:

- Capacity in items and memory.
- Single- or multi-producer and single- or multi-consumer behavior.
- Full and empty behavior, timeout, wakeup, cancellation, and shutdown behavior.
- Ordering guarantee and whether completion may reorder requests.
- Drop counter, high-water mark, queue-age metric, and overload signal.

Handle temporal integrity explicitly:

- Carry capture sequence and timestamp through every stage.
- Reorder outputs only within a bounded window when downstream requires order.
- Drop late results before they mutate a newer state.
- Distinguish intentionally skipped frames from transport loss and processing failure.
- Reset temporal models or adjust `dt` after long gaps according to their contract.
- Synchronize multi-sensor inputs with stated tolerance and missing-input policy.

## Inference runtime and accelerators

Verify runtime behavior from authoritative documentation and a concurrency test:

- Determine whether one model instance supports concurrent calls.
- Determine whether execution contexts, streams, bindings, and scratch buffers are per-request.
- Allocate one isolated context per in-flight request when sharing is unsupported.
- Bind device selection explicitly and verify behavior in multi-device environments.
- Warm up kernels, graph compilation, memory pools, and caches before measurement.
- Separate host preprocessing, host-to-device copy, device execution, device-to-host copy, and
  synchronization in traces.
- Use pinned or unified memory only after measuring transfer and lifetime effects.
- Avoid a global device synchronization when request-scoped events are sufficient.
- Cap in-flight requests to memory and deadline budgets.
- Verify mixed precision or quantization against accuracy tolerances on representative data.
- Record runtime, driver, model, engine, precision, shape, and device versions.

Avoid deceptive concurrency:

- Do not assume asynchronous API submission means device execution overlaps.
- Do not increase worker count beyond serialized model, memory, or device bottlenecks blindly.
- Do not hide synchronization inside timing boundaries.
- Do not benchmark with data already cached if production includes transfer or decoding.
- Do not share mutable input or output bindings between overlapping requests.

## Lifecycle and failure handling

Design shutdown as a state machine:

1. Stop accepting new work.
2. Signal cancellation and wake every blocked producer and consumer.
3. Decide whether to drain or discard queued work according to the freshness contract.
4. Cancel or await in-flight requests with a bounded deadline.
5. Join workers before destroying queues, contexts, devices, or publishers.
6. Release resources in reverse ownership order and make repeated shutdown harmless.

Handle failures explicitly:

- Propagate model-load, allocation, timeout, device-loss, and malformed-input errors with context.
- Quarantine or recreate a corrupted execution context rather than reusing it silently.
- Apply bounded retries only to transient, idempotent operations.
- Use exponential backoff or circuit breaking when a dependency repeatedly fails.
- Emit a defined invalid, stale, degraded, or unavailable state to consumers.
- Keep watchdogs independent enough to detect executor or worker starvation.
- Test partial initialization and exceptions at every acquired resource boundary.

## Decision workflow

Use the following branches:

1. If latency rises over time, inspect queue depth, memory growth, thermal throttling, retries,
   and downstream blocking.
2. If throughput is low but stages are idle, inspect serialization, implicit synchronization,
   callback scheduling, and input starvation.
3. If throughput rises while control worsens, inspect queue age, batching delay, stale outputs,
   and completion reordering.
4. If crashes are nondeterministic, inspect buffer lifetime, shared bindings, shutdown races,
   reconfiguration, and provider callbacks before blaming the model.
5. If performance changes between debug and release, inspect races, timing dependence, allocator
   behavior, logging, compiler options, and undefined behavior.
6. If only overload fails, verify bounded policy, cancellation wakeups, degradation, and recovery.
7. If average latency passes but deadlines fail, optimize the tail and remove rare blocking paths.
8. If added workers do not help, identify the serialized resource and compute saturation first.

| Symptom | Candidate causes | Discriminating evidence |
| --- | --- | --- |
| Ever-growing delay | unbounded queue, overload, blocked consumer | queue age and high-water trace |
| Duplicate or stale result | buffer reuse, late completion, ID loss | item lineage and generation IDs |
| Shutdown hang | blocking I/O, full queue, missed wakeup | thread dump and cancellation trace |
| Sporadic corruption | shared bindings, lifetime race, unsafe runtime | sanitizer and isolated-context test |
| Periodic latency spike | allocation, logging, thermal, batch wait | stage trace plus system telemetry |
| No parallel speedup | serialized engine, transfer, lock, device limit | utilization and scoped concurrency test |

## Metrics, benchmarking, and validation

Instrument without obscuring the system:

- Report input, accepted, processed, dropped, failed, cancelled, stale, and published counts.
- Report per-stage service and queue latency at p50, p95, p99, and maximum.
- Report end-to-end data age, throughput, deadline misses, and recovery time after overload.
- Report queue high-water marks, in-flight count, memory high-water mark, and allocation rate.
- Report CPU, accelerator, memory bandwidth, transfer bandwidth, temperature, power, and clocks.
- Correlate accuracy metrics with precision mode, drop policy, batch size, and data age.

Make benchmarks reproducible:

- Fix input data, rate profile, duration, warmup, configuration, and random seeds.
- Record hardware topology, power mode, runtime versions, model hash, and co-resident load.
- Run long enough to expose thermal and allocator behavior.
- Include steady state, bursts, overload, dependency failure, and recovery phases.
- Compare configurations with identical functional outputs or report accuracy differences.
- Retain raw traces and calculate summary statistics from them.

Validate in increasing scope:

1. Unit-test queue policies, cancellation, ordering, late-result rejection, and lifecycle transitions.
2. Stress producers and consumers with randomized timing and forced capacity limits.
3. Run race, address, and undefined-behavior tooling where supported.
4. Inject allocation failure, malformed input, device timeout, callback error, and shutdown races.
5. Replay fixed data at below-capacity, near-capacity, burst, and sustained-overload rates.
6. Run on the deployment device with production power, cooling, and competing processes.
7. Integrate with downstream estimation and control; validate freshness and deadline behavior.

Require every optimization to state the measured bottleneck, expected causal effect, accuracy and
correctness constraints, acceptance threshold, tests run, tests omitted, rollback condition,
residual risk, and confidence level.
