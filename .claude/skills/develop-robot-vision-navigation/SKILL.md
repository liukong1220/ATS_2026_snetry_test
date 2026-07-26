---
name: develop-robot-vision-navigation
description: Develop, analyze, review, debug, optimize, and validate general robot vision and navigation systems as equal first-class engineering domains. Use for repository-independent work involving visual detection and recognition, target tracking, pose estimation, calibration, sensor fusion, whole-target or vehicle state estimation, concurrent inference, SLAM, mapping, localization, relocalization, behavior planning, path planning, trajectory optimization, MPPI or MPC, vehicle control, visual-navigation fusion, ROS or non-ROS robotics architectures, real-time performance, simulation, recorded-data analysis, code review, algorithm research, and staged real-robot deployment.
---

# Develop Robot Vision And Navigation

Treat vision and navigation as equal first-class domains. Do not assume that either
domain belongs to the other, and do not assume a repository name, directory layout,
robot type, middleware, sensor suite, algorithm, or compute platform.

## Classify The Request

Classify the user's intent before acting:

- Explain, inspect, review, or diagnose: gather evidence and report; do not edit.
- Implement, fix, optimize, or build: make the smallest justified change and verify it.
- Research or propose a new direction: form a falsifiable hypothesis and experiment plan.
- Monitor or run an experiment: preserve raw observations and report stopping conditions.

Classify the technical scope independently:

- Vision: perception, calibration, tracking, estimation, inference, ballistics, or gimbal control.
- Navigation: SLAM, localization, maps, planning, trajectories, behaviors, or vehicle control.
- Integration: contracts that cross perception, estimation, planning, control, or repositories.
- Whole system: trace the complete sensor-to-actuator feedback loop.

Do not force an ambiguous failure into an algorithm category. Timing, frames, units,
configuration ownership, transport, hardware, and deployment are separate root-cause classes.

## Discover The Workspace

Read repository instructions before task files. Inspect version-control state, manifests,
build files, test configuration, entry points, launch/deployment files, public interfaces,
and a shallow tree. Run `scripts/discover_robot_workspace.py` when the project is unfamiliar
or spans multiple repositories:

```bash
python3 <skill-dir>/scripts/discover_robot_workspace.py --root <workspace> --format markdown
```

Identify source repositories, nested repositories, submodules, copied code, generated
code, vendored code, active build/install trees, and uncommitted changes. Treat all unknown
changes as user-owned. Determine which source and configuration are used at runtime instead
of assuming the nearest file or README is authoritative.

## Apply Evidence-First Analysis

Read [evidence-first-analysis.md](references/evidence-first-analysis.md) for every substantial
inspection, review, diagnosis, architecture claim, or research conclusion.

Build a relevance map before broad reading:

1. Read Tier 1: named or changed files, runtime entry points, public interfaces, failures,
   stack locations, and directly relevant tests.
2. Expand to Tier 2 only to resolve a question: callers, callees, parameters, schemas,
   transforms, launch files, focused tests, and specifications.
3. Avoid Tier 3 by default: generated artifacts, dependencies, caches, binaries, media,
   weights, large datasets, logs, and unrelated modules.

Label conclusions as confirmed fact, inference, hypothesis, or unknown. Corroborate important
claims with two independent sources when practical. Never present static inspection as runtime
verification, declared configuration as effective configuration, or correlation as causation.

## Trace The Closed Loop

For system failures, trace only the shortest relevant loop:

```text
sensor -> acquisition -> synchronization -> perception -> estimation
       -> decision -> planning -> trajectory -> control -> actuator -> feedback
```

At every crossed boundary, record:

- producer and consumer ownership;
- type/schema and field semantics;
- units, axes, sign, handedness, and coordinate frame;
- sample/observation time, clock domain, freshness, and ordering;
- transport/QoS, frequency, queue bound, drop policy, and timeout;
- validity, confidence, covariance, saturation, and fallback behavior.

Use `scripts/check_robot_interface_contracts.py` to collect contract candidates and evidence
locations; treat its source scan as an inventory, not proof:

```bash
python3 <skill-dir>/scripts/check_robot_interface_contracts.py --root <workspace> --format markdown
```

Locate the first violated invariant and the layer that owns it. Do not hide upstream contract
failures by tuning downstream thresholds.

## Route Domain Guidance

Load only the references needed for the current task:

- Read [vision-perception-tracking.md](references/vision-perception-tracking.md) for detection,
  keypoints, pose, association, target switching, occlusion, and tracking.
- Read [vision-estimation-calibration.md](references/vision-estimation-calibration.md) for camera
  models, hand-eye calibration, filters, state models, observability, NIS, and NEES.
- Read [concurrency-realtime-inference.md](references/concurrency-realtime-inference.md) for
  multi-camera pipelines, async inference, bounded queues, backpressure, latency, and shutdown.
- Read [slam-localization-mapping.md](references/slam-localization-mapping.md) for VIO/LIO/SLAM,
  deskew, map frames, degeneracy, relocalization, and map quality.
- Read [planning-trajectory-control.md](references/planning-trajectory-control.md) for behaviors,
  costmaps/ESDF, global and local planning, trajectories, MPPI/MPC, and actuator constraints.
- Read [vision-navigation-integration.md](references/vision-navigation-integration.md) whenever
  data or behavior crosses domain, process, middleware, or repository boundaries.
- Read [research-experimentation.md](references/research-experimentation.md) before proposing
  novelty, comparing algorithms, tuning broadly, or designing an experiment.
- Read [validation-real-robot-safety.md](references/validation-real-robot-safety.md) before HIL,
  hardware, field, high-speed, autonomous, or irreversible tests.

## Establish Models And Metrics

Write down state, inputs, observations, dynamics, measurement model, objective, constraints,
frames, clocks, noise assumptions, observability/controllability, compute complexity, and
real-time budget before changing a mathematical component.

Establish a reproducible baseline before optimization. Prefer metrics that expose distributions
and failure tails over averages. Analyze timing CSV data with:

```bash
python3 <skill-dir>/scripts/analyze_timing_metrics.py <data.csv> --column latency_ms --deadline 20
```

Compare baseline and candidate experiment summaries with:

```bash
python3 <skill-dir>/scripts/compare_experiments.py baseline.json candidate.json
```

Change one causally related factor group per experiment. Preserve input data, configuration,
software revision, hardware, environment, warm-up, run duration, random seeds, and raw results.

## Implement Conservatively

Find the component that owns the behavior. Prefer existing abstractions and proven domain
libraries. Preserve public contracts unless the task explicitly changes them. Add or tighten
a focused test that would fail before the fix when practical. Then implement the smallest
coherent change.

For concurrency, make ownership, cancellation, queue bounds, overload behavior, and shutdown
explicit. For controls, retain a deterministic zero/hold/safe fallback. For estimation, reject
invalid numerical states and expose consistency diagnostics. For cross-domain changes, test
both producer and consumer semantics.

Do not weaken assertions, delete safeguards, broaden tolerances without evidence, or suppress
errors merely to make tests pass.

## Validate In Increasing Risk Order

Run the narrowest useful check first, then expand:

1. Static/schema/format and mathematical property checks.
2. Unit and component tests.
3. Recorded data, replay, or deterministic offline evaluation.
4. Integration and closed-loop simulation.
5. Hardware-in-the-loop with actuation constrained or disabled.
6. Low-energy, low-speed real-robot trials.
7. Representative and boundary-condition operation.

Advance only when the previous gate meets explicit acceptance criteria. Record commands,
artifacts, metrics, environment, failures, and tests not run. Define stop and rollback criteria
before hardware motion.

## Report With Decision-Useful Evidence

Answer in the user's language. Lead with the outcome. Include only sections that add value:
conclusion, key evidence, prioritized problems and impact, options and tradeoffs, changes,
verification, assumptions, limitations/unknowns, confidence, and next action.

For review findings, order correctness and invariants first, followed by safety/data loss,
concurrency/real-time behavior, compatibility/integration, recovery/observability, tests,
performance, and maintainability. Cite source locations and distinguish measured results from
expected behavior.
