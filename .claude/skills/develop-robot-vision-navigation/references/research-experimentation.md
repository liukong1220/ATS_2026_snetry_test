# Research And Experimentation

Use this guide before proposing novelty, selecting an algorithm, tuning broadly, comparing
systems, or claiming an improvement. Convert the engineering need into a falsifiable question,
preserve reproducibility, and separate measured evidence from interpretation.

## Contents

- [Frame The Research Question](#frame-the-research-question)
- [Collect Primary Evidence](#collect-primary-evidence)
- [State The Hypothesis](#state-the-hypothesis)
- [Choose Baselines](#choose-baselines)
- [Design Metrics And Datasets](#design-metrics-and-datasets)
- [Plan Controls And Ablations](#plan-controls-and-ablations)
- [Execute Reproducibly](#execute-reproducibly)
- [Analyze Results](#analyze-results)
- [Evaluate Novelty Honestly](#evaluate-novelty-honestly)
- [Report And Decide](#report-and-decide)

## Frame The Research Question

1. Translate the request into a measurable system outcome.
2. Identify the affected loop, operating regime, constraints, and failure mode.
3. Define the independent variable, dependent variables, and controlled variables.
4. State the target population: sensors, environments, motion, robots, targets, and hardware.
5. Separate algorithm quality from implementation quality and deployment quality.
6. Define what result would disprove the proposed direction.
7. Refuse goals expressed only as "better," "smarter," "stable," or "real time."

Use this research statement:

```text
Under <conditions>, changing <factor> is expected to improve <primary metric>
from <baseline> by <minimum effect>, without violating <constraints>.
Reject the hypothesis if <falsification rule>.
```

## Collect Primary Evidence

Search evidence in descending authority and directness:

1. Read original papers, standards, official specifications, and first-party documentation.
2. Inspect the exact source revision and configuration used by an implementation.
3. Inspect datasets, evaluation protocols, issue discussions, and correction notices.
4. Use surveys and independent reproductions to map the field and identify failure modes.
5. Use secondary summaries only for discovery, then verify consequential claims at source.

For every important claim, record:

| Claim | Source | Version/Date | Direct Evidence | Applicability | Limitation |
|---|---|---|---|---|---|
| stated claim | paper/spec/source | identifier | section/code/result | matching conditions | mismatch or unknown |

- Prefer peer-reviewed or officially maintained sources when authority matters.
- Prefer executable artifacts and raw results when reproducibility matters.
- Verify equations, coordinate conventions, dataset splits, and metric definitions directly.
- Check whether reported latency excludes preprocessing, transfer, queuing, or postprocessing.
- Check whether comparisons use equal sensors, compute, training data, and tuning effort.
- Record inaccessible sources and avoid inventing their contents.
- Distinguish publication date from the version actually evaluated.

## State The Hypothesis

Define one primary hypothesis per experiment. Make it directional, bounded, and falsifiable.

| Element | Required Statement |
|---|---|
| Mechanism | Explain why the change should affect the outcome. |
| Intervention | Name the exact algorithm, parameter group, or architecture change. |
| Conditions | Define speed, lighting, texture, dynamics, load, and sensor quality. |
| Primary metric | Select one metric that decides success. |
| Guardrails | Define safety, accuracy, latency, memory, and compute limits. |
| Minimum effect | Define the smallest practically meaningful improvement. |
| Rejection rule | Define results that reject or fail to support the hypothesis. |

Separate exploratory questions from confirmatory hypotheses. Label post-hoc explanations as
new hypotheses and validate them in a fresh run.

## Choose Baselines

1. Include the currently deployed or accepted system as the engineering baseline.
2. Include a simple, strong baseline that exposes whether complexity is justified.
3. Include the closest credible published or open implementation when comparable.
4. Reproduce each baseline under the same inputs, hardware, warm-up, budget, and metrics.
5. Tune baselines with a documented and comparable search budget.
6. Preserve baseline failures; do not silently exclude hard cases.
7. Verify baseline correctness with sanity cases before comparison.

Reject unfair comparisons that change multiple resources at once, including sensor quality,
training data, map prior, compute platform, precision, or latency budget.

## Design Metrics And Datasets

Choose metrics that connect component quality to system behavior.

| Domain | Candidate Primary Metrics | Required Tail Or Failure Metrics |
|---|---|---|
| Detection | precision, recall, mAP, calibration error | class/condition misses, false alarms |
| Tracking | association accuracy, track continuity, pose error | switches, loss, reacquisition time |
| Estimation | RMSE, drift, consistency | NIS/NEES, divergence, covariance failures |
| Localization/SLAM | ATE, RPE, map consistency | relocalization failures, degenerate segments |
| Planning | success, path cost, minimum clearance | no-path, oscillation, unsafe proximity |
| Control | tracking error, settling, energy | overshoot, saturation, deadline misses |
| Runtime | throughput, end-to-end latency | p95/p99, drops, queue growth, memory peaks |

1. Define metric equations, units, aggregation, alignment, and invalid-sample handling.
2. Report distributions, confidence intervals, and per-condition results rather than averages alone.
3. Split development, tuning, validation, and final test data before optimization.
4. Prevent scene, sequence, map, subject, or temporal leakage across splits.
5. Represent normal, boundary, rare, and adversarial operating conditions.
6. Preserve raw sensor data and ground-truth provenance.
7. Validate ground-truth accuracy against the expected improvement scale.
8. Report exclusions and missing data with reasons.

## Plan Controls And Ablations

Use controls to identify causality rather than correlation.

1. Change one causally coherent factor group per confirmatory experiment.
2. Hold data, seeds, compute, compiler flags, precision, and runtime budget constant.
3. Remove each new component independently to measure its contribution.
4. Test interactions with a small factorial design when components may depend on each other.
5. Include a no-op or placebo change when measurement bias is plausible.
6. Test sensitivity around selected thresholds and weights.
7. Test degraded inputs, overload, stale data, and recovery paths.
8. Repeat runs across seeds, sequences, and environmental conditions.

Prepare an experiment matrix before execution:

| Run | Hypothesis | Revision | Config | Data Split | Seed | Hardware | Expected Decision |
|---|---|---|---|---|---|---|---|
| identifier | H1 | immutable ID | config ID | test set | integer | platform ID | accept/reject |

## Execute Reproducibly

Capture enough information for a clean host to repeat the experiment:

- Pin source revisions, dependencies, models, dataset versions, and calibration files.
- Save the complete effective configuration, including defaults and environment overrides.
- Record hardware, firmware, operating system, drivers, accelerators, and power mode.
- Record build type, compiler, optimization flags, numerical precision, and thread settings.
- Record command lines, working directory assumptions, seeds, warm-up, and run duration.
- Synchronize clocks or document alignment uncertainty for multi-device measurements.
- Preserve raw outputs, structured metrics, logs, traces, and failure artifacts.
- Hash immutable inputs and generated results where practical.
- Automate repeated steps and make failures non-silent.
- Separate tuning runs from final evaluation runs.

Do not manually edit result files. Derive summaries from raw artifacts with versioned scripts.

## Analyze Results

1. Validate data completeness and metric computation before interpreting performance.
2. Plot or inspect distributions, sequences, outliers, and condition-level slices.
3. Report effect size and uncertainty, not only statistical significance.
4. Use paired comparisons when runs share the same sequences or initial conditions.
5. Correct for repeated testing when many hypotheses or configurations are evaluated.
6. Separate warm-up, steady state, overload, and recovery behavior.
7. Investigate regressions and failures before averaging them away.
8. Check whether gains come from changed latency, compute, data, or operating constraints.
9. Repeat surprising results from a clean state.

Classify each result as supporting, rejecting, inconclusive, or invalidating the hypothesis.
Avoid interpreting absence of significance as proof of equivalence without an equivalence design.

## Evaluate Novelty Honestly

1. Search terminology variants, adjacent fields, recent papers, patents when relevant, and
   original implementations before claiming novelty.
2. Separate a new scientific mechanism from a new engineering combination, implementation,
   dataset, deployment, optimization, or application.
3. State the closest prior work and the exact differentiator.
4. State whether novelty has been searched, demonstrated, or remains unknown.
5. Avoid words such as "first," "novel," and "state of the art" without exhaustive evidence.
6. Credit reused concepts, equations, code, data, and evaluation protocols.
7. Report negative or neutral results when they constrain future work.

Use calibrated wording: "not found in the searched sources" instead of "does not exist," and
"outperformed the evaluated baselines under these conditions" instead of universal claims.

## Report And Decide

Lead with the decision supported by the experiment. Include:

- the question, hypothesis, mechanism, and rejection rule;
- sources and their applicability limits;
- baseline, intervention, controls, ablations, and resource parity;
- datasets, splits, ground truth, metrics, and measurement uncertainty;
- revisions, effective configuration, hardware, commands, seeds, and artifacts;
- primary result, tails, regressions, failure cases, and confidence interval;
- supported facts, interpretations, alternative explanations, and unknowns;
- novelty classification and closest prior work;
- accept, reject, iterate, or stop decision with deployment implications.

Do not promote a candidate because it wins a single benchmark. Require practical effect,
guardrail compliance, repeatability, and evidence across the intended operating envelope.
