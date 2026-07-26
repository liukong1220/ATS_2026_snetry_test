# Evidence-First Analysis

Use this protocol for code reading, review, diagnosis, architecture analysis, technical
research, and any factual claim that may drive an engineering decision.

## 1. Define The Question

- Restate the concrete decision or failure being investigated.
- Define the minimum evidence that could confirm or refute it.
- Separate the requested scope from adjacent interesting work.
- State the definition of done and the validation boundary before editing.

## 2. Triage Sources

Build a relevance map before opening large files.

| Tier | Read first | Typical evidence |
|---|---|---|
| 1 | Direct behavior | Named/changed files, entry points, public interfaces, failures, focused tests |
| 2 | Resolve dependencies | Callers, callees, configs, schemas, transforms, launch files, specifications |
| 3 | Avoid by default | Generated output, dependencies, caches, binaries, logs, media, datasets |

Respect repository instructions and ignore rules. Treat generated files as outputs; find their
generator or schema. Prefer symbol and reference searches to full-file or full-tree reading.

## 3. Build A Claim Ledger

Record each consequential statement with one of these labels:

- **Confirmed fact**: directly supported by authoritative source or reproducible output.
- **Inference**: follows from facts but has not been directly exercised.
- **Hypothesis**: plausible explanation that needs a discriminating test.
- **Unknown**: evidence is missing, inaccessible, stale, or contradictory.

For each claim, retain:

```text
claim | label | evidence A | evidence B | counterevidence | scope | validation needed
```

Corroborate important claims with independent evidence when practical. Useful pairs include
implementation plus test, interface plus publisher/consumer, specification plus implementation,
or runtime trace plus source. Do not count copies, generated mirrors, or documents derived from
the same source as independent.

When only one authoritative source exists, state `single-source limitation` and explain what
remains unverified. When evidence conflicts, compare authority, directness, recency, runtime
relevance, and reproducibility; report the conflict rather than selecting silently.

## 4. Trace Behavior Minimally

Trace only enough boundaries to answer the question:

1. Input creation and timestamp.
2. Validation and normalization.
3. State transition or algorithm update.
4. Side effect or output publication.
5. Consumer interpretation.
6. Error, timeout, fallback, and recovery path.

Check declared configuration against the effective startup path. Check a message definition
against both assignment and consumption. Check a mathematical formula against units, dimensions,
tests, and numerical behavior. Stop expanding when another file is unlikely to change the
decision; record residual uncertainty instead.

## 5. Challenge Premises

- Identify assumptions embedded in the request, documentation, variable names, and comments.
- Look for contrary runtime paths, duplicate owners, stale configs, unit conversions, and clocks.
- Distinguish correlation from causation and symptom suppression from root-cause repair.
- Prefer the smallest explanation consistent with all observed evidence, while retaining viable
  alternatives until a test separates them.

## 6. Validate Claims

Select the narrowest check that can change confidence:

- schema/parser/build check for structural claims;
- focused unit/property test for local behavior;
- deterministic reproduction for a bug;
- replay or trace for timing/data-flow claims;
- simulation for closed-loop behavior;
- hardware only when lower-risk checks cannot answer the question.

Never claim runtime verification after static inspection. Report commands not run, missing
hardware/data, environmental assumptions, and unrelated failures separately.

## 7. Review By Risk

Prioritize findings in this order unless the task changes the risk model:

1. Correctness and violated invariants.
2. Safety, security, data loss, and irreversible side effects.
3. Concurrency, deadlines, resource exhaustion, and shutdown.
4. Public contract and integration compatibility.
5. Error handling, observability, fallback, and recovery.
6. Missing tests for changed behavior.
7. Demonstrated hot-path performance.
8. Maintainability that materially affects future correctness.

Avoid style-only findings that automated tools can settle.

## 8. Report Concisely

Lead with the decision. Map each key conclusion to evidence. State applicability, assumptions,
unknowns, confidence, and the next experiment that most reduces risk. Do not expose private
chain-of-thought; provide the evidence and concise engineering rationale needed to audit the
decision.
