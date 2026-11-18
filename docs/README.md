# Documentation

Seven documents. They overlap deliberately: the top-level
[README](../README.md) is the tour, each subsystem document is the derivation,
and the project report is what you read before explaining any of it to
somebody else.

| Document | What it is for |
|---|---|
| [project-report.md](project-report.md) | **Start here when explaining the project.** The one-page summary, the interview framing, and what to lead with — which is a failure, not the architecture |
| [architecture.md](architecture.md) | Package decomposition, frame conventions (REP-105), the topic contract and QoS policy |
| [state-estimation.md](state-estimation.md) | The six-state EKF: why those states, the Joseph-form update, chi-square gating, and where the filter is measurably overconfident |
| [path-planning.md](path-planning.md) | A\* and Dijkstra on the occupancy grid, the octile heuristic, costmap inflation |
| [control.md](control.md) | DWA and pure-pursuit/PID, the moving-obstacle work, and the diagnosis of the phantom tracks |
| [fault-injection.md](fault-injection.md) | Six scenarios, what each degrades, and what the estimator did about it |
| [benchmarks.md](benchmarks.md) | Five parameter sweeps and where each boundary lies, plus § 5b: the obstacle-prediction experiment and the three ways it measured nothing first |

## Learning notes

[`learning-notes/`](learning-notes/) holds one note per phase, in Turkish,
written while the phase was being built rather than afterwards. They are not
a translation of the documents above: these record what was *wrong* first and
why the fix is what it is. Thirteen notes, phases 00 through 12.

## What to read for a specific question

* *Does the estimator actually work?* → [state-estimation.md](state-estimation.md), then the Verification status table in the [README](../README.md#verification-status).
* *What happens when sensors degrade?* → [fault-injection.md](fault-injection.md).
* *Where are the limits?* → [benchmarks.md](benchmarks.md).
* *Why is obstacle prediction switched off?* → [benchmarks.md § 5b](benchmarks.md), then [control.md](control.md) for the cause.
* *How do the pieces fit together?* → [architecture.md](architecture.md).
