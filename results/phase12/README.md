# Phase 12 — does predicting obstacle motion help?

Two sweeps of the same paired experiment, six crossing phases each, both
arms run at every phase so they face the same encounter.

| directory | tracker | note |
|---|---|---|
| this one | as originally built | **see the caveat below** |
| `with_aperture_rule/` | plus the aperture rule | the current code |

Read [docs/benchmarks.md § 5b](../../docs/benchmarks.md) for what the
numbers mean. The short version: the aperture rule cut the mean number of
reported movers from 2.37 to 1.66 in a world containing exactly one, and
prediction was not ahead in aggregate in either sweep, so it now ships off.

## Caveat on this directory

`ros2 launch` starts a ruby wrapper which starts `gz sim`. Teardown signals
the wrapper; the wrapper dies and logs that it died; the actual simulator is
a grandchild and can survive. One such orphan was found by hand shortly
after this sweep finished.

Every run's log here shows its own simulator shutting down normally, and the
twelve logs are uniform, which argues the orphan came from the last run's
teardown. **That is an argument, not a proof** — an orphaned process writes
nothing to either log, so a run sharing a transport with one looks exactly
like a run that merely behaved oddly.

So the numbers here are reported with that doubt attached rather than
deleted. `with_aperture_rule/` was run after the harness learned to refuse
to start beside a stray process, and carries no such doubt.

The comparison between the two directories is still a fair one for the
aperture rule's effect in the direction that matters: contamination would
make *this* sweep look worse, so it cannot manufacture the improvement, only
exaggerate it.
