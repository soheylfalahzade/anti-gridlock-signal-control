# Formal calibration of the fuzzy membership functions

The membership-function breakpoints of the controller were originally set by
hand. `de_calibrate.py` replaces that with a search (differential evolution)
under a protocol that separates fitting, selection and testing.

| Stage | Seeds | Purpose |
| --- | --- | --- |
| `train` | 201-204 | Fit the 10 parameters (9 breakpoints + override threshold) |
| `validate` | 101-103 | Held-out check: is the calibrated controller better than the hand-set one? |
| `final-test` | 1-10 | One-shot paired evaluation on the benchmark seeds (refuses to run twice) |

Objective, per candidate, averaged over (regime, seed): relative delay versus
the hand-set controller on the same seed, plus a one-sided penalty if box-gridlock
plus storage-overflow events exceed the hand-set count. The hand-set controller is
in the initial population, so the training objective cannot end worse than 1.0.
Candidates whose membership functions leave gaps in the input range are rejected
before simulation.

```
python calibration/de_calibrate.py train --workers 4
python calibration/de_calibrate.py validate
python calibration/de_calibrate.py final-test --confirm-frozen
```

`train` caches every finished simulation in `calibration/cache/` (git-ignored), so
an interrupted run resumes. Requires scipy >= 1.11. A full default run is about
8 simulations per candidate; budget accordingly, and reduce with `--seeds`,
`--maxiter` or `--regimes` for a trial.

Status: the pipeline is implemented and smoke-tested; the calibration itself has
not yet been run. Any outcome, including no improvement over the hand-set
controller, will be reported as found.
