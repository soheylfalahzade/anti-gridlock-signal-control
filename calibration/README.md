# Formal calibration of the fuzzy membership functions

The membership-function breakpoints of the controller were originally set by
hand. `de_calibrate.py` replaces that with a search (differential evolution)
under a protocol that separates fitting, selection and testing.

| Stage | Seeds | Purpose |
| --- | --- | --- |
| `train` | 201-204 | Fit the 10 parameters (9 breakpoints + override threshold) |
| `validate` | 101-106 | Held-out check: is the calibrated controller better than the hand-set one? |
| `final-test` | 1-10 | One-shot paired evaluation on the benchmark seeds (refuses to run twice) |

Objective, per candidate, over all (regime, seed) pairs: mean relative delay versus
the hand-set controller on the same seed (default weight on incidents: 0). Incidents
(box-gridlock plus storage-overflow events) are checked at validation: pooled incidents
must not exceed 110% of the hand-set controller's. The hand-set controller is in the
initial population, so the training objective cannot end worse than 1.0.
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

## Revision of the objective (run 1 -> run 2)

Run 1 used a one-sided penalty on per-seed incident increases (weight 0.5). It
finished with best J = 1.0000 in all 15 generations. `inspect_population.py`
(reads the cache, no new simulations) showed why: across the 20 initial members,
relative delay lay between 0.991 and 1.017, while pooled incident counts varied
between 42 and 82 against 50 for the hand-set controller. A member with 16% fewer
pooled incidents and a slightly lower delay still scored J = 1.07, because the
penalty counted increases on some seeds and gave no credit for decreases on others.
The penalty therefore tracked count noise rather than controller quality.
Run 1 outputs are kept in `results/run1_per_seed_penalty/`. Run 2 uses the delay-only
objective above. The change was made after seeing run 1, but only on the training
seeds; validation (101-106) and test (1-10) seeds had not been touched.
