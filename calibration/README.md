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

Status: run and validated; see the result section at the end.

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

## Result (run 2)

Training on seeds 201-204 (both regimes; 320 evaluations, 15 generations) reached J = 0.9859, a 1.4% lower mean delay than the hand-set controller on those seeds.

On the held-out validation seeds 101-106 the calibrated controller reached a mean relative delay of 0.9971, with 110 pooled incidents against 98 for the hand-set controller (ratio 1.122, limit 1.100). The acceptance rule was not met, so the calibrated parameters were not adopted and the one-shot test on seeds 1-10 was not run.

By regime on the validation seeds: stress, mean relative delay 0.9870 (lower on 6 of 6 seeds), incidents 62 vs 54; moderate, mean relative delay 1.0072 (lower on 2 of 6 seeds), incidents 48 vs 44.

Reading: the delay response to the membership-function breakpoints is flat (relative delay within about 2% across random candidates), the gain found on the training seeds shrank from 1.4% to 0.3% on held-out seeds, and pooled incidents rose. Within this search budget (10 parameters, 320 evaluations), this objective and these seed sets, the hand-set breakpoints were not improved upon. This is not a proof of optimality: other objectives, regime-specific calibration, larger budgets and more seeds were not explored.
