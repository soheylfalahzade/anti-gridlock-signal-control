# Changelog

## [Critical Fix] Occupancy unit-scaling defect

**Found:** downstream-occupancy sensitivity sweep produced bit-identical
results across every tested value of `OCC_OVERRIDE` (0.65–0.85) and every
tested `critical_shift` (±0.10) — a statistical impossibility if the
parameter genuinely influenced controller decisions.

**Root cause:** `traci.edge.getLastStepOccupancy()` returns a fraction in
`[0, 1]` on SUMO 1.27.1 / this TraCI build, not a percentage in `[0, 100]`
as assumed. `src/controller.py::downstream_occupancy()` divided the
already-fractional value by 100 a second time, shrinking every real
occupancy reading (confirmed up to ~0.68 via direct diagnostic logging,
`diagnose_occupancy.py`) down to ~0.0068 — a value that could never cross
the FIS's "critical" membership function (starts at 0.62) or
`OCC_OVERRIDE` (0.75) under any tested configuration.

**Impact:** the fuzzy controller's entire occupancy-conditioned behavior —
both the hard anti-spillback override and the FIS's "medium"/"critical"
rule branches — was inert in every benchmark run prior to this fix. All
reported results (statistical_report.json, all figures) were regenerated
from scratch after the fix; no prior numbers are carried forward.

**Verification:** `diagnose_occupancy.py` (raw per-step edge telemetry)
confirmed the fraction-not-percentage behavior directly before the fix
was applied. Post-fix, `sensitivity_analysis.py`'s `OCC_OVERRIDE` sweep
range was also corrected (previous range 0.65–0.85 sat entirely above the
occupancy values actually observed at moderate-regime demand, 0–54%; the
corrected range 0.35–0.85 spans the observed distribution).

**Fix:** `src/controller.py::downstream_occupancy()`, single line,
removed the erroneous `/ 100.0`.

## Post-fix corrected findings (supersede all earlier reported numbers)

- Moderate regime (n=10 seeds): Fuzzy delay 241.9s, between Fixed-Time
  (234.8s) and Vanilla MP (261.0s); all three pairwise comparisons
  significant after Holm–Bonferroni correction (p=0.033).
- Stress regime (n=10 seeds): Fuzzy delay 342.6s vs. Vanilla MP 380.3s —
  a materially larger gap than observed pre-fix (was 375.4s vs. 376.8s,
  statistically indistinguishable). This is the expected result: the
  anti-spillback mechanism should matter most exactly where occupancy is
  chronically high.
- Preemption ablation (n=10 seeds): now statistically significant,
  p=0.049, Cohen's d_z=-0.74, 143.3s → 125.0s (~13% reduction).
- Bottleneck-severity sweep: reveals a real, previously invisible cost —
  at loose bottlenecks (duty ≥ 0.56), Fuzzy underperforms both baselines
  on delay and throughput. The anti-spillback mechanism's benefit is
  concentrated in the tight-to-moderate severity range; at low severity
  it throttles green unnecessarily. Reported honestly, not hidden.
- Sensitivity analysis (`OCC_OVERRIDE`): reveals a non-monotonic
  relationship — the most aggressive threshold tested (0.35) triggers
  overrides far more often (83.7/run) but produces *more* storage
  overflow (18.0 events) than a moderately aggressive threshold (0.45:
  37.3 overrides, 0.33 events), suggesting overly frequent forced
  rotation destabilizes the movement it does not currently protect.
