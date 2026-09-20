# anti-gridlock-signal-control

Micro-control layer for a single signalized intersection under a
spillback-inducing bottleneck geometry, developed as the local-control
substrate for a larger real-time emergency-corridor navigation stack
(time-varying fault-tolerant geometric spanners + dynamic green wave).
A corridor-level green wave is only meaningful if every intersection along
it resists junction-box lock-up; this repository isolates and benchmarks
that local guarantee before any network-level routing is introduced.

## 1. Problem Statement

Standard Max-Pressure signal control (Varaiya, 2013) is provably
throughput-optimal under the assumption that receiving links have
effectively unbounded storage. Real urban geometries violate this
assumption at short downstream links, where a discharging phase can push a
queue that the receiving link cannot absorb, causing it to back up into the
junction box itself (spillback) and gridlock the cross street. This project
quantifies that failure mode and evaluates a fuzzy-throttled variant
designed to avoid it.

## 2. System Model

- Four symmetric approaches (N/S/E/W) into junction `C`.
- `app_D` (200 m, 2 lanes) -> `in_D` (50 m, 2 lanes) -> `C` -> `out_T`
  (50 m, 1 lane, metered) -> `exit_T` (200 m, 1 lane).
- Metering signals at each `M_D` node cap egress discharge (~24/3/23 s
  green/yellow/red), physically inducing spillback risk on the 50 m egress
  links under arterial surge demand.
- Base demand: N-S arterial 1500 veh/h (with stochastic surge windows),
  E-W cross street 400 veh/h. Demand is parametrized by a `scale` factor
  for the V/C sweep described in Section 4.

## 3. Control Architecture

**Pressure signal.** `w_p = t_D^p - t_D^q`, a queue-dissipation-time
differential estimated from measured halting queues at saturation flow
(1900 veh/h/lane), rather than raw vehicle counts.

**Fuzzy throttle.** A Mamdani FIS (`skfuzzy`, centroid defuzzification) maps
`(w_p, downstream_occupancy)` to a green duration in `[10s, 60s]`. Critically,
the FIS only ever shortens green relative to unconstrained Max-Pressure; it
never re-weights the phase-selection pressure itself. This means that
whenever no receiving link is near saturation, the controller is
*identical* to vanilla Max-Pressure and inherits its stability argument
unmodified (see `src/controller.py` docstring for the formal statement).

**Hard anti-spillback override.** When the selected phase's receiving link
exceeds 75% occupancy, the controller either rotates to the alternate
phase (if its receiving link has headroom) or holds at `T_MIN`. This is
framed as a control-barrier-style safety layer on `h(x) = 0.75 - occ(x)`,
guaranteeing the controller never *selects* an already-saturated
destination — a deadlock-avoidance property, not a full closed-loop
Lyapunov stability guarantee (see Limitations).

**Emergency preemption (preliminary).** Any `vClass="emergency"` vehicle
triggers an immediate, safety-respecting transition toward its approach,
held until it clears the stop-line section or a 45 s cap is reached. This
is the first integration point toward the corridor-level green wave and is
evaluated as an ablation, not presented as a complete preemption system.

## 4. Experimental Design

Three policies are compared: Webster fixed-time, vanilla Max-Pressure
(full strength, no spillback awareness), and Fuzzy Anti-Spillback
Max-Pressure.

- **Paired multi-seed benchmark** (`evaluate.py`, default 5 seeds): for
  each seed, one demand realization is generated once and shared across
  all three policies, giving a valid paired comparison. Wilcoxon
  signed-rank tests are reported for delay, queue, throughput and
  spillback counts.
- **Demand sweep** across V/C scale `{0.6, 0.8, 1.0, 1.2, 1.5, 1.8}` to
  characterize *where* each policy wins, resolving the apparent paradox
  that Webster fixed-time can outperform total delay at moderate demand
  while still starving the cross street (see Section 5).
- **Jain's Fairness Index** over per-approach mean delay, `J = (Σx)² / (nΣx²)`,
  to separate "low total delay" from "fair distribution of delay."
- **Emergency-vehicle delay** extracted from `tripinfo.xml` for the
  injected ambulance under each policy.

## 5. Known Trade-off (documented, not hidden)

At baseline demand (scale = 1.0), Webster fixed-time achieves lower total
delay than Fuzzy Anti-Spillback in the runs collected here, despite the
fuzzy controller reducing cross-street queue substantially. This is a
genuine **price-of-fairness** effect: Webster's fixed split already gives
the cross street a guaranteed minimum green every cycle, so at this demand
level it does not need dynamic starvation avoidance. The fuzzy controller's
advantage should be expected to concentrate at higher V/C ratios, where
vanilla Max-Pressure would otherwise spillback and Webster's fixed
allocation becomes inefficient. The demand sweep in Section 4 is the
mechanism for validating (or falsifying) this claim quantitatively — see
`results/demand_sweep.png` and `results/statistical_report.json` for the
current run's evidence.

## 6. Limitations

- No closed-loop Lyapunov stability proof for the combined
  fuzzy+override system outside the regime where it reduces to vanilla
  Max-Pressure; this is flagged as open, not claimed as solved.
- Membership function breakpoints are hand-set, not calibrated by a formal
  optimization procedure; a sensitivity/calibration study is future work.
- Emergency preemption is a single-vehicle, single-approach ablation, not
  a validated multi-intersection preemption protocol.
- All results are single-intersection; no claims are made about
  network-level corridor behavior until integrated with the spanner/green
  wave layer.

## 7. Reproducibility
conda activate traffic_research_env
cd ~/anti-gridlock-signal-control
python configs/build_network.py
python evaluate.py --seeds 1 2 3 4 5 --scales 0.6 0.8 1.0 1.2 1.5 1.8

Outputs: `results/comparison_metrics.png`, `results/demand_sweep.png`,
`results/fairness_emergency.png`, `results/statistical_report.json`.

## 8. Roadmap

Phase 2 integrates this controller as the local layer beneath a
network-wide dynamic green wave driven by time-varying, fault-tolerant
geometric spanners for emergency-vehicle routing (separate repository,
not yet linked here pending Phase 1 validation at higher demand regimes).
