# Appendix A: Formal Statement and Proof of the Phase-Selection Reduction

This appendix makes precise a claim stated informally in `README.md` Sec 3.2, and corrects an
imprecision in an earlier draft of that section, which said the Fuzzy Anti-Spillback controller
"reduces to vanilla Max-Pressure" at low occupancy without specifying which Max-Pressure. This
repository implements two distinct pressure signals, and the reduction only holds against one of
them. Stating that precisely is the point of this appendix.

## A.0 Two pressure signals, not one

For a phase group `p` in `{NS, EW}` at decision epoch `t`, define:

- **Raw-count pressure** (used by `baselines/vanilla_max_pressure.py::mp_pressure`):

  ```
  q_p(t) = queue_up(p, t) - queue_down(p, t)
  ```

  where `queue_up`/`queue_down` are halting-vehicle counts on the upstream and downstream edge
  groups for phase `p`.

- **Discharge-time pressure** (used by `src/controller.py::phase_pressure`, following the
  queue-dissipation-time refinement stated in README Sec 3.1):

  ```
  w_p(t) = queue_up(p, t) / mu_up(p) - queue_down(p, t) / mu_down(p)
  ```

  where `mu_up(p)`, `mu_down(p)` are the saturation discharge rates (vehicles/second) of the
  upstream and downstream edge groups for phase `p`, determined by lane count.

`q_p` and `w_p` are related by `w_p = queue_up(p,t)/mu_up(p) - queue_down(p,t)/mu_down(p)`, which is
a **different linear functional** of `(queue_up, queue_down)` than `q_p` whenever `mu_up(p) != mu_down(p)`
for some phase `p`. In this repository's network (`configs/build_network.py`), every approach has
2 upstream lanes and 1 downstream (egress) lane, so `mu_up(p) = 2*mu_down(p)` for both `p = NS` and
`p = EW`. Consequently `argmax_p q_p(t)` and `argmax_p w_p(t)` are **not guaranteed to agree** in
general, and no claim in this repository asserts that they do.

## A.1 Proposition (phase-selection reduction, precisely stated)

**Proposition A.1.** Let `OCC(p, t)` denote the 50-second rolling-average downstream occupancy for
phase `p` at decision epoch `t` (README Sec 3.2), and let `OCC_OVERRIDE` be the hard-override
threshold. If

```
OCC(NS, t) < OCC_OVERRIDE   and   OCC(EW, t) < OCC_OVERRIDE
```

then the phase selected by the Fuzzy Anti-Spillback controller at decision epoch `t` equals

```
argmax_{p in {NS, EW}} w_p(t)
```

i.e., the controller's selection step at `t` is identical to greedy maximization of the
discharge-time pressure `w_p` alone — an unmodified, discharge-time-weighted Max-Pressure
policy — and is **not**, in general, identical to `argmax_p q_p(t)`, the selection rule of this
repository's `Vanilla Max-Pressure` baseline.

**Proof.** Inspect `src/controller.py::run`, the only site where phase selection occurs. At each
decision epoch the controller computes `w_cur = phase_pressure(phase)`,
`w_oth = phase_pressure(other)`, and sets `nxt = phase if w_cur >= w_oth else other` — a direct,
unconditional `argmax` over `w_p` with no dependence on occupancy at this line. The occupancy-gated
branch that follows only executes, and only has the power to overwrite `nxt`, when
`occ_nxt_s > occ_override` for the phase `nxt` just selected. Under the proposition's hypothesis,
`OCC(NS,t) < OCC_OVERRIDE` and `OCC(EW,t) < OCC_OVERRIDE`, so `occ_nxt_s <= OCC(NS,t)` or
`<= OCC(EW,t)` (whichever phase `nxt` is) is `< OCC_OVERRIDE`, the guard condition
`occ_nxt_s > occ_override` is false, the override branch does not execute, and `nxt` remains the
unconditional `argmax_p w_p(t)` computed above. The Mamdani FIS is invoked afterward only to compute
a green *duration* for the already-fixed `nxt`; it has no code path back to the selection variable
`nxt`. Hence the selection step, in isolation, is exactly `argmax_p w_p(t)`, establishing the claim.
The claim does not extend to `q_p` because `phase_pressure` is defined in terms of `w_p`
throughout `src/controller.py`; no code path in this repository computes `argmax_p q_p(t)` outside
`baselines/vanilla_max_pressure.py`. QED.

## A.2 What this does and does not establish

- **Does establish:** a clean, provable reduction of the Fuzzy controller's phase-*selection* rule
  to a well-defined, unmodified baseline policy (discharge-time-weighted Max-Pressure) in the
  low-occupancy regime, with the FIS's influence rigorously confined to green *duration*.
- **Does not establish:** bit-identical behavior to this repository's `Vanilla Max-Pressure`
  baseline, even at low occupancy, because that baseline uses `q_p` rather than `w_p`, and because
  the two policies use different green-duration rules in general (vanilla Max-Pressure uses a fixed
  5-second decision interval with no adaptive duration; Fuzzy always computes a duration via the
  FIS, even when the occupancy-gated override is inactive). The two controllers therefore remain
  distinct policies in the low-occupancy regime as well; Proposition A.1 only isolates and proves
  what changes (nothing) and what doesn't (green duration) about *phase selection specifically*
  when occupancy is comfortably below the override threshold.
- **Does not establish** closed-loop Lyapunov stability for the combined fuzzy+override system.
  Varaiya's (2013) stability argument for Max-Pressure is a drift condition on a quadratic Lyapunov
  function `L(t) = sum_p queue_p(t)^2`, proved for the raw-count pressure `q_p` under a fixed-rate
  service assumption. Proposition A.1 shows the *selection rule* used here is `argmax_p w_p`, not
  `argmax_p q_p`, so Varaiya's drift argument does not transfer without re-derivation for the
  discharge-time-weighted functional, and no such re-derivation is attempted in this repository.
  This is stated as open in README Sec 7 and is not resolved by this appendix.

## A.3 Why this correction matters

An earlier internal draft of README Sec 3.2 stated the reduction as "identical to vanilla
Max-Pressure," which is true only in the degenerate case `mu_up(p) = mu_down(p)` for all `p` — not
the case tested anywhere in this benchmark. Proposition A.1 replaces that claim with the one that is
actually provable from the code as written, and README Sec 3.2 has been updated to cite this
appendix rather than restate the imprecise version.
