#!/usr/bin/env python3
"""Formal calibration of the fuzzy controller's membership functions.

Replaces hand-set breakpoints with a search (scipy differential evolution)
under an explicit, pre-declared protocol that separates the data used to
fit, to select, and to test:

  train       fit on calibration seeds (default 201-204), both regimes
  validate    compare the calibrated and hand-set controllers on held-out
              validation seeds (default 101-103); decides whether to proceed
  final-test  one-shot paired evaluation on the benchmark seeds (1-10);
              refuses to run twice, so the test seeds cannot be tuned on

Objective (minimise), per candidate, averaged over (regime, seed) pairs:

    J = mean( delay / delay_handset )
        + w * mean( max(0, incidents - incidents_handset) / max(incidents_handset, 1) )

where delay_handset / incidents_handset come from the hand-set controller on
the SAME seed and regime, and incidents = box-gridlock events + storage-
overflow events. The second term is one-sided: it never rewards extra
incidents avoided, it only forbids buying lower delay with more incidents.
The hand-set controller is part of the initial population, so on the
training seeds the result cannot be worse than J = 1.

Run from the repository root:
  python calibration/de_calibrate.py train
  python calibration/de_calibrate.py validate
  python calibration/de_calibrate.py final-test --confirm-frozen
"""
import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import time
import uuid

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.chdir(ROOT)

from src.calibration_space import BOUNDS, NAMES, default_vector, vector_to_params  # noqa: E402
from src.fuzzy_engine import coverage_ok  # noqa: E402

OUT_DIR = os.path.join("calibration", "results")
CACHE_DIR = os.path.join("calibration", "cache")
PENALTY_INFEASIBLE = 10.0


# --------------------------------------------------------------------------
# Simulation access
# --------------------------------------------------------------------------
def ambulance_depart(regime):
    from src.demand import AMBULANCE_DEPART_LIGHT, AMBULANCE_DEPART_STRESS
    return AMBULANCE_DEPART_STRESS if regime == "stress" else AMBULANCE_DEPART_LIGHT


def build_configs(seeds, regimes):
    """Write demand/config files once, in the main process, so parallel
    workers only ever read them."""
    from src.demand import make_config
    cfgs = {}
    for regime in regimes:
        for seed in seeds:
            tag = f"cal_{regime}_seed{seed}"
            cfg, _ = make_config(tag, scale=1.0, seed=seed, ambulance=True,
                                 ambulance_depart=ambulance_depart(regime))
            cfgs[f"{regime}:{seed}"] = cfg
    return cfgs


def simulate(cfg, seed, regime, fis_params, occ_override, sim_end):
    from src import controller
    tmp = tempfile.gettempdir()
    uid = uuid.uuid4().hex[:10]
    res = controller.run(sumocfg=cfg, seed=seed, regime=regime, verbose=False,
                         sim_end=sim_end, occ_override=occ_override,
                         fis_params=fis_params,
                         tripinfo_out=os.path.join(tmp, f"cal_{uid}_trip.xml"),
                         metrics_out=os.path.join(tmp, f"cal_{uid}_met.json"))
    for suffix in ("_trip.xml", "_met.json"):
        try:
            os.remove(os.path.join(tmp, f"cal_{uid}{suffix}"))
        except OSError:
            pass
    return {"delay": float(res["avg_delay_s"]),
            "incidents": int(res["box_gridlock_events"] + res["storage_overflow_events"]),
            "box": int(res["box_gridlock_events"]),
            "overflow": int(res["storage_overflow_events"]),
            "throughput": int(res["throughput_completed_trips"])}


def cached_simulate(cfg, seed, regime, fis_params, occ_override, sim_end):
    """Disk cache keyed on every input, so an interrupted run resumes
    without repeating finished simulations."""
    key = json.dumps([cfg, seed, regime, sim_end, round(occ_override, 6),
                      {k: round(v, 6) for k, v in sorted(fis_params.items())}])
    h = hashlib.sha1(key.encode()).hexdigest()
    path = os.path.join(CACHE_DIR, h + ".json")
    if os.path.exists(path):
        with open(path) as fh:
            return json.load(fh)
    out = simulate(cfg, seed, regime, fis_params, occ_override, sim_end)
    os.makedirs(CACHE_DIR, exist_ok=True)
    tmp = path + f".{os.getpid()}.tmp"
    with open(tmp, "w") as fh:
        json.dump(out, fh)
    os.replace(tmp, path)
    return out


def reference_runs(cfgs, sim_end):
    d_params, d_occ = vector_to_params(default_vector())
    ref = {}
    for k, cfg in cfgs.items():
        regime, seed = k.split(":")
        ref[k] = cached_simulate(cfg, int(seed), regime, d_params, d_occ, sim_end)
    return ref


# --------------------------------------------------------------------------
# Objective
# --------------------------------------------------------------------------
def objective(x, cfgs, ref, sim_end, penalty_weight):
    fis_params, occ_override = vector_to_params(x)
    if not coverage_ok(fis_params):
        return PENALTY_INFEASIBLE
    rel, extra = [], []
    for k, cfg in cfgs.items():
        regime, seed = k.split(":")
        try:
            r = cached_simulate(cfg, int(seed), regime, fis_params, occ_override, sim_end)
        except Exception:
            return PENALTY_INFEASIBLE
        base = ref[k]
        rel.append(r["delay"] / base["delay"])
        extra.append(max(0, r["incidents"] - base["incidents"]) / max(base["incidents"], 1))
    return float(np.mean(rel) + penalty_weight * np.mean(extra))


def git_commit():
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except Exception:
        return "unknown"


# --------------------------------------------------------------------------
# Subcommands
# --------------------------------------------------------------------------
def cmd_train(a):
    import scipy
    from scipy.optimize import differential_evolution
    os.makedirs(OUT_DIR, exist_ok=True)
    cfgs = build_configs(a.seeds, a.regimes)
    print(f"[train] {len(cfgs)} (regime, seed) pairs, sim_end={a.sim_end}s, scipy {scipy.__version__}")
    ref = reference_runs(cfgs, a.sim_end)
    print("[train] hand-set reference:",
          {k: (round(v['delay'], 1), v['incidents']) for k, v in ref.items()})

    hist_path = os.path.join(OUT_DIR, "de_history.csv")
    state = {"gen": 0, "t0": time.time()}
    with open(hist_path, "w", newline="") as fh:
        csv.writer(fh).writerow(["generation", "best_J", "convergence", "elapsed_s"] + NAMES)

    def callback(intermediate_result):
        state["gen"] += 1
        with open(hist_path, "a", newline="") as fh:
            csv.writer(fh).writerow(
                [state["gen"], f"{intermediate_result.fun:.6f}",
                 f"{intermediate_result.convergence:.4f}", f"{time.time() - state['t0']:.0f}"]
                + [f"{v:.5f}" for v in intermediate_result.x])
        print(f"[train] generation {state['gen']}: best J = {intermediate_result.fun:.4f}")

    result = differential_evolution(
        objective, BOUNDS, args=(cfgs, ref, a.sim_end, a.penalty),
        x0=default_vector(), strategy="best1bin", popsize=a.popsize, maxiter=a.maxiter,
        mutation=(0.5, 1.0), recombination=0.7, tol=0.0, atol=0.0, seed=a.de_seed,
        init="latinhypercube", polish=False, updating="deferred",
        workers=a.workers, callback=callback)

    fis_params, occ_override = vector_to_params(result.x)
    out = {"fis_params": fis_params, "occ_override": occ_override,
           "train_J": float(result.fun), "handset_train_J": 1.0,
           "evaluations": int(result.nfev), "generations": state["gen"],
           "protocol": {"train_seeds": a.seeds, "regimes": a.regimes, "sim_end": a.sim_end,
                        "popsize": a.popsize, "maxiter": a.maxiter, "de_seed": a.de_seed,
                        "incident_penalty_weight": a.penalty},
           "git_commit": git_commit(), "scipy": scipy.__version__,
           "created": time.strftime("%Y-%m-%d %H:%M:%S")}
    with open(os.path.join(OUT_DIR, "best_params.json"), "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"[train] done. best J = {result.fun:.4f} (hand-set = 1.0000), {result.nfev} evaluations")
    print(f"[train] wrote {OUT_DIR}/best_params.json")


def load_params(path):
    with open(path) as fh:
        d = json.load(fh)
    return d["fis_params"], d["occ_override"], d


def paired_runs(cfgs, fis_params, occ_override, sim_end):
    d_params, d_occ = vector_to_params(default_vector())
    rows = []
    for k, cfg in cfgs.items():
        regime, seed = k.split(":")
        h = cached_simulate(cfg, int(seed), regime, d_params, d_occ, sim_end)
        c = cached_simulate(cfg, int(seed), regime, fis_params, occ_override, sim_end)
        rows.append((regime, int(seed), h, c))
    return rows


def cmd_validate(a):
    fis_params, occ_override, meta = load_params(a.params)
    cfgs = build_configs(a.seeds, a.regimes)
    rows = paired_runs(cfgs, fis_params, occ_override, a.sim_end)
    print(f"{'regime':8s} {'seed':>5s} {'delay hand':>11s} {'delay cal':>10s} {'inc hand':>9s} {'inc cal':>8s}")
    rel, extra, out_rows = [], [], []
    for regime, seed, h, c in rows:
        print(f"{regime:8s} {seed:5d} {h['delay']:11.1f} {c['delay']:10.1f} {h['incidents']:9d} {c['incidents']:8d}")
        rel.append(c["delay"] / h["delay"])
        extra.append(max(0, c["incidents"] - h["incidents"]) / max(h["incidents"], 1))
        out_rows.append({"regime": regime, "seed": seed, "hand": h, "calibrated": c})
    J = float(np.mean(rel) + meta["protocol"]["incident_penalty_weight"] * np.mean(extra))
    verdict = J < 1.0
    print(f"\nvalidation J = {J:.4f} (hand-set = 1.0000) -> calibrated is "
          f"{'BETTER' if verdict else 'NOT better'} on held-out seeds")
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "validation.json"), "w") as fh:
        json.dump({"validation_J": J, "better_than_handset": verdict, "rows": out_rows,
                   "validation_seeds": a.seeds, "regimes": a.regimes}, fh, indent=2)
    print(f"wrote {OUT_DIR}/validation.json")


def cmd_final_test(a):
    out_path = os.path.join(OUT_DIR, "final_test.json")
    if not a.confirm_frozen:
        sys.exit("Refusing: pass --confirm-frozen to confirm the parameters are frozen.")
    if os.path.exists(out_path):
        sys.exit(f"Refusing: {out_path} already exists. The test seeds are one-shot; "
                 "delete the file deliberately only if you accept the evaluation is no longer blind.")
    from evaluate import CORE_KEYS, paired_tests, summarize
    fis_params, occ_override, _ = load_params(a.params)
    result = {"params": {"fis_params": fis_params, "occ_override": occ_override},
              "seeds": a.seeds, "regimes": {}}
    for regime in a.regimes:
        cfgs = build_configs(a.seeds, [regime])
        raw = {"Fuzzy (hand-set)": [], "Fuzzy (DE-calibrated)": []}
        from src import controller
        d_params, d_occ = vector_to_params(default_vector())
        for k, cfg in cfgs.items():
            seed = int(k.split(":")[1])
            for name, (fp, oo) in (("Fuzzy (hand-set)", (d_params, d_occ)),
                                   ("Fuzzy (DE-calibrated)", (fis_params, occ_override))):
                tag = f"{regime}_seed{seed}_{'cal' if 'DE' in name else 'hand'}"
                raw[name].append(controller.run(
                    sumocfg=cfg, seed=seed, regime=regime, sim_end=a.sim_end,
                    occ_override=oo, fis_params=fp,
                    tripinfo_out=f"results/tripinfo_final_{tag}.xml",
                    metrics_out=f"results/metrics_final_{tag}.json"))
        summary = summarize(raw, CORE_KEYS)
        p_raw, p_adj, dz, rb, norm = paired_tests(raw, CORE_KEYS)
        result["regimes"][regime] = {"summary": summary, "p_raw": p_raw, "p_holm": p_adj,
                                     "cohens_dz": dz, "rank_biserial": rb}
        print(f"\n=== {regime} ===")
        for k in CORE_KEYS:
            h = summary["Fuzzy (hand-set)"][k]["mean"]
            c = summary["Fuzzy (DE-calibrated)"][k]["mean"]
            label = f"Fuzzy (hand-set) vs Fuzzy (DE-calibrated) | {k}"
            print(f"{k:34s} hand={h:10.2f} cal={c:10.2f}  p_holm={p_adj.get(label)}")
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(out_path, "w") as fh:
        json.dump(result, fh, indent=2, default=str)
    print(f"\nwrote {out_path}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp, seeds, regimes=("moderate", "stress")):
        sp.add_argument("--seeds", type=int, nargs="+", default=seeds)
        sp.add_argument("--regimes", nargs="+", default=list(regimes), choices=["moderate", "stress"])
        sp.add_argument("--sim-end", type=int, default=3600)

    t = sub.add_parser("train")
    common(t, [201, 202, 203, 204])
    t.add_argument("--popsize", type=int, default=2, help="population = popsize x 10 members")
    t.add_argument("--maxiter", type=int, default=15)
    t.add_argument("--workers", type=int, default=1)
    t.add_argument("--penalty", type=float, default=0.5)
    t.add_argument("--de-seed", type=int, default=0)
    t.set_defaults(fn=cmd_train)

    v = sub.add_parser("validate")
    common(v, [101, 102, 103])
    v.add_argument("--params", default=os.path.join(OUT_DIR, "best_params.json"))
    v.set_defaults(fn=cmd_validate)

    f = sub.add_parser("final-test")
    common(f, list(range(1, 11)))
    f.add_argument("--params", default=os.path.join(OUT_DIR, "best_params.json"))
    f.add_argument("--confirm-frozen", action="store_true")
    f.set_defaults(fn=cmd_final_test)

    a = p.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
