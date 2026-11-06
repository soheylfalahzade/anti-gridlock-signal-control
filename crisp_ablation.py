"""Does the fuzzy engine earn its place?  Fuzzy Anti-Spillback against its non-fuzzy twin.

The twin (src/fuzzy_engine.CrispTwinEngine) keeps the nine rules, the membership breakpoints, the
output range and the occupancy override, and removes only the fuzziness: each input is assigned to one
class and the output is the centroid of one class. Vanilla Max-Pressure is run on the same seeds as the
reference. The three policies are paired by seed.

Protocol, fixed before the runs:
  * seeds 501-530, none used elsewhere in this repository (the seed ledger is in README.md);
  * moderate regime (ambulance at 300 s) and stress regime (ambulance at 1800 s), as in evaluate.py;
  * primary endpoint: mean delay (avg_delay_s), moderate regime, twin against Mamdani engine, two-sided
    Wilcoxon signed-rank test. All other metrics and the stress regime are secondary, and the Holm
    correction is applied over every comparison and metric exactly as in evaluate.py;
  * the report is written whichever way the result goes.

    python crisp_ablation.py --run                      # resumable; add --max-seconds N to stop early
    python crisp_ablation.py --summarize                # writes results/crisp_ablation_report.json
"""
import argparse
import csv
import json
import os
import shutil
import time

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(ROOT, "results", "crisp_ablation_raw.csv")
REPORT = os.path.join(ROOT, "results", "crisp_ablation_report.json")
TMP = os.path.join(ROOT, "results", "_crisp_tmp")
SEEDS = list(range(501, 531))
VANILLA, FUZZY, CRISP = "Vanilla Max-Pressure", "Fuzzy Anti-Spillback", "Crisp-Twin Anti-Spillback"
POLICIES = [VANILLA, FUZZY, CRISP]
KEYS = ["avg_delay_s", "mean_queue_length", "throughput_completed_trips", "gridlock_incidents_per_1000veh",
        "box_gridlock_events", "storage_overflow_events_NS", "storage_overflow_events_EW",
        "deadlock_teleports", "total_co2_kg"]


def read_raw(path=None):
    path = path or RAW
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        r["seed"] = int(r["seed"])
        for k in KEYS:
            r[k] = float(r[k])
    return rows


def append_row(row, path=None):
    path = path or RAW
    os.makedirs(os.path.dirname(path), exist_ok=True)
    new = not os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["regime", "seed", "policy"] + KEYS, lineterminator="\n")
        if new:
            w.writeheader()
        w.writerow(row)


def run_all(seeds, regimes, max_seconds):
    from baselines import vanilla_max_pressure
    from src import controller
    from src.demand import AMBULANCE_DEPART_LIGHT, AMBULANCE_DEPART_STRESS, make_config

    depart = {"moderate": AMBULANCE_DEPART_LIGHT, "stress": AMBULANCE_DEPART_STRESS}
    done = {(r["regime"], r["seed"], r["policy"]) for r in read_raw()}
    t_end = time.time() + max_seconds
    os.makedirs(TMP, exist_ok=True)
    for regime in regimes:
        for seed in seeds:
            todo = [p for p in POLICIES if (regime, seed, p) not in done]
            if not todo:
                continue
            if time.time() > t_end:
                print("time limit reached; rerun to resume")
                return
            tag = f"crisp_{regime}_seed{seed}"
            cfg, n = make_config(tag, scale=1.0, seed=seed, ambulance=True, ambulance_depart=depart[regime])
            print(f"--- [{regime}] seed {seed}: {n} vehicles ---", flush=True)
            for name in todo:
                tri = os.path.join(TMP, f"tripinfo_{tag}_{name[:4]}.xml")
                met = os.path.join(TMP, f"metrics_{tag}_{name[:4]}.json")
                kw = dict(sumocfg=cfg, seed=seed, regime=regime, tripinfo_out=tri, metrics_out=met)
                if name == VANILLA:
                    res = vanilla_max_pressure.run(**kw)
                elif name == FUZZY:
                    res = controller.run(**kw)
                else:
                    res = controller.run(engine_kind="crisp_twin", **kw)
                append_row(dict(regime=regime, seed=seed, policy=name, **{k: res[k] for k in KEYS}))
    shutil.rmtree(TMP, ignore_errors=True)


def summarize():
    import evaluate as ev
    rows = read_raw()
    report = {"seeds": sorted({r["seed"] for r in rows}), "policies": POLICIES}
    for regime in ("moderate", "stress"):
        by = {p: {r["seed"]: r for r in rows if r["regime"] == regime and r["policy"] == p} for p in POLICIES}
        common = sorted(set.intersection(*[set(v) for v in by.values()]))
        if len(common) < 6:
            report[regime] = {"n_pairs": len(common), "note": "fewer than 6 complete seeds; no tests run"}
            continue
        raw = {p: [by[p][s] for s in common] for p in POLICIES}
        p_raw, p_adj, dz, rb, normality = ev.paired_tests(raw, KEYS, min_n=6)
        report[regime] = {"n_pairs": len(common), "seeds": common, "summary": ev.summarize(raw, KEYS),
                          "wilcoxon_p_raw": p_raw, "wilcoxon_p_holm_bonferroni": p_adj,
                          "cohens_dz": dz, "rank_biserial_r": rb, "shapiro_on_differences": normality}
    with open(REPORT, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2)
    print(f"wrote {os.path.relpath(REPORT, ROOT)}")
    return report


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true")
    ap.add_argument("--summarize", action="store_true")
    ap.add_argument("--seeds", type=int, nargs="+", default=SEEDS)
    ap.add_argument("--regimes", nargs="+", default=["moderate", "stress"], choices=["moderate", "stress"])
    ap.add_argument("--max-seconds", type=int, default=10 ** 9)
    a = ap.parse_args()
    if a.run:
        run_all(a.seeds, a.regimes, a.max_seconds)
    if a.summarize:
        summarize()
    if not (a.run or a.summarize):
        ap.print_help()
