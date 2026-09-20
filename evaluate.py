"""
Master benchmark. Fix vs. previous revision: Wilcoxon signed-rank now runs
with 5 paired seeds (was silently skipped below 6) and uses zero_method
"zsplit" so ties across seeds do not raise an exception.
"""

import argparse
import itertools
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

from baselines import fixed_time, vanilla_max_pressure
from src import controller
from src.demand import make_config
from src.metrics import movement_delay_breakdown, jains_index, emergency_metrics

COLORS = {"Fixed-Time (Webster)": "#8e9aaf",
          "Vanilla Max-Pressure": "#e63946",
          "Fuzzy Anti-Spillback": "#2a9d8f"}
POLICY_RUNNERS = {
    "Fixed-Time (Webster)": lambda cfg, seed, tag: fixed_time.run(
        sumocfg=cfg, seed=seed, tripinfo_out=f"results/tripinfo_{tag}_fixed.xml",
        metrics_out=f"results/metrics_{tag}_fixed.json"),
    "Vanilla Max-Pressure": lambda cfg, seed, tag: vanilla_max_pressure.run(
        sumocfg=cfg, seed=seed, tripinfo_out=f"results/tripinfo_{tag}_vmp.xml",
        metrics_out=f"results/metrics_{tag}_vmp.json"),
    "Fuzzy Anti-Spillback": lambda cfg, seed, tag: controller.run(
        sumocfg=cfg, seed=seed, tripinfo_out=f"results/tripinfo_{tag}_fuzzy.xml",
        metrics_out=f"results/metrics_{tag}_fuzzy.json"),
}
TRIPINFO_OF = {
    "Fixed-Time (Webster)": lambda tag: f"results/tripinfo_{tag}_fixed.xml",
    "Vanilla Max-Pressure": lambda tag: f"results/tripinfo_{tag}_vmp.xml",
    "Fuzzy Anti-Spillback": lambda tag: f"results/tripinfo_{tag}_fuzzy.xml",
}


def run_multiseed(seeds):
    raw = {p: [] for p in POLICY_RUNNERS}
    fairness = {p: [] for p in POLICY_RUNNERS}
    emergency = {p: [] for p in POLICY_RUNNERS}

    for seed in seeds:
        tag = f"base_seed{seed}"
        cfg, n = make_config(tag, scale=1.0, seed=seed, ambulance=True)
        print(f"--- seed {seed}: {n} vehicles (baseline demand + ambulance) ---")
        for name, runner in POLICY_RUNNERS.items():
            res = runner(cfg, seed, tag)
            raw[name].append(res)
            breakdown = movement_delay_breakdown(TRIPINFO_OF[name](tag))
            fairness[name].append(jains_index(list(breakdown.values())))
            amb = emergency_metrics(TRIPINFO_OF[name](tag))
            emergency[name].append(amb[0]["delay_s"] if amb else None)
    return raw, fairness, emergency


def summarize(raw, keys):
    summary = {}
    for name, runs in raw.items():
        summary[name] = {}
        for k in keys:
            vals = np.array([r[k] for r in runs], dtype=float)
            summary[name][k] = {"mean": float(vals.mean()),
                               "std": float(vals.std(ddof=1)) if len(vals) > 1 else 0.0}
    return summary


def paired_tests(raw, keys, min_n=5):
    out = {}
    names = list(raw.keys())
    for a, b in itertools.combinations(names, 2):
        for k in keys:
            xa = np.array([r[k] for r in raw[a]], dtype=float)
            xb = np.array([r[k] for r in raw[b]], dtype=float)
            if len(xa) < min_n:
                p = None
            elif np.allclose(xa, xb):
                p = 1.0
            else:
                try:
                    p = float(stats.wilcoxon(xa, xb, zero_method="zsplit").pvalue)
                except ValueError:
                    p = None
            out[f"{a} vs {b} | {k}"] = p
    return out


def run_demand_sweep(scales, seed=1):
    sweep = {p: {"scale": [], "avg_delay_s": [], "throughput_completed_trips": [],
                "spillback_occurrences": [], "deadlock_teleports": []} for p in POLICY_RUNNERS}
    for scale in scales:
        tag = f"sweep_s{str(scale).replace('.', 'p')}"
        cfg, n = make_config(tag, scale=scale, seed=seed, ambulance=False)
        print(f"--- sweep scale={scale} ({n} vehicles) ---")
        for name, runner in POLICY_RUNNERS.items():
            res = runner(cfg, seed, tag)
            sweep[name]["scale"].append(scale)
            for k in ("avg_delay_s", "throughput_completed_trips",
                     "spillback_occurrences", "deadlock_teleports"):
                sweep[name][k].append(res.get(k, 0))
    return sweep


def plot_main(summary, path="results/comparison_metrics.png"):
    bars = [("avg_delay_s", "Average Vehicle Delay [s]", "lower better"),
            ("mean_queue_length", "Mean Queue Length [veh]", "lower better"),
            ("throughput_completed_trips", "Throughput [veh]", "higher better"),
            ("spillback_occurrences", "Spillback Events", "lower better")]
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.5))
    names = list(summary.keys())
    for ax, (key, title, hint) in zip(axes, bars):
        means = [summary[n][key]["mean"] for n in names]
        stds = [summary[n][key]["std"] for n in names]
        ax.bar(names, means, yerr=stds, capsize=5,
              color=[COLORS.get(n, "#777") for n in names], edgecolor="#1d1f21")
        ax.set_title(f"{title}\n({hint})", fontsize=10, fontweight="bold")
        ax.tick_params(axis="x", rotation=15, labelsize=8)
        ax.grid(axis="y", alpha=0.25, linestyle=":")
    fig.suptitle("Multi-seed benchmark (mean +/- std, N seeds, paired demand realizations)",
                fontsize=12, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(path, dpi=160)
    print(f"[chart] {path}")


def plot_sweep(sweep, path="results/demand_sweep.png"):
    fig, axes = plt.subplots(1, 4, figsize=(20, 4.5))
    metrics = [("avg_delay_s", "Average Delay [s]"),
              ("throughput_completed_trips", "Throughput [veh]"),
              ("spillback_occurrences", "Spillback Events"),
              ("deadlock_teleports", "Deadlock Teleports")]
    for ax, (key, title) in zip(axes, metrics):
        for name, data in sweep.items():
            ax.plot(data["scale"], data[key], marker="o", label=name,
                   color=COLORS.get(name, "#777"))
        ax.set_xlabel("Demand scale (x base V/C)")
        ax.set_title(title, fontsize=10, fontweight="bold")
        ax.grid(alpha=0.25, linestyle=":")
    axes[0].legend(fontsize=8)
    fig.suptitle("Demand sweep: where does each policy win?", fontsize=12, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(path, dpi=160)
    print(f"[chart] {path}")


def plot_fairness_emergency(fairness, emergency, path="results/fairness_emergency.png"):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    names = list(fairness.keys())

    j_means = [np.mean(fairness[n]) for n in names]
    axes[0].bar(names, j_means, color=[COLORS.get(n, "#777") for n in names])
    axes[0].set_title("Jain's Fairness Index (movement delay)\n(1.0 = perfectly fair)",
                      fontsize=10, fontweight="bold")
    axes[0].set_ylim(0, 1.05)
    axes[0].tick_params(axis="x", rotation=15, labelsize=8)

    e_means, e_names = [], []
    for n in names:
        vals = [v for v in emergency[n] if v is not None]
        if vals:
            e_means.append(np.mean(vals))
            e_names.append(n)
    if e_names:
        axes[1].bar(e_names, e_means, color=[COLORS.get(n, "#777") for n in e_names])
    else:
        axes[1].text(0.5, 0.5, "no ambulance trip data recorded", ha="center", va="center")
    axes[1].set_title("Ambulance Delay [s]\n(Fuzzy has preemption; baselines do not)",
                      fontsize=10, fontweight="bold")
    axes[1].tick_params(axis="x", rotation=15, labelsize=8)

    fig.tight_layout()
    fig.savefig(path, dpi=160)
    print(f"[chart] {path}")


def main(seeds, scales):
    os.makedirs("results", exist_ok=True)
    print("=== Phase A: paired multi-seed benchmark ===")
    raw, fairness, emergency = run_multiseed(seeds)
    keys = ["avg_delay_s", "mean_queue_length", "throughput_completed_trips",
           "spillback_occurrences", "deadlock_teleports"]
    summary = summarize(raw, keys)
    p_values = paired_tests(raw, keys)
    plot_main(summary)
    plot_fairness_emergency(fairness, emergency)

    print("\n=== Phase B: demand sweep (V/C ratio) ===")
    sweep = run_demand_sweep(scales)
    plot_sweep(sweep)

    report = {"summary": summary, "wilcoxon_p_values": p_values,
             "jains_index": {k: v for k, v in fairness.items()},
             "emergency_delay_s": {k: v for k, v in emergency.items()},
             "demand_sweep": sweep}
    with open("results/statistical_report.json", "w") as f:
        json.dump(report, f, indent=2)

    print("\n--- Mean +/- Std (N=%d seeds) ---" % len(seeds))
    for name in summary:
        d = summary[name]["avg_delay_s"]
        q = summary[name]["mean_queue_length"]
        t = summary[name]["throughput_completed_trips"]
        s = summary[name]["spillback_occurrences"]
        tp = summary[name]["deadlock_teleports"]
        print(f"{name:<24} delay={d['mean']:.1f}+/-{d['std']:.1f}  "
             f"queue={q['mean']:.1f}+/-{q['std']:.1f}  "
             f"throughput={t['mean']:.0f}+/-{t['std']:.0f}  "
             f"spillback={s['mean']:.1f}+/-{s['std']:.1f}  "
             f"teleports={tp['mean']:.1f}+/-{tp['std']:.1f}")
    print("\n--- Wilcoxon signed-rank p-values (paired by seed) ---")
    for k, v in p_values.items():
        print(f"{k}: {'n/a' if v is None else f'p={v:.4f}'}")
    print(f"\nFull report: results/statistical_report.json")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    p.add_argument("--scales", type=float, nargs="+", default=[0.6, 0.8, 1.0, 1.2, 1.5, 1.8])
    a = p.parse_args()
    main(a.seeds, a.scales)
