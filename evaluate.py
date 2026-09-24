"""
Master benchmark, Q1-submission-ready statistical package.

Additions vs. previous revision:
  - Default N=10 paired seeds (was 5): with n=5, Wilcoxon signed-rank can
    never report p<0.05 (min two-sided p = 2/2^5 = 0.0625) regardless of
    true effect size, which is an automatic reviewer rejection for any
    "statistically significant" claim. n=10 allows p as low as 2/2^10
    ~= 0.00195.
  - Holm-Bonferroni correction across the full family of pairwise tests
    per metric, reported alongside raw p-values (raw p alone, uncorrected
    across ~15 comparisons, is itself a common Q1 desk-reject reason).
  - Matched-pairs effect size (Cohen's d_z = mean(diff)/std(diff)) for
    every comparison, since p-values alone do not convey magnitude.
  - Percentile bootstrap 95% CI (2000 resamples) for every summary
    statistic, reported alongside mean +/- std.
  - Composite, demand-normalized anti-gridlock index
    (gridlock_incidents_per_1000veh) as the headline safety metric,
    replacing the two individually noisy raw counts as the primary claim.
  - Phase D: causal preemption ablation. The fuzzy controller is run
    twice per seed, identical demand, with and without emergency
    preemption (--no-preempt), isolating the preemption mechanism's
    effect on ambulance delay from the base algorithm's own queue
    management -- the previous single-arm comparison could not
    distinguish "preemption helped" from "fuzzy's baseline signal timing
    happened to help/hurt this ambulance."
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
from src.demand import make_config, AMBULANCE_DEPART_LIGHT, AMBULANCE_DEPART_STRESS
from src.metrics import movement_delay_breakdown, jains_index, emergency_metrics

COLORS = {"Fixed-Time (Webster)": "#8e9aaf",
          "Vanilla Max-Pressure": "#e63946",
          "Fuzzy Anti-Spillback": "#2a9d8f"}

CORE_KEYS = ["avg_delay_s", "mean_queue_length", "throughput_completed_trips",
             "gridlock_incidents_per_1000veh", "box_gridlock_events",
             "storage_overflow_events_NS", "storage_overflow_events_EW",
             "deadlock_teleports"]


def runners(regime):
    return {
        "Fixed-Time (Webster)": lambda cfg, seed, tag: fixed_time.run(
            sumocfg=cfg, seed=seed, regime=regime,
            tripinfo_out=f"results/tripinfo_{tag}_fixed.xml",
            metrics_out=f"results/metrics_{tag}_fixed.json"),
        "Vanilla Max-Pressure": lambda cfg, seed, tag: vanilla_max_pressure.run(
            sumocfg=cfg, seed=seed, regime=regime,
            tripinfo_out=f"results/tripinfo_{tag}_vmp.xml",
            metrics_out=f"results/metrics_{tag}_vmp.json"),
        "Fuzzy Anti-Spillback": lambda cfg, seed, tag: controller.run(
            sumocfg=cfg, seed=seed, regime=regime,
            tripinfo_out=f"results/tripinfo_{tag}_fuzzy.xml",
            metrics_out=f"results/metrics_{tag}_fuzzy.json"),
    }


def tripinfo_paths(tag):
    return {"Fixed-Time (Webster)": f"results/tripinfo_{tag}_fixed.xml",
           "Vanilla Max-Pressure": f"results/tripinfo_{tag}_vmp.xml",
           "Fuzzy Anti-Spillback": f"results/tripinfo_{tag}_fuzzy.xml"}


# --------------------------------------------------------------------------
# Statistics helpers
# --------------------------------------------------------------------------

def bootstrap_ci(values, n_resamples=2000, ci=0.95, seed=0):
    values = np.asarray(values, dtype=float)
    if len(values) < 2:
        m = float(values.mean()) if len(values) else 0.0
        return m, m, m
    rng = np.random.RandomState(seed)
    means = np.empty(n_resamples)
    n = len(values)
    for i in range(n_resamples):
        sample = values[rng.randint(0, n, n)]
        means[i] = sample.mean()
    lo = float(np.percentile(means, (1 - ci) / 2 * 100))
    hi = float(np.percentile(means, (1 + ci) / 2 * 100))
    return float(values.mean()), lo, hi


def cohens_dz(xa, xb):
    diff = np.asarray(xa, dtype=float) - np.asarray(xb, dtype=float)
    sd = diff.std(ddof=1)
    if sd == 0:
        return 0.0
    return float(diff.mean() / sd)


def holm_bonferroni(pvals_dict):
    """pvals_dict: {label: p or None or str}. Returns {label: adjusted_p}
    for numeric entries; non-numeric entries pass through unchanged."""
    items = [(k, v) for k, v in pvals_dict.items() if isinstance(v, (int, float))]
    items.sort(key=lambda kv: kv[1])
    m = len(items)
    adjusted = {}
    running_max = 0.0
    for i, (k, p) in enumerate(items):
        adj = min(1.0, p * (m - i))
        running_max = max(running_max, adj)
        adjusted[k] = running_max
    out = dict(pvals_dict)
    out.update(adjusted)
    return out


def summarize(raw, keys):
    summary = {}
    for name, runs in raw.items():
        summary[name] = {}
        for k in keys:
            vals = np.array([r[k] for r in runs], dtype=float)
            mean, lo, hi = bootstrap_ci(vals)
            summary[name][k] = {"mean": mean,
                               "std": float(vals.std(ddof=1)) if len(vals) > 1 else 0.0,
                               "ci95_lo": lo, "ci95_hi": hi}
    return summary


def paired_tests(raw, keys, min_n=6):
    p_raw, effect = {}, {}
    names = list(raw.keys())
    for a, b in itertools.combinations(names, 2):
        for k in keys:
            xa = np.array([r[k] for r in raw[a]], dtype=float)
            xb = np.array([r[k] for r in raw[b]], dtype=float)
            label = f"{a} vs {b} | {k}"
            if len(xa) < min_n:
                p_raw[label] = None
            elif np.allclose(xa, xb):
                p_raw[label] = "identical (no variance)"
            else:
                try:
                    p_raw[label] = float(stats.wilcoxon(xa, xb, zero_method="zsplit").pvalue)
                except ValueError:
                    p_raw[label] = None
            effect[label] = cohens_dz(xa, xb) if len(xa) == len(xb) and len(xa) > 1 else None
    p_adj = holm_bonferroni(p_raw)
    return p_raw, p_adj, effect


# --------------------------------------------------------------------------
# Phase A/C: paired multi-seed benchmark
# --------------------------------------------------------------------------

def run_multiseed(seeds, regime="moderate", ambulance_depart=AMBULANCE_DEPART_LIGHT):
    R = runners(regime)
    raw = {p: [] for p in R}
    fairness = {p: [] for p in R}
    emergency = {p: [] for p in R}

    for seed in seeds:
        tag = f"{regime}_seed{seed}"
        cfg, n = make_config(tag, scale=1.0, seed=seed, ambulance=True,
                             ambulance_depart=ambulance_depart)
        print(f"--- [{regime}] seed {seed}: {n} vehicles ---")
        for name, runner in R.items():
            res = runner(cfg, seed, tag)
            raw[name].append(res)
            breakdown = movement_delay_breakdown(tripinfo_paths(tag)[name])
            fairness[name].append(jains_index(list(breakdown.values())))
            amb = emergency_metrics(tripinfo_paths(tag)[name], desired_depart=ambulance_depart)
            emergency[name].append(amb[0] if amb else None)
    return raw, fairness, emergency


# --------------------------------------------------------------------------
# Phase B: demand sweep (3 seeds per scale, averaged, for tractable runtime)
# --------------------------------------------------------------------------

def run_demand_sweep(scales, seeds=(1, 2, 3), regime="moderate"):
    R = runners(regime)
    sweep = {p: {"scale": [], "avg_delay_s": [], "throughput_completed_trips": [],
                "gridlock_incidents_per_1000veh": [], "deadlock_teleports": []}
             for p in R}
    for scale in scales:
        per_scale = {p: {k: [] for k in ("avg_delay_s", "throughput_completed_trips",
                                        "gridlock_incidents_per_1000veh",
                                        "deadlock_teleports")} for p in R}
        for seed in seeds:
            tag = f"sweep_{regime}_s{str(scale).replace('.', 'p')}_seed{seed}"
            cfg, n = make_config(tag, scale=scale, seed=seed, ambulance=False)
            print(f"--- sweep [{regime}] scale={scale} seed={seed} ({n} vehicles) ---")
            for name, runner in R.items():
                res = runner(cfg, seed, tag)
                for k in per_scale[name]:
                    per_scale[name][k].append(res.get(k, 0))
        for name in R:
            sweep[name]["scale"].append(scale)
            for k in per_scale[name]:
                sweep[name][k].append(float(np.mean(per_scale[name][k])))
    return sweep


# --------------------------------------------------------------------------
# Phase D: preemption causal ablation (fuzzy only, with vs. without)
# --------------------------------------------------------------------------

def run_preemption_ablation(seeds, regime="moderate", ambulance_depart=AMBULANCE_DEPART_LIGHT):
    with_amb, without_amb = [], []
    for seed in seeds:
        tag = f"preempt_{regime}_seed{seed}"
        cfg, n = make_config(tag, scale=1.0, seed=seed, ambulance=True,
                             ambulance_depart=ambulance_depart)
        r_with = controller.run(sumocfg=cfg, seed=seed, regime=regime,
                                emergency_preempt=True,
                                tripinfo_out=f"results/tripinfo_{tag}_with.xml",
                                metrics_out=f"results/metrics_{tag}_with.json")
        r_without = controller.run(sumocfg=cfg, seed=seed, regime=regime,
                                   emergency_preempt=False,
                                   tripinfo_out=f"results/tripinfo_{tag}_without.xml",
                                   metrics_out=f"results/metrics_{tag}_without.json")
        amb_with = emergency_metrics(f"results/tripinfo_{tag}_with.xml", desired_depart=ambulance_depart)
        amb_without = emergency_metrics(f"results/tripinfo_{tag}_without.xml", desired_depart=ambulance_depart)
        with_amb.append(amb_with[0]["delay_s"] if amb_with else None)
        without_amb.append(amb_without[0]["delay_s"] if amb_without else None)
    return with_amb, without_amb


# --------------------------------------------------------------------------
# Plotting
# --------------------------------------------------------------------------

def plot_main(summary, path):
    bars = [("avg_delay_s", "Average Vehicle Delay [s]", "lower better"),
           ("mean_queue_length", "Mean Queue Length [veh]", "lower better"),
           ("throughput_completed_trips", "Throughput [veh]", "higher better"),
           ("gridlock_incidents_per_1000veh", "Gridlock Incidents /1000veh\n(box+storage, composite)", "lower better")]
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.5))
    names = list(summary.keys())
    for ax, (key, title, hint) in zip(axes, bars):
        means = [summary[n][key]["mean"] for n in names]
        lo = [summary[n][key]["mean"] - summary[n][key]["ci95_lo"] for n in names]
        hi = [summary[n][key]["ci95_hi"] - summary[n][key]["mean"] for n in names]
        ax.bar(names, means, yerr=[lo, hi], capsize=5,
              color=[COLORS.get(n, "#777") for n in names], edgecolor="#1d1f21")
        ax.set_title(f"{title}\n({hint})", fontsize=10, fontweight="bold")
        ax.tick_params(axis="x", rotation=15, labelsize=8)
        ax.grid(axis="y", alpha=0.25, linestyle=":")
    fig.suptitle("Multi-seed benchmark (mean, bootstrap 95% CI)", fontsize=12, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(path, dpi=160)
    print(f"[chart] {path}")


def plot_storage_breakdown(summary, path):
    names = list(summary.keys())
    ns = [summary[n]["storage_overflow_events_NS"]["mean"] for n in names]
    ew = [summary[n]["storage_overflow_events_EW"]["mean"] for n in names]
    fig, ax = plt.subplots(figsize=(7, 4.5))
    x = np.arange(len(names))
    ax.bar(x - 0.18, ns, width=0.36, label="NS approach overflow", color="#264653")
    ax.bar(x + 0.18, ew, width=0.36, label="EW approach overflow", color="#e9c46a")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15, fontsize=8)
    ax.set_title("Upstream Storage-Overflow Events by Direction\n"
                 "(which movement pays the box-protection cost)", fontsize=10, fontweight="bold")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.25, linestyle=":")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    print(f"[chart] {path}")


def plot_sweep(sweep, path):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    metrics = [("avg_delay_s", "Average Delay [s]"),
              ("throughput_completed_trips", "Throughput [veh]"),
              ("gridlock_incidents_per_1000veh", "Gridlock Incidents /1000veh")]
    for ax, (key, title) in zip(axes, metrics):
        for name, data in sweep.items():
            ax.plot(data["scale"], data[key], marker="o", label=name, color=COLORS.get(name, "#777"))
        ax.set_xlabel("Demand scale (x base V/C)")
        ax.set_title(title, fontsize=10, fontweight="bold")
        ax.grid(alpha=0.25, linestyle=":")
    axes[0].legend(fontsize=8)
    fig.suptitle("Demand sweep (3-seed average per point, moderate regime)",
                fontsize=12, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(path, dpi=160)
    print(f"[chart] {path}")


def plot_fairness(fairness, path):
    fig, ax = plt.subplots(figsize=(6, 4.5))
    names = list(fairness.keys())
    means = [np.mean(fairness[n]) for n in names]
    ax.bar(names, means, color=[COLORS.get(n, "#777") for n in names])
    ax.set_title("Jain's Fairness Index (movement delay)\n(1.0 = perfectly fair)",
                fontsize=10, fontweight="bold")
    ax.set_ylim(0, 1.05)
    ax.tick_params(axis="x", rotation=15, labelsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    print(f"[chart] {path}")


def plot_preemption_ablation(with_amb, without_amb, path):
    with_v = [v for v in with_amb if v is not None]
    without_v = [v for v in without_amb if v is not None]
    fig, ax = plt.subplots(figsize=(6, 4.5))
    means = [np.mean(without_v) if without_v else 0, np.mean(with_v) if with_v else 0]
    ax.bar(["Without preemption", "With preemption"], means, color=["#e76f51", "#2a9d8f"])
    ax.set_title("Ambulance Delay: Causal Effect of Preemption\n"
                 "(same seeds, same base algorithm, only preemption toggled)",
                fontsize=10, fontweight="bold")
    ax.set_ylabel("delay [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    print(f"[chart] {path}")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def emergency_to_json(emergency):
    return {k: [dict(e) if e else None for e in v] for k, v in emergency.items()}


def main(seeds, sweep_seeds, scales):
    os.makedirs("results", exist_ok=True)

    print("\n=== Phase A: primary benchmark (moderate regime) ===")
    raw, fairness, emergency = run_multiseed(seeds, regime="moderate")
    summary = summarize(raw, CORE_KEYS)
    p_raw, p_adj, effect = paired_tests(raw, CORE_KEYS, min_n=6)
    plot_main(summary, "results/comparison_metrics.png")
    plot_storage_breakdown(summary, "results/storage_overflow_breakdown.png")
    plot_fairness(fairness, "results/fairness.png")

    print("\n=== Phase B: demand sweep (moderate regime, 3-seed avg) ===")
    sweep = run_demand_sweep(scales, seeds=sweep_seeds, regime="moderate")
    plot_sweep(sweep, "results/demand_sweep.png")

    print("\n=== Phase C: stress-regime ablation (chronic V/C>1 overload) ===")
    stress_raw, stress_fairness, stress_emergency = run_multiseed(
        seeds, regime="stress", ambulance_depart=AMBULANCE_DEPART_STRESS)
    stress_summary = summarize(stress_raw, CORE_KEYS)
    stress_p_raw, stress_p_adj, stress_effect = paired_tests(stress_raw, CORE_KEYS, min_n=6)
    plot_main(stress_summary, "results/comparison_metrics_stress.png")
    plot_storage_breakdown(stress_summary, "results/storage_overflow_breakdown_stress.png")
    plot_fairness(stress_fairness, "results/fairness_stress.png")

    print("\n=== Phase D: preemption causal ablation (fuzzy, with vs. without) ===")
    with_amb, without_amb = run_preemption_ablation(seeds, regime="moderate")
    plot_preemption_ablation(with_amb, without_amb, "results/preemption_ablation.png")
    valid_pairs = [(w, wo) for w, wo in zip(with_amb, without_amb) if w is not None and wo is not None]
    preempt_stats = {"with": with_amb, "without": without_amb}
    if len(valid_pairs) >= 6:
        xa = np.array([p[0] for p in valid_pairs])
        xb = np.array([p[1] for p in valid_pairs])
        try:
            preempt_stats["wilcoxon_p"] = float(stats.wilcoxon(xa, xb, zero_method="zsplit").pvalue)
        except ValueError:
            preempt_stats["wilcoxon_p"] = None
        preempt_stats["cohens_dz"] = cohens_dz(xa, xb)
    else:
        preempt_stats["wilcoxon_p"] = None
        preempt_stats["cohens_dz"] = None

    report = {
        "moderate_regime": {"summary": summary, "wilcoxon_p_raw": p_raw,
                            "wilcoxon_p_holm_bonferroni": p_adj,
                            "cohens_dz": effect, "jains_index": fairness,
                            "emergency": emergency_to_json(emergency)},
        "demand_sweep_moderate": sweep,
        "stress_regime_ablation": {"summary": stress_summary, "wilcoxon_p_raw": stress_p_raw,
                                   "wilcoxon_p_holm_bonferroni": stress_p_adj,
                                   "cohens_dz": stress_effect,
                                   "jains_index": stress_fairness,
                                   "emergency": emergency_to_json(stress_emergency)},
        "preemption_ablation": preempt_stats,
        "methodology_notes": {
            "seeds_primary": list(seeds),
            "seeds_sweep": list(sweep_seeds),
            "min_seeds_for_wilcoxon": 6,
            "multiple_comparison_correction": "Holm-Bonferroni, family = all pairwise "
                                              "comparisons per regime across CORE_KEYS",
            "effect_size": "Cohen's d_z (paired), mean(diff)/std(diff)",
            "ci_method": "percentile bootstrap, 2000 resamples, 95% CI",
        },
    }
    with open("results/statistical_report.json", "w") as f:
        json.dump(report, f, indent=2)

    print("\n--- Moderate regime: mean [95% CI] ---")
    for name in summary:
        d = summary[name]["avg_delay_s"]
        g = summary[name]["gridlock_incidents_per_1000veh"]
        t = summary[name]["throughput_completed_trips"]
        print(f"{name:<24} delay={d['mean']:.1f} [{d['ci95_lo']:.1f},{d['ci95_hi']:.1f}]  "
             f"throughput={t['mean']:.0f} [{t['ci95_lo']:.0f},{t['ci95_hi']:.0f}]  "
             f"gridlock/1000veh={g['mean']:.2f} [{g['ci95_lo']:.2f},{g['ci95_hi']:.2f}]")
    print("\n--- Holm-Bonferroni corrected p-values (moderate regime) ---")
    for k, v in p_adj.items():
        print(f"{k}: {v}")
    print(f"\nFull report: results/statistical_report.json")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--seeds", type=int, nargs="+", default=list(range(1, 11)))
    p.add_argument("--sweep-seeds", type=int, nargs="+", default=[1, 2, 3])
    p.add_argument("--scales", type=float, nargs="+", default=[0.6, 0.8, 1.0, 1.2, 1.5, 1.8, 2.0])
    a = p.parse_args()
    main(a.seeds, a.sweep_seeds, a.scales)
