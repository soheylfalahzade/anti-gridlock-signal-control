"""
Master benchmark: Fixed-Time (Webster), Vanilla Max-Pressure and Fuzzy
Anti-Spillback Max-Pressure, 3600 s each on identical network, demand and
random seed, measured by identical instrumentation.
"""

import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from baselines import fixed_time, vanilla_max_pressure
from src import controller

SUMOCFG = "configs/intersection.sumocfg"
COLORS = {"Fixed-Time (Webster)": "#8e9aaf",
          "Vanilla Max-Pressure": "#e63946",
          "Fuzzy Anti-Spillback": "#2a9d8f"}

BARS = [
    ("avg_delay_s", "Average Vehicle Delay", "s", "lower is better"),
    ("mean_queue_length", "Mean Queue Length", "veh", "lower is better"),
    ("throughput_completed_trips", "Throughput (completed trips)", "veh", "higher is better"),
    ("spillback_occurrences", "Junction-Box Spillback Events", "count", "lower is better"),
]


def bar_panel(ax, results, key, title, unit, hint):
    names = list(results)
    vals = [results[n][key] for n in names]
    bars = ax.bar(range(len(names)), vals,
                  color=[COLORS.get(n, "#777") for n in names],
                  edgecolor="#1d1f21", linewidth=0.8, width=0.62)
    ax.set_title(f"{title}\n({hint})", fontsize=11, fontweight="bold")
    ax.set_ylabel(unit, fontsize=9)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels([n.replace(" ", "\n", 1) for n in names], fontsize=8.5)
    ax.grid(axis="y", alpha=0.25, linestyle=":")
    ax.set_axisbelow(True)
    span = max(vals) if max(vals) > 0 else 1.0
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.03 * span,
                f"{v:,.1f}" if isinstance(v, float) else f"{v:,}",
                ha="center", fontsize=9.5, fontweight="bold")
    ax.set_ylim(0, span * 1.22)


def series_panel(ax, results, key, title, ylabel):
    for name, r in results.items():
        s = r["series"]
        ax.plot(s["t"], s[key], label=name, color=COLORS.get(name, "#777"),
                linewidth=2.0)
    ax.set_title(title, fontsize=11, fontweight="bold")
    ax.set_xlabel("simulation time [s]", fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.grid(alpha=0.25, linestyle=":")
    ax.set_axisbelow(True)
    ax.legend(fontsize=8.5, frameon=True)


def plot(results, path="results/comparison_metrics.png"):
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 4, hspace=0.42, wspace=0.28)
    for i, (key, title, unit, hint) in enumerate(BARS):
        bar_panel(fig.add_subplot(gs[0, i]), results, key, title, unit, hint)
    series_panel(fig.add_subplot(gs[1, 0:2]), results, "queue",
                 "Network Queue Evolution (60 s means)", "halted vehicles")
    series_panel(fig.add_subplot(gs[1, 2:4]), results, "spillback_cum",
                 "Cumulative Junction-Box Spillback Events", "events")
    fig.suptitle("Anti-Gridlock Signal Control — 3600 s benchmark on a "
                 "spillback-vulnerable arterial intersection\n"
                 "N-S 1500 veh/h with surges, E-W 400 veh/h, 50 m metered egress links",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0.01, 1, 0.93])
    os.makedirs("results", exist_ok=True)
    fig.savefig(path, dpi=170, facecolor="white")
    print(f"[chart] {path}")


def table(results):
    hdr = ("Policy", "Delay[s]", "MaxDelay", "MeanQ", "CrossQ",
           "Throughput", "Spillbacks", "Blocked[s]", "Unserved")
    print("\n" + "{:<24}{:>10}{:>10}{:>9}{:>9}{:>12}{:>12}{:>12}{:>10}".format(*hdr))
    print("-" * 108)
    for n, r in results.items():
        print("{:<24}{:>10.2f}{:>10.1f}{:>9.2f}{:>9.2f}{:>12d}{:>12d}{:>12d}{:>10d}".format(
            n, r["avg_delay_s"], r["max_delay_s"], r["mean_queue_length"],
            r["mean_cross_queue"], r["throughput_completed_trips"],
            r["spillback_occurrences"], r["spillback_duration_s"],
            r["vehicles_unserved"]))
    print()


def main(seed=1, verbose=False):
    os.makedirs("results", exist_ok=True)
    results = {}
    print("=== 1/3  Fixed-Time (Webster) ===")
    results["Fixed-Time (Webster)"] = fixed_time.run(seed=seed)
    print("=== 2/3  Vanilla Max-Pressure ===")
    results["Vanilla Max-Pressure"] = vanilla_max_pressure.run(seed=seed)
    print("=== 3/3  Fuzzy Anti-Spillback ===")
    results["Fuzzy Anti-Spillback"] = controller.run(seed=seed, verbose=verbose)

    with open("results/comparison_summary.json", "w") as f:
        json.dump(results, f, indent=2)
    plot(results)
    table(results)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--verbose", action="store_true",
                   help="stream fuzzy controller telemetry during the benchmark")
    a = p.parse_args()
    main(seed=a.seed, verbose=a.verbose)
