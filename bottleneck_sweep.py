"""
Direct bottleneck-severity sweep: varies NS egress metering green time
(hence duty ratio and sustainable capacity) at FIXED demand (scale=1.0),
for all three policies. This is the counterpart the demand sweep in
evaluate.py cannot substitute for: the demand sweep varies arrival rate
against a fixed bottleneck; this sweep varies the bottleneck itself
against fixed arrival rate, directly testing the core claim ("as the
downstream constraint tightens, does the fuzzy controller degrade more
gracefully than the baselines").
"""

import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from baselines import fixed_time, vanilla_max_pressure
from src import controller
from src.demand import make_config
from src.metering import ns_capacity_vph_per_lane

SEEDS = [1, 2, 3]
NS_GREEN_VALUES = [10, 13, 15, 18, 23, 28, 33]  # seconds, out of 50s cycle
COLORS = {"Fixed-Time (Webster)": "#8e9aaf", "Vanilla Max-Pressure": "#e63946",
          "Fuzzy Anti-Spillback": "#2a9d8f"}


def run_sweep():
    policies = {
        "Fixed-Time (Webster)": lambda cfg, seed, ns_g, tag: fixed_time.run(
            sumocfg=cfg, seed=seed, ns_green_override=ns_g,
            tripinfo_out=f"results/tripinfo_{tag}_fixed.xml",
            metrics_out=f"results/metrics_{tag}_fixed.json"),
        "Vanilla Max-Pressure": lambda cfg, seed, ns_g, tag: vanilla_max_pressure.run(
            sumocfg=cfg, seed=seed, ns_green_override=ns_g,
            tripinfo_out=f"results/tripinfo_{tag}_vmp.xml",
            metrics_out=f"results/metrics_{tag}_vmp.json"),
        "Fuzzy Anti-Spillback": lambda cfg, seed, ns_g, tag: controller.run(
            sumocfg=cfg, seed=seed, ns_green_override=ns_g,
            tripinfo_out=f"results/tripinfo_{tag}_fuzzy.xml",
            metrics_out=f"results/metrics_{tag}_fuzzy.json"),
    }
    sweep = {p: {"ns_green": [], "duty": [], "capacity_vph_lane": [],
                "delay": [], "throughput": [], "box_gridlock": [],
                "storage_overflow": []} for p in policies}

    for ns_g in NS_GREEN_VALUES:
        per = {p: {"delay": [], "throughput": [], "box_gridlock": [], "storage_overflow": []}
              for p in policies}
        for seed in SEEDS:
            tag = f"bneck_ns{ns_g}_seed{seed}"
            cfg, n = make_config(tag, scale=1.0, seed=seed, ambulance=False)
            print(f"--- bottleneck sweep ns_green={ns_g}s seed={seed} ({n} vehicles) ---")
            for name, runner in policies.items():
                r = runner(cfg, seed, ns_g, tag)
                per[name]["delay"].append(r["avg_delay_s"])
                per[name]["throughput"].append(r["throughput_completed_trips"])
                per[name]["box_gridlock"].append(r["box_gridlock_events"])
                per[name]["storage_overflow"].append(r["storage_overflow_events"])
        for name in policies:
            sweep[name]["ns_green"].append(ns_g)
            sweep[name]["duty"].append(ns_g / 50.0)
            sweep[name]["capacity_vph_lane"].append(ns_capacity_vph_per_lane(ns_g))
            for k in ("delay", "throughput", "box_gridlock", "storage_overflow"):
                sweep[name][k].append(float(np.mean(per[name][k])))
    return sweep


def plot(sweep, path="results/bottleneck_sweep.png"):
    fig, axes = plt.subplots(1, 4, figsize=(20, 4.5))
    metrics = [("delay", "Average Delay [s]"), ("throughput", "Throughput [veh]"),
              ("box_gridlock", "Box Gridlock Events"), ("storage_overflow", "Storage Overflow Events")]
    for ax, (key, title) in zip(axes, metrics):
        for name, data in sweep.items():
            ax.plot(data["duty"], data[key], marker="o", label=name, color=COLORS.get(name, "#777"))
        ax.set_xlabel("NS egress duty ratio (green/cycle)")
        ax.set_title(title, fontsize=10, fontweight="bold")
        ax.grid(alpha=0.25, linestyle=":")
        ax.invert_xaxis()  # severity increases leftward (lower duty = tighter bottleneck)
    axes[0].legend(fontsize=8)
    fig.suptitle("Bottleneck-severity sweep (fixed demand, scale=1.0, 3-seed average)\n"
                "x-axis reversed: bottleneck tightens left-to-right",
                fontsize=12, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.90])
    fig.savefig(path, dpi=160)
    print(f"[chart] {path}")


def main():
    os.makedirs("results", exist_ok=True)
    sweep = run_sweep()
    plot(sweep)
    with open("results/bottleneck_sweep_report.json", "w") as f:
        json.dump(sweep, f, indent=2)
    print("Full report: results/bottleneck_sweep_report.json")


if __name__ == "__main__":
    main()
