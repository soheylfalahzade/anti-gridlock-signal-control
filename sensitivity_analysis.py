"""
Sensitivity analysis of the two most consequential hand-set design choices
in the fuzzy controller: OCC_OVERRIDE (the hard-override threshold) and the
"critical" membership function's position (critical_shift). Addresses the
review requirement that hand-tuned parameters not be presented as fixed
given without characterizing how sensitive the results are to them.

Uses 3 seeds per configuration (not 10) for tractable runtime; this is
reported as an indicative sensitivity sweep, not a re-run of the full
statistically-powered benchmark.
"""

import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src import controller
from src.demand import make_config

SEEDS = [1, 2, 3]
OCC_OVERRIDE_VALUES = [0.65, 0.70, 0.75, 0.80, 0.85]
CRITICAL_SHIFT_VALUES = [-0.10, -0.05, 0.0, 0.05, 0.10]


def run_occ_override_sweep():
    results = {"values": OCC_OVERRIDE_VALUES, "delay": [], "box_gridlock": [],
              "storage_overflow": [], "throughput": []}
    for occ in OCC_OVERRIDE_VALUES:
        delays, boxes, storages, throughputs = [], [], [], []
        for seed in SEEDS:
            tag = f"sens_occ{str(occ).replace('.', 'p')}_seed{seed}"
            cfg, _ = make_config(tag, scale=1.0, seed=seed, ambulance=False)
            r = controller.run(sumocfg=cfg, seed=seed, occ_override=occ,
                               tripinfo_out=f"results/tripinfo_{tag}.xml",
                               metrics_out=f"results/metrics_{tag}.json")
            delays.append(r["avg_delay_s"])
            boxes.append(r["box_gridlock_events"])
            storages.append(r["storage_overflow_events"])
            throughputs.append(r["throughput_completed_trips"])
        results["delay"].append(float(np.mean(delays)))
        results["box_gridlock"].append(float(np.mean(boxes)))
        results["storage_overflow"].append(float(np.mean(storages)))
        results["throughput"].append(float(np.mean(throughputs)))
    return results


def run_critical_shift_sweep():
    results = {"values": CRITICAL_SHIFT_VALUES, "delay": [], "box_gridlock": [],
              "storage_overflow": [], "throughput": []}
    for shift in CRITICAL_SHIFT_VALUES:
        delays, boxes, storages, throughputs = [], [], [], []
        for seed in SEEDS:
            tag = f"sens_shift{str(shift).replace('.', 'p').replace('-', 'neg')}_seed{seed}"
            cfg, _ = make_config(tag, scale=1.0, seed=seed, ambulance=False)
            r = controller.run(sumocfg=cfg, seed=seed, critical_shift=shift,
                               tripinfo_out=f"results/tripinfo_{tag}.xml",
                               metrics_out=f"results/metrics_{tag}.json")
            delays.append(r["avg_delay_s"])
            boxes.append(r["box_gridlock_events"])
            storages.append(r["storage_overflow_events"])
            throughputs.append(r["throughput_completed_trips"])
        results["delay"].append(float(np.mean(delays)))
        results["box_gridlock"].append(float(np.mean(boxes)))
        results["storage_overflow"].append(float(np.mean(storages)))
        results["throughput"].append(float(np.mean(throughputs)))
    return results


def plot_sensitivity(occ_results, shift_results, path="results/sensitivity_analysis.png"):
    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    metrics = [("delay", "Avg Delay [s]"), ("box_gridlock", "Box Gridlock Events"),
              ("storage_overflow", "Storage Overflow Events")]

    for ax, (key, title) in zip(axes[0], metrics):
        ax.plot(occ_results["values"], occ_results[key], marker="o", color="#2a9d8f")
        ax.axvline(0.75, color="gray", linestyle="--", alpha=0.6, label="benchmark value")
        ax.set_xlabel("OCC_OVERRIDE threshold")
        ax.set_title(title, fontsize=10, fontweight="bold")
        ax.grid(alpha=0.25, linestyle=":")
        ax.legend(fontsize=7)

    for ax, (key, title) in zip(axes[1], metrics):
        ax.plot(shift_results["values"], shift_results[key], marker="o", color="#e76f51")
        ax.axvline(0.0, color="gray", linestyle="--", alpha=0.6, label="benchmark value")
        ax.set_xlabel("critical MF shift")
        ax.set_title(title, fontsize=10, fontweight="bold")
        ax.grid(alpha=0.25, linestyle=":")
        ax.legend(fontsize=7)

    fig.suptitle("Sensitivity Analysis: OCC_OVERRIDE (top) and Critical-MF Shift (bottom)\n"
                 "3-seed average per point, moderate regime, scale=1.0",
                fontsize=12, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(path, dpi=160)
    print(f"[chart] {path}")


def main():
    os.makedirs("results", exist_ok=True)
    print("=== Sensitivity: OCC_OVERRIDE ===")
    occ_results = run_occ_override_sweep()
    print("=== Sensitivity: critical MF shift ===")
    shift_results = run_critical_shift_sweep()
    plot_sensitivity(occ_results, shift_results)
    with open("results/sensitivity_report.json", "w") as f:
        json.dump({"occ_override_sweep": occ_results,
                  "critical_shift_sweep": shift_results}, f, indent=2)
    print("Full report: results/sensitivity_report.json")


if __name__ == "__main__":
    main()
