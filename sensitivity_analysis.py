"""
Sensitivity analysis of the two most consequential hand-set design choices
in the fuzzy controller.

Range fix: OCC_OVERRIDE_VALUES was previously [0.65, 0.85], entirely above
the occupancy range actually observed at moderate-regime scale=1.0
(0-54%, confirmed via diagnose_occupancy.py and the corrected controller's
own telemetry) -- so even with the occupancy-scaling bug fixed, that sweep
would have stayed uninformative for a different reason (wrong range, not a
bug). The range now spans the observed occupancy distribution so the
sweep can actually show where sensitivity exists.
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
OCC_OVERRIDE_VALUES = [0.35, 0.45, 0.55, 0.65, 0.75, 0.85]
CRITICAL_SHIFT_VALUES = [-0.20, -0.10, -0.05, 0.0, 0.05, 0.10]


def run_occ_override_sweep():
    results = {"values": OCC_OVERRIDE_VALUES, "delay": [], "box_gridlock": [],
              "storage_overflow": [], "throughput": [], "overrides": []}
    for occ in OCC_OVERRIDE_VALUES:
        delays, boxes, storages, throughputs, overrides_list = [], [], [], [], []
        for seed in SEEDS:
            tag = f"sens_occ{str(occ).replace('.', 'p')}_seed{seed}"
            cfg, _ = make_config(tag, scale=1.0, seed=seed, ambulance=False)
            r = controller.run(sumocfg=cfg, seed=seed, occ_override=occ,
                               verbose=False,
                               tripinfo_out=f"results/tripinfo_{tag}.xml",
                               metrics_out=f"results/metrics_{tag}.json")
            delays.append(r["avg_delay_s"])
            boxes.append(r["box_gridlock_events"])
            storages.append(r["storage_overflow_events"])
            throughputs.append(r["throughput_completed_trips"])
            overrides_list.append(r["anti_spillback_overrides"])
        results["delay"].append(float(np.mean(delays)))
        results["box_gridlock"].append(float(np.mean(boxes)))
        results["storage_overflow"].append(float(np.mean(storages)))
        results["throughput"].append(float(np.mean(throughputs)))
        results["overrides"].append(float(np.mean(overrides_list)))
    return results


def run_critical_shift_sweep():
    results = {"values": CRITICAL_SHIFT_VALUES, "delay": [], "box_gridlock": [],
              "storage_overflow": [], "throughput": [], "overrides": []}
    for shift in CRITICAL_SHIFT_VALUES:
        delays, boxes, storages, throughputs, overrides_list = [], [], [], [], []
        for seed in SEEDS:
            tag = f"sens_shift{str(shift).replace('.', 'p').replace('-', 'neg')}_seed{seed}"
            cfg, _ = make_config(tag, scale=1.0, seed=seed, ambulance=False)
            r = controller.run(sumocfg=cfg, seed=seed, critical_shift=shift,
                               verbose=False,
                               tripinfo_out=f"results/tripinfo_{tag}.xml",
                               metrics_out=f"results/metrics_{tag}.json")
            delays.append(r["avg_delay_s"])
            boxes.append(r["box_gridlock_events"])
            storages.append(r["storage_overflow_events"])
            throughputs.append(r["throughput_completed_trips"])
            overrides_list.append(r["anti_spillback_overrides"])
        results["delay"].append(float(np.mean(delays)))
        results["box_gridlock"].append(float(np.mean(boxes)))
        results["storage_overflow"].append(float(np.mean(storages)))
        results["throughput"].append(float(np.mean(throughputs)))
        results["overrides"].append(float(np.mean(overrides_list)))
    return results


def plot_sensitivity(occ_results, shift_results, path="results/sensitivity_analysis.png"):
    fig, axes = plt.subplots(2, 4, figsize=(20, 8))
    metrics = [("delay", "Avg Delay [s]"), ("box_gridlock", "Box Gridlock Events"),
              ("storage_overflow", "Storage Overflow Events"), ("overrides", "Anti-Spillback Overrides")]

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

    fig.suptitle("Sensitivity Analysis (post occupancy-scaling fix): OCC_OVERRIDE (top), "
                 "Critical-MF Shift (bottom)\n3-seed average, moderate regime, scale=1.0",
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
