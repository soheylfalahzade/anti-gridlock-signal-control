"""Webster delay-optimal fixed-time baseline (no emergency preemption)."""

import argparse
import os
import sys

import traci
import sumolib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.metrics import MetricsCollector
from src.controller import TLS_ID, YELLOW_TIME, ALL_RED_TIME, build_state_strings, step
from src.metering import apply_metering, apply_metering_custom, CAPACITY_VPH_PER_LANE

MIN_GREEN = 10.0
FLOW_EW_PER_DIR = 200.0


def webster_plan(scale=1.0, regime="moderate", ns_green_override=None, ew_green_override=30):
    lanes = 2
    if ns_green_override is not None:
        cap_ns_per_lane = 1900 * ns_green_override / 50.0
        cap_ew_per_lane = 1900 * ew_green_override / 50.0
    else:
        cap_ns_per_lane = CAPACITY_VPH_PER_LANE[regime]["NS"]
        cap_ew_per_lane = CAPACITY_VPH_PER_LANE[regime]["EW"]
    flow_ns = min(750.0 * scale, cap_ns_per_lane * lanes * 0.98)
    flow_ew = min(FLOW_EW_PER_DIR * scale, cap_ew_per_lane * lanes * 0.98)
    y_ns, y_ew = flow_ns / (1900.0 * lanes), flow_ew / (1900.0 * lanes)
    Y = min(y_ns + y_ew, 0.95)
    L = 2 * (YELLOW_TIME + ALL_RED_TIME)
    c = min(max((1.5 * L + 5.0) / max(1e-6, 1.0 - Y), 60.0), 120.0)
    eff = c - L
    return {"g_ns": max(MIN_GREEN, eff * y_ns / (y_ns + y_ew)),
            "g_ew": max(MIN_GREEN, eff * y_ew / (y_ns + y_ew)), "cycle": c}


def start_sumo(sumocfg, tripinfo_out, gui, seed, regime,
               ns_green_override=None, ew_green_override=30):
    binary = sumolib.checkBinary("sumo-gui" if gui else "sumo")
    os.makedirs(os.path.dirname(tripinfo_out), exist_ok=True)
    cmd = [binary, "-c", sumocfg, "--seed", str(seed), "--tripinfo-output", tripinfo_out,
          "--tripinfo-output.write-unfinished", "true",
          "--no-step-log", "true", "--duration-log.disable", "true", "--no-warnings", "true"]
    traci.start(cmd)
    if ns_green_override is not None:
        apply_metering_custom(ns_green_override, ew_green_override)
    else:
        apply_metering(regime)


def run(sumocfg="configs/intersection.sumocfg", gui=False, seed=1, verbose=False,
        scale=1.0, sim_end=3600, regime="moderate",
        ns_green_override=None, ew_green_override=30,
        tripinfo_out="results/tripinfo_fixed_time.xml",
        metrics_out="results/metrics_fixed_time.json"):
    plan = webster_plan(scale, regime, ns_green_override, ew_green_override)
    start_sumo(sumocfg, tripinfo_out, gui, seed, regime, ns_green_override, ew_green_override)
    states = build_state_strings()
    metrics = MetricsCollector()

    schedule = [("NS_green", plan["g_ns"]), ("NS_yellow", YELLOW_TIME),
               ("all_red", ALL_RED_TIME), ("EW_green", plan["g_ew"]),
               ("EW_yellow", YELLOW_TIME), ("all_red", ALL_RED_TIME)]
    idx, remaining, inserted = 0, schedule[0][1], 0
    traci.trafficlight.setRedYellowGreenState(TLS_ID, states[schedule[0][0]])

    while traci.simulation.getTime() < sim_end:
        step(metrics)
        inserted += traci.simulation.getDepartedNumber()
        remaining -= 1
        if remaining <= 0:
            idx = (idx + 1) % len(schedule)
            traci.trafficlight.setRedYellowGreenState(TLS_ID, states[schedule[idx][0]])
            remaining = schedule[idx][1]

    pending = traci.simulation.getMinExpectedNumber() - traci.vehicle.getIDCount()
    traci.close()
    results = metrics.finalize(tripinfo_out, inserted, max(pending, 0))
    results["policy"] = "Fixed-Time (Webster)"
    MetricsCollector.save(results, metrics_out)
    print(f"[Fixed-Time:{regime}] delay={results['avg_delay_s']:.1f}s "
         f"queue={results['mean_queue_length']:.1f} "
         f"throughput={results['throughput_completed_trips']} "
         f"box_gridlock={results['box_gridlock_events']} "
         f"storage_overflow={results['storage_overflow_events']}")
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--sumocfg", default="configs/intersection.sumocfg")
    p.add_argument("--gui", action="store_true")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--regime", default="moderate", choices=["moderate", "stress"])
    run(**vars(p.parse_args()))
