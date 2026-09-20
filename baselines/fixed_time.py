"""Webster delay-optimal fixed-time baseline (no emergency preemption)."""

import argparse
import os
import sys

import traci

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.metrics import MetricsCollector
from src.controller import TLS_ID, YELLOW_TIME, ALL_RED_TIME, build_state_strings, start_sumo, step

SAT_PER_LANE, LANES = 1900.0, 2
FLOW_NS, FLOW_EW = 750.0, 200.0
MIN_GREEN = 10.0


def webster_plan(scale=1.0):
    cap = SAT_PER_LANE * LANES
    y_ns, y_ew = (FLOW_NS * scale) / cap, (FLOW_EW * scale) / cap
    Y = min(y_ns + y_ew, 0.95)
    L = 2 * (YELLOW_TIME + ALL_RED_TIME)
    c = min(max((1.5 * L + 5.0) / max(1e-6, 1.0 - Y), 60.0), 120.0)
    eff = c - L
    return {"g_ns": max(MIN_GREEN, eff * y_ns / (y_ns + y_ew)),
            "g_ew": max(MIN_GREEN, eff * y_ew / (y_ns + y_ew)), "cycle": c}


def run(sumocfg="configs/intersection.sumocfg", gui=False, seed=1, verbose=False,
        scale=1.0, sim_end=3600,
        tripinfo_out="results/tripinfo_fixed_time.xml",
        metrics_out="results/metrics_fixed_time.json"):
    plan = webster_plan(scale)
    start_sumo(sumocfg, tripinfo_out, gui, seed)
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
    print(f"[Fixed-Time] delay={results['avg_delay_s']:.1f}s "
         f"queue={results['mean_queue_length']:.1f} "
         f"throughput={results['throughput_completed_trips']} "
         f"spillbacks={results['spillback_occurrences']}")
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--sumocfg", default="configs/intersection.sumocfg")
    p.add_argument("--gui", action="store_true")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--scale", type=float, default=1.0)
    run(**vars(p.parse_args()))
