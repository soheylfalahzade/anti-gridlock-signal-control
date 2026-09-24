"""Vanilla Max-Pressure (Varaiya, 2013), full strength, no spillback awareness."""

import argparse
import os
import sys

import traci

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.metrics import MetricsCollector
from src.controller import (TLS_ID, PHASE_UP, PHASE_DOWN, OPPOSITE,
                            build_state_strings, step, switch, queue)
from baselines.fixed_time import start_sumo

DECISION_INTERVAL, MIN_GREEN = 5, 10


def mp_pressure(group):
    return queue(PHASE_UP[group]) - queue(PHASE_DOWN[group])


def run(sumocfg="configs/intersection.sumocfg", gui=False, seed=1, verbose=False,
        sim_end=3600, regime="moderate", ns_green_override=None, ew_green_override=30,
        tripinfo_out="results/tripinfo_vanilla_mp.xml",
        metrics_out="results/metrics_vanilla_mp.json"):
    start_sumo(sumocfg, tripinfo_out, gui, seed, regime, ns_green_override, ew_green_override)
    states = build_state_strings()
    metrics = MetricsCollector()

    phase = "NS"
    traci.trafficlight.setRedYellowGreenState(TLS_ID, states["NS_green"])
    elapsed, inserted = 0, 0

    while traci.simulation.getTime() < sim_end:
        step(metrics)
        inserted += traci.simulation.getDepartedNumber()
        elapsed += 1
        if elapsed >= MIN_GREEN and elapsed % DECISION_INTERVAL == 0:
            other = OPPOSITE[phase]
            if mp_pressure(other) > mp_pressure(phase):
                switch(states, metrics, phase, other)
                phase, elapsed = other, 0

    pending = traci.simulation.getMinExpectedNumber() - traci.vehicle.getIDCount()
    traci.close()
    results = metrics.finalize(tripinfo_out, inserted, max(pending, 0))
    results["policy"] = "Vanilla Max-Pressure"
    MetricsCollector.save(results, metrics_out)
    print(f"[Vanilla Max-Pressure:{regime}] delay={results['avg_delay_s']:.1f}s "
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
    p.add_argument("--regime", default="moderate", choices=["moderate", "stress"])
    run(**vars(p.parse_args()))
