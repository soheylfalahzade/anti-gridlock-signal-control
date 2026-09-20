"""
Vanilla Max-Pressure baseline (Varaiya, 2013).

Textbook formulation, deliberately implemented at full strength: the phase
maximizing the pressure differential (upstream queue minus downstream queue,
in vehicles) is activated at every decision epoch, subject only to a minimum
green. No max-green cap and no occupancy feedback exist in the original
algorithm, and none are added here. Its failure on short downstream links is
a property of the algorithm under this geometry, not of this implementation:
a 50 m egress saturates at a handful of vehicles, so the pressure term stays
strongly positive for the arterial long after the receiving link can no
longer accept discharge.
"""

import argparse
import os
import sys

import traci

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.metrics import MetricsCollector
from src.controller import (TLS_ID, SIM_STEPS, PHASE_UP, PHASE_DOWN, OPPOSITE,
                            build_state_strings, start_sumo, step, switch, queue)

DECISION_INTERVAL = 5
MIN_GREEN = 10


def mp_pressure(group):
    """Classic max-pressure weight in vehicles."""
    return queue(PHASE_UP[group]) - queue(PHASE_DOWN[group])


def run(sumocfg="configs/intersection.sumocfg", gui=False, seed=1, verbose=False,
        tripinfo_out="results/tripinfo_vanilla_mp.xml",
        metrics_out="results/metrics_vanilla_mp.json"):
    start_sumo(sumocfg, tripinfo_out, gui, seed)
    states = build_state_strings()
    metrics = MetricsCollector()

    phase = "NS"
    traci.trafficlight.setRedYellowGreenState(TLS_ID, states["NS_green"])
    elapsed, inserted = 0, 0

    while traci.simulation.getTime() < SIM_STEPS:
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
    print(f"[Vanilla Max-Pressure] delay={results['avg_delay_s']:.1f}s  "
          f"queue={results['mean_queue_length']:.1f}  "
          f"throughput={results['throughput_completed_trips']}  "
          f"spillbacks={results['spillback_occurrences']}")
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--sumocfg", default="configs/intersection.sumocfg")
    p.add_argument("--gui", action="store_true")
    p.add_argument("--seed", type=int, default=1)
    run(**vars(p.parse_args()))
