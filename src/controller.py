"""
Fuzzy Anti-Spillback Max-Pressure controller (TraCI).

Phase selection uses the queue-dissipation pressure
    w_p = t_D^p - t_D^q
with t_D obtained from measured halting queues at saturation flow. The green
duration for the selected phase is produced by the Mamdani FIS, which
throttles green as the receiving egress link approaches saturation.

A hard anti-spillback override sits on top of the FIS: once the receiving
link exceeds 75 % occupancy, the phase is held at T_min and the controller
rotates to the opposing movement so the cross street cannot be gridlocked by
a queue that physically cannot discharge.

This module also exposes the shared simulation primitives used by the two
baselines so that every policy runs on identical instrumentation.
"""

import argparse
import os
import sys

import traci
import sumolib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.fuzzy_engine import FuzzyAntiSpillbackEngine, T_MIN
from src.metrics import MetricsCollector

TLS_ID = "C"
SAT_FLOW_VPH_PER_LANE = 1900.0
YELLOW_TIME = 3
ALL_RED_TIME = 2
SIM_STEPS = 3600
OCC_OVERRIDE = 0.75

PHASE_UP = {"NS": ["app_N", "in_N", "app_S", "in_S"],
            "EW": ["app_E", "in_E", "app_W", "in_W"]}
PHASE_STOPLINE = {"NS": ["in_N", "in_S"], "EW": ["in_E", "in_W"]}
PHASE_DOWN = {"NS": ["out_S", "out_N"], "EW": ["out_W", "out_E"]}
OPPOSITE = {"NS": "EW", "EW": "NS"}


def controlled_incoming_edges(tls_id=TLS_ID):
    edges = []
    for link in traci.trafficlight.getControlledLinks(tls_id):
        edges.append(traci.lane.getEdgeID(link[0][0]) if link else None)
    return edges


def build_state_strings(tls_id=TLS_ID):
    edges = controlled_incoming_edges(tls_id)

    def mk(group, char):
        return "".join(char if e in PHASE_STOPLINE[group] else "r" for e in edges)

    return {"NS_green": mk("NS", "G"), "EW_green": mk("EW", "G"),
            "NS_yellow": mk("NS", "y"), "EW_yellow": mk("EW", "y"),
            "all_red": "r" * len(edges)}


def queue(edges):
    return sum(traci.edge.getLastStepHaltingNumber(e) for e in edges)


def discharge_time(edges, stopline_edges):
    lanes = sum(traci.edge.getLaneNumber(e) for e in stopline_edges)
    rate = SAT_FLOW_VPH_PER_LANE * max(lanes, 1) / 3600.0
    return queue(edges) / rate


def phase_pressure(group):
    """w_p = upstream discharge time - downstream discharge time [s]."""
    up = discharge_time(PHASE_UP[group], PHASE_STOPLINE[group])
    down = discharge_time(PHASE_DOWN[group], PHASE_DOWN[group])
    return up - down


def downstream_occupancy(group):
    return max(traci.edge.getLastStepOccupancy(e) / 100.0 for e in PHASE_DOWN[group])


def start_sumo(sumocfg, tripinfo_out, gui=False, seed=1):
    binary = sumolib.checkBinary("sumo-gui" if gui else "sumo")
    os.makedirs(os.path.dirname(tripinfo_out), exist_ok=True)
    cmd = [binary, "-c", sumocfg, "--seed", str(seed),
           "--tripinfo-output", tripinfo_out,
           "--no-step-log", "true", "--duration-log.disable", "true",
           "--no-warnings", "true"]
    if gui:
        cmd += ["--start", "true", "--quit-on-end", "true"]
    traci.start(cmd)


def step(metrics):
    traci.simulationStep()
    metrics.sample_step()


def switch(states, metrics, frm, to):
    traci.trafficlight.setRedYellowGreenState(TLS_ID, states[f"{frm}_yellow"])
    for _ in range(YELLOW_TIME):
        step(metrics)
    traci.trafficlight.setRedYellowGreenState(TLS_ID, states["all_red"])
    for _ in range(ALL_RED_TIME):
        step(metrics)
    traci.trafficlight.setRedYellowGreenState(TLS_ID, states[f"{to}_green"])


def run(sumocfg="configs/intersection.sumocfg", gui=False, seed=1, verbose=True,
        tripinfo_out="results/tripinfo_fuzzy.xml",
        metrics_out="results/metrics_fuzzy.json"):
    start_sumo(sumocfg, tripinfo_out, gui, seed)
    states = build_state_strings()
    engine = FuzzyAntiSpillbackEngine()
    metrics = MetricsCollector()

    phase = "NS"
    traci.trafficlight.setRedYellowGreenState(TLS_ID, states["NS_green"])
    remaining = T_MIN
    overrides = 0
    inserted = 0

    if verbose:
        print(f"{'t[s]':>6} {'phase':>6} {'q_NS':>5} {'q_EW':>5} "
              f"{'w_p[s]':>7} {'occ_dn':>7} {'green':>6}  note")

    while traci.simulation.getTime() < SIM_STEPS:
        step(metrics)
        inserted += traci.simulation.getDepartedNumber()
        remaining -= 1
        if remaining > 0:
            continue

        other = OPPOSITE[phase]
        w_cur, w_oth = phase_pressure(phase), phase_pressure(other)
        occ_cur, occ_oth = downstream_occupancy(phase), downstream_occupancy(other)

        # Max-pressure phase selection on the queue-dissipation differential.
        nxt = phase if w_cur >= w_oth else other

        # Hard anti-spillback override: never keep serving a movement whose
        # receiving link cannot accept the discharge.
        note = "fuzzy"
        if downstream_occupancy(nxt) > OCC_OVERRIDE:
            if downstream_occupancy(other if nxt == phase else phase) <= OCC_OVERRIDE:
                nxt = other if nxt == phase else phase
                note = "ANTI-SPILLBACK rotate"
            else:
                note = "ANTI-SPILLBACK hold@Tmin"

        w_sel = w_cur if nxt == phase else w_oth
        occ_sel = occ_cur if nxt == phase else occ_oth
        green = engine.compute(w_sel, occ_sel)
        if note != "fuzzy":
            green = T_MIN
            overrides += 1

        if verbose:
            print(f"{traci.simulation.getTime():6.0f} {nxt:>6} "
                  f"{queue(PHASE_UP['NS']):5d} {queue(PHASE_UP['EW']):5d} "
                  f"{w_sel:7.2f} {occ_sel * 100:6.1f}% {green:6.1f}  {note}")

        if nxt != phase:
            switch(states, metrics, phase, nxt)
            phase = nxt
        remaining = green

    pending = traci.simulation.getMinExpectedNumber() - traci.vehicle.getIDCount()
    traci.close()

    results = metrics.finalize(tripinfo_out, inserted, max(pending, 0))
    results["anti_spillback_overrides"] = overrides
    results["policy"] = "Fuzzy Anti-Spillback"
    MetricsCollector.save(results, metrics_out)
    print(f"[Fuzzy Anti-Spillback] delay={results['avg_delay_s']:.1f}s  "
          f"queue={results['mean_queue_length']:.1f}  "
          f"throughput={results['throughput_completed_trips']}  "
          f"spillbacks={results['spillback_occurrences']}")
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--sumocfg", default="configs/intersection.sumocfg")
    p.add_argument("--gui", action="store_true")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--quiet", dest="verbose", action="store_false")
    run(**vars(p.parse_args()))
