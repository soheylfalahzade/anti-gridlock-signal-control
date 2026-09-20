"""
Fuzzy Anti-Spillback Max-Pressure controller.

start_sumo() now passes --tripinfo-output.write-unfinished true, so an
ambulance still en route at sim end is still recorded (with its
in-progress timeLoss) instead of being silently absent from tripinfo.xml,
which was the direct cause of every emergency_delay_s being null.
"""

import argparse
import os
import sys
from collections import deque

import traci
import sumolib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.fuzzy_engine import FuzzyAntiSpillbackEngine, T_MIN
from src.metrics import MetricsCollector

TLS_ID = "C"
SAT_FLOW_VPH_PER_LANE = 1900.0
YELLOW_TIME = 3
ALL_RED_TIME = 2
OCC_OVERRIDE = 0.75
PREEMPT_MAX_HOLD = 45.0
OCC_SMOOTH_WINDOW = 50

PHASE_UP = {"NS": ["app_N", "in_N", "app_S", "in_S"], "EW": ["app_E", "in_E", "app_W", "in_W"]}
PHASE_STOPLINE = {"NS": ["in_N", "in_S"], "EW": ["in_E", "in_W"]}
PHASE_DOWN = {"NS": ["out_S", "out_N"], "EW": ["out_W", "out_E"]}
OPPOSITE = {"NS": "EW", "EW": "NS"}
DIR_GROUP = {"N": "NS", "S": "NS", "E": "EW", "W": "EW"}


def controlled_incoming_edges(tls_id=TLS_ID):
    edges = []
    for link in traci.trafficlight.getControlledLinks(tls_id):
        edges.append(traci.lane.getEdgeID(link[0][0]) if link else None)
    return edges


def build_state_strings(tls_id=TLS_ID):
    edges = controlled_incoming_edges(tls_id)

    def mk(group, ch):
        return "".join(ch if e in PHASE_STOPLINE[group] else "r" for e in edges)

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
    up = discharge_time(PHASE_UP[group], PHASE_STOPLINE[group])
    down = discharge_time(PHASE_DOWN[group], PHASE_DOWN[group])
    return up - down


def downstream_occupancy(group):
    return max(traci.edge.getLastStepOccupancy(e) / 100.0 for e in PHASE_DOWN[group])


class _OccupancySmoother:
    def __init__(self, window=OCC_SMOOTH_WINDOW):
        self.hist = {"NS": deque(maxlen=window), "EW": deque(maxlen=window)}

    def update(self):
        self.hist["NS"].append(downstream_occupancy("NS"))
        self.hist["EW"].append(downstream_occupancy("EW"))

    def value(self, group):
        h = self.hist[group]
        return sum(h) / len(h) if h else 0.0


def detect_emergency_group():
    for vid in traci.vehicle.getIDList():
        if not vid.startswith("amb"):
            continue
        edge = traci.vehicle.getRoadID(vid)
        if edge and not edge.startswith(":"):
            d = edge.split("_")[-1]
            if d in DIR_GROUP:
                return DIR_GROUP[d], vid, edge
    return None, None, None


def start_sumo(sumocfg, tripinfo_out, gui=False, seed=1):
    binary = sumolib.checkBinary("sumo-gui" if gui else "sumo")
    os.makedirs(os.path.dirname(tripinfo_out), exist_ok=True)
    cmd = [binary, "-c", sumocfg, "--seed", str(seed),
          "--tripinfo-output", tripinfo_out,
          "--tripinfo-output.write-unfinished", "true",
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


def run(sumocfg="configs/intersection.sumocfg", gui=False, seed=1, verbose=False,
        emergency_preempt=True, sim_end=3600,
        tripinfo_out="results/tripinfo_fuzzy.xml",
        metrics_out="results/metrics_fuzzy.json"):
    start_sumo(sumocfg, tripinfo_out, gui, seed)
    states = build_state_strings()
    engine = FuzzyAntiSpillbackEngine()
    metrics = MetricsCollector()
    smoother = _OccupancySmoother()

    def do_step():
        step(metrics)
        smoother.update()

    phase = "NS"
    traci.trafficlight.setRedYellowGreenState(TLS_ID, states["NS_green"])
    remaining = T_MIN
    overrides, preemptions = 0, 0
    inserted = 0
    preempt_active, preempt_hold = False, 0.0

    while traci.simulation.getTime() < sim_end:
        if emergency_preempt and not preempt_active:
            grp, vid, edge = detect_emergency_group()
            if grp is not None and grp != phase:
                if verbose:
                    print(f"{traci.simulation.getTime():6.0f}  EMERGENCY PREEMPT -> {grp} for {vid} on {edge}")
                switch(states, metrics, phase, grp)
                phase = grp
                preempt_active, preempt_hold = True, PREEMPT_MAX_HOLD
                preemptions += 1
                inserted += traci.simulation.getDepartedNumber()
                continue

        if preempt_active:
            grp, _, _ = detect_emergency_group()
            do_step()
            inserted += traci.simulation.getDepartedNumber()
            preempt_hold -= 1
            if grp != phase or preempt_hold <= 0:
                preempt_active = False
                remaining = T_MIN
            continue

        do_step()
        inserted += traci.simulation.getDepartedNumber()
        remaining -= 1
        if remaining > 0:
            continue

        other = OPPOSITE[phase]
        w_cur, w_oth = phase_pressure(phase), phase_pressure(other)
        occ_cur_s, occ_oth_s = smoother.value(phase), smoother.value(other)
        nxt = phase if w_cur >= w_oth else other

        note = "fuzzy"
        occ_nxt_s = occ_cur_s if nxt == phase else occ_oth_s
        if occ_nxt_s > OCC_OVERRIDE:
            alt = other if nxt == phase else phase
            occ_alt_s = occ_oth_s if nxt == phase else occ_cur_s
            if occ_alt_s <= OCC_OVERRIDE:
                nxt, note = alt, "ANTI-SPILLBACK rotate"
            else:
                note = "ANTI-SPILLBACK hold@Tmin"

        w_sel = w_cur if nxt == phase else w_oth
        occ_sel = occ_cur_s if nxt == phase else occ_oth_s
        green = engine.compute(w_sel, occ_sel)
        if note != "fuzzy":
            green = T_MIN
            overrides += 1

        if verbose:
            print(f"{traci.simulation.getTime():6.0f} {nxt:>6} w={w_sel:7.2f} "
                 f"occ(smoothed)={occ_sel*100:5.1f}% green={green:5.1f}  {note}")

        if nxt != phase:
            switch(states, metrics, phase, nxt)
            phase = nxt
        remaining = green

    pending = traci.simulation.getMinExpectedNumber() - traci.vehicle.getIDCount()
    traci.close()

    results = metrics.finalize(tripinfo_out, inserted, max(pending, 0))
    results.update(policy="Fuzzy Anti-Spillback", anti_spillback_overrides=overrides,
                   emergency_preemptions=preemptions)
    MetricsCollector.save(results, metrics_out)
    print(f"[Fuzzy Anti-Spillback] delay={results['avg_delay_s']:.1f}s "
         f"queue={results['mean_queue_length']:.1f} "
         f"throughput={results['throughput_completed_trips']} "
         f"spillbacks={results['spillback_occurrences']} "
         f"teleports={results['deadlock_teleports']} "
         f"preemptions={preemptions}")
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--sumocfg", default="configs/intersection.sumocfg")
    p.add_argument("--gui", action="store_true")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--quiet", dest="verbose", action="store_false")
    p.add_argument("--no-preempt", dest="emergency_preempt", action="store_false")
    run(**vars(p.parse_args()))
