#!/usr/bin/env python3
"""Three-junction corridor: does the local anti-spillback layer still help when
spillback can propagate between adjacent signalised junctions?

Scope (proof of concept, not a network-control result):
  * three junctions C1-C3 on an east-west arterial, 200 m apart, each with a
    north and south cross street; through movements only
  * the same engineered bottleneck as the single-junction study, at the east
    exit only (metered, 1 lane, 50 m) so queues can back up C3 -> C2 -> C1
  * every junction runs its OWN controller with NO communication and no
    offsets: Webster fixed-time, Vanilla Max-Pressure, or the fuzzy
    anti-spillback controller (same decision logic as src/controller.py)
  * no emergency vehicle, to keep the comparison free of preemption effects

Run from the repository root:
    python multi/corridor.py build
    python multi/corridor.py run --policy fuzzy --seed 401 --regime moderate
"""
import argparse
import math
import os
import random
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import deque

import sumolib
import traci

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from src.fuzzy_engine import FuzzyAntiSpillbackEngine, T_MIN  # noqa: E402

GEN = os.path.join(ROOT, "multi", "gen")
NOD, EDG, CON, TLL, NET = (os.path.join(GEN, f"corridor.{e}.xml") for e in ("nod", "edg", "con", "tll", "net"))

SPACING, R_OUTER, R_METER = 200.0, 250.0, 50.0
V_ART, V_BOT = 13.89, 8.33
K = 3
XS = [i * SPACING for i in range(K)]

SIM_END = 3600
SAT_FLOW = 1900.0
YELLOW, ALL_RED = 3, 2
OCC_OVERRIDE = 0.75
OCC_WINDOW = 50
MIN_GREEN, MP_INTERVAL = 10, 5

METER_PLANS = {"moderate": (30, 3, 17), "stress": (20, 3, 27)}   # M_E: green, yellow, red (s)
EB_VPH, WB_VPH, NS_VPH = 1000.0, 350.0, 350.0
BURST_FACTOR, BURST_LEN, BURST_STARTS = 2.0, 300, [500, 1400, 2400]

LINKS = ["lk_E1", "lk_E2", "lk_W2", "lk_W3"]
LINK_CAP_VEH = int(SPACING / 7.5 * 2)
SPILL_FRACTION, PERSIST_SEC = 0.90, 10
BOX_STALL_SPEED, BOX_PERSIST_SEC = 0.3, 15

# per junction: queue-counting edges, stop-line edges, downstream edges, per phase group
JUNCTIONS = {
    "C1": {"up": {"NS": ["in_N1", "in_S1"], "EW": ["in_W1", "lk_W2"]},
           "stop": {"NS": ["in_N1", "in_S1"], "EW": ["in_W1", "lk_W2"]},
           "down": {"NS": ["out_S1", "out_N1"], "EW": ["lk_E1", "out_W1"]}},
    "C2": {"up": {"NS": ["in_N2", "in_S2"], "EW": ["lk_E1", "lk_W3"]},
           "stop": {"NS": ["in_N2", "in_S2"], "EW": ["lk_E1", "lk_W3"]},
           "down": {"NS": ["out_S2", "out_N2"], "EW": ["lk_E2", "lk_W2"]}},
    "C3": {"up": {"NS": ["in_N3", "in_S3"], "EW": ["lk_E2", "in_E3", "app_E3"]},
           "stop": {"NS": ["in_N3", "in_S3"], "EW": ["lk_E2", "in_E3"]},
           "down": {"NS": ["out_S3", "out_N3"], "EW": ["out_E3", "lk_W3"]}},
}
OPP = {"NS": "EW", "EW": "NS"}


# ------------------------------------------------------------------ network
def write_inputs():
    """Write the SUMO node/edge/connection files (no SUMO binary needed)."""
    os.makedirs(GEN, exist_ok=True)
    n = ["<nodes>"]
    for k, x in enumerate(XS, 1):
        n.append(f'  <node id="C{k}" x="{x}" y="0" type="traffic_light"/>')
        n.append(f'  <node id="O_N{k}" x="{x}" y="{R_OUTER}" type="priority"/>')
        n.append(f'  <node id="O_S{k}" x="{x}" y="{-R_OUTER}" type="priority"/>')
    n.append(f'  <node id="O_W" x="{-R_OUTER}" y="0" type="priority"/>')
    xe = XS[-1]
    n.append(f'  <node id="M_E" x="{xe + R_METER}" y="0" type="traffic_light"/>')
    n.append(f'  <node id="O_E" x="{xe + R_OUTER}" y="0" type="priority"/>')
    n.append("</nodes>")
    open(NOD, "w").write("\n".join(n) + "\n")

    def e(i, a, b, lanes, speed, prio=3):
        return f'  <edge id="{i}" from="{a}" to="{b}" numLanes="{lanes}" speed="{speed}" priority="{prio}"/>'
    ed = ["<edges>", e("in_W1", "O_W", "C1", 2, V_ART), e("out_W1", "C1", "O_W", 2, V_ART),
          e("lk_E1", "C1", "C2", 2, V_ART), e("lk_W2", "C2", "C1", 2, V_ART),
          e("lk_E2", "C2", "C3", 2, V_ART), e("lk_W3", "C3", "C2", 2, V_ART),
          e("app_E3", "O_E", "M_E", 2, V_ART), e("in_E3", "M_E", "C3", 2, V_ART),
          e("out_E3", "C3", "M_E", 1, V_BOT, 2), e("exit_E3", "M_E", "O_E", 1, V_BOT, 2)]
    for k in range(1, K + 1):
        ed += [e(f"in_N{k}", f"O_N{k}", f"C{k}", 2, V_ART), e(f"out_N{k}", f"C{k}", f"O_N{k}", 2, V_ART),
               e(f"in_S{k}", f"O_S{k}", f"C{k}", 2, V_ART), e(f"out_S{k}", f"C{k}", f"O_S{k}", 2, V_ART)]
    ed.append("</edges>")
    open(EDG, "w").write("\n".join(ed) + "\n")

    def c(a, b, fl, tl):
        return f'  <connection from="{a}" to="{b}" fromLane="{fl}" toLane="{tl}"/>'
    cn = ["<connections>"]
    for a, b in [("in_W1", "lk_E1"), ("lk_E1", "lk_E2"), ("lk_W3", "lk_W2"), ("lk_W2", "out_W1"),
                 ("app_E3", "in_E3"), ("in_E3", "lk_W3")]:
        cn += [c(a, b, 0, 0), c(a, b, 1, 1)]
    cn += [c("lk_E2", "out_E3", 0, 0), c("lk_E2", "out_E3", 1, 0), c("out_E3", "exit_E3", 0, 0)]
    for k in range(1, K + 1):
        for a, b in [(f"in_N{k}", f"out_S{k}"), (f"in_S{k}", f"out_N{k}")]:
            cn += [c(a, b, 0, 0), c(a, b, 1, 1)]
    cn.append("</connections>")
    open(CON, "w").write("\n".join(cn) + "\n")


def build():
    write_inputs()

    def netconvert(tll=None):
        cmd = ["netconvert", "--node-files", NOD, "--edge-files", EDG, "--connection-files", CON,
               "--output-file", NET, "--tls.default-type", "static", "--no-turnarounds", "true",
               "--junctions.corner-detail", "8", "--no-warnings", "true"]
        if tll:
            cmd += ["--tllogic-files", tll]
        subprocess.run(cmd, check=True)

    netconvert()
    net = sumolib.net.readNet(NET)
    conns = net.getTLS("M_E").getConnections()
    size = max(cn_[2] for cn_ in conns) + 1
    role = ["G"] * size
    for in_lane, _o, idx in conns:
        role[idx] = "M" if in_lane.getEdge().getID().startswith("out_") else "G"
    st = lambda ch: "".join(ch if r == "M" else "G" for r in role)  # noqa: E731
    g, y, r = METER_PLANS["moderate"]
    open(TLL, "w").write(
        f'<additional>\n <tlLogic id="M_E" type="static" programID="0" offset="0">\n'
        f'  <phase duration="{g}" state="{st("G")}"/>\n  <phase duration="{y}" state="{st("y")}"/>\n'
        f'  <phase duration="{r}" state="{st("r")}"/>\n </tlLogic>\n</additional>\n')
    netconvert(TLL)
    print("[corridor] network built:", NET)


def arrivals(vph, end, bursty, rng):
    t, out = 0.0, []
    while t < end:
        surge = bursty and any(b <= t < b + BURST_LEN for b in BURST_STARTS)
        t += rng.expovariate(max(vph * (BURST_FACTOR if surge else 1.0) / 3600.0, 1e-6))
        if t < end:
            out.append(t)
    return out


def write_config(tag, seed, scale=1.0, sim_end=SIM_END):
    rng = random.Random(seed)
    routes = {"EB": "in_W1 lk_E1 lk_E2 out_E3 exit_E3", "WB": "app_E3 in_E3 lk_W3 lk_W2 out_W1"}
    demand = {"EB": EB_VPH * scale, "WB": WB_VPH * scale}
    for k in range(1, K + 1):
        routes[f"N{k}"], routes[f"S{k}"] = f"in_N{k} out_S{k}", f"in_S{k} out_N{k}"
        demand[f"N{k}"] = demand[f"S{k}"] = NS_VPH * scale
    veh = []
    for d, vph in demand.items():
        for t in arrivals(vph, sim_end, bursty=(d == "EB"), rng=rng):
            veh.append((t, d))
    veh.sort()
    out = ["<routes>",
           '  <vType id="car" accel="2.6" decel="4.5" sigma="0.5" length="5.0" minGap="2.5" tau="1.1" '
           'maxSpeed="16.67" emissionClass="HBEFA3/PC_G_EU4"/>']
    out += [f'  <route id="r_{d}" edges="{e}"/>' for d, e in routes.items()]
    cnt = {}
    for t, d in veh:
        cnt[d] = cnt.get(d, -1) + 1
        out.append(f'  <vehicle id="{d}_{cnt[d]}" type="car" route="r_{d}" depart="{t:.2f}" '
                   f'departLane="best" departSpeed="max"/>')
    out.append("</routes>")
    rou = os.path.join(GEN, f"{tag}.rou.xml")
    cfg = os.path.join(GEN, f"{tag}.sumocfg")
    open(rou, "w").write("\n".join(out) + "\n")
    open(cfg, "w").write(
        f'<configuration>\n <input><net-file value="corridor.net.xml"/><route-files value="{tag}.rou.xml"/></input>\n'
        f' <time><begin value="0"/><end value="{sim_end}"/><step-length value="1"/></time>\n'
        f' <processing><collision.action value="warn"/><time-to-teleport value="300"/>'
        f'<ignore-junction-blocker value="-1"/></processing>\n'
        f' <report><no-step-log value="true"/><duration-log.disable value="true"/><no-warnings value="true"/></report>\n'
        f'</configuration>\n')
    return cfg, len(veh)


# -------------------------------------------------------------- controllers
def queue(edges):
    return sum(traci.edge.getLastStepHaltingNumber(e) for e in edges)


def discharge_time(q_edges, lane_edges):
    lanes = sum(traci.edge.getLaneNumber(e) for e in lane_edges)
    return queue(q_edges) / (SAT_FLOW * max(lanes, 1) / 3600.0)


class Junction:
    def __init__(self, jid, policy, fixed_plan=None, occ_override=OCC_OVERRIDE, fis_params=None):
        self.id, self.cfg, self.policy = jid, JUNCTIONS[jid], policy
        self.occ_override, self.fixed_plan = occ_override, fixed_plan
        edges = []
        for link in traci.trafficlight.getControlledLinks(jid):
            edges.append(traci.lane.getEdgeID(link[0][0]) if link else None)

        def mk(group, ch):
            return "".join(ch if e in self.cfg["stop"][group] else "r" for e in edges)
        self.states = {f"{g}_{k}": mk(g, ch) for g in ("NS", "EW") for k, ch in (("green", "G"), ("yellow", "y"))}
        self.states["all_red"] = "r" * len(edges)
        self.engine = FuzzyAntiSpillbackEngine(fis_params=fis_params) if policy == "fuzzy" else None
        self.hist = {"NS": deque(maxlen=OCC_WINDOW), "EW": deque(maxlen=OCC_WINDOW)}
        self.phase, self.mode, self.target, self.elapsed = "NS", "green", None, 0
        self.timer = T_MIN if policy == "fuzzy" else (fixed_plan["NS"] if policy == "fixed" else 10 ** 9)
        self.pending_green = None
        self.overrides = 0
        traci.trafficlight.setRedYellowGreenState(jid, self.states["NS_green"])

    def pressure(self, g):
        return discharge_time(self.cfg["up"][g], self.cfg["stop"][g]) - discharge_time(self.cfg["down"][g], self.cfg["down"][g])

    def mp_pressure(self, g):
        return queue(self.cfg["up"][g]) - queue(self.cfg["down"][g])

    def occ(self, g):
        return max(traci.edge.getLastStepOccupancy(e) for e in self.cfg["down"][g])

    def smoothed(self, g):
        h = self.hist[g]
        return sum(h) / len(h) if h else 0.0

    def begin_switch(self, target, green_after):
        traci.trafficlight.setRedYellowGreenState(self.id, self.states[f"{self.phase}_yellow"])
        self.mode, self.timer, self.target, self.pending_green = "yellow", YELLOW, target, green_after

    def tick(self):
        if self.policy == "fuzzy":
            for g in ("NS", "EW"):
                self.hist[g].append(self.occ(g))
        self.timer -= 1
        if self.mode == "yellow":
            if self.timer <= 0:
                traci.trafficlight.setRedYellowGreenState(self.id, self.states["all_red"])
                self.mode, self.timer = "allred", ALL_RED
        elif self.mode == "allred":
            if self.timer <= 0:
                self.phase = self.target
                traci.trafficlight.setRedYellowGreenState(self.id, self.states[f"{self.phase}_green"])
                self.mode, self.timer, self.elapsed = "green", self.pending_green, 0
        else:
            self.elapsed += 1
            getattr(self, f"decide_{self.policy}")()

    def decide_fixed(self):
        if self.timer <= 0:
            other = OPP[self.phase]
            self.begin_switch(other, self.fixed_plan[other])

    def decide_vanilla(self):
        if self.elapsed >= MIN_GREEN and self.elapsed % MP_INTERVAL == 0:
            other = OPP[self.phase]
            if self.mp_pressure(other) > self.mp_pressure(self.phase):
                self.begin_switch(other, 10 ** 9)

    def decide_fuzzy(self):
        if self.timer > 0:
            return
        cur, oth = self.phase, OPP[self.phase]
        w_cur, w_oth = self.pressure(cur), self.pressure(oth)
        o_cur, o_oth = self.smoothed(cur), self.smoothed(oth)
        nxt = cur if w_cur >= w_oth else oth
        note = "fuzzy"
        o_nxt = o_cur if nxt == cur else o_oth
        if o_nxt > self.occ_override:
            alt = oth if nxt == cur else cur
            o_alt = o_oth if nxt == cur else o_cur
            note = "rotate" if o_alt <= self.occ_override else "hold"
            if note == "rotate":
                nxt = alt
        w_sel = w_cur if nxt == cur else w_oth
        o_sel = o_cur if nxt == cur else o_oth
        green = T_MIN if note != "fuzzy" else self.engine.compute(w_sel, o_sel)
        if note != "fuzzy":
            self.overrides += 1
        if nxt != cur:
            self.begin_switch(nxt, green)
        else:
            self.timer = green


def webster_plan(scale):
    lanes = 2
    y_ew = EB_VPH * scale / (SAT_FLOW * lanes)
    y_ns = NS_VPH * scale / (SAT_FLOW * lanes)
    Y = min(y_ns + y_ew, 0.95)
    L = 2 * (YELLOW + ALL_RED)
    c = min(max((1.5 * L + 5.0) / max(1e-6, 1.0 - Y), 60.0), 120.0)
    eff = c - L
    return {"NS": max(MIN_GREEN, eff * y_ns / (y_ns + y_ew)), "EW": max(MIN_GREEN, eff * y_ew / (y_ns + y_ew))}


# ------------------------------------------------------------------ metrics
class Metrics:
    def __init__(self):
        self.box_events = {j: 0 for j in JUNCTIONS}
        self.stall, self.armed = {}, {}
        self.spill_events = {l: 0 for l in LINKS}
        self.spill_run = {l: 0 for l in LINKS}
        self.spill_armed = {l: False for l in LINKS}
        self.queue_sum, self.steps = 0.0, 0

    def sample(self):
        self.steps += 1
        self.queue_sum += queue(["in_W1", "lk_E1", "lk_E2", "app_E3", "in_E3"])
        present = set()
        for vid in traci.vehicle.getIDList():
            lane = traci.vehicle.getLaneID(vid)
            if not lane.startswith(":C"):
                continue
            present.add(vid)
            if traci.vehicle.getSpeed(vid) < BOX_STALL_SPEED:
                self.stall[vid] = self.stall.get(vid, 0) + 1
            else:
                self.stall[vid], self.armed[vid] = 0, False
            if self.stall[vid] >= BOX_PERSIST_SEC and not self.armed.get(vid, False):
                self.box_events[lane[1:3]] += 1
                self.armed[vid] = True
        for vid in list(self.stall):
            if vid not in present:
                del self.stall[vid]
                self.armed.pop(vid, None)
        for l in LINKS:
            over = traci.edge.getLastStepHaltingNumber(l) >= SPILL_FRACTION * LINK_CAP_VEH
            self.spill_run[l] = self.spill_run[l] + 1 if over else 0
            sustained = self.spill_run[l] >= PERSIST_SEC
            if sustained and not self.spill_armed[l]:
                self.spill_events[l] += 1
            self.spill_armed[l] = sustained


def tripinfo_delays(path):
    d = []
    if os.path.exists(path):
        for t in ET.parse(path).getroot().findall("tripinfo"):
            d.append(float(t.attrib.get("timeLoss", 0.0)))
    return d


# ------------------------------------------------------------------- runner
def run(policy="fuzzy", seed=401, regime="moderate", scale=1.0, sim_end=SIM_END, tag=None,
        occ_override=OCC_OVERRIDE, fis_params=None, out_dir=None, verbose=True, meter_green=None):
    tag = tag or f"{policy}_{regime}_s{seed}_{os.getpid()}"
    cfg, n_veh = write_config(tag, seed, scale, sim_end)
    out_dir = out_dir or os.path.join(ROOT, "multi", "out")
    os.makedirs(out_dir, exist_ok=True)
    trip = os.path.join(out_dir, f"trip_{tag}.xml")
    traci.start([sumolib.checkBinary("sumo"), "-c", cfg, "--seed", str(seed), "--tripinfo-output", trip,
                 "--tripinfo-output.write-unfinished", "true", "--no-step-log", "true",
                 "--duration-log.disable", "true", "--no-warnings", "true"])
    g, y, r = METER_PLANS[regime] if meter_green is None else (meter_green, 3, 50 - meter_green - 3)
    logic = traci.trafficlight.getAllProgramLogics("M_E")[0]
    role = ["M" if any(p.state[i] in "rR" for p in logic.phases) else "G"
            for i in range(len(logic.phases[0].state))]
    st = lambda ch: "".join(ch if x == "M" else "G" for x in role)  # noqa: E731
    traci.trafficlight.setProgramLogic("M_E", traci.trafficlight.Logic(
        logic.programID, logic.type, 0,
        [traci.trafficlight.Phase(g, st("G")), traci.trafficlight.Phase(y, st("y")),
         traci.trafficlight.Phase(r, st("r"))]))
    plan = webster_plan(scale)
    juncs = [Junction(j, policy, plan, occ_override, fis_params) for j in JUNCTIONS]
    metrics, inserted = Metrics(), 0
    while traci.simulation.getTime() < sim_end:
        traci.simulationStep()
        metrics.sample()
        inserted += traci.simulation.getDepartedNumber()
        for j in juncs:
            j.tick()
    pending = max(traci.simulation.getMinExpectedNumber() - traci.vehicle.getIDCount(), 0)
    teleports = traci.simulation.getStartingTeleportNumber()
    traci.close()
    delays = tripinfo_delays(trip)
    try:
        os.remove(trip)
    except OSError:
        pass
    for ext in (".rou.xml", ".sumocfg"):
        try:
            os.remove(os.path.join(GEN, tag + ext))
        except OSError:
            pass
    res = {"policy": policy, "seed": seed, "regime": regime, "scale": scale,
           "avg_delay_s": sum(delays) / max(len(delays), 1), "throughput_completed_trips": len(delays),
           "box_gridlock_events": sum(metrics.box_events.values()),
           "link_spillback_events": sum(metrics.spill_events.values()),
           "box_gridlock_by_junction": metrics.box_events, "spillback_by_link": metrics.spill_events,
           "mean_corridor_queue": metrics.queue_sum / max(metrics.steps, 1),
           "vehicles_inserted": inserted, "vehicles_never_departed": pending, "deadlock_teleports": teleports,
           "anti_spillback_overrides": sum(j.overrides for j in juncs)}
    if verbose:
        print(f"[corridor:{policy}:{regime} seed {seed}] delay={res['avg_delay_s']:.1f}s "
              f"completed={res['throughput_completed_trips']}/{inserted} box={res['box_gridlock_events']} "
              f"link_spillback={res['link_spillback_events']} teleports={teleports} "
              f"overrides={res['anti_spillback_overrides']}")
    return res


# --------------------------------------------------------------- evaluation
LEVELS = {"mild": 14, "medium": 10, "severe": 7}      # east-exit metering green s / 50 s cycle
POLICIES = ["fixed", "vanilla", "fuzzy"]
# "fuzzy60" = the same controller with the hard override at 0.60 instead of 0.75. The
# physical ceiling of occupancy for 5 m vehicles with 2.5 m minimum gap is 5/7.5 = 0.667,
# so 0.75 can never be reached; 0.60 is 90% of that ceiling.
OCC_OVERRIDE_CEILING_BASED = 0.60
POLICY_NAMES = {"fixed": "Fixed-Time", "vanilla": "Vanilla MP", "fuzzy": "Fuzzy AS-MP",
                "fuzzy60": "Fuzzy AS-MP (override 0.60)"}
EVAL_KEYS = ["avg_delay_s", "throughput_completed_trips", "mean_corridor_queue",
             "box_gridlock_events", "link_spillback_events"]


def _eval_job(job):
    policy, level, seed, sim_end = job
    base = "fuzzy" if policy == "fuzzy60" else policy
    occ = OCC_OVERRIDE_CEILING_BASED if policy == "fuzzy60" else OCC_OVERRIDE
    res = run(base, seed, "moderate", 1.0, sim_end, meter_green=LEVELS[level], occ_override=occ, verbose=False)
    res["policy"] = policy
    return level, res


def evaluate_all(seeds, workers, sim_end, out_path, policies=None):
    policies = policies or POLICIES
    import json
    from multiprocessing import Pool
    from evaluate import paired_tests, summarize
    if os.path.exists(out_path):
        sys.exit(f"Refusing: {out_path} exists. The evaluation seeds are blind; delete it deliberately "
                 "only if you accept that the evaluation is no longer blind.")
    jobs = [(p, lv, sd, sim_end) for lv in LEVELS for p in policies for sd in seeds]
    print(f"[eval] {len(jobs)} runs, {workers} workers", flush=True)
    with Pool(workers, maxtasksperchild=1) as pool:
        done = pool.map(_eval_job, jobs, chunksize=1)
    raw = {lv: {p: [] for p in policies} for lv in LEVELS}
    for (lv, res), job in zip(done, jobs):
        raw[lv][job[0]].append(res)
    names = POLICY_NAMES
    out = {"seeds": seeds, "levels": LEVELS, "sim_end": sim_end, "policies": policies,
           "overrides_mean": {}, "results": {}}
    for lv in LEVELS:
        named = {names[p]: raw[lv][p] for p in policies}
        out['overrides_mean'][lv] = {names[p]: sum(r['anti_spillback_overrides'] for r in raw[lv][p]) / len(raw[lv][p]) for p in policies}
        summ = summarize(named, EVAL_KEYS)
        p_raw, p_adj, dz, rb, _ = paired_tests(named, EVAL_KEYS)
        out["results"][lv] = {"summary": summ, "p_holm": p_adj, "cohens_dz": dz, "rank_biserial": rb,
                              "runs": raw[lv]}
        print(f"\n=== metering level: {lv} (green {LEVELS[lv]} s per 50 s) ===")
        print(f"{'metric':30s}" + "".join(f"{n:>22s}" for n in named))
        for k in EVAL_KEYS:
            print(f"{k:30s}" + "".join(
                f"{summ[n][k]['mean']:>10.2f} [{summ[n][k]['ci95_lo']:.1f},{summ[n][k]['ci95_hi']:.1f}]".rjust(22)
                for n in named))
        print(f"   mean override engagements per run: {out['overrides_mean'][lv]}")
        targets = [n for n in named if n.startswith("Fuzzy")]
        for t in targets:
            for other in named:
                if other == t or (other.startswith("Fuzzy") and targets.index(t) < targets.index(other)):
                    continue
                for k in EVAL_KEYS:
                    pv = p_adj.get(f"{other} vs {t} | {k}", p_adj.get(f"{t} vs {other} | {k}"))
                    shown = pv if pv is None or isinstance(pv, str) else f"{pv:.4f}"
                    print(f"   Holm p, {t} vs {other} | {k:28s} = {shown}")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    json.dump(out, open(out_path, "w"), indent=2, default=str)
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("build")
    r = sub.add_parser("run")
    r.add_argument("--policy", choices=["fixed", "vanilla", "fuzzy"], default="fuzzy")
    r.add_argument("--seed", type=int, default=401)
    r.add_argument("--regime", choices=["moderate", "stress"], default="moderate")
    r.add_argument("--scale", type=float, default=1.0)
    r.add_argument("--sim-end", type=int, default=SIM_END)
    r.add_argument("--meter-green", type=int, default=None,
                   help="east-exit metering green seconds per 50 s cycle (overrides --regime)")
    e = sub.add_parser("eval")
    e.add_argument("--seeds", type=int, nargs="+", default=list(range(301, 311)))
    e.add_argument("--workers", type=int, default=1)
    e.add_argument("--policies", nargs="+", default=POLICIES, choices=list(POLICY_NAMES))
    e.add_argument("--sim-end", type=int, default=SIM_END)
    e.add_argument("--out", default=os.path.join(ROOT, "multi", "results", "corridor_eval.json"))
    a = ap.parse_args()
    if a.cmd == "eval":
        evaluate_all(a.seeds, a.workers, a.sim_end, a.out, a.policies)
    elif a.cmd == "build":
        build()
    else:
        run(a.policy, a.seed, a.regime, a.scale, a.sim_end, meter_green=a.meter_green)
