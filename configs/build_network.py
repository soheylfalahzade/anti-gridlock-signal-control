#!/usr/bin/env python3
"""
Symmetrical 4-way intersection generator with an engineered downstream
bottleneck, for the anti-gridlock signal control benchmark.

Topology (perfectly symmetric, no lateral offsets)
--------------------------------------------------
    O_N (0, 250)  --app_N-->  M_N (0, 50)  --in_N-->  C (0, 0)
    C             --out_N-->  M_N          --exit_N-> O_N
  (identical for S, E, W)

  app_* : 200 m, 2 lanes, 13.89 m/s   (arterial storage)
  in_*  :  50 m, 2 lanes, 13.89 m/s   (stop-line section)
  out_* :  50 m, 1 lane,   8.33 m/s   (bottleneck egress -> spillback)
  exit_*: 200 m, 1 lane,   8.33 m/s   (sink)

The M_* nodes are metering signals that choke egress discharge to roughly
900 veh/h per direction. North-South base demand (750 veh/h/direction)
clears; arterial surges (x2) do not, so the 50 m egress links fill and
queues physically back up into junction box C.

The metering programs are injected through `netconvert --tllogic-files`
(never through runtime additional-files) so that no duplicate programID="0"
is ever defined for a traffic light.
"""

import os
import random
import subprocess

import sumolib

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(BASE_DIR, "configs")

NOD = os.path.join(CFG, "intersection.nod.xml")
EDG = os.path.join(CFG, "intersection.edg.xml")
CON = os.path.join(CFG, "intersection.con.xml")
TLL = os.path.join(CFG, "metering.tll.xml")
NET = os.path.join(CFG, "intersection.net.xml")
ROU = os.path.join(CFG, "intersection.rou.xml")

R_OUTER = 250.0          # approach origin distance from C
R_METER = 50.0           # metering node distance from C (= egress length)
V_ART = 13.89            # arterial free-flow speed  [m/s]
V_BOT = 8.33             # bottleneck egress speed   [m/s]

SIM_END = 3600
NS_TOTAL_VPH = 1500.0    # heavy arterial, both directions
EW_TOTAL_VPH = 400.0     # cross street, both directions
BURST_FACTOR = 2.0
BURST_LEN = 300
BURST_STARTS = [500, 1400, 2400]

# Metering cycle: 24 s green + 3 s yellow + 23 s red  ->  ~0.5 green ratio
METER_GREEN, METER_YELLOW, METER_RED = 24, 3, 23

DIRS = {
    "N": (0.0, 1.0),
    "S": (0.0, -1.0),
    "E": (1.0, 0.0),
    "W": (-1.0, 0.0),
}
# Through movements only: entering from N exits to S, etc.
THROUGH = {"N": "S", "S": "N", "E": "W", "W": "E"}

random.seed(11)


def write_nodes():
    lines = ['<nodes>', '    <node id="C" x="0.0" y="0.0" type="traffic_light"/>']
    for d, (ux, uy) in DIRS.items():
        lines.append(f'    <node id="M_{d}" x="{ux * R_METER:.1f}" '
                     f'y="{uy * R_METER:.1f}" type="traffic_light"/>')
        lines.append(f'    <node id="O_{d}" x="{ux * R_OUTER:.1f}" '
                     f'y="{uy * R_OUTER:.1f}" type="priority"/>')
    lines.append("</nodes>")
    open(NOD, "w").write("\n".join(lines) + "\n")


def write_edges():
    lines = ["<edges>"]
    for d in DIRS:
        lines += [
            f'    <edge id="app_{d}" from="O_{d}" to="M_{d}" numLanes="2" '
            f'speed="{V_ART}" priority="3"/>',
            f'    <edge id="in_{d}" from="M_{d}" to="C" numLanes="2" '
            f'speed="{V_ART}" priority="3"/>',
            f'    <edge id="out_{d}" from="C" to="M_{d}" numLanes="1" '
            f'speed="{V_BOT}" priority="2"/>',
            f'    <edge id="exit_{d}" from="M_{d}" to="O_{d}" numLanes="1" '
            f'speed="{V_BOT}" priority="2"/>',
        ]
    lines.append("</edges>")
    open(EDG, "w").write("\n".join(lines) + "\n")


def write_connections():
    """Explicit connections: through movements only, 2 lanes merging into the
    single-lane egress. Declaring connections for an edge removes all of its
    default connections, which keeps the junction free of phantom turns."""
    lines = ["<connections>"]
    for d in DIRS:
        tgt = THROUGH[d]
        lines.append(f'    <connection from="in_{d}" to="out_{tgt}" fromLane="0" toLane="0"/>')
        lines.append(f'    <connection from="in_{d}" to="out_{tgt}" fromLane="1" toLane="0"/>')
        lines.append(f'    <connection from="app_{d}" to="in_{d}" fromLane="0" toLane="0"/>')
        lines.append(f'    <connection from="app_{d}" to="in_{d}" fromLane="1" toLane="1"/>')
        lines.append(f'    <connection from="out_{d}" to="exit_{d}" fromLane="0" toLane="0"/>')
    lines.append("</connections>")
    open(CON, "w").write("\n".join(lines) + "\n")


def netconvert(tllogic=None):
    cmd = ["netconvert",
           "--node-files", NOD,
           "--edge-files", EDG,
           "--connection-files", CON,
           "--output-file", NET,
           "--tls.default-type", "static",
           "--no-turnarounds", "true",
           "--junctions.corner-detail", "8",
           "--no-warnings", "true"]
    if tllogic:
        cmd += ["--tllogic-files", tllogic]
    subprocess.run(cmd, check=True)


def write_metering_tll():
    """Build metering programs with link-index-correct state strings by
    reading the link ordering that netconvert actually assigned."""
    net = sumolib.net.readNet(NET)
    lines = ["<additional>"]
    for d in DIRS:
        tls_id = f"M_{d}"
        conns = net.getTLS(tls_id).getConnections()   # (inLane, outLane, index)
        n = max(c[2] for c in conns) + 1
        role = ["G"] * n
        for in_lane, _out_lane, idx in conns:
            role[idx] = "M" if in_lane.getEdge().getID().startswith("out_") else "G"

        def state(meter_char):
            return "".join(meter_char if r == "M" else "G" for r in role)

        lines.append(f'    <tlLogic id="{tls_id}" type="static" programID="0" offset="0">')
        lines.append(f'        <phase duration="{METER_GREEN}"  state="{state("G")}"/>')
        lines.append(f'        <phase duration="{METER_YELLOW}" state="{state("y")}"/>')
        lines.append(f'        <phase duration="{METER_RED}"    state="{state("r")}"/>')
        lines.append("    </tlLogic>")
    lines.append("</additional>")
    open(TLL, "w").write("\n".join(lines) + "\n")


def arrivals(vph_base, begin, end, bursty):
    """Poisson arrivals with deterministic surge windows superimposed."""
    times, t = [], float(begin)
    while t < end:
        surge = bursty and any(b <= t < b + BURST_LEN for b in BURST_STARTS)
        rate = max(vph_base * (BURST_FACTOR if surge else 1.0) / 3600.0, 1e-6)
        t += random.expovariate(rate)
        if t < end:
            times.append(t)
    return times


def write_routes():
    routes = {d: f"app_{d} in_{d} out_{THROUGH[d]} exit_{THROUGH[d]}" for d in DIRS}
    demand = {"N": NS_TOTAL_VPH / 2, "S": NS_TOTAL_VPH / 2,
              "E": EW_TOTAL_VPH / 2, "W": EW_TOTAL_VPH / 2}

    vehicles = []
    for d, vph in demand.items():
        for t in arrivals(vph, 0, SIM_END, bursty=d in ("N", "S")):
            vehicles.append({"route": f"r_{d}", "depart": t, "dir": d})

    # SUMO silently drops unsorted entries -> enforce strict departure order.
    vehicles.sort(key=lambda x: float(x["depart"]))

    out = ["<routes>",
           '    <vType id="car" accel="2.6" decel="4.5" sigma="0.5" length="5.0" '
           'minGap="2.5" tau="1.1" maxSpeed="16.67" jmDriveAfterRedTime="-1" '
           'jmIgnoreFoeProb="0" emissionClass="HBEFA3/PC_G_EU4" color="0.2,0.8,0.4"/>']
    for d, edges in routes.items():
        out.append(f'    <route id="r_{d}" edges="{edges}"/>')
    for i, v in enumerate(vehicles):
        out.append(f'    <vehicle id="{v["dir"]}{i}" type="car" route="{v["route"]}" '
                   f'depart="{v["depart"]:.2f}" departLane="best" departSpeed="max"/>')
    out.append("</routes>")
    open(ROU, "w").write("\n".join(out) + "\n")
    print(f"[routes] {len(vehicles)} vehicles, strictly sorted by depart time")


if __name__ == "__main__":
    os.makedirs(CFG, exist_ok=True)
    write_nodes()
    write_edges()
    write_connections()
    netconvert()                 # pass 1: discover TLS link indices
    write_metering_tll()
    netconvert(tllogic=TLL)      # pass 2: inject metering programs
    write_routes()
    print("[net] configs/intersection.net.xml built with metered egress nodes")
