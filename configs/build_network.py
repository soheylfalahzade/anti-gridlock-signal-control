#!/usr/bin/env python3
"""
Symmetrical 4-way intersection with an engineered downstream bottleneck.

Metering asymmetry: NS egress metering duty ratio ~0.30 (green=15s of a
50s cycle) => sustainable capacity ~570 veh/h/lane, deliberately BELOW the
750 veh/h/direction baseline NS demand, so the arterial is continuously
oversaturated even without burst windows. EW egress metering duty ratio
~0.60 => capacity ~1140 veh/h/lane, comfortably above the 200 veh/h/direction
cross-street demand, so the cross street's own downstream is never the
constraint -- isolating the benchmark question to how well each policy at
C protects the cross street from an arterial that a fixed, uncoordinated
downstream signal cannot drain fast enough.
"""

import os
import subprocess
import sys

import sumolib

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)
from src import demand as dmd

CFG = os.path.join(BASE_DIR, "configs")
NOD, EDG, CON = (os.path.join(CFG, f"intersection.{e}.xml") for e in ("nod", "edg", "con"))
TLL = os.path.join(CFG, "metering.tll.xml")
NET = os.path.join(CFG, "intersection.net.xml")
ROU = os.path.join(CFG, "intersection.rou.xml")
SUMOCFG = os.path.join(CFG, "intersection.sumocfg")

R_OUTER, R_METER = 250.0, 50.0
V_ART, V_BOT = 13.89, 8.33
DIRS = {"N": (0.0, 1.0), "S": (0.0, -1.0), "E": (1.0, 0.0), "W": (-1.0, 0.0)}

# (green, yellow, red) per direction group, cycle = 50s in both cases
METERING_PLAN = {
    "N": (15, 3, 32), "S": (15, 3, 32),   # NS egress: duty 0.30 -> ~570 veh/h/lane
    "E": (30, 3, 17), "W": (30, 3, 17),   # EW egress: duty 0.60 -> ~1140 veh/h/lane
}


def write_nodes():
    lines = ['<nodes>', '    <node id="C" x="0.0" y="0.0" type="traffic_light"/>']
    for d, (ux, uy) in DIRS.items():
        lines.append(f'    <node id="M_{d}" x="{ux*R_METER:.1f}" y="{uy*R_METER:.1f}" type="traffic_light"/>')
        lines.append(f'    <node id="O_{d}" x="{ux*R_OUTER:.1f}" y="{uy*R_OUTER:.1f}" type="priority"/>')
    lines.append("</nodes>")
    open(NOD, "w").write("\n".join(lines) + "\n")


def write_edges():
    lines = ["<edges>"]
    for d in DIRS:
        lines += [
            f'    <edge id="app_{d}" from="O_{d}" to="M_{d}" numLanes="2" speed="{V_ART}" priority="3"/>',
            f'    <edge id="in_{d}" from="M_{d}" to="C" numLanes="2" speed="{V_ART}" priority="3"/>',
            f'    <edge id="out_{d}" from="C" to="M_{d}" numLanes="1" speed="{V_BOT}" priority="2"/>',
            f'    <edge id="exit_{d}" from="M_{d}" to="O_{d}" numLanes="1" speed="{V_BOT}" priority="2"/>',
        ]
    lines.append("</edges>")
    open(EDG, "w").write("\n".join(lines) + "\n")


def write_connections():
    lines = ["<connections>"]
    for d in dmd.DIRS:
        t = dmd.THROUGH[d]
        lines.append(f'    <connection from="in_{d}" to="out_{t}" fromLane="0" toLane="0"/>')
        lines.append(f'    <connection from="in_{d}" to="out_{t}" fromLane="1" toLane="0"/>')
        lines.append(f'    <connection from="app_{d}" to="in_{d}" fromLane="0" toLane="0"/>')
        lines.append(f'    <connection from="app_{d}" to="in_{d}" fromLane="1" toLane="1"/>')
        lines.append(f'    <connection from="out_{d}" to="exit_{d}" fromLane="0" toLane="0"/>')
    lines.append("</connections>")
    open(CON, "w").write("\n".join(lines) + "\n")


def netconvert(tllogic=None):
    cmd = ["netconvert", "--node-files", NOD, "--edge-files", EDG,
           "--connection-files", CON, "--output-file", NET,
           "--tls.default-type", "static", "--no-turnarounds", "true",
           "--junctions.corner-detail", "8", "--no-warnings", "true"]
    if tllogic:
        cmd += ["--tllogic-files", tllogic]
    subprocess.run(cmd, check=True)


def write_metering_tll():
    net = sumolib.net.readNet(NET)
    lines = ["<additional>"]
    for d in DIRS:
        tls_id = f"M_{d}"
        g, y, r = METERING_PLAN[d]
        conns = net.getTLS(tls_id).getConnections()
        n = max(c[2] for c in conns) + 1
        role = ["G"] * n
        for in_lane, _out, idx in conns:
            role[idx] = "M" if in_lane.getEdge().getID().startswith("out_") else "G"

        def state(ch):
            return "".join(ch if r_ == "M" else "G" for r_ in role)

        lines += [f'    <tlLogic id="{tls_id}" type="static" programID="0" offset="0">',
                  f'        <phase duration="{g}" state="{state("G")}"/>',
                  f'        <phase duration="{y}" state="{state("y")}"/>',
                  f'        <phase duration="{r}" state="{state("r")}"/>',
                  "    </tlLogic>"]
    lines.append("</additional>")
    open(TLL, "w").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    os.makedirs(CFG, exist_ok=True)
    write_nodes()
    write_edges()
    write_connections()
    netconvert()
    write_metering_tll()
    netconvert(tllogic=TLL)
    n = dmd.write_routes(ROU, scale=1.0, seed=1, ambulance=True)
    dmd.write_sumocfg(SUMOCFG, net_file="intersection.net.xml",
                      route_file="intersection.rou.xml",
                      view_file="viewsettings.xml")
    print(f"[net] rebuilt with asymmetric metering (NS duty=0.30, EW duty=0.60); "
         f"{n} vehicles in baseline route file")
