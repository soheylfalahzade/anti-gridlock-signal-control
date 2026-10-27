"""Side-by-side animation of the three-junction corridor under three controllers.

Same demand, same seed (a tuning seed, not an evaluation seed), severe east-exit metering.
Vehicles are coloured by speed (red: stopped, green: free flow). Run from the repository root:

    python multi/make_animation.py record fixed     # also: vanilla, fuzzy60   (one simulation each)
    python multi/make_animation.py render           # -> figures/corridor_comparison.gif
"""
import os
import pickle
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import sumolib
import traci
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from multi import corridor as cr  # noqa: E402

SEED, LEVEL, T0, T1, STEP = 401, "severe", 300, 1800, 15
PANELS = [("fixed", "Fixed-Time (Webster)"), ("vanilla", "Vanilla Max-Pressure"), ("fuzzy60", "Fuzzy anti-spillback (override 0.60)")]
OUT = os.path.join(ROOT, "multi", "out")
ALL_EDGES = ["in_W1", "lk_E1", "lk_E2", "app_E3", "in_E3", "lk_W2", "lk_W3", "out_W1", "out_E3"] + \
            [f"{d}{k}" for k in (1, 2, 3) for d in ("in_N", "in_S")]


def record(policy):
    os.makedirs(OUT, exist_ok=True)
    frames = []

    def on_step(t):
        if T0 <= t <= T1 and int(t) % STEP == 0:
            veh = [(*traci.vehicle.getPosition(v), traci.vehicle.getSpeed(v)) for v in traci.vehicle.getIDList()]
            packed = sum(traci.edge.getLastStepHaltingNumber(l) >= cr.SPILL_FRACTION * cr.LINK_CAP_VEH for l in cr.LINKS)
            frames.append(dict(t=t, veh=np.array(veh).reshape(-1, 3), queue=cr.queue(ALL_EDGES), packed=packed))
    base = "fuzzy" if policy == "fuzzy60" else policy
    res = cr.run(base, SEED, "moderate", 1.0, cr.SIM_END, meter_green=cr.LEVELS[LEVEL],
                 occ_override=cr.OCC_OVERRIDE_CEILING_BASED if policy == "fuzzy60" else cr.OCC_OVERRIDE, on_step=on_step, verbose=True)
    pickle.dump(dict(frames=frames, res=res), open(os.path.join(OUT, f"rec_{policy}.pkl"), "wb"))
    print("recorded", policy, len(frames), "frames")


def render():
    recs = {p: pickle.load(open(os.path.join(OUT, f"rec_{p}.pkl"), "rb")) for p, _ in PANELS}
    net = sumolib.net.readNet(cr.NET)
    segs = [[(x, y) for x, y in e.getShape()] for e in net.getEdges()]
    n = len(recs["fixed"]["frames"])
    x0, y0, x1, y1 = net.getBoundary()          # netconvert shifts the origin, so read the real extent
    cy = (y0 + y1) / 2
    fig, axes = plt.subplots(3, 1, figsize=(8.4, 6.9), dpi=60)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.95, bottom=0.01, hspace=0.18)
    cmap = plt.get_cmap("RdYlGn")

    def draw(i):
        for ax, (p, title) in zip(axes, PANELS):
            ax.clear()
            ax.set_xlim(x0 - 15, x1 + 15)
            ax.set_ylim(cy - 125, cy + 125)
            ax.set_aspect("equal")
            ax.axis("off")
            ax.add_collection(LineCollection(segs, colors="#cbd5e1", linewidths=5, zorder=1))
            f = recs[p]["frames"][i]
            if len(f["veh"]):
                ax.scatter(f["veh"][:, 0], f["veh"][:, 1], c=np.clip(f["veh"][:, 2] / 14.0, 0, 1), cmap=cmap, vmin=0, vmax=1,
                           s=16, zorder=3, edgecolors="none")
            ax.set_title(f"{title}", fontsize=11, loc="left")
            ax.text(x1 + 10, cy + 108, f"stopped: {f['queue']} veh   packed links: {f['packed']}/4", fontsize=10, ha="right", color="#111827")
        fig.suptitle(f"Three-junction corridor, severe east-exit metering, t = {recs['fixed']['frames'][i]['t']:.0f} s", fontsize=11, y=0.995)
    anim = FuncAnimation(fig, draw, frames=n, interval=100)
    out = os.path.join(ROOT, "figures", "corridor_comparison.gif")
    anim.save(out, writer=PillowWriter(fps=10))
    print("saved", out, os.path.getsize(out) // 1024, "KiB")


if __name__ == "__main__":
    if sys.argv[1] == "record":
        record(sys.argv[2])
    else:
        render()
