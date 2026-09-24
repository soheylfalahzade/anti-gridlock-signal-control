"""
Shared demand model.

Fix vs. previous revision: AMBULANCE_DEPART moved from 1800s (deep into
chronic oversaturation, where it experienced ~985s of pure insertion delay
before ever entering the network -- confounding signal-preemption benefit
with a queue-storage problem no signal policy at C can fix) to 300s, while
traffic is still light. The chronic-demand ambulance scenario is kept as
an explicit, separately labeled ablation (see evaluate.py) rather than the
default, so the two effects are never silently mixed into one number again.
"""

import random

DIRS = ["N", "S", "E", "W"]
THROUGH = {"N": "S", "S": "N", "E": "W", "W": "E"}

SIM_END = 3600
NS_BASE_VPH = 1500.0
EW_BASE_VPH = 400.0
BURST_FACTOR = 2.0
BURST_LEN = 300
BURST_STARTS = [500, 1400, 2400]
TIME_TO_TELEPORT = 300

AMBULANCE_DEPART_LIGHT = 300.0    # primary ablation: light traffic
AMBULANCE_DEPART_STRESS = 1800.0  # secondary ablation: deep congestion
AMBULANCE_DIR = "N"


def _arrivals(vph_base, end, bursty, rng):
    times, t = [], 0.0
    while t < end:
        surge = bursty and any(b <= t < b + BURST_LEN for b in BURST_STARTS)
        rate = max(vph_base * (BURST_FACTOR if surge else 1.0) / 3600.0, 1e-6)
        t += rng.expovariate(rate)
        if t < end:
            times.append(t)
    return times


def write_routes(path, scale=1.0, seed=1, ambulance=False,
                 ambulance_depart=AMBULANCE_DEPART_LIGHT, sim_end=SIM_END):
    rng = random.Random(seed)
    routes = {d: f"app_{d} in_{d} out_{THROUGH[d]} exit_{THROUGH[d]}" for d in DIRS}
    demand = {"N": NS_BASE_VPH * scale / 2, "S": NS_BASE_VPH * scale / 2,
              "E": EW_BASE_VPH * scale / 2, "W": EW_BASE_VPH * scale / 2}

    vehicles = []
    for d, vph in demand.items():
        for t in _arrivals(vph, sim_end, bursty=(d in ("N", "S")), rng=rng):
            vehicles.append({"route": f"r_{d}", "depart": t, "id_prefix": d})

    if ambulance:
        vehicles.append({"route": f"r_{AMBULANCE_DIR}", "depart": ambulance_depart,
                         "id_prefix": "amb", "vtype": "ambulance"})

    vehicles.sort(key=lambda v: float(v["depart"]))

    out = ["<routes>",
          '    <vType id="car" accel="2.6" decel="4.5" sigma="0.5" length="5.0" '
          'minGap="2.5" tau="1.1" maxSpeed="16.67" emissionClass="HBEFA3/PC_G_EU4" '
          'color="0.2,0.8,0.4"/>',
          '    <vType id="ambulance" vClass="emergency" accel="3.5" decel="5.0" '
          'sigma="0.2" length="6.5" minGap="2.0" tau="0.8" maxSpeed="22.0" '
          'guiShape="emergency" color="1,0,0"/>']
    for d, edges in routes.items():
        out.append(f'    <route id="r_{d}" edges="{edges}"/>')
    counters = {}
    for v in vehicles:
        prefix = v["id_prefix"]
        counters[prefix] = counters.get(prefix, -1) + 1
        vtype = v.get("vtype", "car")
        out.append(f'    <vehicle id="{prefix}{counters[prefix]}" type="{vtype}" '
                   f'route="{v["route"]}" depart="{v["depart"]:.2f}" '
                   f'departLane="best" departSpeed="max"/>')
    out.append("</routes>")
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")
    return len(vehicles)


def write_sumocfg(path, net_file, route_file, view_file=None, sim_end=SIM_END,
                  teleport_time=TIME_TO_TELEPORT):
    gui_line = (f'<gui-settings-file value="{view_file}"/>' if view_file else "")
    with open(path, "w") as f:
        f.write(f"""<configuration>
    <input>
        <net-file value="{net_file}"/>
        <route-files value="{route_file}"/>
        {gui_line}
    </input>
    <time>
        <begin value="0"/>
        <end value="{sim_end}"/>
        <step-length value="1"/>
    </time>
    <processing>
        <collision.action value="warn"/>
        <time-to-teleport value="{teleport_time}"/>
        <ignore-junction-blocker value="-1"/>
    </processing>
    <report>
        <no-step-log value="true"/>
        <duration-log.disable value="true"/>
        <no-warnings value="true"/>
    </report>
</configuration>
""")


def make_config(tag, scale=1.0, seed=1, ambulance=False,
                ambulance_depart=AMBULANCE_DEPART_LIGHT,
                config_dir="configs", gen_dir="configs/gen",
                net_file="../intersection.net.xml", view_file="../viewsettings.xml"):
    import os
    os.makedirs(gen_dir, exist_ok=True)
    rou_path = os.path.join(gen_dir, f"{tag}.rou.xml")
    cfg_path = os.path.join(gen_dir, f"{tag}.sumocfg")
    n = write_routes(rou_path, scale=scale, seed=seed, ambulance=ambulance,
                     ambulance_depart=ambulance_depart)
    write_sumocfg(cfg_path, net_file=net_file, route_file=f"{tag}.rou.xml",
                 view_file=view_file)
    return cfg_path, n
