"""
Runtime metering control for the egress signals M_N/M_S/M_E/M_W.

Adds apply_metering_custom(): sets an arbitrary NS green duration (duty
ratio = ns_green/cycle) while EW stays at the moderate-regime value,
enabling a direct bottleneck-severity sweep (varying the metering
constraint itself) independent of the demand sweep (which varies arrival
rate at a FIXED bottleneck). These are two different, non-substitutable
axes of the same design question and a Q1 reviewer will expect both.
"""

import traci

METER_CYCLE = 50

PLANS = {
    "moderate": {"N": (23, 3, 24), "S": (23, 3, 24), "E": (30, 3, 17), "W": (30, 3, 17)},
    "stress":   {"N": (15, 3, 32), "S": (15, 3, 32), "E": (30, 3, 17), "W": (30, 3, 17)},
}

CAPACITY_VPH_PER_LANE = {
    "moderate": {"NS": 1900 * 23 / METER_CYCLE, "EW": 1900 * 30 / METER_CYCLE},
    "stress":   {"NS": 1900 * 15 / METER_CYCLE, "EW": 1900 * 30 / METER_CYCLE},
}


def _infer_roles(logic):
    n = len(logic.phases[0].state)
    role = ["G"] * n
    for phase in logic.phases:
        for i, ch in enumerate(phase.state):
            if ch in "rR":
                role[i] = "M"
    return role


def _state(role, meter_char):
    return "".join(meter_char if r == "M" else "G" for r in role)


def _set_plan(tls, g, y, r):
    try:
        logic = traci.trafficlight.getAllProgramLogics(tls)[0]
    except (traci.exceptions.TraCIException, IndexError):
        return
    role = _infer_roles(logic)
    phases = [traci.trafficlight.Phase(g, _state(role, "G")),
             traci.trafficlight.Phase(y, _state(role, "y")),
             traci.trafficlight.Phase(r, _state(role, "r"))]
    traci.trafficlight.setProgramLogic(tls, traci.trafficlight.Logic(
        logic.programID, logic.type, 0, phases))


def apply_metering(regime="moderate"):
    plan = PLANS[regime]
    for d, (g, y, r) in plan.items():
        _set_plan(f"M_{d}", g, y, r)


def apply_metering_custom(ns_green, ew_green=30, cycle=METER_CYCLE, yellow=3):
    """Direct bottleneck-severity control: NS/EW egress green set explicitly
    (duty = green/cycle). Used by bottleneck_sweep.py to vary the
    constraint itself, holding demand fixed, as the counterpart to the
    demand-side sweep in evaluate.py."""
    ns_red = cycle - ns_green - yellow
    ew_red = cycle - ew_green - yellow
    assert ns_red > 0 and ew_red > 0, "invalid metering plan: green+yellow >= cycle"
    for d in ("N", "S"):
        _set_plan(f"M_{d}", ns_green, yellow, ns_red)
    for d in ("E", "W"):
        _set_plan(f"M_{d}", ew_green, yellow, ew_red)


def ns_capacity_vph_per_lane(ns_green, cycle=METER_CYCLE):
    return 1900 * ns_green / cycle
