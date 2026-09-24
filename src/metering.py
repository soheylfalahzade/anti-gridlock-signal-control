"""
Runtime metering control for the egress signals M_N/M_S/M_E/M_W.

Root-cause fix: in every previous revision, the metering timing was baked
into intersection.net.xml at build time, so the same fixed plan governed
every scenario. Computing the actual V/C ratio it implied shows why every
benchmark so far has reported spillback_occurrences = 0 across all three
policies and why fuzzy vs. baselines looked noisy rather than genuinely
different:

  NS egress duty 0.30 (old) -> capacity = 1900*0.30 = 570 veh/h/lane,
  while base NS demand per direction is 750 veh/h even BEFORE the x2
  burst multiplier. The egress was permanently oversaturated for the
  entire 3600s regardless of what the controller at C did, so the queue
  simply grew monotonically all run, upstream of C -- not because of a
  spillback event, but because of a structurally undersized bottleneck.
  All three policies were being compared under permanent, unrecoverable
  gridlock, which swamps any real algorithmic difference with noise.

This module defines two regimes, applied at simulation start via TraCI
(not baked into the net file), so both can share one net.xml:

  "moderate": NS duty 0.46 -> capacity ~874 veh/h/lane, V/C ~0.86 sustained,
              rising to ~1.7 during the 300s burst windows. This is a
              near-saturation-with-recoverable-bursts regime: the correct
              setting for the primary controller-comparison benchmark,
              since it lets each policy's own signal logic (not a
              permanently undersized downstream) determine the outcome.
  "stress":   NS duty 0.30 (the old value) -> V/C ~1.32 sustained. Kept as
              an explicit, separately labeled ablation: "can the
              controller avoid total collapse under permanent, deliberate
              chronic overload." Not used for the headline comparison.

EW duty is fixed at 0.60 (capacity ~1140 veh/h/lane) in both regimes,
comfortably above the 200 veh/h/direction cross-street demand, so the
cross street's own egress is never the constraint -- isolating the
question to how well each policy at C protects it from the arterial.
"""

import traci

METER_CYCLE = 50  # seconds, common to all plans below

PLANS = {
    "moderate": {"N": (23, 3, 24), "S": (23, 3, 24), "E": (30, 3, 17), "W": (30, 3, 17)},
    "stress":   {"N": (15, 3, 32), "S": (15, 3, 32), "E": (30, 3, 17), "W": (30, 3, 17)},
}

CAPACITY_VPH_PER_LANE = {
    "moderate": {"NS": 1900 * 23 / METER_CYCLE, "EW": 1900 * 30 / METER_CYCLE},
    "stress":   {"NS": 1900 * 15 / METER_CYCLE, "EW": 1900 * 30 / METER_CYCLE},
}


def apply_metering(regime="moderate"):
    """Overrides each M_D traffic light's program via TraCI to match the
    requested regime, replacing whatever static plan netconvert baked into
    the net file. Must be called once, right after traci.start()."""
    plan = PLANS[regime]
    for d, (g, y, r) in plan.items():
        tls = f"M_{d}"
        try:
            logic = traci.trafficlight.getAllProgramLogics(tls)[0]
        except (traci.exceptions.TraCIException, IndexError):
            continue
        n_links = len(logic.phases[0].state) if logic.phases else 0
        # Preserve which link index is the metered ("out_*") one by reusing
        # the role pattern encoded in the existing default phase (state
        # char 'r' during the default red phase marks the metered link;
        # any link permanently 'G' in every phase is the free-flow one).
        role = _infer_roles(logic)
        new_phases = [
            traci.trafficlight.Phase(g, _state(role, "G")),
            traci.trafficlight.Phase(y, _state(role, "y")),
            traci.trafficlight.Phase(r, _state(role, "r")),
        ]
        new_logic = traci.trafficlight.Logic(logic.programID, logic.type, 0, new_phases)
        traci.trafficlight.setProgramLogic(tls, new_logic)


def _infer_roles(logic):
    """role[i] = 'M' if link i is ever red across the logic's phases (the
    metered egress link), else 'G' (the always-open through link)."""
    n = len(logic.phases[0].state)
    role = ["G"] * n
    for phase in logic.phases:
        for i, ch in enumerate(phase.state):
            if ch in "rR":
                role[i] = "M"
    return role


def _state(role, meter_char):
    return "".join(meter_char if r == "M" else "G" for r in role)
