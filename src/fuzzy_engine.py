"""
Mamdani fuzzy inference engine for anti-spillback green-time allocation.

Adds critical_shift: horizontally shifts the "critical" occupancy
membership function's breakpoints by this amount (in occupancy units,
e.g. +0.05), enabling a sensitivity analysis of the single most
consequential design choice in this controller -- where "critical"
begins -- rather than presenting the hand-set breakpoints as fixed given.

Adds fis_params: an optional dict that overrides the hand-set membership
breakpoints listed in DEFAULT_FIS_PARAMS. With fis_params=None (the
default) the engine is numerically identical to the original hand-set
controller; tests/test_fuzzy_params.py freezes that behaviour. The
override exists so that the breakpoints can be calibrated by a formal
search (see calibration/de_calibrate.py) instead of being asserted.
"""

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

T_MIN = 10.0
T_MAX = 60.0

# Hand-set breakpoints of the original controller.
DEFAULT_FIS_PARAMS = {
    "occ_low_a": 0.25,    # low:      trapezoid [0, 0, a, b]
    "occ_low_b": 0.45,
    "occ_med_a": 0.32,    # medium:   triangle  [a, b, c]
    "occ_med_b": 0.55,
    "occ_med_c": 0.76,
    "occ_crit_a": 0.62,   # critical: trapezoid [a, b, 1, 1]
    "occ_crit_b": 0.78,
    "pres_zero_hw": 8.0,  # zero:     triangle  [-hw, 0, +hw]
    "pres_pos_peak": 18.0,  # positive: triangle  [2, peak, 34]
}

PRESSURE_UNIVERSE = np.arange(-60.0, 60.01, 0.5)
OCCUPANCY_UNIVERSE = np.arange(0.0, 1.001, 0.01)


def resolve_params(fis_params=None):
    p = dict(DEFAULT_FIS_PARAMS)
    if fis_params:
        unknown = set(fis_params) - set(p)
        if unknown:
            raise KeyError(f"unknown fis_params keys: {sorted(unknown)}")
        p.update({k: float(v) for k, v in fis_params.items()})
    return p


def membership_points(fis_params=None, critical_shift=0.0):
    """Breakpoints of every antecedent membership function."""
    p = resolve_params(fis_params)
    s = critical_shift
    return {
        "occ_low": sorted(np.clip([0.00, 0.00, p["occ_low_a"] + s, p["occ_low_b"] + s], 0, 1)),
        "occ_medium": sorted(np.clip([p["occ_med_a"] + s, p["occ_med_b"] + s, p["occ_med_c"] + s], 0, 1)),
        "occ_critical": sorted(np.clip([p["occ_crit_a"] + s, p["occ_crit_b"] + s, 1.0, 1.0], 0, 1)),
        "pres_negative": [-60, -60, -18, -2],
        "pres_zero": [-p["pres_zero_hw"], 0.0, p["pres_zero_hw"]],
        "pres_positive": sorted([2.0, p["pres_pos_peak"], 34.0]),
        "pres_high": [26, 42, 60, 60],
    }


def coverage_ok(fis_params=None, critical_shift=0.0, tol=0.05):
    """True if every occupancy and pressure value activates at least one
    membership function above `tol`. Candidate parameter sets with gaps are
    rejected before any simulation, so a poor candidate cannot hide behind
    the engine's exception fallback."""
    try:
        pts = membership_points(fis_params, critical_shift)
        occ = np.vstack([
            fuzz.trapmf(OCCUPANCY_UNIVERSE, pts["occ_low"]),
            fuzz.trimf(OCCUPANCY_UNIVERSE, pts["occ_medium"]),
            fuzz.trapmf(OCCUPANCY_UNIVERSE, pts["occ_critical"]),
        ])
        pres = np.vstack([
            fuzz.trapmf(PRESSURE_UNIVERSE, pts["pres_negative"]),
            fuzz.trimf(PRESSURE_UNIVERSE, pts["pres_zero"]),
            fuzz.trimf(PRESSURE_UNIVERSE, pts["pres_positive"]),
            fuzz.trapmf(PRESSURE_UNIVERSE, pts["pres_high"]),
        ])
        return bool(occ.max(axis=0).min() >= tol and pres.max(axis=0).min() >= tol)
    except Exception:
        return False


class FuzzyAntiSpillbackEngine:
    def __init__(self, critical_shift=0.0, fis_params=None):
        pts = membership_points(fis_params, critical_shift)
        pressure = ctrl.Antecedent(PRESSURE_UNIVERSE, "pressure")
        occupancy = ctrl.Antecedent(OCCUPANCY_UNIVERSE, "occupancy")
        green = ctrl.Consequent(np.arange(T_MIN, T_MAX + 0.01, 0.5), "green",
                                defuzzify_method="centroid")

        pressure["negative"] = fuzz.trapmf(pressure.universe, pts["pres_negative"])
        pressure["zero"] = fuzz.trimf(pressure.universe, pts["pres_zero"])
        pressure["positive"] = fuzz.trimf(pressure.universe, pts["pres_positive"])
        pressure["high"] = fuzz.trapmf(pressure.universe, pts["pres_high"])

        occupancy["low"] = fuzz.trapmf(occupancy.universe, pts["occ_low"])
        occupancy["medium"] = fuzz.trimf(occupancy.universe, pts["occ_medium"])
        occupancy["critical"] = fuzz.trapmf(occupancy.universe, pts["occ_critical"])

        green["short"] = fuzz.trapmf(green.universe, [10, 10, 14, 22])
        green["medium"] = fuzz.trimf(green.universe, [18, 32, 46])
        green["long"] = fuzz.trapmf(green.universe, [40, 50, 60, 60])

        rules = [
            ctrl.Rule(occupancy["critical"], green["short"]),
            ctrl.Rule(pressure["high"] & occupancy["low"], green["long"]),
            ctrl.Rule(pressure["high"] & occupancy["medium"], green["medium"]),
            ctrl.Rule(pressure["positive"] & occupancy["low"], green["medium"]),
            ctrl.Rule(pressure["positive"] & occupancy["medium"], green["medium"]),
            ctrl.Rule(pressure["zero"] & occupancy["low"], green["short"]),
            ctrl.Rule(pressure["zero"] & occupancy["medium"], green["short"]),
            ctrl.Rule(pressure["negative"] & occupancy["low"], green["short"]),
            ctrl.Rule(pressure["negative"] & occupancy["medium"], green["short"]),
        ]
        self.system = ctrl.ControlSystem(rules)
        self.sim = ctrl.ControlSystemSimulation(self.system)
        self._crit_start = pts["occ_critical"][0]

    def compute(self, pressure_value, occupancy_value):
        p = float(np.clip(pressure_value, -60.0, 60.0))
        o = float(np.clip(occupancy_value, 0.0, 1.0))
        self.sim.input["pressure"] = p
        self.sim.input["occupancy"] = o
        try:
            self.sim.compute()
            g = float(self.sim.output["green"])
        except Exception:
            g = T_MIN if o >= self._crit_start else 0.5 * (T_MIN + T_MAX)
        return float(np.clip(g, T_MIN, T_MAX))
