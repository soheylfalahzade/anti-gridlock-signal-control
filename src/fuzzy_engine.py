"""
Mamdani fuzzy inference engine for anti-spillback green-time allocation.

Adds critical_shift: horizontally shifts the "critical" occupancy
membership function's breakpoints by this amount (in occupancy units,
e.g. +0.05), enabling a sensitivity analysis of the single most
consequential design choice in this controller -- where "critical"
begins -- rather than presenting the hand-set breakpoints as fixed given.
"""

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

T_MIN = 10.0
T_MAX = 60.0


class FuzzyAntiSpillbackEngine:
    def __init__(self, critical_shift=0.0):
        pressure = ctrl.Antecedent(np.arange(-60.0, 60.01, 0.5), "pressure")
        occupancy = ctrl.Antecedent(np.arange(0.0, 1.001, 0.01), "occupancy")
        green = ctrl.Consequent(np.arange(T_MIN, T_MAX + 0.01, 0.5), "green",
                                defuzzify_method="centroid")

        pressure["negative"] = fuzz.trapmf(pressure.universe, [-60, -60, -18, -2])
        pressure["zero"] = fuzz.trimf(pressure.universe, [-8, 0, 8])
        pressure["positive"] = fuzz.trimf(pressure.universe, [2, 18, 34])
        pressure["high"] = fuzz.trapmf(pressure.universe, [26, 42, 60, 60])

        s = critical_shift
        low_pts = np.clip([0.00, 0.00, 0.25 + s, 0.45 + s], 0, 1)
        med_pts = np.clip([0.32 + s, 0.55 + s, 0.76 + s], 0, 1)
        crit_pts = np.clip([0.62 + s, 0.78 + s, 1.0, 1.0], 0, 1)
        occupancy["low"] = fuzz.trapmf(occupancy.universe, sorted(low_pts))
        occupancy["medium"] = fuzz.trimf(occupancy.universe, sorted(med_pts))
        occupancy["critical"] = fuzz.trapmf(occupancy.universe, sorted(crit_pts))

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

    def compute(self, pressure_value, occupancy_value):
        p = float(np.clip(pressure_value, -60.0, 60.0))
        o = float(np.clip(occupancy_value, 0.0, 1.0))
        self.sim.input["pressure"] = p
        self.sim.input["occupancy"] = o
        try:
            self.sim.compute()
            g = float(self.sim.output["green"])
        except Exception:
            g = T_MIN if o >= 0.62 else 0.5 * (T_MIN + T_MAX)
        return float(np.clip(g, T_MIN, T_MAX))
