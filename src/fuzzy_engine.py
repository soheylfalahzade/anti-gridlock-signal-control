"""
Mamdani fuzzy inference engine for anti-spillback green-time allocation.

Inputs
------
pressure  : w_p(m) = t_D^p(m) - t_D^q(m)   [s]  queue-dissipation differential
occupancy : downstream egress occupancy / normalized queue growth, in [0, 1]

Output
------
green : centroid (centre-of-area) defuzzified green duration,
        strictly bounded to [T_MIN, T_MAX] = [10 s, 60 s]
"""

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

T_MIN = 18.0
T_MAX = 60.0


class FuzzyAntiSpillbackEngine:
    def __init__(self):
        pressure = ctrl.Antecedent(np.arange(-60.0, 60.01, 0.5), "pressure")
        occupancy = ctrl.Antecedent(np.arange(0.0, 1.001, 0.01), "occupancy")
        green = ctrl.Consequent(np.arange(T_MIN, T_MAX + 0.01, 0.5), "green",
                                defuzzify_method="centroid")

        pressure["negative"] = fuzz.trapmf(pressure.universe, [-60, -60, -18, -2])
        pressure["zero"] = fuzz.trimf(pressure.universe, [-8, 0, 8])
        pressure["positive"] = fuzz.trimf(pressure.universe, [2, 18, 34])
        pressure["high"] = fuzz.trapmf(pressure.universe, [26, 42, 60, 60])

        occupancy["low"] = fuzz.trapmf(occupancy.universe, [0.00, 0.00, 0.25, 0.45])
        occupancy["medium"] = fuzz.trimf(occupancy.universe, [0.32, 0.55, 0.76])
        occupancy["critical"] = fuzz.trapmf(occupancy.universe, [0.62, 0.78, 1.0, 1.0])

        green["short"] = fuzz.trapmf(green.universe, [10, 10, 14, 22])
        green["medium"] = fuzz.trimf(green.universe, [18, 32, 46])
        green["long"] = fuzz.trapmf(green.universe, [40, 50, 60, 60])

        rules = [
            # Anti-spillback dominance: a critical receiving link throttles the
            # green to its minimum irrespective of how large upstream pressure is.
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
