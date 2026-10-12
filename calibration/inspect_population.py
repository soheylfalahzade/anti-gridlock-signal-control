"""Read-only diagnostic: rebuild the initial DE population and score each
member from the simulation cache only (no new simulations).
Prints per-member relative delay and incident change, so it is visible
whether the objective's incident penalty or the delay term drives J."""
import argparse, importlib.util, os, sys
import numpy as np
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT); os.chdir(ROOT)
spec = importlib.util.spec_from_file_location("dc", "calibration/de_calibrate.py")
dc = importlib.util.module_from_spec(spec); spec.loader.exec_module(dc)
from scipy.optimize._differentialevolution import DifferentialEvolutionSolver
from src.calibration_space import BOUNDS, default_vector, vector_to_params
from src.fuzzy_engine import coverage_ok

ap = argparse.ArgumentParser()
ap.add_argument("--seeds", type=int, nargs="+", default=[201, 202, 203, 204])
ap.add_argument("--regimes", nargs="+", default=["moderate", "stress"])
ap.add_argument("--sim-end", type=int, default=3600)
ap.add_argument("--popsize", type=int, default=2)
ap.add_argument("--penalty", type=float, default=0.5)
a = ap.parse_args()

def no_sim(*args, **kw):
    raise RuntimeError("cache miss")
dc.simulate = no_sim
cfgs = {f"{r}:{s}": f"cal_{r}_seed{s}" for r in a.regimes for s in a.seeds}
real = dc.build_configs(a.seeds, a.regimes)
ref = dc.reference_runs(real, a.sim_end)
print("hand-set reference (delay, incidents):",
      {k: (round(v["delay"], 1), v["incidents"]) for k, v in ref.items()})

import inspect
_rng_kw = "rng" if "rng" in inspect.signature(DifferentialEvolutionSolver.__init__).parameters else "seed"
solver = DifferentialEvolutionSolver(lambda x: 0.0, BOUNDS, x0=default_vector(), popsize=a.popsize,
                                     init="latinhypercube", polish=False, updating="deferred", **{_rng_kw: 0})
pop = solver._scale_parameters(solver.population)
print(f"\n{'#':>2s} {'feas':>4s} {'relDelay':>9s} {'incHand':>8s} {'incCand':>8s} {'extraPooled':>11s} {'J_perseed':>9s} {'J_delayonly':>11s}")
for i, x in enumerate(pop):
    fp, oo = vector_to_params(x)
    if not coverage_ok(fp):
        print(f"{i:2d} {'no':>4s}"); continue
    try:
        rel, extra, ih, ic = [], [], 0, 0
        for k, cfg in real.items():
            reg, seed = k.split(":")
            r = dc.cached_simulate(cfg, int(seed), reg, fp, oo, a.sim_end)
            b = ref[k]; rel.append(r["delay"] / b["delay"])
            extra.append(max(0, r["incidents"] - b["incidents"]) / max(b["incidents"], 1))
            ih += b["incidents"]; ic += r["incidents"]
    except RuntimeError:
        print(f"{i:2d} {'yes':>4s}   (not in cache)"); continue
    pooled = max(0, ic - ih) / max(ih, 1)
    print(f"{i:2d} {'yes':>4s} {np.mean(rel):9.4f} {ih:8d} {ic:8d} {pooled:11.3f} {np.mean(rel)+a.penalty*np.mean(extra):9.4f} {np.mean(rel):11.4f}")
