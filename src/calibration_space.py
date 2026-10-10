"""Search space for calibrating the fuzzy controller (differential evolution).

Ten decision variables: nine membership-function breakpoints (see
DEFAULT_FIS_PARAMS) plus the hard-override occupancy threshold. Bounds are
deliberately wide around the hand-set values, and the hand-set values are
inside the bounds so they can seed the initial population.
"""
from src.fuzzy_engine import DEFAULT_FIS_PARAMS

OCC_OVERRIDE_DEFAULT = 0.75

# (name, lower, upper, hand-set default)
PARAM_SPEC = [
    ("occ_low_a",     0.10, 0.40, DEFAULT_FIS_PARAMS["occ_low_a"]),
    ("occ_low_b",     0.30, 0.60, DEFAULT_FIS_PARAMS["occ_low_b"]),
    ("occ_med_a",     0.20, 0.50, DEFAULT_FIS_PARAMS["occ_med_a"]),
    ("occ_med_b",     0.40, 0.70, DEFAULT_FIS_PARAMS["occ_med_b"]),
    ("occ_med_c",     0.60, 0.90, DEFAULT_FIS_PARAMS["occ_med_c"]),
    ("occ_crit_a",    0.45, 0.75, DEFAULT_FIS_PARAMS["occ_crit_a"]),
    ("occ_crit_b",    0.65, 0.95, DEFAULT_FIS_PARAMS["occ_crit_b"]),
    ("pres_zero_hw",  4.0, 14.0, DEFAULT_FIS_PARAMS["pres_zero_hw"]),
    ("pres_pos_peak", 8.0, 28.0, DEFAULT_FIS_PARAMS["pres_pos_peak"]),
    ("occ_override",  0.60, 0.90, OCC_OVERRIDE_DEFAULT),
]
NAMES = [n for n, *_ in PARAM_SPEC]
BOUNDS = [(lo, hi) for _, lo, hi, _ in PARAM_SPEC]


def default_vector():
    return [d for *_, d in PARAM_SPEC]


def vector_to_params(x):
    """Decision vector -> (fis_params dict, occ_override)."""
    d = {n: float(v) for n, v in zip(NAMES, x)}
    occ_override = d.pop("occ_override")
    return d, occ_override
