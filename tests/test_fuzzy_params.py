"""Regression tests for the parametrised fuzzy engine and its search space.
No SUMO execution required.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.calibration_space import BOUNDS, NAMES, PARAM_SPEC, default_vector, vector_to_params
from src.fuzzy_engine import DEFAULT_FIS_PARAMS, FuzzyAntiSpillbackEngine, coverage_ok

# Outputs of the ORIGINAL hand-set engine, frozen before it was parametrised.
FROZEN = [
    (-40, 0.0, 14.333333), (-40, 0.6, 14.649105), (-10, 0.7, 15.066667),
    (0, 0.3, 14.666667), (10, 0.0, 32.0), (10, 0.7, 25.006667),
    (25, 0.9, 14.333333), (50, 0.0, 52.222222), (50, 0.3, 51.730769),
    (50, 0.7, 25.006667),
]


@pytest.mark.parametrize("kwargs", [{}, {"fis_params": dict(DEFAULT_FIS_PARAMS)}])
def test_default_engine_matches_original_handset_outputs(kwargs):
    e = FuzzyAntiSpillbackEngine(**kwargs)
    for p, o, expected in FROZEN:
        assert e.compute(p, o) == pytest.approx(expected, abs=1e-5)


def test_critical_shift_still_supported():
    e = FuzzyAntiSpillbackEngine(critical_shift=0.05)
    assert e.compute(50, 0.7) == pytest.approx(29.594171, abs=1e-5)


def test_search_space_contains_handset_and_maps_back():
    assert all(lo < hi for lo, hi in BOUNDS)
    assert all(lo <= d <= hi for _, lo, hi, d in PARAM_SPEC)
    fis, occ = vector_to_params(default_vector())
    assert fis == DEFAULT_FIS_PARAMS and occ == 0.75
    assert set(fis) | {"occ_override"} == set(NAMES)


def test_coverage_check_accepts_handset_and_rejects_gaps():
    assert coverage_ok(DEFAULT_FIS_PARAMS)
    gapped = dict(DEFAULT_FIS_PARAMS, occ_low_a=0.10, occ_low_b=0.12,
                  occ_med_a=0.50, occ_med_b=0.55, occ_med_c=0.60)
    assert not coverage_ok(gapped)


def test_unknown_parameter_name_is_rejected():
    with pytest.raises(KeyError):
        FuzzyAntiSpillbackEngine(fis_params={"occ_lwo_a": 0.2})
