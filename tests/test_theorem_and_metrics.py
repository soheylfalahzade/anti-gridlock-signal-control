"""
Regression tests for claims made in README.md Sec 3.2 and docs/THEOREM.md.

These guard against two classes of regression:
  1. Someone "fixing" phase_pressure() to match mp_pressure() (or vice
     versa), which would silently invalidate Proposition A.1's premise
     that the two pressure signals are distinct functionals.
  2. Someone removing total_co2_kg from MetricsCollector.finalize(),
     which would break evaluate.py's emissions reporting (Sec 13 of
     README.md) without raising any error, since it's only ever read
     via .get() / dict indexing that would need to be checked separately.

No SUMO execution required; source-level and static-dict checks only.
"""

import inspect
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


def test_fuzzy_and_vanilla_pressure_are_distinct_functionals():
    """Regression test for the overclaim corrected in docs/THEOREM.md
    Sec A.0: the Fuzzy controller's phase_pressure() must remain a
    discharge-TIME differential (division by a saturation rate), and
    vanilla_max_pressure's mp_pressure() must remain a raw-count
    differential (no division). If either changes to match the other,
    Proposition A.1 as proven no longer describes the code, and the
    README's precisely-qualified claim becomes either wrong or vacuous.
    """
    from src import controller
    from baselines import vanilla_max_pressure

    fuzzy_src = inspect.getsource(controller.phase_pressure)
    vanilla_src = inspect.getsource(vanilla_max_pressure.mp_pressure)

    assert "discharge_time" in fuzzy_src, (
        "controller.phase_pressure() no longer appears to use "
        "discharge_time() -- if it now matches vanilla's raw-count "
        "formula, docs/THEOREM.md Proposition A.1 must be revised, not "
        "silently invalidated."
    )
    assert "discharge_time" not in vanilla_src, (
        "baselines.vanilla_max_pressure.mp_pressure() now references "
        "discharge_time() -- it was written to be a raw vehicle-count "
        "differential (Varaiya's original formulation) specifically so "
        "it differs from the Fuzzy controller's discharge-time pressure. "
        "If this is intentional, docs/THEOREM.md Sec A.0 must be revised."
    )


def test_metrics_finalize_reports_co2():
    """Regression test: MetricsCollector.finalize() must keep exposing
    total_co2_kg, which evaluate.py's emissions reporting (README Sec 13)
    reads directly by key. A silent removal would make plot_emissions()
    raise a KeyError deep inside a long benchmark run instead of failing
    fast here.
    """
    src_text = inspect.getsource(__import__("src.metrics", fromlist=["MetricsCollector"]).MetricsCollector.finalize)
    assert "total_co2_kg" in src_text, (
        "MetricsCollector.finalize() no longer returns total_co2_kg -- "
        "evaluate.py's plot_emissions() and CORE_KEYS depend on this key."
    )


def test_evaluate_core_keys_includes_co2():
    """Regression test: evaluate.py's CORE_KEYS must include
    total_co2_kg for the emissions comparison (README Sec 13) to be
    computed at all."""
    import evaluate
    assert "total_co2_kg" in evaluate.CORE_KEYS, (
        "evaluate.CORE_KEYS no longer includes total_co2_kg -- the "
        "emissions summary/plot will be silently empty."
    )
