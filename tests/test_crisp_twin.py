"""Tests for the non-fuzzy twin engine and the ablation summary. No SUMO execution required."""
import csv
import inspect
import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.fuzzy_engine import T_MAX, T_MIN, CrispTwinEngine, FuzzyAntiSpillbackEngine

# Outputs of the Mamdani engine at points where one rule dominates (frozen in test_fuzzy_params.py).
PROTOTYPES = [((-40, 0.0), 14.333333), ((10, 0.0), 32.0), ((25, 0.9), 14.333333), ((50, 0.0), 52.222222)]

# The nine rules of the Mamdani engine, as (occupancy class, pressure class) -> output class.
RULES = {
    ("critical", "high"): "short", ("critical", "positive"): "short", ("critical", "zero"): "short",
    ("critical", "negative"): "short",
    ("low", "high"): "long", ("medium", "high"): "medium",
    ("low", "positive"): "medium", ("medium", "positive"): "medium",
    ("low", "zero"): "short", ("medium", "zero"): "short",
    ("low", "negative"): "short", ("medium", "negative"): "short",
}


def test_centres_are_ordered_and_equal_the_mamdani_prototype_outputs():
    c = CrispTwinEngine().centres
    assert T_MIN <= c["short"] < c["medium"] < c["long"] <= T_MAX
    assert c["short"] == pytest.approx(14.333333, abs=1e-5)
    assert c["medium"] == pytest.approx(32.0, abs=1e-5)
    assert c["long"] == pytest.approx(52.222222, abs=1e-5)


@pytest.mark.parametrize("point,expected", PROTOTYPES)
def test_agrees_with_mamdani_where_one_rule_dominates(point, expected):
    assert CrispTwinEngine().compute(*point) == pytest.approx(expected, abs=1e-5)
    assert FuzzyAntiSpillbackEngine().compute(*point) == pytest.approx(expected, abs=1e-5)


def test_rule_table_matches_the_nine_rules():
    for (occ, pres), out in RULES.items():
        assert CrispTwinEngine.rule(occ, pres) == out, (occ, pres)


def test_output_takes_only_the_three_centres_and_stays_in_range():
    e = CrispTwinEngine()
    allowed = {round(v, 6) for v in e.centres.values()}
    seen = set()
    for p in np.arange(-70, 70.1, 2.5):
        for o in np.arange(-0.1, 1.11, 0.05):
            g = e.compute(p, o)
            assert T_MIN <= g <= T_MAX
            seen.add(round(g, 6))
    assert seen == allowed


def test_critical_occupancy_always_gives_the_short_green():
    e = CrispTwinEngine()
    for p in (-50, 0, 20, 50):
        assert e.compute(p, 0.95) == pytest.approx(e.centres["short"])


def test_it_is_a_different_function_from_the_mamdani_engine():
    # between two rules the Mamdani engine interpolates and the twin does not
    assert FuzzyAntiSpillbackEngine().compute(10, 0.7) == pytest.approx(25.006667, abs=1e-5)
    assert CrispTwinEngine().compute(10, 0.7) == pytest.approx(14.333333, abs=1e-5)


def test_engine_kind_defaults_to_mamdani_and_rejects_unknown_values():
    pytest.importorskip("traci")
    from src import controller
    assert inspect.signature(controller.run).parameters["engine_kind"].default == "mamdani"
    with pytest.raises(ValueError):
        controller.run(engine_kind="bogus")


def test_ablation_summary_runs_on_recorded_rows(tmp_path, monkeypatch):
    pytest.importorskip("traci")
    import crisp_ablation as ca
    raw, rep = tmp_path / "raw.csv", tmp_path / "report.json"
    monkeypatch.setattr(ca, "RAW", str(raw))
    monkeypatch.setattr(ca, "REPORT", str(rep))
    rng = np.random.RandomState(0)
    for regime in ("moderate", "stress"):
        for seed in range(501, 509):
            for i, pol in enumerate(ca.POLICIES):
                row = dict(regime=regime, seed=seed, policy=pol)
                row.update({k: 100.0 + 5 * i + rng.rand() for k in ca.KEYS})
                ca.append_row(row, str(raw))
    assert len(ca.read_raw(str(raw))) == 48
    report = ca.summarize()
    assert report["moderate"]["n_pairs"] == 8 and report["stress"]["n_pairs"] == 8
    label = f"{ca.FUZZY} vs {ca.CRISP} | avg_delay_s"
    assert 0.0 <= report["moderate"]["wilcoxon_p_raw"][label] <= 1.0
    assert json.loads(rep.read_text())["seeds"] == list(range(501, 509))
    with open(raw, newline="") as fh:
        assert next(csv.reader(fh))[:3] == ["regime", "seed", "policy"]


def _fake_report(tmp_path, monkeypatch, shift_stress=-3.0):
    import crisp_ablation as ca
    raw, rep = tmp_path / "results" / "crisp_ablation_raw.csv", tmp_path / "results" / "crisp_ablation_report.json"
    monkeypatch.setattr(ca, "RAW", str(raw))
    monkeypatch.setattr(ca, "REPORT", str(rep))
    rng = np.random.RandomState(1)
    for regime in ("moderate", "stress"):
        for seed in range(501, 521):
            for pol in ca.POLICIES:
                base = {ca.VANILLA: 120.0, ca.FUZZY: 100.0, ca.CRISP: 100.0}[pol]
                if regime == "stress" and pol == ca.CRISP:
                    base -= shift_stress
                row = dict(regime=regime, seed=seed, policy=pol)
                row.update({k: base + rng.rand() for k in ca.KEYS})
                ca.append_row(row, str(raw))
    ca.summarize()


def test_readme_block_is_generated_from_the_report(tmp_path, monkeypatch):
    pytest.importorskip("traci")
    _fake_report(tmp_path, monkeypatch)
    import render_readme_tables as rr
    monkeypatch.setattr(rr, "ROOT", str(tmp_path))
    block = rr.table_crisp()
    assert block.startswith("Seeds 501-520 (n = 20 per regime)")
    assert block.count("| moderate |") == 3 + 3 and block.count("| stress |") == 3 + 3
    assert "Reading, generated from the table above:" in block
    # the stress regime was built with a clear difference between the engines, the moderate one with none
    assert "- stress, mean delay [s]: the fuzzy engine is lower than its crisp twin" in block
    assert "- moderate, mean delay [s]: no significant difference" in block
    import re
    share = {k: float(v) for k, v in re.findall(r"(moderate|stress) (\d+)%\.?", block.split("already achieves:")[1])}
    assert 97 <= share["moderate"] <= 103          # twin equals the fuzzy engine in the moderate fake data
    assert 83 <= share["stress"] <= 87             # (120 - 103) / (120 - 100)


def test_readme_documents_the_ablation_seeds():
    import crisp_ablation as ca
    text = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "README.md"), encoding="utf-8").read()
    assert f"{ca.SEEDS[0]}-{ca.SEEDS[-1]}" in text and "BEGIN:crisp" in text
