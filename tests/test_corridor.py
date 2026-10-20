"""Structural tests for the corridor proof of concept. No SUMO run required;
the network-build test is skipped when netconvert is not installed."""
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from multi import corridor as cr


@pytest.fixture(scope="module")
def inputs():
    cr.write_inputs()
    edges = {e.get("id"): e for e in ET.parse(cr.EDG).getroot()}
    conns = {(c.get("from"), c.get("to")) for c in ET.parse(cr.CON).getroot()}
    return edges, conns


def test_controller_edges_exist(inputs):
    edges, _ = inputs
    for j, cfg in cr.JUNCTIONS.items():
        for key in ("up", "stop", "down"):
            for group, names in cfg[key].items():
                for n in names:
                    assert n in edges, f"{j}.{key}.{group}: unknown edge {n}"
        for group in ("NS", "EW"):
            assert set(cfg["stop"][group]) <= set(cfg["up"][group])


def test_every_route_follows_declared_connections(inputs):
    _, conns = inputs
    routes = {"EB": "in_W1 lk_E1 lk_E2 out_E3 exit_E3", "WB": "app_E3 in_E3 lk_W3 lk_W2 out_W1"}
    for k in range(1, cr.K + 1):
        routes[f"N{k}"], routes[f"S{k}"] = f"in_N{k} out_S{k}", f"in_S{k} out_N{k}"
    for name, r in routes.items():
        seq = r.split()
        for a, b in zip(seq, seq[1:]):
            assert (a, b) in conns, f"route {name}: no connection {a}->{b}"


def test_demand_is_reproducible_and_sorted(tmp_path, monkeypatch):
    monkeypatch.setattr(cr, "GEN", str(tmp_path))
    cfg1, n1 = cr.write_config("t1", seed=7)
    text1 = open(os.path.join(str(tmp_path), "t1.rou.xml")).read()
    cfg2, n2 = cr.write_config("t2", seed=7)
    text2 = open(os.path.join(str(tmp_path), "t2.rou.xml")).read()
    assert n1 == n2 and text1 == text2
    departs = [float(v.get("depart")) for v in ET.fromstring(text1).findall("vehicle")]
    assert departs == sorted(departs)


def test_webster_plan_is_sane():
    plan = cr.webster_plan(1.0)
    assert plan["NS"] >= cr.MIN_GREEN and plan["EW"] >= cr.MIN_GREEN
    assert plan["EW"] > plan["NS"]


@pytest.mark.skipif(shutil.which("netconvert") is None, reason="netconvert not installed")
def test_network_builds_and_has_expected_signals():
    cr.build()
    import sumolib
    net = sumolib.net.readNet(cr.NET)
    ids = {t.getID() for t in net.getTrafficLights()}
    assert {"C1", "C2", "C3", "M_E"} <= ids
