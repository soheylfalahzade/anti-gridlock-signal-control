"""
Structural sanity tests -- no SUMO execution required, fast enough for CI.
Verifies the specific defects found and fixed during this project's
development are not silently reintroduced by a future edit: unsorted
route files (SUMO drops vehicles), metering plans that violate the cycle
constraint, and basic network-file well-formedness.

Run with: pytest tests/ -v
"""

import os
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.demand import write_routes
from src.metering import ns_capacity_vph_per_lane, METER_CYCLE


@pytest.fixture
def tmp_route_file(tmp_path):
    path = str(tmp_path / "test.rou.xml")
    write_routes(path, scale=1.0, seed=1, ambulance=True)
    return path


def test_routes_sorted_by_departure(tmp_route_file):
    """Regression test for the original 'Route file should be sorted by
    departure time' defect that caused SUMO to silently drop vehicles."""
    tree = ET.parse(tmp_route_file)
    departs = [float(v.attrib["depart"]) for v in tree.getroot().findall("vehicle")]
    assert departs == sorted(departs), "vehicle entries are not sorted by depart time"
    assert len(departs) > 100, "suspiciously few vehicles generated"


def test_ambulance_present_exactly_once(tmp_route_file):
    tree = ET.parse(tmp_route_file)
    amb_vehicles = [v for v in tree.getroot().findall("vehicle")
                   if v.attrib["id"].startswith("amb")]
    assert len(amb_vehicles) == 1
    assert amb_vehicles[0].attrib["type"] == "ambulance"


def test_all_vehicles_reference_declared_routes(tmp_route_file):
    tree = ET.parse(tmp_route_file)
    declared = {r.attrib["id"] for r in tree.getroot().findall("route")}
    used = {v.attrib["route"] for v in tree.getroot().findall("vehicle")}
    assert used.issubset(declared), f"undeclared routes referenced: {used - declared}"


def test_metering_capacity_below_demand_in_moderate_regime():
    """Regression test for the original defect where the baked-in metering
    plan made the NS egress permanently oversaturated regardless of any
    controller's behavior, corrupting every downstream comparison."""
    from src.metering import CAPACITY_VPH_PER_LANE
    moderate_ns_cap_total = CAPACITY_VPH_PER_LANE["moderate"]["NS"] * 2  # 2 lanes
    ns_base_demand_per_direction = 750.0
    ns_total_base_demand = ns_base_demand_per_direction * 2
    assert moderate_ns_cap_total > ns_base_demand_per_direction, (
        "moderate regime's NS egress capacity is below even a single "
        "direction's base demand -- this reproduces the permanent-gridlock defect")


def test_ns_green_override_never_exceeds_cycle():
    for ns_green in [10, 15, 23, 33, 40]:
        cap = ns_capacity_vph_per_lane(ns_green)
        assert cap > 0
        assert ns_green + 3 < METER_CYCLE, (
            f"ns_green={ns_green} + yellow(3) must be < cycle({METER_CYCLE}) "
            "or apply_metering_custom's red-phase computation goes negative")


def test_network_files_exist_after_build(tmp_path):
    """Smoke test: confirms build_network.py's required outputs exist.
    Run configs/build_network.py before this test; skipped if absent so
    this file works standalone in a fresh checkout without requiring
    netconvert to have run."""
    net_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "configs", "intersection.net.xml")
    if not os.path.exists(net_path):
        pytest.skip("configs/intersection.net.xml not built yet; run "
                   "python configs/build_network.py first")
    tree = ET.parse(net_path)
    junction_ids = {j.attrib["id"] for j in tree.getroot().findall("junction")}
    assert "C" in junction_ids, "central junction 'C' missing from built network"


def test_downstream_occupancy_is_fraction_not_double_scaled():
    """Regression test for the critical occupancy unit-scaling defect
    (see CHANGELOG.md): downstream_occupancy() must NOT divide TraCI's
    getLastStepOccupancy() by an additional 100. Guards against silently
    reintroducing a bug that, in this project's history, made the entire
    anti-spillback mechanism inert without raising any error -- the
    single most dangerous kind of defect because nothing crashes.

    This inspects the source directly rather than running SUMO, so it
    stays fast and dependency-free; a full runtime check is performed
    separately by diagnose_occupancy.py, which is not part of automated
    tests because it requires a SUMO binary and network build.
    """
    import inspect
    from src import controller
    source = inspect.getsource(controller.downstream_occupancy)
    assert "/ 100" not in source.replace(" ", "") and "/100" not in source.replace(" ", ""), (
        "downstream_occupancy() appears to divide by 100 again -- "
        "traci.edge.getLastStepOccupancy() already returns a fraction "
        "in [0,1] on this SUMO/TraCI version, not a percentage. See "
        "CHANGELOG.md for the history of this exact defect."
    )
