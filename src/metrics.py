"""
Shared instrumentation.

Fixes vs. previous revision:

1. box_gridlock detection was a lane-aggregate mean-speed stall check.
   With 0-1 vehicles typically present on any single internal junction
   lane at once, "lane mean speed" is really "that one vehicle's speed",
   so the metric was really an individual-vehicle stall detector wearing
   an aggregate-statistic disguise -- and a noisy one, since a single
   vehicle briefly yielding to a conflicting through movement (a normal,
   momentary right-of-way interaction, not gridlock) could trip it. This
   explains the non-monotonic box_gridlock counts across the demand sweep
   (4,2,1,0,5,2,5 for Fixed-Time -- uncorrelated with V/C, i.e. noise).
   Detection is now explicitly per-vehicle: a vehicle must remain on an
   internal (":C_*") lane below BOX_STALL_SPEED for BOX_PERSIST_SEC
   CONSECUTIVE seconds before one event is armed for it, and it cannot
   re-arm until it either clears the box or resumes moving and re-stalls.
   This filters out transient right-of-way yields (which resolve in a
   few seconds) and only counts genuine, sustained immobilization.

2. storage_overflow_events previously fired for Fuzzy Anti-Spillback in
   effectively every run and for the two baselines in effectively none,
   which is not evidence of a metric bug: the fuzzy controller's hard
   override deliberately extends green for a protected movement up to
   T_MAX (60s) while occupancy on the OTHER movement's egress is high,
   which is exactly the mechanism that can let the un-served approach's
   own upstream storage fill up. This is a real, reportable trade-off,
   not an artifact -- but it must be reported per-direction (NS vs EW)
   so a reviewer can see which movement pays the storage-overflow price
   for the box-gridlock protection the other movement receives, rather
   than one opaque aggregate number.

3. A single combined, demand-normalized index is added
   (gridlock_incidents_per_1000veh) so the two separate, individually
   noisy counts (box_gridlock_events, storage_overflow_events) can be
   compared across policies and regimes on one defensible scale instead
   of requiring the reader to reconcile two small integers by eye.
"""

import json
import os
import xml.etree.ElementTree as ET

import traci

UPSTREAM_EDGES = ["app_N", "in_N", "app_S", "in_S", "app_E", "in_E", "app_W", "in_W"]
CROSS_EDGES = ["app_E", "in_E", "app_W", "in_W"]
DOWNSTREAM_EDGES = ["out_N", "out_S", "out_E", "out_W"]
EMISSION_EDGES = UPSTREAM_EDGES + DOWNSTREAM_EDGES

GROUP_UP = {"NS": ["app_N", "in_N", "app_S", "in_S"], "EW": ["app_E", "in_E", "app_W", "in_W"]}

VEH_SPACING_M = 7.5
LINK_LEN_M = 250.0
LANES = 2
DIRECTIONS_PER_GROUP = 2
GROUP_CAPACITY_VEH = int((LINK_LEN_M / VEH_SPACING_M) * LANES * DIRECTIONS_PER_GROUP)
STORAGE_OVERFLOW_FRACTION = 0.90

BOX_STALL_SPEED = 0.3
BOX_PERSIST_SEC = 15
PERSIST_SEC = 10
SERIES_INTERVAL = 60


class MetricsCollector:
    def __init__(self, junction_id="C"):
        self.junction_id = junction_id
        self.queue_samples, self.cross_queue_samples = [], []
        self.total_co2_mg = 0.0

        self.box_gridlock_events, self.box_gridlock_seconds = 0, 0
        self._veh_stall_run, self._veh_stall_armed = {}, {}

        self.storage_overflow_events = {"NS": 0, "EW": 0}
        self.storage_overflow_seconds = {"NS": 0, "EW": 0}
        self._storage_run = {"NS": 0, "EW": 0}
        self._storage_armed = {"NS": False, "EW": False}

        self.series = {"t": [], "queue": [], "box_gridlock_cum": [],
                       "storage_overflow_cum_NS": [], "storage_overflow_cum_EW": []}
        self._bin = {"q": [], "n": 0}
        self._step = 0
        self._box_prefix = ":" + junction_id

    def sample_step(self):
        self._step += 1

        q = sum(traci.edge.getLastStepHaltingNumber(e) for e in UPSTREAM_EDGES)
        cq = sum(traci.edge.getLastStepHaltingNumber(e) for e in CROSS_EDGES)
        self.queue_samples.append(q)
        self.cross_queue_samples.append(cq)

        # --- per-vehicle sustained box stall -------------------------------
        current_box_vids = set()
        any_box = False
        for vid in traci.vehicle.getIDList():
            lane = traci.vehicle.getLaneID(vid)
            if not lane.startswith(self._box_prefix):
                continue
            current_box_vids.add(vid)
            speed = traci.vehicle.getSpeed(vid)
            if speed < BOX_STALL_SPEED:
                self._veh_stall_run[vid] = self._veh_stall_run.get(vid, 0) + 1
            else:
                self._veh_stall_run[vid] = 0
                self._veh_stall_armed[vid] = False
            if self._veh_stall_run[vid] >= BOX_PERSIST_SEC:
                any_box = True
                if not self._veh_stall_armed.get(vid, False):
                    self.box_gridlock_events += 1
                    self._veh_stall_armed[vid] = True
        for vid in list(self._veh_stall_run):
            if vid not in current_box_vids:
                del self._veh_stall_run[vid]
                self._veh_stall_armed.pop(vid, None)
        if any_box:
            self.box_gridlock_seconds += 1

        # --- per-direction storage overflow --------------------------------
        for group, edges in GROUP_UP.items():
            n_halt = sum(traci.edge.getLastStepHaltingNumber(e) for e in edges)
            over = n_halt >= STORAGE_OVERFLOW_FRACTION * GROUP_CAPACITY_VEH
            self._storage_run[group] = self._storage_run[group] + 1 if over else 0
            sustained = self._storage_run[group] >= PERSIST_SEC
            if sustained and not self._storage_armed[group]:
                self.storage_overflow_events[group] += 1
            self._storage_armed[group] = sustained
            if sustained:
                self.storage_overflow_seconds[group] += 1

        for e in EMISSION_EDGES:
            self.total_co2_mg += traci.edge.getCO2Emission(e)

        self._bin["q"].append(q)
        self._bin["n"] += 1
        if self._step % SERIES_INTERVAL == 0:
            n = max(self._bin["n"], 1)
            self.series["t"].append(self._step)
            self.series["queue"].append(sum(self._bin["q"]) / n)
            self.series["box_gridlock_cum"].append(self.box_gridlock_events)
            self.series["storage_overflow_cum_NS"].append(self.storage_overflow_events["NS"])
            self.series["storage_overflow_cum_EW"].append(self.storage_overflow_events["EW"])
            self._bin = {"q": [], "n": 0}

    def finalize(self, tripinfo_path, inserted, pending):
        delays, completed = [], 0
        if os.path.exists(tripinfo_path):
            for trip in ET.parse(tripinfo_path).getroot().findall("tripinfo"):
                if trip.attrib.get("id", "").startswith("amb"):
                    continue
                completed += 1
                delays.append(float(trip.attrib.get("timeLoss", 0.0)))

        try:
            teleports = traci.simulation.getStartingTeleportNumber() if traci.isLoaded() else 0
        except Exception:
            teleports = 0

        n = max(len(self.queue_samples), 1)
        total_overflow = self.storage_overflow_events["NS"] + self.storage_overflow_events["EW"]
        composite = self.box_gridlock_events + total_overflow
        per_1000 = composite / max(inserted, 1) * 1000.0

        return {
            "avg_delay_s": (sum(delays) / len(delays)) if delays else 0.0,
            "mean_queue_length": sum(self.queue_samples) / n,
            "mean_cross_queue": sum(self.cross_queue_samples) / n,
            "throughput_completed_trips": completed,
            "box_gridlock_events": self.box_gridlock_events,
            "box_gridlock_seconds": self.box_gridlock_seconds,
            "storage_overflow_events": total_overflow,
            "storage_overflow_events_NS": self.storage_overflow_events["NS"],
            "storage_overflow_events_EW": self.storage_overflow_events["EW"],
            "storage_capacity_veh_per_group": GROUP_CAPACITY_VEH,
            "gridlock_incidents_per_1000veh": per_1000,
            "vehicles_inserted": inserted,
            "vehicles_never_departed": max(pending, 0),
            "deadlock_teleports": teleports,
            "total_co2_kg": self.total_co2_mg / 1e6,
            "series": self.series,
        }

    @staticmethod
    def save(results, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(results, f, indent=2)


def movement_delay_breakdown(tripinfo_path):
    groups = {"N": [], "S": [], "E": [], "W": []}
    if not os.path.exists(tripinfo_path):
        return {k: 0.0 for k in groups}
    for trip in ET.parse(tripinfo_path).getroot().findall("tripinfo"):
        vid = trip.attrib.get("id", "")
        if vid and vid[0] in groups:
            groups[vid[0]].append(float(trip.attrib.get("timeLoss", 0.0)))
    return {k: (sum(v) / len(v) if v else 0.0) for k, v in groups.items()}


def jains_index(values):
    values = [v for v in values if v is not None]
    if not values or sum(values) == 0:
        return 1.0
    n = len(values)
    return (sum(values) ** 2) / (n * sum(v ** 2 for v in values))


def emergency_metrics(tripinfo_path, prefix="amb", desired_depart=None):
    out = []
    if not os.path.exists(tripinfo_path):
        return out
    for trip in ET.parse(tripinfo_path).getroot().findall("tripinfo"):
        vid = trip.attrib.get("id", "")
        if vid.startswith(prefix):
            actual_depart = float(trip.attrib.get("depart", 0.0))
            entry = {"id": vid,
                    "delay_s": float(trip.attrib.get("timeLoss", 0.0)),
                    "duration_s": float(trip.attrib.get("duration", 0.0)),
                    "actual_depart_s": actual_depart}
            if desired_depart is not None:
                entry["insertion_delay_s"] = max(0.0, actual_depart - desired_depart)
            out.append(entry)
    return out
