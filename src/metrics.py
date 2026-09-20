"""
Shared instrumentation.

Fixes vs. previous revision:
  - `deadlock_teleports`: counts SUMO-forced teleports (vehicles stuck
    beyond time-to-teleport). This is the honest replacement for the
    anomalous scale=1.5 vanilla-Max-Pressure sample -- gridlock now shows
    up as a large, comparable number instead of corrupting the sample.
  - `vehicles_never_departed`: loaded-but-never-inserted count at sim end,
    reported explicitly instead of silently distorting throughput.
"""

import json
import os
from collections import deque
import xml.etree.ElementTree as ET

import traci

UPSTREAM_EDGES = ["app_N", "in_N", "app_S", "in_S", "app_E", "in_E", "app_W", "in_W"]
CROSS_EDGES = ["app_E", "in_E", "app_W", "in_W"]
DOWNSTREAM_EDGES = ["out_N", "out_S", "out_E", "out_W"]
EMISSION_EDGES = UPSTREAM_EDGES + DOWNSTREAM_EDGES

GROUP_UP = {"NS": ["in_N", "in_S"], "EW": ["in_E", "in_W"]}
GROUP_DOWN = {"NS": ["out_N", "out_S"], "EW": ["out_E", "out_W"]}

OCC_JAM = 0.85
JAM_WINDOW = 50
PERSIST_SEC = 10
WASTED_GREEN_OCC = 0.90
SERIES_INTERVAL = 60


class MetricsCollector:
    def __init__(self, junction_id="C"):
        self.junction_id = junction_id
        self.queue_samples, self.cross_queue_samples = [], []
        self.spillback_events, self.spillback_steps = 0, 0
        self.wasted_green_steps = 0
        self.total_co2_mg = 0.0

        self.jam_hist = {e: deque(maxlen=JAM_WINDOW) for e in DOWNSTREAM_EDGES}
        self.jam_run = {e: 0 for e in DOWNSTREAM_EDGES}
        self._armed = {e: False for e in DOWNSTREAM_EDGES}

        self.series = {"t": [], "queue": [], "spillback_cum": [], "wasted_green_cum": []}
        self._bin = {"q": [], "n": 0}
        self._step = 0
        self._tls_edges = None
        self._last_teleport_count = 0

    def _init_tls_mapping(self):
        if self._tls_edges is not None:
            return
        edges = []
        for link in traci.trafficlight.getControlledLinks(self.junction_id):
            edges.append(traci.lane.getEdgeID(link[0][0]) if link else None)
        self._tls_edges = edges

    def _group_has_green(self, group, rygs):
        stopline = GROUP_UP[group]
        return any(e in stopline and rygs[i] in "Gg"
                  for i, e in enumerate(self._tls_edges) if e is not None)

    def sample_step(self):
        self._init_tls_mapping()
        self._step += 1

        q = sum(traci.edge.getLastStepHaltingNumber(e) for e in UPSTREAM_EDGES)
        cq = sum(traci.edge.getLastStepHaltingNumber(e) for e in CROSS_EDGES)
        self.queue_samples.append(q)
        self.cross_queue_samples.append(cq)

        sustained_any = False
        for e in DOWNSTREAM_EDGES:
            occ = traci.edge.getLastStepOccupancy(e) / 100.0
            self.jam_hist[e].append(occ)
            smoothed = sum(self.jam_hist[e]) / len(self.jam_hist[e])
            self.jam_run[e] = self.jam_run[e] + 1 if smoothed >= OCC_JAM else 0
            sustained = self.jam_run[e] >= PERSIST_SEC
            if sustained and not self._armed[e]:
                self.spillback_events += 1
            self._armed[e] = sustained
            sustained_any = sustained_any or sustained
        if sustained_any:
            self.spillback_steps += 1

        try:
            rygs = traci.trafficlight.getRedYellowGreenState(self.junction_id)
            for group in ("NS", "EW"):
                if not self._group_has_green(group, rygs):
                    continue
                if queue_edges(GROUP_UP[group]) == 0:
                    continue
                occ = max(traci.edge.getLastStepOccupancy(e) / 100.0 for e in GROUP_DOWN[group])
                if occ >= WASTED_GREEN_OCC:
                    self.wasted_green_steps += 1
        except traci.exceptions.TraCIException:
            pass

        for e in EMISSION_EDGES:
            self.total_co2_mg += traci.edge.getCO2Emission(e)

        self._bin["q"].append(q)
        self._bin["n"] += 1
        if self._step % SERIES_INTERVAL == 0:
            n = max(self._bin["n"], 1)
            self.series["t"].append(self._step)
            self.series["queue"].append(sum(self._bin["q"]) / n)
            self.series["spillback_cum"].append(self.spillback_events)
            self.series["wasted_green_cum"].append(self.wasted_green_steps)
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
        return {
            "avg_delay_s": (sum(delays) / len(delays)) if delays else 0.0,
            "mean_queue_length": sum(self.queue_samples) / n,
            "mean_cross_queue": sum(self.cross_queue_samples) / n,
            "throughput_completed_trips": completed,
            "spillback_occurrences": self.spillback_events,
            "spillback_duration_s": self.spillback_steps,
            "wasted_green_seconds": self.wasted_green_steps,
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


def queue_edges(edges):
    return sum(traci.edge.getLastStepHaltingNumber(e) for e in edges)


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


def emergency_metrics(tripinfo_path, prefix="amb"):
    """Reads finished-and-unfinished ambulance entries. Requires the sumo
    process to be started with --tripinfo-output.write-unfinished true
    (set in src/controller.py::start_sumo), otherwise an ambulance that has
    not completed its route by sim end never appears in tripinfo.xml at
    all -- which is exactly why every emergency_delay_s value was null in
    the previous run."""
    out = []
    if not os.path.exists(tripinfo_path):
        return out
    for trip in ET.parse(tripinfo_path).getroot().findall("tripinfo"):
        vid = trip.attrib.get("id", "")
        if vid.startswith(prefix):
            out.append({"id": vid,
                       "delay_s": float(trip.attrib.get("timeLoss", 0.0)),
                       "duration_s": float(trip.attrib.get("duration", 0.0)),
                       "unfinished": trip.attrib.get("timeLoss") is None})
    return out
