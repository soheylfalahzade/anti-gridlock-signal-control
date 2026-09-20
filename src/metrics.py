"""
Metric collection shared by every controller, so all three policies are
measured by exactly the same instrument.

Spillback event definition (rising edge, per egress link):
    downstream egress occupancy > 75 %  AND  at least one vehicle standing
    (< 0.1 m/s) on an internal lane of junction box C.
"""

import json
import os
import xml.etree.ElementTree as ET

import traci

UPSTREAM_EDGES = ["app_N", "in_N", "app_S", "in_S",
                  "app_E", "in_E", "app_W", "in_W"]
CROSS_EDGES = ["app_E", "in_E", "app_W", "in_W"]
DOWNSTREAM_EDGES = ["out_N", "out_S", "out_E", "out_W"]
EMISSION_EDGES = UPSTREAM_EDGES + DOWNSTREAM_EDGES

OCC_SPILLBACK = 0.75
BOX_SPEED_STOPPED = 0.1
SERIES_INTERVAL = 60


class MetricsCollector:
    def __init__(self, junction_id="C"):
        self.box_lanes = [l for l in traci.lane.getIDList()
                          if l.startswith(":" + junction_id)]
        self.queue_samples = []
        self.cross_queue_samples = []
        self.spillback_events = 0
        self.spillback_steps = 0
        self.total_co2_mg = 0.0
        self._armed = {e: False for e in DOWNSTREAM_EDGES}
        self.series = {"t": [], "queue": [], "cross_queue": [],
                       "spillback_cum": [], "blocked_pct": []}
        self._bin = {"q": [], "cq": [], "blocked": 0, "n": 0}
        self._step = 0

    def box_blocked(self):
        for lane in self.box_lanes:
            if (traci.lane.getLastStepVehicleNumber(lane) > 0 and
                    traci.lane.getLastStepMeanSpeed(lane) < BOX_SPEED_STOPPED):
                return True
        return False

    def sample_step(self):
        self._step += 1
        q = sum(traci.edge.getLastStepHaltingNumber(e) for e in UPSTREAM_EDGES)
        cq = sum(traci.edge.getLastStepHaltingNumber(e) for e in CROSS_EDGES)
        self.queue_samples.append(q)
        self.cross_queue_samples.append(cq)

        blocked = self.box_blocked()
        active = False
        for e in DOWNSTREAM_EDGES:
            occ = traci.edge.getLastStepOccupancy(e) / 100.0
            hot = (occ > OCC_SPILLBACK) and blocked
            if hot:
                active = True
                if not self._armed[e]:
                    self.spillback_events += 1
            self._armed[e] = hot
        if active:
            self.spillback_steps += 1

        for e in EMISSION_EDGES:
            self.total_co2_mg += traci.edge.getCO2Emission(e)

        self._bin["q"].append(q)
        self._bin["cq"].append(cq)
        self._bin["blocked"] += 1 if active else 0
        self._bin["n"] += 1
        if self._step % SERIES_INTERVAL == 0:
            n = max(self._bin["n"], 1)
            self.series["t"].append(self._step)
            self.series["queue"].append(sum(self._bin["q"]) / n)
            self.series["cross_queue"].append(sum(self._bin["cq"]) / n)
            self.series["spillback_cum"].append(self.spillback_events)
            self.series["blocked_pct"].append(100.0 * self._bin["blocked"] / n)
            self._bin = {"q": [], "cq": [], "blocked": 0, "n": 0}

    def finalize(self, tripinfo_path, inserted, pending):
        delays = []
        completed = 0
        if os.path.exists(tripinfo_path):
            for trip in ET.parse(tripinfo_path).getroot().findall("tripinfo"):
                completed += 1
                delays.append(float(trip.attrib.get("timeLoss", 0.0)))
        n = max(len(self.queue_samples), 1)
        return {
            "avg_delay_s": (sum(delays) / len(delays)) if delays else 0.0,
            "max_delay_s": max(delays) if delays else 0.0,
            "mean_queue_length": sum(self.queue_samples) / n,
            "mean_cross_queue": sum(self.cross_queue_samples) / n,
            "throughput_completed_trips": completed,
            "spillback_occurrences": self.spillback_events,
            "spillback_duration_s": self.spillback_steps,
            "vehicles_inserted": inserted,
            "vehicles_unserved": pending,
            "total_co2_kg": self.total_co2_mg / 1e6,
            "series": self.series,
        }

    @staticmethod
    def save(results, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(results, f, indent=2)
