"""
Diagnostic: logs raw per-step values needed to determine why downstream
occupancy on out_N/out_S stays near zero in the moderate regime, despite
substantial upstream (in_N/in_S) queueing. Not part of the benchmark
suite -- a one-off investigation script.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import traci
import sumolib
from src.metering import apply_metering
from src.demand import make_config

cfg, n = make_config("diag", scale=1.0, seed=1, ambulance=False)
binary = sumolib.checkBinary("sumo")
traci.start([binary, "-c", cfg, "--seed", "1", "--no-step-log", "true",
            "--no-warnings", "true"])
apply_metering("moderate")

print(f"{'t':>5} {'q_in_N':>7} {'q_in_S':>7} {'occ_out_N':>10} {'occ_out_S':>10} "
     f"{'veh_out_N':>10} {'halt_out_N':>10} {'M_N_state':>10}")

for t in range(3600):
    traci.simulationStep()
    if t % 30 == 0 and t > 0:
        q_in_n = traci.edge.getLastStepHaltingNumber("in_N")
        q_in_s = traci.edge.getLastStepHaltingNumber("in_S")
        occ_out_n = traci.edge.getLastStepOccupancy("out_N")
        occ_out_s = traci.edge.getLastStepOccupancy("out_S")
        veh_out_n = traci.edge.getLastStepVehicleNumber("out_N")
        halt_out_n = traci.edge.getLastStepHaltingNumber("out_N")
        try:
            m_n_state = traci.trafficlight.getRedYellowGreenState("M_N")
        except Exception:
            m_n_state = "N/A"
        print(f"{t:5d} {q_in_n:7d} {q_in_s:7d} {occ_out_n:10.3f} {occ_out_s:10.3f} "
             f"{veh_out_n:10d} {halt_out_n:10d} {m_n_state:>10}")

traci.close()
