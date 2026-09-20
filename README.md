# Anti-Gridlock Signal Control (SUMO)

An adaptive traffic signal control framework implemented in SUMO via TraCI. 
This project integrates Queue-Dissipation Max-Pressure with Fuzzy Inference System (FIS) 
to eliminate intersection spillback and gridlock under asymmetric and heavy traffic demands.

## Architecture
- `configs/`: SUMO network definitions, detector layouts, and trip demands.
- `src/`: Core Fuzzy Max-Pressure controller and TraCI simulation runtime.
- `baselines/`: Fixed-Time and Vanilla Max-Pressure controllers for benchmarking.
- `evaluate.py`: Performance metrics extraction (Average Delay, Queue Length, Throughput).
