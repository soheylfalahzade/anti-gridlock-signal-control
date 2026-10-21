# Three-junction corridor (proof of concept)

Question: does the local anti-spillback layer still help when spillback can
propagate between adjacent signalised junctions?

Scope. Three junctions on an east-west arterial, 200 m apart, each with a north
and a south cross street; through movements only. The engineered bottleneck of the
single-junction study is kept at the east exit only (metered, one lane, 50 m), so a
queue can back up C3, then C2, then C1. Every junction runs its own controller with
no communication and no signal offsets: Webster fixed-time, Vanilla Max-Pressure, or
the fuzzy anti-spillback controller (same decision logic as `src/controller.py`,
reimplemented as a per-junction state machine). No emergency vehicle is simulated.
This tests a local layer on three junctions; it is not a network-control result.

Scenario selection. A first corridor setting (east-exit green 30 s and 20 s per 50 s
cycle) produced no inter-junction spillback under the fixed-time baseline, so it could
not test the question. Before looking at the other controllers, the scenario was
required to show spillback under fixed-time on the tuning seeds 401-403. Three
metering levels meet that criterion and are all reported: mild (14 s), medium (10 s)
and severe (7 s of green per 50 s). Controllers were not tuned for the corridor.
In the first setting, with the controllers as they are, the fuzzy controller was
slightly worse than both baselines on seed 401, and its override never engaged.

Seeds. 401-403 were used only to choose the scenario. The evaluation seeds 301-310
are blind and `eval` refuses to run twice.

```
python multi/corridor.py build
python multi/corridor.py run --policy fuzzy --seed 401 --meter-green 10
python multi/corridor.py eval --workers 20
```

Metrics: mean delay (time loss), completed trips, mean corridor queue, box-blocking
events at each junction (same sustained-stall rule as the single-junction study),
and link-spillback events (an inter-junction link at least 90% full for 10 s).

Known limits: one demand pattern, identical cycle and no offsets for the fixed-time
baseline (a coordinated green wave would be a stronger baseline), three junctions only.

## Results, first evaluation (blind seeds 301-310)

Ten paired seeds, three metering levels. Cells are means. Wilcoxon signed-rank with Holm correction per level.

**mild** (east-exit green 14 s per 50 s); mean override engagements per fuzzy run: 0.00

| metric | Fixed-Time | Vanilla MP | Fuzzy AS-MP |
| --- | --- | --- | --- |
| mean delay (s) | 175.37 | 176.89 | 176.34 |
| box-blocking events | 2.80 | 13.30 | 8.90 |
| link-spillback events | 8.20 | 0.00 | 21.90 |
| mean corridor queue | 76.56 | 82.27 | 84.94 |

**medium** (east-exit green 10 s per 50 s); mean override engagements per fuzzy run: 0.00

| metric | Fixed-Time | Vanilla MP | Fuzzy AS-MP |
| --- | --- | --- | --- |
| mean delay (s) | 189.40 | 195.59 | 195.69 |
| box-blocking events | 0.60 | 3.40 | 4.10 |
| link-spillback events | 27.60 | 0.00 | 31.20 |
| mean corridor queue | 89.66 | 93.01 | 94.82 |

**severe** (east-exit green 7 s per 50 s); mean override engagements per fuzzy run: 0.00

| metric | Fixed-Time | Vanilla MP | Fuzzy AS-MP |
| --- | --- | --- | --- |
| mean delay (s) | 198.87 | 209.62 | 207.19 |
| box-blocking events | 3.80 | 20.10 | 12.80 |
| link-spillback events | 44.90 | 0.00 | 34.60 |
| mean corridor queue | 98.85 | 99.58 | 101.03 |

Reading. The fuzzy controller was never better than Fixed-Time on delay (no difference at the mild level; Fixed-Time lower by about 6 s at the medium and 8 s at the severe level, Holm p = 0.029, the smallest value attainable with ten pairs, i.e. all seeds agree in direction). Against Vanilla MP delay was equal at the mild and medium levels and about 1% lower at the severe level. Vanilla MP had no sustained link-spillback events but several times more box-blocking events than Fixed-Time. The fuzzy controller had fewer link-spillback events than Fixed-Time only at the severe level.

The hard override did not engage in these runs. Downstream occupancy peaks at about 0.67, the physical ceiling for 5 m vehicles with a 2.5 m minimum gap (5 / 7.5), so a threshold of 0.75 cannot be reached. This evaluation therefore tests the fuzzy pressure logic, not the occupancy barrier. A follow-up with the override at 0.60 (90% of the ceiling), declared before running, uses new seeds 321-330.

