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


## Results, follow-up with the override at 0.60 (new blind seeds 321-330)

Declared before running: the override threshold 0.60 is 90% of the occupancy ceiling (0.667), chosen from physics, not from performance. Ten paired seeds, four controllers, 30 comparisons per level.

**mild** (east-exit green 14 s per 50 s); mean override engagements per run at 0.60: 144.4

| metric | Fixed-Time | Vanilla MP | Fuzzy (0.75) | Fuzzy (0.60) |
| --- | --- | --- | --- | --- |
| mean delay (s) | 175.45 | 175.79 | 175.74 | 173.48 |
| box-blocking events | 1.30 | 12.40 | 5.60 | 3.40 |
| link-spillback events | 6.70 | 0.00 | 18.40 | 20.60 |
| mean corridor queue | 76.21 | 81.65 | 84.61 | 88.60 |

**medium** (east-exit green 10 s per 50 s); mean override engagements per run at 0.60: 235.6

| metric | Fixed-Time | Vanilla MP | Fuzzy (0.75) | Fuzzy (0.60) |
| --- | --- | --- | --- | --- |
| mean delay (s) | 189.94 | 195.38 | 195.71 | 190.85 |
| box-blocking events | 0.50 | 4.00 | 3.50 | 2.80 |
| link-spillback events | 25.80 | 0.00 | 31.10 | 36.80 |
| mean corridor queue | 89.21 | 92.62 | 94.65 | 102.96 |

**severe** (east-exit green 7 s per 50 s); mean override engagements per run at 0.60: 289.0

| metric | Fixed-Time | Vanilla MP | Fuzzy (0.75) | Fuzzy (0.60) |
| --- | --- | --- | --- | --- |
| mean delay (s) | 199.43 | 209.54 | 207.16 | 199.85 |
| box-blocking events | 5.80 | 20.70 | 13.70 | 5.50 |
| link-spillback events | 43.70 | 0.00 | 34.50 | 36.80 |
| mean corridor queue | 98.51 | 99.26 | 100.70 | 108.81 |

Statistics. With ten pairs and 30 comparisons per level the smallest attainable Holm-adjusted p is 0.0586, so no comparison can be formally significant at 0.05; a value of 0.0586 means all ten seeds moved in the same direction. The statements below are about consistent direction and effect size.

Reading. With the barrier reachable, the fuzzy controller needed less delay than the 0.75 version (about 1.3%, 2.5% and 3.5% at the mild, medium and severe levels) and, at the severe level, had box-blocking and delay comparable to Fixed-Time (5.5 vs 5.8 events; 199.9 s vs 199.4 s) and better than Vanilla MP. It did not reduce link-spillback events (they were higher than with the 0.75 version at all three levels), and it had the longest mean corridor queue at every level. Vanilla MP kept links from staying packed (no sustained spillback event) at the price of several times more box-blocking events. At the mild and medium levels Fixed-Time had the fewest box-blocking events. In no setting was the fuzzy controller clearly better than Fixed-Time.

Baseline means agree between the two seed sets within about 1.1 s of delay, so the setup reproduces. Limits: one demand pattern, one threshold value tested, no signal offsets for Fixed-Time, three junctions.

