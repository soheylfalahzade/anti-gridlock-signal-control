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
