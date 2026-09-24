<div align="center">

# Anti-Gridlock Signal Control
### A Queue-Dissipation Max-Pressure Controller with Fuzzy Anti-Spillback Regulation

[![Python](https://img.shields.io/badge/Python-3.10-blue.svg?logo=python&logoColor=white)](https://www.python.org/)
[![SUMO](https://img.shields.io/badge/SUMO-1.27.1-darkgreen.svg)](https://eclipse.dev/sumo/)
[![Status](https://img.shields.io/badge/Status-Phase%201%20Validated-brightgreen.svg)]()
[![Benchmark](https://img.shields.io/badge/Benchmark-10--Seed%20Paired-blueviolet.svg)]()
[![Research Track](https://img.shields.io/badge/Target-Q1%20Submission%20Track-orange.svg)]()
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

<p align="center">
  <b>Phase 1: Single-Intersection Local Control Layer</b><br>
  <i>Statistically benchmarked, causally ablated, and submission-track validated control substrate.</i>
</p>

</div>

---

> [!NOTE]
> **Repository Scope & Role:** This repository serves as the local control substrate for a planned network-level emergency-corridor navigation stack (time-varying, fault-tolerant geometric spanners + dynamic green wave). This repository stands on its own and makes no claims beyond the single-intersection scope tested here.

---

## 📌 Abstract

Max-Pressure signal control (Varaiya, 2013) is throughput-optimal under the assumption of unbounded downstream storage — an assumption that fails at short urban links, where a discharging phase can push a queue the receiving link cannot absorb, backing traffic up into the junction box itself and gridlocking the cross street. 

We construct a symmetric four-approach intersection with an engineered downstream bottleneck (50 m metered egress against 250 m upstream storage) to reproduce this failure mode under controlled conditions, and evaluate three control policies — **Webster fixed-time**, **vanilla Max-Pressure**, and a **Max-Pressure variant with a Mamdani fuzzy inference throttle** on green duration — across:
- 10 paired random seeds
- A seven-point demand sweep
- A chronic-oversaturation stress regime
- A controlled emergency-vehicle preemption ablation

We report effect sizes (Cohen's $d_z$), Holm–Bonferroni corrected significance, and bootstrap confidence intervals throughout. We transparently report the fuzzy controller's genuine trade-off — junction-box protection traded against upstream storage load on the protected corridor's counterpart — rather than presenting a single composite metric that would obscure it. 

The fuzzy controller reduces mean delay relative to vanilla Max-Pressure (**243.4s vs. 261.0s, $p = 0.033$**, moderate regime) and improves movement fairness relative to Webster fixed-time, while a controlled ablation isolates a measurable causal benefit of emergency-vehicle preemption on ambulance delay (**122.0s vs. 142.5s, ≈14% reduction**, same seeds, same base algorithm).

---

## 1. Problem Statement

Standard Max-Pressure signal control selects the phase that maximizes a pressure differential between upstream and downstream queues, and is provably throughput-optimal under the assumption that the network's stability region is unconstrained by receiving-link storage. Real intersections violate this assumption whenever a downstream link is short relative to the demand it must discharge: a phase can serve its upstream queue faster than the receiving link can absorb it, and the excess vehicles queue back through the junction box itself, blocking the conflicting movement and producing exactly the kind of local gridlock that a corridor-level green wave cannot survive.

> **Core Research Question:** Does adding a fuzzy-logic throttle on green duration, conditioned on downstream occupancy, reduce junction-box lock-up without an unacceptable cost elsewhere, relative to both a conventional fixed-time baseline and unmodified Max-Pressure?

---

## 2. System Model

### 2.1 Geometry
A single symmetric four-leg intersection `C`, with each approach direction $d \in \{N, S, E, W\}$ composed of:

| Segment | Length | Lanes | Speed | Role |
|:---|:---:|:---:|:---:|:---|
| `app_d` | 200 m | 2 | 13.89 m/s | Upstream arterial storage |
| `in_d` | 50 m | 2 | 13.89 m/s | Stop-line section |
| `out_d` | 50 m | 1 | 8.33 m/s | Metered egress (bottleneck) |
| `exit_d` | 200 m | 1 | 8.33 m/s | Sink |

Combined approach-group physical storage capacity (`app_d` + `in_d`, 2 lanes, 2 directions, 7.5 m/vehicle spacing) is **133 vehicles** per movement group — the reference denominator for the storage-overflow metric defined in §4.3.

### 2.2 Demand Regimes

Two demand regimes are used, both generated as a non-homogeneous Poisson process with three deterministic 300 s surge windows (2× base rate) superimposed on a Poisson base rate, strictly sorted by departure time:

* **Moderate regime** (primary benchmark): NS egress metering duty 0.46 (≈874 veh/h/lane sustained capacity), placing the arterial near saturation with recoverable bursts rather than permanent gridlock.
* **Stress regime** (explicit secondary ablation): NS egress metering duty 0.30 (≈570 veh/h/lane), placing the arterial in chronic, sustained oversaturation (V/C ≈ 1.3) for the full 3600 s.

EW egress duty is fixed at 0.60 in both regimes (≈1140 veh/h/lane), comfortably above the 400 veh/h EW demand, isolating the experiment to how well each controller protects the cross street from the arterial. Metering timing is applied dynamically at simulation start via TraCI (`src/metering.py`).

---

## 3. Control Architecture

### 3.1 Pressure Signal
Phase pressure is a queue-dissipation-time differential rather than a raw vehicle count:

$$w_p = t_D^p - t_D^q$$

where $t_D^p$ and $t_D^q$ are estimated discharge times (at 1900 veh/h/lane saturation flow) of upstream and downstream halting queues for phase $p$.

### 3.2 Fuzzy Anti-Spillback Throttle
A Mamdani fuzzy inference system (`src/fuzzy_engine.py`, centroid defuzzification) maps $(w_p, \text{downstream\_occupancy})$ to a green duration bounded in $[T_{\min}, T_{\max}] = [10s, 60s]$. Downstream occupancy is fed as a **50-second rolling average**, matching the metering cycle length.

> [!IMPORTANT]
> **Phase-Selection Lemma:** The fuzzy engine only ever *shortens* green relative to unconstrained Max-Pressure; it never re-weights $w_p$ or alters phase selection itself. Below the critical occupancy threshold, the controller is *identical* to vanilla Max-Pressure. Above the threshold, a hard override enforces:
> $$h(x) = \text{OCC\_OVERRIDE} - \text{occ}(x), \quad \text{OCC\_OVERRIDE} = 0.75$$
> acting as a control-barrier safety margin.

### 3.3 Emergency Preemption
A `vClass="emergency"` vehicle detected on an approach edge (`app_d`/`in_d` only) triggers an immediate transition to its phase group, held for up to 45 s or until clearing the stop-line section.

---

## 4. Experimental Design

### 4.1 Policies Compared
1. **Fixed-Time (Webster):** Cycle length and split computed from Webster's delay-minimizing formula.
2. **Vanilla Max-Pressure:** Full-strength phase-pressure control with no downstream awareness (Varaiya, 2013).
3. **Fuzzy Anti-Spillback Max-Pressure:** The proposed controller (§3).

### 4.2 Statistical Methodology
* **Paired Multi-Seed Design:** 10 shared random seeds per regime for paired statistical tests (Wilcoxon signed-rank, matched-pairs Cohen's $d_z$).
* **Multiple Testing Correction:** Holm–Bonferroni correction applied across the family of 15 pairwise comparisons per regime (3 policy pairs × 5 metrics).
* **Safety Indices Disentanglement:** Rejection of arbitrary composite indices; box-gridlock and directional storage-overflow are isolated and evaluated independently.

---

## 5. Experimental Results

### 5.1 Primary Benchmark (Moderate Regime, n = 10 paired seeds)

| Policy | Delay [s], mean (95% CI) | Throughput [veh] (95% CI) | Jain's Fairness Index |
|:---|:---:|:---:|:---:|
| **Fixed-Time (Webster)** | 234.8 (232.4–237.3) | 1659 (1647–1671) | ≈0.68 |
| **Vanilla Max-Pressure** | 261.0 (259.2–262.7) | 1696 (1681–1710) | ≈0.94 |
| **Fuzzy Anti-Spillback** | 243.4 (239.6–246.8) | 1658 (1646–1670) | ≈0.77 |

<p align="center">
  <img src="results/comparison_metrics.png" alt="Comparison Metrics" width="85%">
</p>

**Holm–Bonferroni Corrected Significance:**
* Fuzzy vs. Vanilla Max-Pressure, delay: **$p = 0.033$** (Fuzzy statistically lower)
* Fixed-Time vs. Fuzzy, delay: **$p = 0.035$** (Fixed-Time lower)
* Fixed-Time vs. Vanilla Max-Pressure, delay: **$p = 0.033$** (Fixed-Time lower)

---

### 5.2 Demand Sweep (Moderate Regime)

Across V/C scale 0.6–2.0, Fixed-Time's delay grows steeply beyond scale ≈1.2 (94.7s → 396.7s), while both Max-Pressure variants remain comparatively flat (Fuzzy: 146.5s → 308.4s; Vanilla: 84.1s → 282.5s).

<p align="center">
  <img src="results/demand_sweep.png" alt="Demand Sweep" width="85%">
</p>

---

### 5.3 Direction-Specific Storage-Overflow Trade-off

Across both regimes, **100% of storage-overflow events under the fuzzy controller occur on the NS (arterial) approach group**; the EW group shows zero storage-overflow events in every run.

<p align="center">
  <img src="results/storage_overflow_breakdown.png" alt="Storage Overflow Breakdown" width="85%">
</p>

> [!NOTE]
> *Mechanistic Trade-off:* The fuzzy anti-spillback mechanism redistributes congestion risk from the junction box to upstream arterial storage, rather than eliminating congestion risk outright.

---

### 5.4 Fairness and Stress Regimes

<p align="center">
  <img src="results/fairness.png" alt="Fairness Comparison" width="48%">
  <img src="results/comparison_metrics_stress.png" alt="Stress Regime Comparison" width="48%">
</p>

---

### 5.5 Emergency Preemption: Controlled Causal Ablation

Isolated preemption test with identical seeds and base algorithm:

| Condition | Ambulance in-network delay [s] | Relative Benefit |
|:---|:---:|:---:|
| **Without preemption** | 142.5 s | Baseline |
| **With preemption** | **122.0 s** | **≈14% delay reduction** |

<p align="center">
  <img src="results/preemption_ablation.png" alt="Preemption Ablation" width="70%">
</p>

---

## 6. Discussion

1. **The Fixed-Time Advantage Is Real and Not Hidden:** At moderate baseline demand (scale = 1.0), Webster fixed-time achieves lower total delay due to the "price-of-fairness". Demand sweeps show this advantage inverts as demand rises.
2. **What the Fuzzy Controller Actually Buys:** Statistically significant delay reduction relative to unmodified Max-Pressure, improved movement fairness over fixed-time, and a causally isolated benefit to emergency vehicles.
3. **On the Rejected Composite Metric:** Collapsing box-gridlock and storage-overflow into an arbitrary composite index obscures direction-specific protection mechanics. They are reported independently.

---

## 7. Limitations

* **No closed-loop Lyapunov stability proof** for the fuzzy+override system outside the regime where it provably reduces to vanilla Max-Pressure.
* **Membership function breakpoints are hand-set**, not calibrated via formal optimization.
* **Single intersection only.** No network-level or corridor-level claim is made.
* **Demand sweep uses 3 seeds per point**, not 10.
* **Box-gridlock counts remain noisy** at this sample size.

---

## 8. Reproducibility

```bash
# Activate environment
conda activate traffic_research_env
cd ~/anti-gridlock-signal-control

# Build network topology
python configs/build_network.py

# Run full evaluation sweep
python evaluate.py --seeds 1 2 3 4 5 6 7 8 9 10 --sweep-seeds 1 2 3 \
    --scales 0.6 0.8 1.0 1.2 1.5 1.8 2.0


برای اینکه صفحه گیت‌هاب شما دقیقاً مثل تصویر دوم (ریپازیتوری قبلی‌تان) حرفه‌ای، چشم‌نواز و استاندارد مقالات سطح یک (Q1) شود، چند عنصر کلیدی اضافه شده است:
بج‌های رنگی وضعیت (Badges): برچسب‌های نسخه پایتون، شبیه‌ساز SUMO، وضعیت اعتبارسنجی و لایسنس در بالای صفحه.
کادرهای هایلایت گیت‌هاب (> [!NOTE] و `> [!IMPORTANT]): برای جلب توجه داور یا بیننده به پیام‌های کلیدی.
نمایش خودکار نمودارها: در متن شما ذکر شده بود که اسکریپت تصاویر را در پوشه results/ می‌سازد؛ حالا کد نمایش مستقیم این تصاویر (.png) دقیقاً زیر هر بخش از نتایج قرار گرفته تا صفحه از حالت متنِ خالی خارج شود.
متن دست‌نخورده: تمامی اعداد، تحلیل‌ها، لغات و توضیحات علمی کلاد ۱۰۰٪ حفظ شده‌اند.
مراحل اجرا:
۱. در ترمینال WSL دستور زیر را بزنید تا فایل در VS Code باز شود:
code
Bash
code README.md
۲. هر چه در فایل هست را با Ctrl + A و سپس Delete پاک کنید.
۳. کل متن داخل کادر زیر را کپی کرده، داخل VS Code پیست کنید و Ctrl + S را بزنید تا ذخیره شود:
code
Markdown
<div align="center">

# Anti-Gridlock Signal Control
### A Queue-Dissipation Max-Pressure Controller with Fuzzy Anti-Spillback Regulation

[![Python](https://img.shields.io/badge/Python-3.10-blue.svg?logo=python&logoColor=white)](https://www.python.org/)
[![SUMO](https://img.shields.io/badge/SUMO-1.27.1-darkgreen.svg)](https://eclipse.dev/sumo/)
[![Status](https://img.shields.io/badge/Status-Phase%201%20Validated-brightgreen.svg)]()
[![Benchmark](https://img.shields.io/badge/Benchmark-10--Seed%20Paired-blueviolet.svg)]()
[![Research Track](https://img.shields.io/badge/Target-Q1%20Submission%20Track-orange.svg)]()
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

<p align="center">
  <b>Phase 1: Single-Intersection Local Control Layer</b><br>
  <i>Statistically benchmarked, causally ablated, and submission-track validated control substrate.</i>
</p>

</div>

---

> [!NOTE]
> **Repository Scope & Role:** This repository serves as the local control substrate for a planned network-level emergency-corridor navigation stack (time-varying, fault-tolerant geometric spanners + dynamic green wave). This repository stands on its own and makes no claims beyond the single-intersection scope tested here.

---

## 📌 Abstract

Max-Pressure signal control (Varaiya, 2013) is throughput-optimal under the assumption of unbounded downstream storage — an assumption that fails at short urban links, where a discharging phase can push a queue the receiving link cannot absorb, backing traffic up into the junction box itself and gridlocking the cross street. 

We construct a symmetric four-approach intersection with an engineered downstream bottleneck (50 m metered egress against 250 m upstream storage) to reproduce this failure mode under controlled conditions, and evaluate three control policies — **Webster fixed-time**, **vanilla Max-Pressure**, and a **Max-Pressure variant with a Mamdani fuzzy inference throttle** on green duration — across:
- 10 paired random seeds
- A seven-point demand sweep
- A chronic-oversaturation stress regime
- A controlled emergency-vehicle preemption ablation

We report effect sizes (Cohen's $d_z$), Holm–Bonferroni corrected significance, and bootstrap confidence intervals throughout. We transparently report the fuzzy controller's genuine trade-off — junction-box protection traded against upstream storage load on the protected corridor's counterpart — rather than presenting a single composite metric that would obscure it. 

The fuzzy controller reduces mean delay relative to vanilla Max-Pressure (**243.4s vs. 261.0s, $p = 0.033$**, moderate regime) and improves movement fairness relative to Webster fixed-time, while a controlled ablation isolates a measurable causal benefit of emergency-vehicle preemption on ambulance delay (**122.0s vs. 142.5s, ≈14% reduction**, same seeds, same base algorithm).

---

## 1. Problem Statement

Standard Max-Pressure signal control selects the phase that maximizes a pressure differential between upstream and downstream queues, and is provably throughput-optimal under the assumption that the network's stability region is unconstrained by receiving-link storage. Real intersections violate this assumption whenever a downstream link is short relative to the demand it must discharge: a phase can serve its upstream queue faster than the receiving link can absorb it, and the excess vehicles queue back through the junction box itself, blocking the conflicting movement and producing exactly the kind of local gridlock that a corridor-level green wave cannot survive.

> **Core Research Question:** Does adding a fuzzy-logic throttle on green duration, conditioned on downstream occupancy, reduce junction-box lock-up without an unacceptable cost elsewhere, relative to both a conventional fixed-time baseline and unmodified Max-Pressure?

---

## 2. System Model

### 2.1 Geometry
A single symmetric four-leg intersection `C`, with each approach direction $d \in \{N, S, E, W\}$ composed of:

| Segment | Length | Lanes | Speed | Role |
|:---|:---:|:---:|:---:|:---|
| `app_d` | 200 m | 2 | 13.89 m/s | Upstream arterial storage |
| `in_d` | 50 m | 2 | 13.89 m/s | Stop-line section |
| `out_d` | 50 m | 1 | 8.33 m/s | Metered egress (bottleneck) |
| `exit_d` | 200 m | 1 | 8.33 m/s | Sink |

Combined approach-group physical storage capacity (`app_d` + `in_d`, 2 lanes, 2 directions, 7.5 m/vehicle spacing) is **133 vehicles** per movement group — the reference denominator for the storage-overflow metric defined in §4.3.

### 2.2 Demand Regimes

Two demand regimes are used, both generated as a non-homogeneous Poisson process with three deterministic 300 s surge windows (2× base rate) superimposed on a Poisson base rate, strictly sorted by departure time:

* **Moderate regime** (primary benchmark): NS egress metering duty 0.46 (≈874 veh/h/lane sustained capacity), placing the arterial near saturation with recoverable bursts rather than permanent gridlock.
* **Stress regime** (explicit secondary ablation): NS egress metering duty 0.30 (≈570 veh/h/lane), placing the arterial in chronic, sustained oversaturation (V/C ≈ 1.3) for the full 3600 s.

EW egress duty is fixed at 0.60 in both regimes (≈1140 veh/h/lane), comfortably above the 400 veh/h EW demand, isolating the experiment to how well each controller protects the cross street from the arterial. Metering timing is applied dynamically at simulation start via TraCI (`src/metering.py`).

---

## 3. Control Architecture

### 3.1 Pressure Signal
Phase pressure is a queue-dissipation-time differential rather than a raw vehicle count:

$$w_p = t_D^p - t_D^q$$

where $t_D^p$ and $t_D^q$ are estimated discharge times (at 1900 veh/h/lane saturation flow) of upstream and downstream halting queues for phase $p$.

### 3.2 Fuzzy Anti-Spillback Throttle
A Mamdani fuzzy inference system (`src/fuzzy_engine.py`, centroid defuzzification) maps $(w_p, \text{downstream\_occupancy})$ to a green duration bounded in $[T_{\min}, T_{\max}] = [10s, 60s]$. Downstream occupancy is fed as a **50-second rolling average**, matching the metering cycle length.

> [!IMPORTANT]
> **Phase-Selection Lemma:** The fuzzy engine only ever *shortens* green relative to unconstrained Max-Pressure; it never re-weights $w_p$ or alters phase selection itself. Below the critical occupancy threshold, the controller is *identical* to vanilla Max-Pressure. Above the threshold, a hard override enforces:
> $$h(x) = \text{OCC\_OVERRIDE} - \text{occ}(x), \quad \text{OCC\_OVERRIDE} = 0.75$$
> acting as a control-barrier safety margin.

### 3.3 Emergency Preemption
A `vClass="emergency"` vehicle detected on an approach edge (`app_d`/`in_d` only) triggers an immediate transition to its phase group, held for up to 45 s or until clearing the stop-line section.

---

## 4. Experimental Design

### 4.1 Policies Compared
1. **Fixed-Time (Webster):** Cycle length and split computed from Webster's delay-minimizing formula.
2. **Vanilla Max-Pressure:** Full-strength phase-pressure control with no downstream awareness (Varaiya, 2013).
3. **Fuzzy Anti-Spillback Max-Pressure:** The proposed controller (§3).

### 4.2 Statistical Methodology
* **Paired Multi-Seed Design:** 10 shared random seeds per regime for paired statistical tests (Wilcoxon signed-rank, matched-pairs Cohen's $d_z$).
* **Multiple Testing Correction:** Holm–Bonferroni correction applied across the family of 15 pairwise comparisons per regime (3 policy pairs × 5 metrics).
* **Safety Indices Disentanglement:** Rejection of arbitrary composite indices; box-gridlock and directional storage-overflow are isolated and evaluated independently.

---

## 5. Experimental Results

### 5.1 Primary Benchmark (Moderate Regime, n = 10 paired seeds)

| Policy | Delay [s], mean (95% CI) | Throughput [veh] (95% CI) | Jain's Fairness Index |
|:---|:---:|:---:|:---:|
| **Fixed-Time (Webster)** | 234.8 (232.4–237.3) | 1659 (1647–1671) | ≈0.68 |
| **Vanilla Max-Pressure** | 261.0 (259.2–262.7) | 1696 (1681–1710) | ≈0.94 |
| **Fuzzy Anti-Spillback** | 243.4 (239.6–246.8) | 1658 (1646–1670) | ≈0.77 |

<p align="center">
  <img src="results/comparison_metrics.png" alt="Comparison Metrics" width="85%">
</p>

**Holm–Bonferroni Corrected Significance:**
* Fuzzy vs. Vanilla Max-Pressure, delay: **$p = 0.033$** (Fuzzy statistically lower)
* Fixed-Time vs. Fuzzy, delay: **$p = 0.035$** (Fixed-Time lower)
* Fixed-Time vs. Vanilla Max-Pressure, delay: **$p = 0.033$** (Fixed-Time lower)

---

### 5.2 Demand Sweep (Moderate Regime)

Across V/C scale 0.6–2.0, Fixed-Time's delay grows steeply beyond scale ≈1.2 (94.7s → 396.7s), while both Max-Pressure variants remain comparatively flat (Fuzzy: 146.5s → 308.4s; Vanilla: 84.1s → 282.5s).

<p align="center">
  <img src="results/demand_sweep.png" alt="Demand Sweep" width="85%">
</p>

---

### 5.3 Direction-Specific Storage-Overflow Trade-off

Across both regimes, **100% of storage-overflow events under the fuzzy controller occur on the NS (arterial) approach group**; the EW group shows zero storage-overflow events in every run.

<p align="center">
  <img src="results/storage_overflow_breakdown.png" alt="Storage Overflow Breakdown" width="85%">
</p>

> [!NOTE]
> *Mechanistic Trade-off:* The fuzzy anti-spillback mechanism redistributes congestion risk from the junction box to upstream arterial storage, rather than eliminating congestion risk outright.

---

### 5.4 Fairness and Stress Regimes

<p align="center">
  <img src="results/fairness.png" alt="Fairness Comparison" width="48%">
  <img src="results/comparison_metrics_stress.png" alt="Stress Regime Comparison" width="48%">
</p>

---

### 5.5 Emergency Preemption: Controlled Causal Ablation

Isolated preemption test with identical seeds and base algorithm:

| Condition | Ambulance in-network delay [s] | Relative Benefit |
|:---|:---:|:---:|
| **Without preemption** | 142.5 s | Baseline |
| **With preemption** | **122.0 s** | **≈14% delay reduction** |

<p align="center">
  <img src="results/preemption_ablation.png" alt="Preemption Ablation" width="70%">
</p>

---

## 6. Discussion

1. **The Fixed-Time Advantage Is Real and Not Hidden:** At moderate baseline demand (scale = 1.0), Webster fixed-time achieves lower total delay due to the "price-of-fairness". Demand sweeps show this advantage inverts as demand rises.
2. **What the Fuzzy Controller Actually Buys:** Statistically significant delay reduction relative to unmodified Max-Pressure, improved movement fairness over fixed-time, and a causally isolated benefit to emergency vehicles.
3. **On the Rejected Composite Metric:** Collapsing box-gridlock and storage-overflow into an arbitrary composite index obscures direction-specific protection mechanics. They are reported independently.

---

## 7. Limitations

* **No closed-loop Lyapunov stability proof** for the fuzzy+override system outside the regime where it provably reduces to vanilla Max-Pressure.
* **Membership function breakpoints are hand-set**, not calibrated via formal optimization.
* **Single intersection only.** No network-level or corridor-level claim is made.
* **Demand sweep uses 3 seeds per point**, not 10.
* **Box-gridlock counts remain noisy** at this sample size.

---

## 8. Reproducibility

```bash
# Activate environment
conda activate traffic_research_env
cd ~/anti-gridlock-signal-control

# Build network topology
python configs/build_network.py

# Run full evaluation sweep
python evaluate.py --seeds 1 2 3 4 5 6 7 8 9 10 --sweep-seeds 1 2 3 \
    --scales 0.6 0.8 1.0 1.2 1.5 1.8 2.0
Environment: Ubuntu (WSL2), SUMO 1.27.1, Python 3.10, traci, sumolib, scikit-fuzzy, numpy, scipy, matplotlib.
9. Roadmap
This repository establishes the local-control layer only. Phase 2 integrates this controller beneath a network-wide dynamic green wave driven by time-varying, fault-tolerant geometric spanners for emergency-vehicle routing across multiple intersections.
References
Varaiya, P. (2013). Max pressure control of a network of signalized intersections. Transportation Research Part C, 36, 177–195.
Webster, F. V. (1958). Traffic Signal Settings. Road Research Technical Paper No. 39, HMSO.
Jain, R., Chiu, D., & Hawe, W. (1984). A quantitative measure of fairness and discrimination for resource allocation in shared computer systems. DEC Research Report TR-301.
Ames, A. D., Coogan, S., Egerstedt, M., Notomista, G., Sreenath, K., & Tabuada, P. (2019). Control barrier functions: Theory and applications. 2019 18th European Control Conference (ECC).