"""Render the results tables of README.md from the recorded evaluation files.

Tables sit between markers  <!-- BEGIN:name --> ... <!-- END:name -->  and are generated here,
so no number inside them is typed by hand.

    python render_readme_tables.py            # rewrite the tables in README.md
    python render_readme_tables.py --check    # exit 1 if README.md differs from the data

Sources: results/statistical_report.json (single junction), multi/results/corridor_eval.json and
multi/results/corridor_eval_followup.json (three-junction corridor), results/crisp_ablation_report.json
(crisp-twin ablation).
"""
import argparse
import json
import os
import re
import sys

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
README = os.path.join(ROOT, "README.md")
FIXED, VANILLA, FUZZY = "Fixed-Time (Webster)", "Vanilla Max-Pressure", "Fuzzy Anti-Spillback"
POLICIES = [FIXED, VANILLA, FUZZY]


def load(path):
    with open(os.path.join(ROOT, path), encoding="utf-8") as fh:
        return json.load(fh)


def md(header, rows):
    out = ["| " + " | ".join(header) + " |", "| " + " | ".join(["---"] * len(header)) + " |"]
    return "\n".join(out + ["| " + " | ".join(str(c) for c in r) + " |" for r in rows])


def mean(rep, regime, policy, key):
    return rep[regime]["summary"][policy][key]["mean"]


def holm(rep, regime, a, b, key):
    d = rep[regime]["wilcoxon_p_holm_bonferroni"]
    return d.get(f"{a} vs {b} | {key}", d.get(f"{b} vs {a} | {key}"))


def pct(new, old):
    return 100.0 * (new - old) / old


def table_glance(rep):
    m, s = "moderate_regime", "stress_regime_ablation"
    dv = (mean(rep, m, FUZZY, "avg_delay_s"), mean(rep, m, VANILLA, "avg_delay_s"))
    ds = (mean(rep, s, FUZZY, "avg_delay_s"), mean(rep, s, VANILLA, "avg_delay_s"))
    box = {p: mean(rep, m, p, "box_gridlock_events") for p in POLICIES}
    lowest = min(box, key=box.get) == FUZZY
    w, wo = np.mean(rep["preemption_ablation"]["with"]), np.mean(rep["preemption_ablation"]["without"])
    co_m = pct(mean(rep, m, FUZZY, "total_co2_kg"), mean(rep, m, VANILLA, "total_co2_kg"))
    co_s = pct(mean(rep, s, FUZZY, "total_co2_kg"), mean(rep, s, VANILLA, "total_co2_kg"))
    rows = [
        ["Delay vs. Vanilla Max-Pressure, moderate regime",
         f"**{pct(*dv):+.1f}%** ({dv[0]:.1f} s vs. {dv[1]:.1f} s), Holm *p* = {holm(rep, m, VANILLA, FUZZY, 'avg_delay_s'):.3f}"],
        ["Junction-box gridlock events, moderate regime",
         ("**lowest of the three policies** " if lowest else "not the lowest ") +
         f"({box[FUZZY]:.1f} vs. {min(box[FIXED], box[VANILLA]):.1f} to {max(box[FIXED], box[VANILLA]):.1f} per run)"],
        ["Delay vs. Vanilla Max-Pressure, chronic stress",
         f"**{pct(*ds):+.1f}%** ({ds[0]:.1f} s vs. {ds[1]:.1f} s)"],
        ["Emergency-vehicle delay, preemption on vs. off",
         f"**{pct(w, wo):+.0f}%** ({w:.1f} s vs. {wo:.1f} s), Wilcoxon *p* = {rep['preemption_ablation']['wilcoxon_p']:.3f}"],
        ["CO2 vs. Vanilla Max-Pressure", f"**{co_m:+.1f}%** (moderate) and **{co_s:+.1f}%** (stress)"],
        ["Cost, stated plainly", "underperforms both baselines at loose bottlenecks (Section 10.2); no clear advantage over Fixed-Time in the three-junction corridor (Section 5.6)"],
    ]
    return md(["Finding", "Value"], rows)


def table_holm(rep):
    m = "moderate_regime"
    pairs = [(FIXED, VANILLA, "Fixed-Time vs. Vanilla MP"), (FIXED, FUZZY, "Fixed-Time vs. Fuzzy"), (VANILLA, FUZZY, "Vanilla MP vs. Fuzzy")]
    metrics = [("avg_delay_s", "Delay"), ("mean_queue_length", "Mean queue length"), ("throughput_completed_trips", "Throughput"),
               ("box_gridlock_events", "Box-gridlock events"), ("storage_overflow_events_NS", "Storage overflow, NS"), ("total_co2_kg", "CO2")]
    rows = []
    for key, lab in metrics:
        cells = []
        for a, b, _ in pairs:
            p = holm(rep, m, a, b, key)
            cells.append("identical" if not isinstance(p, (int, float)) else (f"**{p:.3f}**" if p < 0.05 else f"{p:.3f}"))
        rows.append([lab] + cells)
    return md(["Metric"] + [n for _, _, n in pairs], rows) + "\n\nHolm-adjusted p-values, moderate regime; bold entries are below 0.05."


def table_primary(rep):
    m = "moderate_regime"
    rows = []
    for p in POLICIES:
        jain = np.mean(rep[m]["jains_index"][p])
        rows.append([p, f"{mean(rep, m, p, 'avg_delay_s'):.1f}", f"{mean(rep, m, p, 'throughput_completed_trips'):.0f}",
                     f"{mean(rep, m, p, 'box_gridlock_events'):.1f}", f"{jain:.3f}"])
    return md(["Policy", "Delay [s], mean", "Throughput [veh]", "Box-gridlock events", "Jain's Fairness Index"], rows)


def table_stress(rep):
    s = "stress_regime_ablation"
    rows = [[p, f"{mean(rep, s, p, 'avg_delay_s'):.1f}", f"{mean(rep, s, p, 'throughput_completed_trips'):.0f}"] for p in POLICIES]
    return md(["Policy", "Delay [s], mean", "Throughput [veh]"], rows)


def table_preemption(rep):
    a = rep["preemption_ablation"]
    return md(["Condition", "Ambulance in-network delay [s], mean"],
              [["Without preemption", f"{np.mean(a['without']):.1f}"], ["With preemption", f"{np.mean(a['with']):.1f}"]])


def table_corridor():
    d = load("multi/results/corridor_eval.json")
    names = ["Fixed-Time", "Vanilla MP", "Fuzzy AS-MP"]
    keys = [("avg_delay_s", "Mean delay (s)"), ("box_gridlock_events", "Box-blocking events"),
            ("link_spillback_events", "Link-spillback events"), ("mean_corridor_queue", "Mean corridor queue")]
    rows = []
    for lv, g in d["levels"].items():
        for k, lab in keys:
            rows.append([f"{lv} ({g} s)", lab] + [f"{d['results'][lv]['summary'][n][k]['mean']:.2f}" for n in names])
    return md(["Metering level (east-exit green per 50 s)", "Metric"] + names, rows) + \
        f"\n\nSeeds {d['seeds'][0]}-{d['seeds'][-1]} (blind), ten paired runs per cell. Source: `multi/results/corridor_eval.json`."


def table_corridor_followup():
    d = load("multi/results/corridor_eval_followup.json")
    names = {"fixed": "Fixed-Time", "vanilla": "Vanilla MP", "fuzzy": "Fuzzy (override 0.75)", "fuzzy60": "Fuzzy (override 0.60)"}
    full = {"fixed": "Fixed-Time", "vanilla": "Vanilla MP", "fuzzy": "Fuzzy AS-MP", "fuzzy60": "Fuzzy AS-MP (override 0.60)"}
    keys = [("avg_delay_s", "Mean delay (s)"), ("box_gridlock_events", "Box-blocking events"),
            ("link_spillback_events", "Link-spillback events"), ("mean_corridor_queue", "Mean corridor queue")]
    rows = []
    for lv, g in d["levels"].items():
        for k, lab in keys:
            rows.append([f"{lv} ({g} s)", lab] + [f"{d['results'][lv]['summary'][full[p]][k]['mean']:.2f}" for p in d["policies"]])
    ov = d["overrides_mean"]["severe"][full["fuzzy60"]]
    return md(["Metering level", "Metric"] + [names[p] for p in d["policies"]], rows) + \
        f"\n\nSeeds {d['seeds'][0]}-{d['seeds'][-1]}. With the threshold at 0.60 the override engaged on average {ov:.0f} times per run at the severe level; at 0.75 it never engaged. " \
        "Source: `multi/results/corridor_eval_followup.json`."


CRISP = "Crisp-Twin Anti-Spillback"
CRISP_METRICS = [("avg_delay_s", "Mean delay [s]", 1), ("mean_queue_length", "Mean queue [veh]", 1),
                 ("box_gridlock_events", "Box-blocking events", 2)]
ALPHA = 0.05


def fmt_p(p):
    return "<0.001" if p < 0.001 else f"{p:.3f}"


def eq_p(p):
    return "p < 0.001" if p < 0.001 else f"p = {p:.3f}"


def table_crisp():
    rep = load("results/crisp_ablation_report.json")
    seeds = rep["seeds"]
    out = [f"Seeds {seeds[0]}-{seeds[-1]} (n = {len(seeds)} per regime), paired by seed; mean with 95% bootstrap interval.", ""]
    rows = []
    for regime in ("moderate", "stress"):
        for pol in (VANILLA, FUZZY, CRISP):
            cells = []
            for key, _, nd in CRISP_METRICS:
                v = rep[regime]["summary"][pol][key]
                cells.append(f"{v['mean']:.{nd}f} ({v['ci95_lo']:.{nd}f} to {v['ci95_hi']:.{nd}f})")
            rows.append([regime, pol] + cells)
    out += [md(["Regime", "Policy"] + [m[1] for m in CRISP_METRICS], rows), "",
            "Paired difference, Fuzzy Anti-Spillback minus Crisp-Twin Anti-Spillback (negative favours the fuzzy engine); "
            f"Holm correction over the {len(rep['moderate']['wilcoxon_p_raw'])} comparisons of each regime "
            "(three policy pairs, nine metrics).", ""]
    rows, lines = [], []
    for regime in ("moderate", "stress"):
        d = rep[regime]
        for key, label, nd in CRISP_METRICS:
            pair = f"{FUZZY} vs {CRISP} | {key}"
            diff = mean(rep, regime, FUZZY, key) - mean(rep, regime, CRISP, key)
            p_raw, p_holm, dz = d["wilcoxon_p_raw"][pair], d["wilcoxon_p_holm_bonferroni"][pair], d["cohens_dz"][pair]
            rows.append([regime, label, f"{diff:+.{nd}f}", fmt_p(p_raw), fmt_p(p_holm), f"{dz:+.2f}"])
            if p_holm < ALPHA:
                side = "lower" if diff < 0 else "higher"
                lines.append(f"- {regime}, {label.lower()}: the fuzzy engine is {side} than its crisp twin "
                             f"({abs(100 * diff / mean(rep, regime, CRISP, key)):.1f}%, Holm-adjusted p = {fmt_p(p_holm)}).")
            else:
                lines.append(f"- {regime}, {label.lower()}: no significant difference (Holm-adjusted {eq_p(p_holm)}).")
    out += [md(["Regime", "Metric", "Difference", "Wilcoxon p", "Holm p", "Cohen's dz"], rows), "",
            "Reading, generated from the table above:", ""] + lines + [""]
    share = []
    for regime in ("moderate", "stress"):
        v, f, c = (mean(rep, regime, p, "avg_delay_s") for p in (VANILLA, FUZZY, CRISP))
        share.append(f"{regime} {100 * (v - c) / (v - f):.0f}%")
    out.append("Share of the fuzzy engine's mean-delay reduction relative to Vanilla Max-Pressure that the crisp twin already achieves: "
               + ", ".join(share) + ". Source: `results/crisp_ablation_report.json`.")
    return "\n".join(out)


def render(text):
    rep = load("results/statistical_report.json")
    blocks = {"glance": lambda: table_glance(rep), "holm": lambda: table_holm(rep), "primary": lambda: table_primary(rep), "stress": lambda: table_stress(rep),
              "preemption": lambda: table_preemption(rep), "corridor": table_corridor, "corridor_followup": table_corridor_followup,
              "crisp": table_crisp}
    for name, fn in blocks.items():
        pat = re.compile(rf"(<!-- BEGIN:{name} -->)\n.*?(<!-- END:{name} -->)", re.S)
        if not pat.search(text):
            sys.exit(f"README.md has no marker block for '{name}'")
        text = pat.sub(lambda m: m.group(1) + "\n" + fn() + "\n" + m.group(2), text)
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    current = open(README, encoding="utf-8", newline="").read()
    nl = "\r\n" if "\r\n" in current else "\n"
    rendered = render(current.replace("\r\n", "\n")).replace("\n", nl)
    if args.check:
        if rendered != current:
            print("README.md tables differ from the recorded data; run: python render_readme_tables.py")
            sys.exit(1)
        print("README.md tables match the recorded data.")
        return
    open(README, "w", encoding="utf-8", newline="").write(rendered)
    print("README.md tables rendered from the recorded data")


if __name__ == "__main__":
    main()
