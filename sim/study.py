"""One command that evaluates the controller on every road type and writes a report.

    python study.py                       # all road types, all scenarios, 3 seeds (long: see below)
    python study.py --quick               # crossroads + T-junction, 3 scenarios, 1 seed (a few minutes)
    python study.py --layouts crossroads t_junction --seeds 5
    python study.py --miss-rates 0 0.15   # also test a camera that misses 15% of vehicles
    python study.py --site ../site.json   # add the junction your camera learned
    python study.py --no-pedestrians      # vehicles only

For each road type it builds the SUMO network (build_network.py), then runs
fixed-time, longest-queue and adaptive controllers on identical traffic (compare.py), and
finally combines everything into:

    results/study/study_summary.csv   one row per layout x scenario x controller
    results/study/STUDY_REPORT.md     headline table + per-layout tables, ready to paste into a report
    results/study/study_chart.png     change in delay versus fixed-time (needs matplotlib)

Time: every run is 30 simulated minutes. Expect roughly 10 to 60 seconds of real time each
(an estimate, it depends on your machine), so the full study is 7 road types x 6 scenarios x
3 controllers x 3 seeds = 378 runs, which can take hours. Use --quick first. Finished road
types are skipped when you run it again (use --force to redo them), so it can be stopped and resumed.
A road type that fails to build is reported and skipped; the others continue.
"""
import argparse
import csv
import os
import subprocess
import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
ALL_LAYOUTS = ["crossroads", "t_junction", "y_junction", "five_way", "one_way_cross",
               "shuttle", "road_crossing"]
QUICK_LAYOUTS = ["crossroads", "t_junction"]
ALL_SCENARIOS = ["balanced", "main_peak", "tidal", "one_sided", "surge", "ped_heavy"]
QUICK_SCENARIOS = ["balanced", "tidal", "surge"]
CONTROLLERS = ["fixed", "longest_queue", "adaptive"]
NUMERIC = ["mean_delay_s", "p95_delay_s", "max_wait_s", "mean_queue_veh", "max_queue_veh",
           "throughput_vph", "phase_changes", "unfinished", "ped_mean_delay_s",
           "ped_p95_delay_s", "ped_max_wait_s", "peds_completed"]


def run_layout(layout, scenarios, seeds, miss_rates, out_dir, pedestrians):
    """Build one road type and run the comparison on it. Returns (ok, message)."""
    env = dict(os.environ, SIM_LAYOUT=str(layout))
    build = [sys.executable, "build_network.py"] + ([] if pedestrians else ["--no-pedestrians"])
    done = subprocess.run(build, cwd=SIM_DIR, env=env, capture_output=True, text=True)
    if done.returncode != 0:
        tail = (done.stdout + done.stderr).strip().splitlines()[-6:]
        return False, "network build failed:\n    " + "\n    ".join(tail)
    cmd = [sys.executable, "compare.py", "--scenarios", *scenarios, "--controllers", *CONTROLLERS,
           "--seeds", str(seeds), "--miss-rates", *map(str, miss_rates), "--out", str(out_dir)]
    if not pedestrians:
        cmd.append("--no-pedestrians")
    done = subprocess.run(cmd, cwd=SIM_DIR, env=env)
    if done.returncode != 0:
        return False, "simulation run failed (see the messages above)"
    return True, "ok"


def read_summary(path, layout):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            row = {"layout": layout, "scenario": r["scenario"], "controller": r["controller"],
                   "miss_rate": float(r["miss_rate"]), "runs": int(float(r["runs"]))}
            for k in NUMERIC:
                row[k] = float(r[k]) if r.get(k) not in (None, "") else 0.0
            rows.append(row)
    return rows


def pct_change(value, base):
    return 100.0 * (value - base) / base if base else None


def changes_vs_fixed(rows, metric="mean_delay_s"):
    """{(layout, scenario, miss_rate, controller): % change of the metric against fixed-time}."""
    base = {(r["layout"], r["scenario"], r["miss_rate"]): r[metric] for r in rows if r["controller"] == "fixed"}
    out = {}
    for r in rows:
        b = base.get((r["layout"], r["scenario"], r["miss_rate"]))
        c = pct_change(r[metric], b) if b is not None else None
        if c is not None:
            out[(r["layout"], r["scenario"], r["miss_rate"], r["controller"])] = c
    return out


def headline(rows, miss_rate):
    """Per layout and controller: average % change vs fixed over scenarios, and scenarios won."""
    delay = changes_vs_fixed(rows, "mean_delay_s")
    p95 = changes_vs_fixed(rows, "p95_delay_s")
    wait = changes_vs_fixed(rows, "max_wait_s")
    table = []
    for layout in dict.fromkeys(r["layout"] for r in rows):
        for ctl in CONTROLLERS[1:]:
            keys = [k for k in delay if k[0] == layout and k[2] == miss_rate and k[3] == ctl]
            if not keys:
                continue
            avg = lambda d: sum(d[k] for k in keys if k in d) / max(1, sum(1 for k in keys if k in d))
            table.append({"layout": layout, "controller": ctl, "scenarios": len(keys),
                          "delay": avg(delay), "p95": avg(p95), "max_wait": avg(wait),
                          "wins": sum(1 for k in keys if delay[k] < 0)})
    return table


def markdown_report(rows, miss_rates, seeds, pedestrians, failures):
    base_miss = miss_rates[0]
    lines = ["# Multi-road-type simulation study", "",
             f"Seeds per combination: {seeds}. Pedestrians: {'on' if pedestrians else 'off'}. "
             f"Camera miss rate for the headline table: {base_miss:.0%}.",
             "Negative percentages mean less delay than the fixed-time plan on identical traffic.", ""]
    lines += ["## Headline: change versus fixed-time, averaged over scenarios", "",
              "| Road type | Controller | Mean delay | 95th-percentile delay | Longest wait | Scenarios better than fixed |",
              "|---|---|---|---|---|---|"]
    for h in headline(rows, base_miss):
        lines.append(f"| {h['layout']} | {h['controller']} | {h['delay']:+.1f}% | {h['p95']:+.1f}% | "
                     f"{h['max_wait']:+.1f}% | {h['wins']} of {h['scenarios']} |")
    lines.append("")
    adaptive = [h for h in headline(rows, base_miss) if h["controller"] == "adaptive"]
    if adaptive:
        best = min(adaptive, key=lambda h: h["delay"])
        worst = max(adaptive, key=lambda h: h["delay"])
        lines += [f"Adaptive control changed mean delay by {best['delay']:+.1f}% on {best['layout']} (best) and "
                  f"{worst['delay']:+.1f}% on {worst['layout']} (worst) relative to fixed-time.", ""]
    delay = changes_vs_fixed(rows, "mean_delay_s")
    for layout in dict.fromkeys(r["layout"] for r in rows):
        lines += [f"## {layout}", "",
                  "| Scenario | Controller | Delay (s) | vs fixed | 95th pct (s) | Longest wait (s) | Max queue (veh) | Pedestrian delay (s) |",
                  "|---|---|---|---|---|---|---|---|"]
        for r in sorted((r for r in rows if r["layout"] == layout and r["miss_rate"] == base_miss),
                        key=lambda r: (r["scenario"], CONTROLLERS.index(r["controller"]))):
            c = delay.get((layout, r["scenario"], r["miss_rate"], r["controller"]))
            lines.append(f"| {r['scenario']} | {r['controller']} | {r['mean_delay_s']:.1f} | "
                         f"{'-' if c is None or r['controller'] == 'fixed' else f'{c:+.0f}%'} | "
                         f"{r['p95_delay_s']:.1f} | {r['max_wait_s']:.1f} | {r['max_queue_veh']:.1f} | "
                         f"{r['ped_mean_delay_s']:.1f} |")
        lines.append("")
    if len(miss_rates) > 1:
        lines += ["## Robustness to camera errors (adaptive controller)", "",
                  "| Road type | Camera misses | Mean delay vs fixed | 95th-percentile delay vs fixed |", "|---|---|---|---|"]
        p95 = changes_vs_fixed(rows, "p95_delay_s")
        for layout in dict.fromkeys(r["layout"] for r in rows):
            for m in miss_rates:
                ks = [k for k in delay if k[0] == layout and k[2] == m and k[3] == "adaptive"]
                if ks:
                    lines.append(f"| {layout} | {m:.0%} | {sum(delay[k] for k in ks) / len(ks):+.1f}% | "
                                 f"{sum(p95[k] for k in ks if k in p95) / len(ks):+.1f}% |")
        lines.append("")
    if failures:
        lines += ["## Not run", ""] + [f"* {name}: {why.splitlines()[0]}" for name, why in failures.items()] + [""]
    lines += ["These are simulation results on illustrative demand, not measurements from Lusaka. "
              "Replace the demand in make_demand.py with real counts before quoting them.", ""]
    return "\n".join(lines)


def write_chart(rows, miss_rate, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    delay = changes_vs_fixed(rows, "mean_delay_s")
    layouts = list(dict.fromkeys(r["layout"] for r in rows))
    scenarios = list(dict.fromkeys(r["scenario"] for r in rows))
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), sharey=True)
    for ax, ctl in zip(axes, CONTROLLERS[1:]):
        width = 0.8 / max(1, len(scenarios))
        for i, sc in enumerate(scenarios):
            vals = [delay.get((l, sc, miss_rate, ctl), 0.0) for l in layouts]
            ax.bar([x + i * width for x in range(len(layouts))], vals, width, label=sc)
        ax.axhline(0, color="black", linewidth=0.8)
        ax.set_xticks([x + 0.4 - width / 2 for x in range(len(layouts))])
        ax.set_xticklabels(layouts, rotation=30, ha="right")
        ax.set_title(f"{ctl} vs fixed-time")
    axes[0].set_ylabel("Change in mean delay (%), below zero is better")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--layouts", nargs="+", default=None, help=f"default: {' '.join(ALL_LAYOUTS)}")
    parser.add_argument("--scenarios", nargs="+", default=None)
    parser.add_argument("--seeds", type=int, default=None)
    parser.add_argument("--miss-rates", nargs="+", type=float, default=[0.0])
    parser.add_argument("--quick", action="store_true", help="2 road types, 3 scenarios, 1 seed")
    parser.add_argument("--site", default=None, help="also run the layout in this site.json")
    parser.add_argument("--no-pedestrians", action="store_true")
    parser.add_argument("--force", action="store_true", help="redo road types that already have results")
    parser.add_argument("--out", default=str(SIM_DIR / "results" / "study"))
    args = parser.parse_args()

    layouts = args.layouts or (QUICK_LAYOUTS if args.quick else ALL_LAYOUTS)
    scenarios = args.scenarios or (QUICK_SCENARIOS if args.quick else ALL_SCENARIOS)
    seeds = args.seeds or (1 if args.quick else 3)
    pedestrians = not args.no_pedestrians
    if not pedestrians:
        scenarios = [s for s in scenarios if s != "ped_heavy"]
    if args.site:
        layouts = layouts + [str(Path(args.site).resolve())]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rows, failures = [], {}
    for n, layout in enumerate(layouts, 1):
        name = Path(layout).stem if layout.endswith(".json") else layout
        sub = out / name
        print(f"\n=== [{n}/{len(layouts)}] {name} ===", flush=True)
        if (sub / "summary.csv").exists() and not args.force:
            print("  results found, skipping (use --force to redo)")
        else:
            ok, msg = run_layout(layout, scenarios, seeds, args.miss_rates, sub, pedestrians)
            if not ok:
                failures[name] = msg
                print(f"  SKIPPED {name}: {msg}")
                continue
        rows += read_summary(sub / "summary.csv", name)

    if not rows:
        sys.exit("No results to report.")
    with open(out / "study_summary.csv", "w", newline="") as f:
        fields = ["layout", "scenario", "miss_rate", "controller", "runs"] + NUMERIC
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    report = markdown_report(rows, args.miss_rates, seeds, pedestrians, failures)
    (out / "STUDY_REPORT.md").write_text(report)
    chart = write_chart(rows, args.miss_rates[0], out / "study_chart.png")
    print("\n" + report)
    print(f"Saved {out / 'STUDY_REPORT.md'}, {out / 'study_summary.csv'}"
          + (f" and {out / 'study_chart.png'}" if chart else " (install matplotlib for a chart)"))


if __name__ == "__main__":
    main()
