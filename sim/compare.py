"""Run every controller on every scenario and print a comparison.

    python compare.py                          # all scenarios, 3 seeds, 3 controllers
    python compare.py --seeds 5
    python compare.py --scenarios tidal surge --controllers fixed adaptive
    python compare.py --miss-rates 0 0.1 0.2   # how much do camera errors hurt?

Writes results/runs.csv (one row per run) and results/summary.csv (averages),
and, if matplotlib is installed, results/delay_by_scenario.png.
"""
import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))

import make_demand                                    # noqa: E402
import run_sim                                        # noqa: E402

METRICS = ["mean_delay_s", "p95_delay_s", "max_wait_s", "mean_queue_veh",
           "max_queue_veh", "throughput_vph", "phase_changes", "unfinished",
           "ped_mean_delay_s", "ped_p95_delay_s", "ped_max_wait_s", "peds_completed"]


def average(rows, key):
    values = [r[key] for r in rows if r[key] is not None]
    return sum(values) / len(values) if values else 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenarios", nargs="+", default=list(make_demand.SCENARIOS))
    parser.add_argument("--controllers", nargs="+", default=list(run_sim.CONTROLLERS),
                        choices=run_sim.CONTROLLERS)
    parser.add_argument("--seeds", type=int, default=3, help="runs per combination")
    parser.add_argument("--miss-rates", nargs="+", type=float, default=[0.0])
    parser.add_argument("--out", default=str(SIM_DIR / "results"))
    parser.add_argument("--no-pedestrians", action="store_true", help="vehicles only")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = []
    total = len(args.scenarios) * len(args.controllers) * args.seeds * len(args.miss_rates)
    done = 0
    for scenario in args.scenarios:
        for miss in args.miss_rates:
            for controller in args.controllers:
                for seed in range(1, args.seeds + 1):
                    done += 1
                    print(f"[{done}/{total}] {scenario} | {controller} | seed {seed} | miss {miss}",
                          flush=True)
                    runs.append(run_sim.run_one(scenario, controller, seed=seed, miss_rate=miss,
                                                out_dir=out,
                                                pedestrians=False if args.no_pedestrians else None))

    # one row per run
    fields = ["scenario", "controller", "seed", "miss_rate", "completed"] + METRICS \
             + ["mean_travel_s", "mean_stops", "clear_time_s"]
    with open(out / "runs.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(runs)

    # averages over seeds
    groups = defaultdict(list)
    for r in runs:
        groups[(r["scenario"], r["miss_rate"], r["controller"])].append(r)
    summary = []
    for (scenario, miss, controller), rows in groups.items():
        row = {"scenario": scenario, "miss_rate": miss, "controller": controller, "runs": len(rows)}
        for m in METRICS:
            row[m] = round(average(rows, m), 2)
        summary.append(row)
    with open(out / "summary.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["scenario", "miss_rate", "controller", "runs"] + METRICS)
        writer.writeheader()
        writer.writerows(summary)

    # printed table, with the change against the fixed-time plan
    fixed_delay = {(r["scenario"], r["miss_rate"]): r["mean_delay_s"]
                   for r in summary if r["controller"] == "fixed"}
    print(f"\n{'scenario':<11}{'miss':>5} {'controller':<14}{'delay s':>9}{'vs fixed':>10}"
          f"{'p95 s':>8}{'max wait':>10}{'max queue':>10}{'unfinished':>11}{'ped delay':>10}{'ped max':>9}")
    for r in sorted(summary, key=lambda r: (r["scenario"], r["miss_rate"], args.controllers.index(r["controller"]))):
        base = fixed_delay.get((r["scenario"], r["miss_rate"]))
        change = f"{100 * (r['mean_delay_s'] - base) / base:+.0f}%" if base else "-"
        print(f"{r['scenario']:<11}{r['miss_rate']:>5.2f} {r['controller']:<14}{r['mean_delay_s']:>9.1f}"
              f"{change:>10}{r['p95_delay_s']:>8.1f}{r['max_wait_s']:>10.1f}"
              f"{r['max_queue_veh']:>10.1f}{r['unfinished']:>11.1f}"
              f"{r['ped_mean_delay_s']:>10.1f}{r['ped_max_wait_s']:>9.1f}")
    print(f"\nSaved {out / 'runs.csv'} and {out / 'summary.csv'}")

    try:                                              # optional chart
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(install matplotlib to also get a chart)")
        return
    base_rows = [r for r in summary if r["miss_rate"] == args.miss_rates[0]]
    scenarios = list(dict.fromkeys(r["scenario"] for r in base_rows))
    width = 0.8 / len(args.controllers)
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for i, c in enumerate(args.controllers):
        vals = [next((r["mean_delay_s"] for r in base_rows
                      if r["scenario"] == s and r["controller"] == c), 0) for s in scenarios]
        ax.bar([x + i * width for x in range(len(scenarios))], vals, width, label=c)
    ax.set_xticks([x + 0.4 - width / 2 for x in range(len(scenarios))])
    ax.set_xticklabels(scenarios)
    ax.set_ylabel("Mean delay per vehicle (s)")
    ax.set_title("Average delay by scenario and controller")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "delay_by_scenario.png", dpi=150)
    print(f"Saved {out / 'delay_by_scenario.png'}")


if __name__ == "__main__":
    main()
