"""Compare fixed, isolated and connected control in SUMO on a network.

    python sumo_build.py networks/corridor.json     # once per network
    python sumo_study.py --quick                    # corridor, balanced + peak, 1 seed
    python sumo_study.py                            # both networks, 4 scenarios, 3 seeds
    python sumo_study.py --networks corridor --seeds 5

Needs SUMO installed (see ../sim/README.md). Writes results_sumo/SUMO_NETWORK_STUDY.md and a CSV.
These ARE the numbers for the report (unlike study_net.py, which uses a simplified model), once
the first runs have shown that the SUMO export behaves.
Variants: fixed, isolated, connected, connected_ai (needs models/predictor.json).
"""
import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import sumo_run                                    # noqa: E402
from predictor import Predictor                    # noqa: E402

NETWORKS = ["corridor", "grid_mixed"]
SCENARIOS = ["light", "balanced", "peak", "surge"]
METRICS = ["mean_delay_s", "p95_delay_s", "mean_travel_s", "throughput_vph", "max_link_fill",
           "ped_mean_delay_s", "unfinished"]


def avg(rows, key):
    vals = [r[key] for r in rows if r.get(key) is not None]
    return sum(vals) / len(vals) if vals else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--networks", nargs="+", default=None)
    ap.add_argument("--scenarios", nargs="+", default=None)
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--loss", type=float, default=0.0)
    ap.add_argument("--no-ai", action="store_true")
    ap.add_argument("--out", default=str(HERE / "results_sumo"))
    args = ap.parse_args()
    networks = args.networks or (NETWORKS[:1] if args.quick else NETWORKS)
    scenarios = args.scenarios or (["balanced", "peak"] if args.quick else SCENARIOS)
    seeds = 1 if args.quick else args.seeds
    model = HERE / "models" / "predictor.json"
    predictor = Predictor.load(model) if model.exists() and not args.no_ai else None

    variants = [("fixed", "fixed", None), ("isolated", "isolated", None), ("connected", "coordinated", None)]
    if predictor:
        variants.append(("connected_ai", "coordinated", predictor))

    rows = []
    for netname in networks:
        path = HERE / "networks" / f"{netname}.json"
        for scen in scenarios:
            for label, mode, pred in variants:
                for seed in range(1, seeds + 1):
                    print(f"{netname} | {scen} | {label} | seed {seed}", flush=True)
                    r = sumo_run.run_one(path, scen, mode, seed=seed, predictor=pred, loss=args.loss)
                    r.update(netname=netname, variant=label)
                    rows.append(r)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "sumo_network_study.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["netname", "scenario", "variant", "seed"] + METRICS,
                           extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    groups = defaultdict(list)
    for r in rows:
        groups[(r["netname"], r["scenario"], r["variant"])].append(r)
    lines = ["# Connected signals in SUMO", "",
             f"Seeds per cell: {seeds}. Messages lost: {args.loss:.0%}. Delay is SUMO's time loss per vehicle. "
             "Change is against isolated adaptive control.", ""]
    for netname in networks:
        lines += [f"## {netname}", "",
                  "| scenario | variant | mean delay s | vs isolated | p95 s | max link fill | ped delay s | unfinished |",
                  "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
        for scen in scenarios:
            base = avg(groups[(netname, scen, "isolated")], "mean_delay_s")
            for label, _, _ in variants:
                g = groups.get((netname, scen, label))
                if not g:
                    continue
                d = avg(g, "mean_delay_s")
                chg = f"{100 * (d - base) / base:+.0f}%" if base else "-"
                pd = avg(g, "ped_mean_delay_s")
                lines.append(f"| {scen} | {label} | {d:.0f} | {chg} | {avg(g, 'p95_delay_s'):.0f} | "
                             f"{avg(g, 'max_link_fill'):.2f} | {'-' if pd is None else f'{pd:.0f}'} | "
                             f"{avg(g, 'unfinished'):.0f} |")
        lines.append("")
    (out / "SUMO_NETWORK_STUDY.md").write_text("\n".join(lines))
    print("\n".join(lines))
    print(f"\nSaved {out / 'SUMO_NETWORK_STUDY.md'}")


if __name__ == "__main__":
    main()
