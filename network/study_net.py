"""Compare fixed-time, stand-alone adaptive, and connected (neighbour-aware) control.

    python study_net.py                    # both networks, 4 scenarios, 3 seeds
    python study_net.py --quick            # corridor only, 2 scenarios, 1 seed
    python study_net.py --loss 0.3         # drop 30% of messages: does it still work?
    python study_net.py --no-gmaps         # leave out the Google-weighted variant

Runs the simplified simulator in netsim.py, NOT SUMO. Use it to check how the coordination
behaves, not as evidence about real roads. Writes results/NETWORK_STUDY.md and network_study.csv.

Variants: fixed (30 s greens, no offsets), isolated (each node adaptive, no messages),
connected (neighbour messages, simple arrival rule), connected_ai (messages + learned predictor),
connected_gmaps (connected_ai with link weights from a congestion profile; the profile here is
SYNTHETIC unless you pass --profile).
"""
import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "gmaps"))

from graph import Network                       # noqa: E402
from netsim import NetSim, build_demand         # noqa: E402
from predictor import Predictor                 # noqa: E402
import priorities                               # noqa: E402

SCENARIOS = {"light": (120, 1.0), "balanced": (200, 1.0), "peak": (300, 1.8), "surge": (400, 1.8)}
NETWORKS = ["corridor", "grid_mixed"]
METRICS = ["mean_delay_s", "p95_delay_s", "mean_travel_s", "throughput_vph", "blocked_share",
           "max_link_fill", "ped_mean_wait_s", "unfinished"]
PEAK_HOUR = 8


def variants(net, predictor, profile, use_gmaps):
    out = [("fixed", dict(mode="fixed")), ("isolated", dict(mode="isolated")),
           ("connected", dict(mode="coordinated"))]
    if predictor is not None:
        out.append(("connected_ai", dict(mode="coordinated", predictor=predictor)))
        if use_gmaps:
            out.append(("connected_gmaps", dict(mode="coordinated", predictor=predictor,
                                                link_weight=priorities.link_weight_fn(profile, PEAK_HOUR))))
    return out


def avg(rows, key):
    vals = [r[key] for r in rows if r.get(key) is not None]
    return sum(vals) / len(vals) if vals else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--duration", type=float, default=1800.0)
    ap.add_argument("--loss", type=float, default=0.0, help="share of messages lost (0 to 1)")
    ap.add_argument("--profile", default=None, help="a profile.json from gmaps/fetch_profile.py")
    ap.add_argument("--no-gmaps", action="store_true")
    ap.add_argument("--out", default=str(HERE / "results"))
    args = ap.parse_args()

    model_path = HERE / "models" / "predictor.json"
    predictor = Predictor.load(model_path) if model_path.exists() else None
    if predictor is None:
        print("No models/predictor.json yet (run train_predictor.py): skipping the AI variants.")
    networks = NETWORKS[:1] if args.quick else NETWORKS
    scenarios = {k: SCENARIOS[k] for k in (["balanced", "peak"] if args.quick else SCENARIOS)}
    seeds = 1 if args.quick else args.seeds

    rows = []
    for netname in networks:
        net = Network.load(HERE / "networks" / f"{netname}.json")
        profile = (priorities.load_profile(args.profile) if args.profile
                   else priorities.synthetic_profile(list(net.links)))
        for scen, (rate, boost) in scenarios.items():
            demand = build_demand(net, rate, boost)
            for name, kw in variants(net, predictor, profile, not args.no_gmaps):
                for seed in range(1, seeds + 1):
                    print(f"{netname} | {scen} | {name} | seed {seed}", flush=True)
                    kw2 = dict(kw)
                    mode = kw2.pop("mode")
                    r = NetSim(net, mode, demand, seed=seed, duration=args.duration,
                               msg_loss=args.loss, **kw2).run()
                    r.update(network=netname, scenario=scen, variant=name, seed=seed)
                    rows.append(r)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    fields = ["network", "scenario", "variant", "seed"] + METRICS
    with open(out / "network_study.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    groups = defaultdict(list)
    for r in rows:
        groups[(r["network"], r["scenario"], r["variant"])].append(r)
    order = ["fixed", "isolated", "connected", "connected_ai", "connected_gmaps"]
    lines = ["# Connected-signals study (simplified simulator, not SUMO)", "",
             f"Seeds per cell: {seeds}. Duration {args.duration:.0f} s. Messages lost: {args.loss:.0%}. "
             f"Google weights: {'SYNTHETIC profile' if not args.profile else args.profile}.", "",
             "Delay is time spent waiting in queues per vehicle, including vehicles still inside "
             "when the run ends. Change is against isolated adaptive control.", ""]
    for netname in networks:
        lines += [f"## {netname}", "",
                  "| scenario | variant | mean delay s | vs isolated | p95 s | blocked share | ped wait s | unfinished |",
                  "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
        for scen in scenarios:
            base = avg(groups[(netname, scen, "isolated")], "mean_delay_s")
            for v in order:
                g = groups.get((netname, scen, v))
                if not g:
                    continue
                d = avg(g, "mean_delay_s")
                chg = f"{100 * (d - base) / base:+.0f}%" if base else "-"
                pw = avg(g, "ped_mean_wait_s")
                lines.append(f"| {scen} | {v} | {d:.0f} | {chg} | {avg(g, 'p95_delay_s'):.0f} | "
                             f"{avg(g, 'blocked_share'):.2f} | {'-' if pw is None else f'{pw:.0f}'} | "
                             f"{avg(g, 'unfinished'):.0f} |")
        lines.append("")
    (out / "NETWORK_STUDY.md").write_text("\n".join(lines))
    print("\n".join(lines))
    print(f"\nSaved {out / 'NETWORK_STUDY.md'} and {out / 'network_study.csv'}")


if __name__ == "__main__":
    main()
