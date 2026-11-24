"""Train the arrival predictor on the simplified simulator and report how well it does.

    python train_predictor.py

Trains on the mixed corridor, then tests on (1) the same corridor with seeds it has not seen and
(2) a different network (the grid with a one-way street), which shows whether it learned something
general or just memorised one layout. Saves models/predictor.json.

The simulator is a model, so this proves the pipeline works, not that the predictor is accurate on
real roads. Retrain on logs from real cameras (features at message time, arrivals 20 s later).
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from graph import Network                      # noqa: E402
from netsim import NetSim, build_demand        # noqa: E402
from predictor import Predictor, evaluate      # noqa: E402

SCENARIOS = {"light": (120, 1.0), "balanced": (200, 1.0), "peak": (300, 1.8), "surge": (400, 1.8)}


def collect(net, seeds, duration=1800.0):
    X, y = [], []
    for scenario, (rate, boost) in SCENARIOS.items():
        demand = build_demand(net, rate, boost)
        for seed in seeds:
            # observe only: messages flow and features are recorded, but nothing is adjusted
            sim = NetSim(net, "coordinated", demand, seed=seed, duration=duration,
                         record=True, apply_coordination=False)
            sim.run()
            xs, ys = sim.training_rows()
            X += xs
            y += ys
    return X, y


def main():
    corridor = Network.load(HERE / "networks" / "corridor.json")
    grid = Network.load(HERE / "networks" / "grid_mixed.json")
    X, y = collect(corridor, seeds=range(1, 5))
    model = Predictor.fit(X, y, ridge=1.0, meta={"trained_on": "simulator, corridor.json",
                                                 "samples": len(y), "horizon_s": 20})
    model.save(HERE / "models" / "predictor.json")
    print(f"trained on {len(y)} samples; saved models/predictor.json")
    for label, net in (("same corridor, unseen seeds", corridor), ("different network (grid)", grid)):
        Xt, yt = collect(net, seeds=range(11, 14))
        r = evaluate(model, Xt, yt)
        print(f"{label:30s} model MAE {r['model_mae']:.2f} | rule MAE {r['rule_mae']:.2f} | "
              f"mean-guess MAE {r['mean_guess_mae']:.2f}  ({r['samples']} samples)")


if __name__ == "__main__":
    main()
