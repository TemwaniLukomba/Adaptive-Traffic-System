"""Checks for the network layer. Run:  python test_network.py"""
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np                                            # noqa: E402
from coordination import Message                              # noqa: E402
from graph import Network                                     # noqa: E402
from netsim import NetSim, build_demand                       # noqa: E402
from predictor import Predictor                               # noqa: E402

fails = []


def check(name, ok, detail=""):
    print(("PASS  " if ok else "FAIL  ") + name + (f"  ({detail})" if detail and not ok else ""))
    if not ok:
        fails.append(name)


corr = Network.load(HERE / "networks" / "corridor.json")
grid = Network.load(HERE / "networks" / "grid_mixed.json")

# graph
check("both networks are connected", corr.is_connected() and grid.is_connected())
for net in (corr, grid):
    d = build_demand(net, 200, 1.0)
    check(f"{net.name[:20]}: every demand flow has a route", all(net.route(*k) for k in d), len(d))
check("one-way street creates a one-way link", "C.north>A.south" in grid.links and "A.south>C.north" not in grid.links)
try:
    Network.from_dict({"nodes": [{"id": "A", "layout": "crossroads"}, {"id": "B", "layout": "crossroads"}],
                       "links": [{"from": "A.east", "to": "B.west"}, {"from": "A.east", "to": "B.north"}]})
    check("an arm cannot be used by two links", False)
except ValueError:
    check("an arm cannot be used by two links", True)
route = corr.route(("A", "west"), ("D", "east"))
check("a route crosses all four node types", route is not None and [r[0] for r in route] == ["A", "B", "C", "D"], route)

check("an arm that belongs to a one-way link is not an edge arm",
      ("A", "south") not in grid.boundary_exits() and ("A", "south") not in grid.boundary_entries())

# messages
m = Message("A", "B", "east", "west", 3.0, 4.5, True, [1, 2, 3], 2.5, True)
check("message survives JSON", Message.from_json(m.to_json()) == m)

# predictor
rng = np.random.default_rng(0)
X = rng.random((500, 7)) * 5
y = X @ np.array([1, 0.5, 0, 0.2, 0, 0.3, 0]) + 0.7
p = Predictor.fit(X, y, ridge=1e-6)
check("predictor recovers a known linear rule", np.allclose(p.predict(X[0]), y[0], atol=1e-3))

# safety: coordination never breaks the signal rules, with and without message loss
for loss in (0.0, 0.5, 1.0):
    sim = NetSim(corr, "coordinated", build_demand(corr, 300, 1.8), seed=3, duration=900, msg_loss=loss)
    bad_phase = bad_green = 0
    for _ in range(900):
        sim.step()
        for n, c in sim.ctrl.items():
            live = {p for p, apps in c.phases.items()
                    if any(c.signals.get(a) != "red" for a in apps)}
            bad_phase += len(live) > 1
    for n, c in sim.ctrl.items():
        ev = c.events
        for i, e in enumerate(ev):
            if e["state"] == "GREEN" and i + 1 < len(ev):
                if ev[i + 1]["time"] - e["time"] < c.min_green - 1e-9:
                    bad_green += 1
    check(f"loss {loss:.0%}: never two phases live at once", bad_phase == 0, bad_phase)
    check(f"loss {loss:.0%}: minimum green always respected", bad_green == 0, bad_green)

# lost messages: with everything lost, no adjustment is ever applied
sim = NetSim(corr, "coordinated", build_demand(corr, 300, 1.8), seed=3, duration=300, msg_loss=1.0)
sim.run()
check("with all messages lost every adjustment is zero",
      all(abs(v) < 1e-9 for c in sim.ctrl.values() for v in c.score_adjust.values()))
sim = NetSim(corr, "coordinated", build_demand(corr, 300, 1.8), seed=3, duration=300)
sim.run()
check("with messages, some adjustment is applied",
      any(abs(v) > 1e-9 for c in sim.ctrl.values() for v in c.score_adjust.values()))

# stale messages expire
co = next(iter(sim.coord.values()))
co.inbox = {k: Message(**{**m.__dict__, "t": 0.0}) for k, m in co.inbox.items()}
adj = co.adjustments(1000.0, {a: 5.0 for a in co.layout.approaches}, {}, sim.shares[co.id])
check("messages older than the time-to-live are ignored", all(abs(v) < 1e-9 for v in adj.values()))

# mass is conserved
sim = NetSim(corr, "isolated", build_demand(corr, 200, 1.0), seed=5, duration=900)
sim.run()
inside = sum(len(q) for n in sim.q.values() for q in n.values()) + sum(len(t) for t in sim.transit.values())
check("every spawned vehicle is finished or still inside", len(sim.spawned) == len(sim.finished) + inside,
      (len(sim.spawned), len(sim.finished), inside))

print("\nAll checks passed." if not fails else f"\n{len(fails)} check(s) failed: {fails}")
sys.exit(1 if fails else 0)
