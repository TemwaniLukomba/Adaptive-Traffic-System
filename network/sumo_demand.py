"""Write a SUMO route file (vehicle flows and walkers) for a network.

    python sumo_demand.py networks/corridor.json balanced

Scenarios match study_net.py: light, balanced, peak, surge (entry rate per edge arm in veh/h, and a
boost for the through traffic along the corridor). Every flow follows the shortest route through
the nodes (graph.Network.route), so any mix of road types works. Vehicle types and their mix are the
same as in ../sim/make_demand.py. The numbers are illustrative, not measured.
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "sim"))

from graph import Network                       # noqa: E402
from netsim import build_demand                 # noqa: E402
from sumo_build import in_edge, out_edge        # noqa: E402

SCENARIOS = {"light": (120, 1.0), "balanced": (200, 1.0), "peak": (300, 1.8), "surge": (400, 1.8)}
PED_RATE = 120.0          # walkers per hour on each crossing (both directions together)
ROUTE_FILE_HEAD = ('<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
                   'xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">')


def route_edges(net, route):
    """SUMO edge ids for a route [(node, in_arm, out_arm), ...]."""
    node0, arm0, _ = route[0]
    edges = [in_edge(net, node0, arm0)]
    for node, _, out_arm in route:
        edges.append(out_edge(net, node, out_arm))
    return edges


def build(net, scenario, map_path, duration=1800, ped_rate=PED_RATE):
    import make_demand as md                     # vehicle types and mix
    rate, boost = SCENARIOS[scenario]
    demand = build_demand(net, rate, boost)
    info = json.loads(Path(map_path).read_text())
    lines = [ROUTE_FILE_HEAD, md.VTYPES.rstrip("\n")]
    walking = any(info["crossings"].values()) and ped_rate > 0
    if walking:
        lines.append(md.PEDTYPE)
    n = 0
    for (entry, ex), vph in demand.items():
        route = net.route(entry, ex)
        if not route:
            continue
        rid = f"r{n}"
        lines.append(f'    <route id="{rid}" edges="{" ".join(route_edges(net, route))}"/>')
        for vtype, share in md.MIX.items():
            p = vph * share / 3600.0
            if p > 0:
                lines.append(f'    <flow id="f{n}_{vtype}" type="{vtype}" route="{rid}" begin="0" '
                             f'end="{duration}" probability="{p:.5f}" departLane="best" departSpeed="max"/>')
        n += 1
    if walking:
        for node, arms in info["crossings"].items():
            for arm in arms:
                a, b = in_edge(net, node, arm), out_edge(net, node, arm)
                near_a = max(5.0, info["edge_length"][a] - 30.0)       # 30 m before the crossing
                near_b = min(30.0, info["edge_length"][b] - 5.0)       # 30 m after it
                for d, (frm, to, p0, p1) in (("fwd", (a, b, near_a, near_b)), ("back", (b, a, near_b, near_a))):
                    lines.append(
                        f'    <personFlow id="pf_{node}_{arm}_{d}" type="ped" begin="0" end="{duration}" '
                        f'probability="{ped_rate / 2 / 3600.0:.5f}">\n'
                        f'        <walk from="{frm}" to="{to}" departPos="{p0:g}" arrivalPos="{p1:g}"/>\n'
                        f'    </personFlow>')
    lines.append("</routes>")
    return "\n".join(lines) + "\n"


def main():
    if len(sys.argv) < 3 or sys.argv[2] not in SCENARIOS:
        sys.exit(f"usage: python sumo_demand.py networks/corridor.json {{{'|'.join(SCENARIOS)}}}")
    net = Network.load(sys.argv[1])
    out = HERE / "sumo" / Path(sys.argv[1]).stem
    (out / "demand").mkdir(parents=True, exist_ok=True)
    path = out / "demand" / f"{sys.argv[2]}.rou.xml"
    path.write_text(build(net, sys.argv[2], out / "map.json"))
    print("wrote", path)


if __name__ == "__main__":
    main()
