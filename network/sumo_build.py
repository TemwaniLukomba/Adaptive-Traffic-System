"""Build a SUMO network from a network.json (any mix of road types).

    python sumo_build.py networks/corridor.json
    python sumo_build.py networks/grid_mixed.json --pedestrians all
    python sumo_build.py networks/corridor.json --files-only      # write the XML, skip netconvert

Writes sumo/<network name>/ with nodes.nod.xml, edges.edg.xml, connections.con.xml, net.net.xml
and map.json (which edge belongs to which node and arm; the runner reads it).

How the geometry is made:
  * The first node sits at (0, 0). A linked node is placed one link length away from its
    neighbour, in the direction of the neighbour's linked arm (compass bearing, north = up), so
    junctions of any shape line up. A link that closes a loop is drawn as a straight road between
    the two nodes already placed; its length is then the distance between them, and a warning is
    printed when that differs from the length in network.json or the arm directions do not line up.
  * Arms not linked to anything get an outer end node 300 m away (vehicles enter and leave there).
  * Every node is a SUMO traffic light named after the node. Each layout's own arm bearings, lane
    counts and one-way flags decide its edges, as in ../sim/build_network.py. Driving side follows
    the layouts (they must agree: SUMO builds one network on one side).

Edge ids:  A.east__B.west   from node A's arm east to node B's arm west (a link, one direction)
           in__A.west       from the outside into node A's arm west
           out__A.west      from node A's arm west to the outside

Pedestrians: --pedestrians phase (default) puts a zebra crossing only on nodes with a pedestrian
phase; all puts one on every node's two-way arms; none builds vehicles only. Crossings are
declared explicitly, so a two-arm node such as a mid-block crossing gets one too.

NOT YET RUN IN SUMO: written against SUMO's documented file formats, and tested only by checking
the files it writes. Expect to fix small things on the first netconvert run.
"""
import argparse
import json
import math
import subprocess
import sys
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "sim"))

from graph import Network                      # noqa: E402

OUTER_M = 300.0
ANGLE_WARN = 15.0


def place_nodes(net):
    """{node: (x, y)} and a list of warnings."""
    pos, warnings = {}, []
    first = next(iter(net.nodes))
    pos[first] = (0.0, 0.0)
    todo = deque([first])
    while todo:
        n = todo.popleft()
        for link in net.links.values():
            for a, b, a_arm, b_arm in ((link.src, link.dst, link.src_arm, link.dst_arm),
                                       (link.dst, link.src, link.dst_arm, link.src_arm)):
                if a == n and b not in pos:
                    bearing = math.radians(net.nodes[a].layout.arm(a_arm).bearing)
                    pos[b] = (pos[a][0] + link.length * math.sin(bearing),
                              pos[a][1] + link.length * math.cos(bearing))
                    todo.append(b)
    for n in net.nodes:
        if n not in pos:
            raise ValueError(f"node {n} is not connected to the rest of the network")
    for link in net.links.values():
        (x1, y1), (x2, y2) = pos[link.src], pos[link.dst]
        dist = math.hypot(x2 - x1, y2 - y1)
        if abs(dist - link.length) > 0.1 * link.length:
            warnings.append(f"{link.id}: drawn {dist:.0f} m long, network.json says {link.length:.0f} m")
        heading = math.degrees(math.atan2(x2 - x1, y2 - y1)) % 360
        want = net.nodes[link.src].layout.arm(link.src_arm).bearing
        if min(abs(heading - want), 360 - abs(heading - want)) > ANGLE_WARN:
            warnings.append(f"{link.id}: leaves {link.src}.{link.src_arm} at bearing {want:.0f} "
                            f"but the road is drawn at {heading:.0f}")
    return pos, warnings


def in_edge(net, node, arm):
    link = net.in_link.get((node, arm))
    return link.id.replace(">", "__") if link else f"in__{node}.{arm}"


def out_edge(net, node, arm):
    link = net.out_link.get((node, arm))
    return link.id.replace(">", "__") if link else f"out__{node}.{arm}"


def crossing_arms(net, node_id, mode):
    lay = net.nodes[node_id].layout
    if mode == "none" or (mode == "phase" and not lay.pedestrian_phase):
        return []
    return [a for a in lay.crossing_arm_names() if lay.arm(a).oneway == "both"]


def build_files(net, out_dir, pedestrians="phase"):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    pos, warnings = place_nodes(net)
    sides = {node.layout.drive_on_left for node in net.nodes.values()}
    if len(sides) > 1:
        raise ValueError("all nodes must drive on the same side: SUMO builds one network on one side")
    left = sides.pop()

    nodes = ["<nodes>"]
    for n, (x, y) in pos.items():
        nodes.append(f'    <node id="{n}" x="{x:.2f}" y="{y:.2f}" type="traffic_light"/>')
    edges = ["<edges>"]
    edge_to, edge_from = {}, {}                     # edge id -> (node, arm) it ends / starts at
    edge_length = {}

    for link in net.links.values():
        eid = link.id.replace(">", "__")
        (x1, y1), (x2, y2) = pos[link.src], pos[link.dst]
        length = math.hypot(x2 - x1, y2 - y1)
        edges.append(f'    <edge id="{eid}" from="{link.src}" to="{link.dst}" numLanes="{link.lanes}" '
                     f'speed="{link.speed:.2f}" length="{length:.1f}"/>')
        edge_to[eid], edge_from[eid] = (link.dst, link.dst_arm), (link.src, link.src_arm)
        edge_length[eid] = round(length, 1)

    for n, node in net.nodes.items():
        for arm in node.layout.arms:
            x0, y0 = pos[n]
            b = math.radians(arm.bearing)
            speed = arm.speed_kmh / 3.6
            if (n, arm.name) in net.in_link or (n, arm.name) in net.out_link:
                continue                                              # a linked arm: edges exist
            outer = f"X_{n}.{arm.name}"
            nodes.append(f'    <node id="{outer}" x="{x0 + OUTER_M * math.sin(b):.2f}" '
                         f'y="{y0 + OUTER_M * math.cos(b):.2f}" type="priority"/>')
            if arm.oneway in ("both", "in"):
                eid = f"in__{n}.{arm.name}"
                edges.append(f'    <edge id="{eid}" from="{outer}" to="{n}" numLanes="{arm.lanes}" speed="{speed:.2f}"/>')
                edge_to[eid], edge_from[eid] = (n, arm.name), None
                edge_length[eid] = OUTER_M
            if arm.oneway in ("both", "out"):
                eid = f"out__{n}.{arm.name}"
                edges.append(f'    <edge id="{eid}" from="{n}" to="{outer}" numLanes="{arm.lanes}" speed="{speed:.2f}"/>')
                edge_from[eid], edge_to[eid] = (n, arm.name), None
                edge_length[eid] = OUTER_M
    nodes.append("</nodes>")
    edges.append("</edges>")

    cons, crossings = ["<connections>"], {}
    known = set(edge_to) | set(edge_from)
    for n in net.nodes:
        arms = [a for a in crossing_arms(net, n, pedestrians)
                if in_edge(net, n, a) in known and out_edge(net, n, a) in known]
        crossings[n] = arms
        for arm in arms:
            ie, oe = in_edge(net, n, arm), out_edge(net, n, arm)
            cons.append(f'    <crossing node="{n}" edges="{ie} {oe}" priority="true"/>')
    cons.append("</connections>")

    (out_dir / "nodes.nod.xml").write_text("\n".join(nodes) + "\n")
    (out_dir / "edges.edg.xml").write_text("\n".join(edges) + "\n")
    (out_dir / "connections.con.xml").write_text("\n".join(cons) + "\n")
    (out_dir / "map.json").write_text(json.dumps({
        "network": net.name, "drive_on_left": left, "positions": pos,
        "edge_to": {k: v for k, v in edge_to.items() if v},
        "edge_from": {k: v for k, v in edge_from.items() if v},
        "edge_length": edge_length, "crossings": crossings, "warnings": warnings}, indent=1))
    return warnings, any(crossings.values()), left


def run_netconvert(out_dir, with_peds, left):
    from find_sumo import find_binary
    cmd = [find_binary("netconvert"),
           "--node-files", str(out_dir / "nodes.nod.xml"),
           "--edge-files", str(out_dir / "edges.edg.xml"),
           "--connection-files", str(out_dir / "connections.con.xml"),
           "--output-file", str(out_dir / "net.net.xml"),
           "--no-turnarounds", "true", "--lefthand", "true" if left else "false"]
    if with_peds:
        cmd += ["--sidewalks.guess", "true", "--sidewalks.guess.min-speed", "5",
                "--sidewalks.guess.max-speed", "20", "--walkingareas", "true"]
    print("Running:", " ".join(cmd))
    if subprocess.run(cmd).returncode != 0:
        sys.exit("netconvert failed; see its messages above.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("network")
    ap.add_argument("--pedestrians", choices=["phase", "all", "none"], default="phase")
    ap.add_argument("--out", default=None)
    ap.add_argument("--files-only", action="store_true")
    args = ap.parse_args()
    net = Network.load(args.network)
    out = Path(args.out) if args.out else HERE / "sumo" / Path(args.network).stem
    print(net.describe())
    warnings, with_peds, left = build_files(net, out, args.pedestrians)
    for w in warnings:
        print("Warning:", w)
    print(f"Wrote the network files to {out}")
    if not args.files_only:
        run_netconvert(out, with_peds, left)
        print(f"Network written to {out / 'net.net.xml'}")


if __name__ == "__main__":
    main()
