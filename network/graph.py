"""A road network made of signalised nodes joined by links.

A node is ANY signalised point described by a layout (see ../layout.py): a crossroads, a
T-junction, a five-way junction, a one-way street crossing, a pedestrian crossing in the middle of
a road, a road-works shuttle, or a layout learned from video. Nodes are joined by links between
two of their arms, so a corridor can mix road types freely.

network.json:
    {"name": "Great East corridor",
     "nodes": [{"id": "A", "layout": "crossroads"},
               {"id": "B", "layout": "t_junction"},
               {"id": "C", "layout": "road_crossing", "rotation": 90}],
     "links": [{"from": "A.east", "to": "B.west", "length": 250},
               {"from": "B.east", "to": "C.west", "length": 200}]}

A node spec may also give lanes, drive_on_left, rotation, or a path to a layout.json / site.json.
A link joins one arm of one node to one arm of another. It carries traffic in each direction the
two arms allow (an arm marked one-way "in" or "out" makes the link one-way), so the graph handles
one-way streets without special cases. Arms that are not linked are the edges of the network:
vehicles enter and leave there.
"""
import json
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import layout as L                                           # noqa: E402

JAM_SPACING_M = 7.5          # road length one passenger-car equivalent occupies in a queue


@dataclass
class Node:
    id: str
    layout: L.Layout


@dataclass
class Link:
    """One direction of travel between two nodes."""
    id: str                  # "A.east>B.west": leaves node A by arm east, enters node B by arm west
    src: str
    src_arm: str
    dst: str
    dst_arm: str
    length: float
    lanes: int
    speed: float             # m/s, free flow

    @property
    def capacity(self):
        """How many passenger-car equivalents fit on the road when it is jammed."""
        return max(4.0, self.length * self.lanes / JAM_SPACING_M)

    @property
    def free_time(self):
        return self.length / self.speed


class Network:
    def __init__(self, name, nodes, link_specs):
        self.name = name
        self.nodes = {n.id: n for n in nodes}
        self.links = {}
        self.out_link = {}            # (node, exit arm)     -> Link
        self.in_link = {}             # (node, approach arm) -> Link
        for spec in link_specs:
            self._add_link(spec)
        self._moves = {nid: {} for nid in self.nodes}
        for nid, node in self.nodes.items():
            for m in node.layout.movements():
                self._moves[nid].setdefault(m.approach, []).append(m)

    # ---- building ------------------------------------------------------------------------
    @staticmethod
    def _split(ref):
        node, _, arm = ref.partition(".")
        if not arm:
            raise ValueError(f"'{ref}' should look like NODE.arm, for example A.east")
        return node, arm

    def _add_link(self, spec):
        (a, a_arm), (b, b_arm) = self._split(spec["from"]), self._split(spec["to"])
        for n, arm in ((a, a_arm), (b, b_arm)):
            if n not in self.nodes:
                raise ValueError(f"link refers to unknown node '{n}'")
            self.nodes[n].layout.arm(arm)                       # KeyError if the arm does not exist
            if (n, arm) in self.out_link or (n, arm) in self.in_link:
                raise ValueError(f"arm {n}.{arm} is already used by another link")
        length = float(spec.get("length", 300.0))
        made = 0
        for sn, sa, dn, da in ((a, a_arm, b, b_arm), (b, b_arm, a, a_arm)):
            s, d = self.nodes[sn].layout.arm(sa), self.nodes[dn].layout.arm(da)
            if s.oneway in ("both", "out") and d.oneway in ("both", "in"):
                speed = min(s.speed_kmh, d.speed_kmh) / 3.6
                link = Link(f"{sn}.{sa}>{dn}.{da}", sn, sa, dn, da, length, min(s.lanes, d.lanes), speed)
                self.links[link.id] = link
                self.out_link[(sn, sa)] = link
                self.in_link[(dn, da)] = link
                made += 1
        if not made:
            raise ValueError(f"link {spec['from']} - {spec['to']} carries no traffic in either direction")

    @classmethod
    def from_dict(cls, data):
        nodes = []
        for spec in data["nodes"]:
            kw = {k: spec[k] for k in ("lanes", "drive_on_left", "rotation") if k in spec}
            layout = L.resolve(spec["layout"], **kw)
            nodes.append(Node(spec["id"], layout))
        return cls(data.get("name", "network"), nodes, data["links"])

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text()))

    # ---- structure -----------------------------------------------------------------------
    def neighbours(self, node_id):
        out = set()
        for link in self.links.values():
            if link.src == node_id:
                out.add(link.dst)
            if link.dst == node_id:
                out.add(link.src)
        return out

    def _linked(self, node, arm):
        return (node, arm) in self.in_link or (node, arm) in self.out_link

    def boundary_entries(self):
        """Approach arms on the edge of the network. An arm that belongs to a link is inside the
        network even if the link is one-way, so it is never an edge arm."""
        return [(n, a) for n, node in self.nodes.items() for a in node.layout.approaches
                if not self._linked(n, a)]

    def boundary_exits(self):
        return [(n, a) for n, node in self.nodes.items() for a in node.layout.exits
                if not self._linked(n, a)]

    def is_connected(self):
        if not self.nodes:
            return True
        seen, todo = set(), [next(iter(self.nodes))]
        while todo:
            n = todo.pop()
            if n not in seen:
                seen.add(n)
                todo.extend(self.neighbours(n))
        return seen == set(self.nodes)

    def movements_from(self, node_id, in_arm):
        return self._moves[node_id].get(in_arm, [])

    def route(self, origin, dest):
        """Shortest route between two boundary arms.

        origin = (node, arm it enters by); dest = (node, arm it leaves by).
        Returns [(node, in_arm, out_arm), ...] or None when there is no way through."""
        parent = {origin: None}
        todo = deque([origin])
        while todo:
            state = todo.popleft()
            node, in_arm = state
            for m in self.movements_from(node, in_arm):
                step = (node, in_arm, m.dest)
                if (node, m.dest) == dest:
                    path = [step]
                    while parent[state] is not None:
                        state, prev = parent[state]
                        path.append(prev)
                    return path[::-1]
                link = self.out_link.get((node, m.dest))
                if link:
                    nxt = (link.dst, link.dst_arm)
                    if nxt not in parent:
                        parent[nxt] = (state, step)
                        todo.append(nxt)
        return None

    def describe(self):
        lines = [f"{self.name}: {len(self.nodes)} nodes, {len(self.links)} directed links, "
                 f"{'connected' if self.is_connected() else 'NOT connected'}"]
        for n, node in self.nodes.items():
            lay = node.layout
            lines.append(f"  {n}: {lay.name}, arms {', '.join(a.name for a in lay.arms)}")
        for link in self.links.values():
            lines.append(f"  link {link.id}  {link.length:.0f} m  {link.lanes} lane(s)  "
                         f"holds {link.capacity:.0f} PCE")
        lines.append("  network edges (traffic enters/leaves): "
                     + ", ".join(f"{n}.{a}" for n, a in self.boundary_entries()))
        return "\n".join(lines)


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: python graph.py networks/corridor.json")
    print(Network.load(sys.argv[1]).describe())


if __name__ == "__main__":
    main()
