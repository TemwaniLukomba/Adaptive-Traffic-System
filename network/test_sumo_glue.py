"""Tests for the SUMO export and runner, using a stand-in for SUMO's TraCI.

    python test_sumo_glue.py

This checks OUR code: the files we write are consistent (every route edge exists, every crossing
refers to real edges), the runner maps SUMO's signal links to movements, builds legal state strings,
never shows two conflicting phases at once, and exchanges messages. It does NOT check SUMO itself
(netconvert, car-following, pedestrians): that needs a real run.
"""
import json
import random
import re
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import sumo_build                                   # noqa: E402
import sumo_demand                                  # noqa: E402
import sumo_run                                     # noqa: E402
from graph import Network                           # noqa: E402

fails = []


def check(name, ok, detail=""):
    print(("PASS  " if ok else "FAIL  ") + name + (f"  ({detail})" if detail and not ok else ""))
    if not ok:
        fails.append(name)


class FakeTraci:
    """A very small stand-in: vehicles drive along their route edges and pass a node when the
    light for their movement is green, one vehicle per second per approach edge."""

    def __init__(self, net, info, work):
        self.net, self.info, self.work = net, info, work
        self.time, self.vehs, self.n = 0.0, {}, 0
        self.edges = {}
        for e, (n, a) in info["edge_to"].items():
            self.edges.setdefault(e, {})["to"] = n
        for e, (n, a) in info["edge_from"].items():
            self.edges.setdefault(e, {})["from"] = n
        self.states, self.history, self.finished = {}, [], []
        self.trafficlight, self.edge, self.vehicle = self, self, self
        self.lane, self.simulation, self.person = self, self, self
        self.links = {n: self._links_for(n) for n in net.nodes}

    # --- controlled links, like netconvert would make them
    def _links_for(self, node):
        ins = [e for e, d in self.edges.items() if d.get("to") == node]
        outs = [e for e, d in self.edges.items() if d.get("from") == node]
        out = []
        for i in ins:
            for o in outs:
                if self.info["edge_to"][i][1] != self.info["edge_from"][o][1]:      # no U-turns
                    out.append([(f"{i}_0", f"{o}_0", "")])
        for arm in self.info["crossings"].get(node, []):
            out.append([(f":{node}_c0_0", f":{node}_c0_0", "")])
        return out

    def getControlledLinks(self, node):
        return self.links[node]

    def setRedYellowGreenState(self, node, state):
        assert len(state) == len(self.links[node]), (node, len(state), len(self.links[node]))
        assert set(state) <= set("rygG"), state
        self.states[node] = state
        self.history.append((self.time, node, state))

    # --- world
    def start(self, cmd):
        self.rou = ET.parse(cmd[cmd.index("-r") + 1]).getroot()
        self.out = cmd[cmd.index("--tripinfo-output") + 1]
        self.routes = {r.get("id"): r.get("edges").split() for r in self.rou.findall("route")}
        self.flows = [f for f in self.rou.findall("flow")]
        self.rng = random.Random(1)

    def getTime(self):
        return self.time

    def getMinExpectedNumber(self):
        return len(self.vehs) + (1 if self.time < 600 else 0)

    def getIDList(self):
        return []

    def simulationStep(self):
        t = self.time
        for f in self.flows:
            if self.rng.random() < float(f.get("probability")):
                self.n += 1
                self.vehs[f"v{self.n}"] = {"type": f.get("type"), "route": self.routes[f.get("route")],
                                           "i": 0, "pos": 0.0, "t0": t, "wait": 0.0}
        used = set()
        for vid, v in list(self.vehs.items()):
            e = v["route"][v["i"]]
            length = self._len(e)
            v["pos"] = min(length, v["pos"] + 10.0)
            if v["pos"] >= length - 0.01:
                if v["i"] == len(v["route"]) - 1:
                    self._finish(vid, v)
                    continue
                node = self.edges[e]["to"]
                nxt = v["route"][v["i"] + 1]
                idx = next((k for k, g in enumerate(self.links[node])
                            if g[0][0] == f"{e}_0" and g[0][1] == f"{nxt}_0"), None)
                state = self.states.get(node, "")
                if idx is not None and idx < len(state) and state[idx] in "Gg" and e not in used:
                    used.add(e)
                    v["i"] += 1
                    v["pos"] = 0.0
                else:
                    v["wait"] += 1
        self.time += 1.0

    def _len(self, e):
        return self.info["edge_length"].get(e, 300.0)

    def _finish(self, vid, v):
        self.finished.append((self.time - v["t0"], v["wait"]))
        del self.vehs[vid]

    def getLastStepVehicleIDs(self, edge):
        return [k for k, v in self.vehs.items() if v["route"][v["i"]] == edge]

    def getLastStepHaltingNumber(self, edge):
        return sum(1 for k, v in self.vehs.items() if v["route"][v["i"]] == edge and v["wait"] > 0
                   and v["pos"] >= self._len(edge) - 0.01)

    def getLaneID(self, veh):
        return f"{self.vehs[veh]['route'][self.vehs[veh]['i']]}_0"

    def getLanePosition(self, veh):
        return self.vehs[veh]["pos"]

    def getTypeID(self, veh):
        return self.vehs[veh]["type"]

    def getLength(self, lane):
        return self._len(lane.rsplit("_", 1)[0])

    def close(self):
        rows = "".join(f'<tripinfo id="{i}" duration="{d}" timeLoss="{w}" waitingCount="{1 if w else 0}"/>'
                       for i, (d, w) in enumerate(self.finished))
        Path(self.out).write_text(f"<tripinfos>{rows}</tripinfos>")


def fake_net_xml(path, info):
    rows = ["<net>"]
    link_count = {}
    for node, arms in info["crossings"].items():
        for arm in arms:
            ie = [e for e, (n, a) in info["edge_to"].items() if n == node and a == arm][0]
            oe = [e for e, (n, a) in info["edge_from"].items() if n == node and a == arm][0]
            rows.append(f'<edge id=":{node}_c0" function="crossing" crossingEdges="{ie} {oe}"/>')
            rows.append(f'<connection from=":{node}_c0" to=":{node}_w0" tl="{node}" linkIndex="LINKIDX_{node}"/>')
    rows.append("</net>")
    path.write_text("\n".join(rows))


def one_network(name):
    net_path = HERE / "networks" / f"{name}.json"
    net = Network.load(net_path)
    tmp = Path(tempfile.mkdtemp())
    warnings, with_peds, left = sumo_build.build_files(net, tmp)
    info = json.loads((tmp / "map.json").read_text())
    edges = set(info["edge_to"]) | set(info["edge_from"])

    # files
    for stem in ("nodes.nod", "edges.edg", "connections.con"):
        ET.parse(tmp / f"{stem}.xml")
    declared = {e.get("id") for e in ET.parse(tmp / "edges.edg.xml").getroot()}
    check(f"{name}: every mapped edge is declared", edges <= declared, edges - declared)
    for scen in sumo_demand.SCENARIOS:
        rou = tmp / f"{scen}.rou.xml"
        rou.write_text(sumo_demand.build(net, scen, tmp / "map.json"))
        root = ET.parse(rou).getroot()
        bad = [r.get("id") for r in root.findall("route") if not set(r.get("edges").split()) <= declared]
        check(f"{name}/{scen}: every route uses declared edges", not bad, bad[:3])
        # consecutive route edges must meet at a node
        ok = True
        for r in root.findall("route"):
            es = r.get("edges").split()
            for a, b in zip(es, es[1:]):
                ok &= info["edge_to"].get(a, [None])[0] == info["edge_from"].get(b, [None])[0]
        check(f"{name}/{scen}: consecutive route edges meet at a node", ok)
    check(f"{name}: no placement warnings", not warnings, warnings)
    return net, info, tmp


for name in ("corridor", "grid_mixed"):
    net, info, tmp = one_network(name)
    for mode in ("fixed", "isolated", "coordinated"):
        work = tmp / f"run_{mode}"
        shutil.copytree(tmp, work, ignore=shutil.ignore_patterns("run_*"))
        fake_net_xml(work / "net.net.xml", info)
        fake = FakeTraci(net, info, work)
        # fill the crossing link indexes the way netconvert would (after the vehicle links)
        text = (work / "net.net.xml").read_text()
        for node in net.nodes:
            vehicle_links = len(fake.links[node]) - len(info["crossings"].get(node, []))
            text = text.replace(f"LINKIDX_{node}", str(vehicle_links))
        (work / "net.net.xml").write_text(text)
        result = sumo_run.run_one(HERE / "networks" / f"{name}.json", "balanced", mode, seed=1,
                                  traci_module=fake, work_dir=work, sumo_binary="sumo",
                                  max_time=600, out_dir=work / "results")
        check(f"{name}/{mode}: the run completes and vehicles finish", result["completed"] > 20, result)
        # never two conflicting phases live at once
        bad = 0
        for t, node, state in fake.history:
            groups = fake.links[node]
            lay = net.nodes[node].layout
            live = set()
            for i, ch in enumerate(state):
                if ch in "Gg" and groups[i][0][0].startswith(":") is False:
                    e_in = groups[i][0][0].rsplit("_", 1)[0]
                    live.add(info["edge_to"][e_in][1])
            phases_live = {p for p, apps in lay.phases().items() if set(apps) & live}
            bad += len(phases_live) > 1
        check(f"{name}/{mode}: never two phases green at once", bad == 0, bad)
        if mode == "coordinated":
            check(f"{name}: messages were exchanged", result["messages_sent"] > 100, result["messages_sent"])
        if mode != "fixed":
            maps = 0
            for node, groups in fake.links.items():
                nb = sumo_run.NodeBinding(net, node, info, mode, work / "net.net.xml")
                nb.read_links(fake)
                vehicle_ok = all(x is not None for g, x in zip(groups, nb.link_info) if not g[0][0].startswith(":"))
                maps += vehicle_ok
            check(f"{name}/{mode}: every vehicle link maps to a movement", maps == len(net.nodes))
    shutil.rmtree(tmp)

print("\nAll checks passed." if not fails else f"\n{len(fails)} check(s) failed: {fails}")
sys.exit(1 if fails else 0)
