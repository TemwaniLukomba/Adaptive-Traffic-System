"""Run a whole network in SUMO under fixed, stand-alone adaptive, or connected control.

    python sumo_build.py networks/corridor.json          # once
    python sumo_run.py networks/corridor.json --scenario peak --mode coordinated --gui
    python sumo_run.py networks/corridor.json --scenario peak --mode isolated
    python sumo_run.py networks/corridor.json --scenario peak --mode coordinated --ai
    python sumo_run.py networks/corridor.json --scenario peak --mode coordinated --gmaps profile.json

Modes: fixed (30 s greens), isolated (each node's AdaptiveController alone),
coordinated (neighbour messages; --ai adds the learned arrival predictor).

Each simulated second, for every node:
  1. measure the weighted load in the last 60 m of each approach, like the camera zones do;
  2. (coordinated) send and receive neighbour messages and set the controller's score adjustments;
  3. let the node's controller decide (the same controller.py the prototype uses);
  4. translate its colours into SUMO's traffic-light state, with give-way greens (g) for the
     movements the layout says must yield, and walk signals for pedestrian crossings.

NOT YET RUN IN SUMO from where it was written; it is tested against a stand-in for TraCI
(test_sumo_glue.py). Expect to fix small things on the first real run.
"""
import argparse
import json
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "sim"))
sys.path.insert(0, str(HERE.parent / "gmaps"))

import config                                                         # noqa: E402
from controller import AdaptiveController, FixedTimeController        # noqa: E402
from coordination import Bus, CoordParams, NodeCoordinator            # noqa: E402
from graph import JAM_SPACING_M, Network                              # noqa: E402
from netsim import ZONE_M                                             # noqa: E402
from predictor import Predictor                                       # noqa: E402
from sumo_build import in_edge, out_edge                              # noqa: E402
from sumo_demand import SCENARIOS, build as build_routes             # noqa: E402

MODES = ("fixed", "isolated", "coordinated")
SIM_TYPE_TO_COCO = {"car": 2, "minibus": 5, "truck": 7, "motorcycle": 3}
COLOUR = {"green": "G", "yellow": "y", "red": "r"}
STEP = 1.0
MAX_TIME = 5400


def crossing_links(net_xml, node, crossing_arm_of_edge):
    """{signal index: arm} for the pedestrian crossings of one node in the built network."""
    root = ET.parse(net_xml).getroot()
    arm_of = {}
    for edge in root.findall("edge"):
        if edge.get("function") == "crossing" and edge.get("id", "").startswith(f":{node}_c") \
                and edge.get("crossingEdges"):
            first = edge.get("crossingEdges").split()[0]
            arm_of[edge.get("id")] = crossing_arm_of_edge.get(first)
    found = {}
    for con in root.findall("connection"):
        if con.get("tl") != node or con.get("linkIndex") is None:
            continue
        for key in ("from", "to"):
            if arm_of.get(con.get(key)):
                found[int(con.get("linkIndex"))] = arm_of[con.get(key)]
    return found


class NodeBinding:
    """Everything the runner needs to drive one traffic light."""

    def __init__(self, net, node_id, info, mode, net_xml):
        self.id = node_id
        self.layout = net.nodes[node_id].layout
        self.phases = self.layout.phases()
        make = FixedTimeController if mode == "fixed" else AdaptiveController
        self.ctrl = make(phases=self.phases)
        self.in_edge = {a: in_edge(net, node_id, a) for a in self.layout.approaches}
        self.yields = self.layout.yielding_movements()
        self.shares = {(a, b): s for a in self.layout.approaches
                       for b, s in self.layout.movement_shares(a).items()}
        self.lanes = {a.name: a.lanes for a in self.layout.arms}
        self.crossing_arms = set(info["crossings"].get(node_id, []))
        arm_of_edge = {}
        for a in self.crossing_arms:
            arm_of_edge[in_edge(net, node_id, a)] = a
            arm_of_edge[out_edge(net, node_id, a)] = a
        self.crossings = crossing_links(net_xml, node_id, arm_of_edge) if self.crossing_arms else {}
        self.edge_to = {e: tuple(v) for e, v in info["edge_to"].items()}
        self.edge_from = {e: tuple(v) for e, v in info["edge_from"].items()}
        self.link_info = []

    def read_links(self, traci):
        """Which movement each signal index controls: (approach arm, destination arm) or None."""
        self.link_info = []
        for group in traci.trafficlight.getControlledLinks(self.id):
            info = None
            if group:
                lane_in, lane_out = group[0][0], group[0][1]
                if not lane_in.startswith(":") and not lane_out.startswith(":"):
                    e_in, e_out = lane_in.rsplit("_", 1)[0], lane_out.rsplit("_", 1)[0]
                    a = self.edge_to.get(e_in)
                    b = self.edge_from.get(e_out)
                    if a and b and a[0] == self.id and b[0] == self.id:
                        if b[1] in {m.dest for m in self.layout.movements() if m.approach == a[1]}:
                            info = (a[1], b[1])
            self.link_info.append(info)

    def walk_arms(self, status):
        if not self.crossings or status["state"] != "GREEN":
            return set()
        sig = status["signals"]
        return {a.name for a in self.layout.arms if sig.get(a.name, "red") == "red"}

    def state_string(self, signals, walk):
        chars = []
        for i, info in enumerate(self.link_info):
            if i in self.crossings:
                chars.append("G" if self.crossings[i] in walk else "r")
            elif info is None:
                chars.append("r")
            else:
                approach, dest = info
                colour = signals.get(approach, "red")
                if colour == "green":
                    chars.append("g" if (approach, dest) in self.yields or dest in walk else "G")
                else:
                    chars.append(COLOUR[colour])
        return "".join(chars)


def read_tripinfo(path):
    delays, travel, stops = [], [], []
    ped = []
    root = ET.parse(path).getroot()
    for t in root.findall("tripinfo"):
        delays.append(float(t.get("timeLoss")))
        travel.append(float(t.get("duration")))
        stops.append(float(t.get("waitingCount", 0) or 0))
    for p in root.findall("personinfo"):
        walks = p.findall("walk")
        if walks:
            ped.append(sum(float(w.get("timeLoss", 0) or 0) for w in walks))
    return delays, travel, stops, ped


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(q * (len(xs) - 1))))] if xs else 0.0


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def run_one(net_path, scenario, mode, seed=1, gui=False, predictor=None, link_weight=None,
            params=None, loss=0.0, latency=1.0, pedestrians=True, out_dir=None, max_time=MAX_TIME,
            traci_module=None, sumo_binary=None, work_dir=None):
    """Run one simulation and return a dictionary of results."""
    traci = traci_module
    if traci is None:
        import traci                                           # noqa: F811
    net_path = Path(net_path)
    net = Network.load(net_path)
    work = Path(work_dir) if work_dir else HERE / "sumo" / net_path.stem
    net_xml, map_path = work / "net.net.xml", work / "map.json"
    if not net_xml.exists() and traci_module is None:
        raise SystemExit(f"Build the network first:  python sumo_build.py {net_path}")
    info = json.loads(map_path.read_text())
    if not pedestrians:
        info["crossings"] = {n: [] for n in info["crossings"]}
    out_dir = Path(out_dir) if out_dir else work / "results"
    out_dir.mkdir(parents=True, exist_ok=True)
    rou = work / "demand" / f"{scenario}.rou.xml"
    rou.parent.mkdir(exist_ok=True)
    rou.write_text(build_routes(net, scenario, map_path, ped_rate=120.0 if pedestrians else 0.0))
    tag = f"{scenario}_{mode}{'_ai' if predictor else ''}{'_gm' if link_weight else ''}_s{seed}"
    tripinfo = out_dir / f"tripinfo_{tag}.xml"

    if sumo_binary is None:
        from find_sumo import find_binary
        sumo_binary = find_binary("sumo-gui" if gui else "sumo")
    cmd = [sumo_binary, "-n", str(net_xml), "-r", str(rou), "--step-length", str(STEP),
           "--seed", str(seed), "--tripinfo-output", str(tripinfo), "--no-step-log", "true",
           "--no-warnings", "true", "--time-to-teleport", "-1", "--duration-log.disable", "true"]
    if gui:
        cmd += ["--start", "true", "--quit-on-end", "true", "--delay", "50"]
        settings = HERE.parent / "sim" / "gui_settings.xml"
        if settings.exists():
            cmd += ["--gui-settings-file", str(settings)]

    rng = random.Random(seed)
    nodes = {n: NodeBinding(net, n, info, mode, net_xml) for n in net.nodes}
    bus = Bus(latency, loss, rng)
    coords = {}
    if mode == "coordinated":
        coords = {n: NodeCoordinator(n, net, bus, params, predictor, link_weight) for n in net.nodes}
    lane_len, ped_w = {}, config.PERSON_WEIGHT
    blocked_steps = node_steps = 0
    max_fill = 0.0
    queue_sum = steps = 0

    traci.start(cmd)
    try:
        for nb in nodes.values():
            nb.read_links(traci)
        last = {n: None for n in nodes}

        def weight(veh):
            return config.VEHICLE_WEIGHTS.get(SIM_TYPE_TO_COCO.get(traci.vehicle.getTypeID(veh), 2), 1.0)

        while traci.simulation.getMinExpectedNumber() > 0 and traci.simulation.getTime() < max_time:
            now = traci.simulation.getTime()
            for msg in bus.deliver(now):
                if msg.dst in coords:
                    coords[msg.dst].receive(msg)
            waiting = {}
            if pedestrians and any(nb.crossings for nb in nodes.values()):
                try:
                    for pid in traci.person.getIDList():
                        if traci.person.getSpeed(pid) < 0.1 and traci.person.getWaitingTime(pid) > 0:
                            parts = pid.split(".")[0].split("_")
                            if len(parts) >= 3 and parts[0] == "pf":
                                waiting[(parts[1], parts[2])] = waiting.get((parts[1], parts[2]), 0) + 1
                except Exception as exc:                          # keep the vehicle run alive
                    print(f"Pedestrian measurement switched off ({exc}).")
                    pedestrians = False

            for n, nb in nodes.items():
                seen, halting = {}, 0
                for a, edge in nb.in_edge.items():
                    load = 0.0
                    for veh in traci.edge.getLastStepVehicleIDs(edge):
                        lane = traci.vehicle.getLaneID(veh)
                        if lane not in lane_len:
                            lane_len[lane] = traci.lane.getLength(lane)
                        if lane_len[lane] - traci.vehicle.getLanePosition(veh) <= ZONE_M:
                            load += weight(veh)
                    seen[a] = load
                    halting += traci.edge.getLastStepHaltingNumber(edge)
                loads = dict(seen)
                for (nn, arm), count in waiting.items():
                    if nn != n or not count:
                        continue
                    if "PED" in nb.phases:                          # a phase of their own
                        loads["ped"] = loads.get("ped", 0.0) + ped_w * count
                    else:                                           # served by phases that do not use the arm
                        for apps in nb.phases.values():
                            if arm not in apps:
                                for a in apps:
                                    loads[a] = loads.get(a, 0.0) + ped_w * count / len(apps)
                if n in coords:
                    sensors = _sensors(traci, net, nb, seen, weight, now)
                    coords[n].send_updates(now, sensors)
                    en_route = {l.src_arm: sensors["_on_link"][l.id]
                                for l in net.links.values() if l.src == n}
                    nb.ctrl.score_adjust = coords[n].adjustments(now, seen, en_route, nb.shares)
                status = nb.ctrl.update(loads, STEP)
                state = nb.state_string(status["signals"], nb.walk_arms(status))
                if state != last[n]:
                    traci.trafficlight.setRedYellowGreenState(n, state)
                    last[n] = state
                queue_sum += halting
            traci.simulationStep()
            steps += 1
            for l in net.links.values():
                occ = sum(weight(v) for v in traci.edge.getLastStepVehicleIDs(l.id.replace(">", "__")))
                max_fill = max(max_fill, occ / l.capacity)
        unfinished = traci.simulation.getMinExpectedNumber()
        sim_time = traci.simulation.getTime()
    finally:
        traci.close()

    delays, travel, stops, ped = read_tripinfo(tripinfo)
    return {
        "network": net.name, "scenario": scenario, "mode": mode, "seed": seed,
        "completed": len(delays), "unfinished": unfinished,
        "mean_delay_s": round(mean(delays), 2), "p95_delay_s": round(pct(delays, 0.95), 2),
        "mean_travel_s": round(mean(travel), 2), "mean_stops": round(mean(stops), 2),
        "throughput_vph": round(len(delays) / (sim_time / 3600.0), 1) if sim_time else 0.0,
        "max_link_fill": round(max_fill, 2),
        "mean_queue_veh": round(queue_sum / max(1, steps), 2),
        "ped_mean_delay_s": round(mean(ped), 2) if ped else None,
        "peds_completed": len(ped),
        "messages_sent": bus.sent, "messages_dropped": bus.dropped,
    }


def _sensors(traci, net, nb, seen, weight, now):
    """The numbers a node tells its neighbours, measured from SUMO."""
    sig = nb.ctrl.signals
    sensors = {"queue": {}, "zone_cap": {}, "eta": {}, "green_to": {}, "queued_for": {}, "_on_link": {}}
    for a in nb.layout.approaches:
        sensors["queue"][a] = seen[a]
        sensors["zone_cap"][a] = ZONE_M / JAM_SPACING_M * nb.lanes[a]
    for link in net.links.values():
        if link.src != nb.id:
            continue
        edge = link.id.replace(">", "__")
        buckets, total = [0.0, 0.0, 0.0], 0.0
        length = traci.lane.getLength(f"{edge}_0")
        for veh in traci.edge.getLastStepVehicleIDs(edge):
            w = weight(veh)
            total += w
            eta = (length - traci.vehicle.getLanePosition(veh)) / link.speed
            for i, (lo, hi) in enumerate(((0, 10), (10, 20), (20, 40))):
                if lo <= eta < hi:
                    buckets[i] += w
        sensors["eta"][link.id] = buckets
        sensors["_on_link"][link.id] = total
        sensors["green_to"][link.id] = any(
            sig.get(a) == "green" and nb.shares.get((a, link.src_arm), 0) > 0
            for a in nb.layout.approaches)
        sensors["queued_for"][link.id] = sum(
            seen[a] * nb.shares.get((a, link.src_arm), 0.0) for a in nb.layout.approaches)
    return sensors


def load_predictor():
    path = HERE / "models" / "predictor.json"
    if not path.exists():
        raise SystemExit("No models/predictor.json yet. Run:  python train_predictor.py")
    return Predictor.load(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("network")
    ap.add_argument("--scenario", default="peak", choices=list(SCENARIOS))
    ap.add_argument("--mode", default="coordinated", choices=MODES)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--ai", action="store_true", help="use the learned arrival predictor")
    ap.add_argument("--gmaps", default=None, help="profile.json from gmaps/fetch_profile.py")
    ap.add_argument("--hour", type=int, default=8, help="hour of day for the Google weights")
    ap.add_argument("--loss", type=float, default=0.0)
    ap.add_argument("--no-pedestrians", action="store_true")
    args = ap.parse_args()
    link_weight = None
    if args.gmaps:
        import priorities
        link_weight = priorities.link_weight_fn(priorities.load_profile(args.gmaps), args.hour)
    result = run_one(args.network, args.scenario, args.mode, seed=args.seed, gui=args.gui,
                     predictor=load_predictor() if args.ai else None, link_weight=link_weight,
                     loss=args.loss, pedestrians=not args.no_pedestrians)
    print(json.dumps(result, indent=2))
    if result["unfinished"]:
        print(f"\nWarning: {result['unfinished']} vehicles had not finished when the run stopped.")


if __name__ == "__main__":
    main()
