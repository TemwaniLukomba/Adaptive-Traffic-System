"""Run the junction in SUMO under one signal controller and measure the result.

    python run_sim.py --scenario tidal --controller adaptive
    python run_sim.py --scenario tidal --controller adaptive --gui
    python run_sim.py --scenario ns_peak --controller fixed
    python run_sim.py --scenario surge --controller adaptive --miss-rate 0.15
    python run_sim.py --scenario one_sided --controller adaptive --set demand_threshold=0.3
    python run_sim.py --scenario ped_heavy --controller adaptive --gui
    python run_sim.py --scenario tidal --controller adaptive --no-pedestrians

Controllers: adaptive (the project's AdaptiveController), fixed (FixedTimeController),
longest_queue (simple demand-responsive baseline).

Every simulated second:
  1. measure the weighted load in the last ZONE_LENGTH metres of each approach,
     the way the camera zones do (optionally missing some vehicles, --miss-rate),
  2. give those loads to the controller (the same controller.py the prototype uses),
  3. translate its signal colours into SUMO's traffic-light state and step SUMO.

The road comes from the layout in sim_config.py (crossroads, T-junction, five-way, one-way
streets, a road-works shuttle, a pedestrian crossing, or a layout learned from video). Every
movement the layout allows carries traffic, and the movements that must give way while their
phase is green (for example right turns when driving on the left) are worked out from the
geometry by layout.py.

Pedestrians (sim_config.PEDESTRIANS): each arm has a zebra crossing. It shows walk only while
the controller is GREEN for the other phase, and closes the moment that green turns yellow, so
the yellow plus all-red time (5 s) is the clearance for anyone still on the crossing. Vehicles
turning onto an arm whose crossing is open give way ('g'). Pedestrians waiting at a crossing
add sim_config.PED_WEIGHT each to the phase that would let them cross, so the adaptive
controller sees them as demand (--ped-weight 0 turns that off).
"""
import argparse
import json
import random
import sys
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR.parent))   # the project's controller.py and config.py
sys.path.insert(0, str(SIM_DIR))

import config                                       # noqa: E402
import make_demand                                  # noqa: E402
import sim_config as sc                             # noqa: E402
from baselines import LongestQueueController        # noqa: E402
from controller import AdaptiveController, FixedTimeController   # noqa: E402
from find_sumo import find_binary                   # noqa: E402

CONTROLLERS = ("fixed", "longest_queue", "adaptive")
COLOUR = {"green": "G", "yellow": "y", "red": "r"}


def make_controller(name, overrides=None):
    overrides = overrides or {}
    if name == "adaptive":
        return AdaptiveController(phases=sc.PHASES, **overrides)
    if name == "fixed":
        return FixedTimeController(phases=sc.PHASES, **overrides)
    if name == "longest_queue":
        return LongestQueueController(phases=sc.PHASES, **overrides)
    raise SystemExit(f"Unknown controller '{name}'. Choose from: {', '.join(CONTROLLERS)}")


def resolve_scenario(scenario, pedestrians=None):
    """A scenario is a name from make_demand.SCENARIOS or a path to a .rou.xml file."""
    path = Path(scenario)
    if path.suffix == ".xml" and path.exists():
        return path
    if scenario in make_demand.SCENARIOS:
        return make_demand.write(scenario, pedestrians)   # always regenerate, so edits take effect
    raise SystemExit(f"Unknown scenario '{scenario}'. Choose from: "
                     f"{', '.join(make_demand.SCENARIOS)} or give a .rou.xml path.")


def link_movement(in_lane, out_lane):
    """Identify a signal link as (approach, destination arm), e.g. ('east', 'south').

    Lane ids look like 'E2C_0' (into the junction) and 'C2S_1' (out of it); arm codes can be
    several letters ('NE2C_0'). Internal lanes (':C_w0_0', crossings) return None."""
    if in_lane.startswith(":") or out_lane.startswith(":"):
        return None
    code_to_arm = {code: arm for arm, code in sc.ARM_LETTER.items()}
    e_in, e_out = in_lane.rsplit("_", 1)[0], out_lane.rsplit("_", 1)[0]
    if not e_in.endswith(f"2{sc.TLS_ID}") or not e_out.startswith(f"{sc.TLS_ID}2"):
        return None
    approach = code_to_arm.get(e_in[: -len(f"2{sc.TLS_ID}")])
    dest = code_to_arm.get(e_out[len(f"{sc.TLS_ID}2"):])
    if approach is None or dest is None or dest not in sc.MOVES.get(approach, {}):
        return None                                    # a U-turn or something not modelled
    return approach, dest


def crossing_links(net_path):
    """{signal index: arm} for the pedestrian crossings in the built network ({} if none).

    A crossing is an edge with function="crossing" whose crossingEdges name the road it
    crosses (for example 'N2C C2N' is the crossing over the north arm)."""
    root = ET.parse(net_path).getroot()
    arm_of = {}
    for edge in root.findall("edge"):
        if edge.get("function") == "crossing" and edge.get("crossingEdges"):
            first = edge.get("crossingEdges").split()[0]
            a, b = first.split("2")
            letter = a if b == sc.TLS_ID else b
            arm_of[edge.get("id")] = next((n for n, l in sc.ARM_LETTER.items() if l == letter), None)
    found = {}
    for con in root.findall("connection"):
        if con.get("tl") != sc.TLS_ID or con.get("linkIndex") is None:
            continue
        for key in ("from", "to"):
            if con.get(key) in arm_of and arm_of[con.get(key)]:
                found[int(con.get("linkIndex"))] = arm_of[con.get(key)]
    return found


def walk_arms(status, crossings):
    """Arms whose crossing shows walk: only while the OTHER phase is green (not yellow,
    not all-red), so anyone still crossing has the yellow + all-red time to clear."""
    if not crossings or status["state"] != "GREEN":
        return set()
    signals = status["signals"]
    return {arm.name for arm in sc.LAYOUT.arms if signals.get(arm.name, "red") == "red"}


def state_string(link_info, signals, crossings=None, walk=()):
    """SUMO wants one character per controlled link.

    r = red, y = yellow, G = green with priority, g = green but give way.
    While a phase is green, movements the layout says must give way (for example right turns
    crossing oncoming traffic when driving on the left) get 'g'.
    A vehicle turning onto an arm whose crossing is open (walk) also gets 'g'.
    crossings maps signal index -> arm for pedestrian links: 'G' when walking, else 'r'."""
    crossings = crossings or {}
    chars = []
    for i, info in enumerate(link_info):
        if i in crossings:
            chars.append("G" if crossings[i] in walk else "r")
            continue
        if info is None:
            chars.append("r")
            continue
        approach, dest = info
        colour = signals[approach]
        if colour == "green":
            yields = (approach, dest) in sc.YIELD_MOVES or dest in walk
            chars.append("g" if yields else "G")
        else:
            chars.append(COLOUR[colour])
    return "".join(chars)


def read_personinfo(path):
    """Delay and wait for each pedestrian that finished walking."""
    delays, waits = [], []
    for person in ET.parse(path).getroot().findall("personinfo"):
        walks = person.findall("walk")
        if not walks:
            continue
        loss = sum(float(w.get("timeLoss", 0) or 0) for w in walks)
        wait = sum(float(w.get("waitingTime", w.get("timeLoss", 0)) or 0) for w in walks)
        delays.append(loss)
        waits.append(wait)
    return delays, waits


def read_tripinfo(path):
    delays, waits, travel, stops = [], [], [], []
    for trip in ET.parse(path).getroot().findall("tripinfo"):
        delays.append(float(trip.get("timeLoss")))
        waits.append(float(trip.get("waitingTime")))
        travel.append(float(trip.get("duration")))
        if trip.get("waitingCount") is not None:
            stops.append(float(trip.get("waitingCount")))
    return delays, waits, travel, stops


def percentile(values, q):
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(q * (len(ordered) - 1))))]


def mean(values):
    return sum(values) / len(values) if values else 0.0


def run_one(scenario, controller_name, seed=1, miss_rate=0.0, gui=False, overrides=None,
            max_time=sc.MAX_TIME, zone_length=sc.ZONE_LENGTH, out_dir=None, trace=False,
            pedestrians=None, ped_weight=None):
    """Run one simulation and return a dictionary of results.

    pedestrians: None follows sim_config.PEDESTRIANS; False forces a vehicles-only run.
    ped_weight: load one waiting pedestrian adds to the phase that lets them cross."""
    import traci

    net = SIM_DIR / "network" / "junction.net.xml"
    if not net.exists():
        raise SystemExit("The network has not been built yet. Run:  python build_network.py")
    saved = net.parent / "layout.json"
    if saved.exists() and json.loads(saved.read_text()) != json.loads(json.dumps(sc.LAYOUT.to_dict())):
        raise SystemExit("The built network is for a different road layout than sim_config.py / "
                         "SIM_LAYOUT now selects. Rebuild it:  python build_network.py")
    crossings = crossing_links(net)
    if pedestrians is None:
        pedestrians = sc.PEDESTRIANS
    if pedestrians and not crossings:
        print("Note: the network has no pedestrian crossings (it was built before pedestrians "
              "were added), so this run has no pedestrians. Rebuild with:  python build_network.py")
        pedestrians = False
    if not pedestrians:
        crossings = {}
    ped_weight = sc.PED_WEIGHT if ped_weight is None else ped_weight
    rou = resolve_scenario(scenario, pedestrians)
    out_dir = Path(out_dir) if out_dir else SIM_DIR / "results"
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = f"{rou.name.replace('.rou.xml', '')}_{controller_name}_s{seed}"
    if not pedestrians:
        tag += "_nopeds"
    if miss_rate:
        tag += f"_miss{int(round(miss_rate * 100))}"
    tripinfo = out_dir / f"tripinfo_{tag}.xml"

    cmd = [find_binary("sumo-gui" if gui else "sumo"),
           "-n", str(net), "-r", str(rou),
           "--step-length", str(sc.STEP_LENGTH), "--seed", str(seed),
           "--tripinfo-output", str(tripinfo),
           "--no-step-log", "true", "--no-warnings", "true",
           "--time-to-teleport", "-1", "--duration-log.disable", "true"]
    if gui:
        cmd += ["--start", "true", "--quit-on-end", "true", "--delay", "50"]
        gui_settings = SIM_DIR / "gui_settings.xml"
        if gui_settings.exists():              # draws cars, vans, trucks, motorcycles, people
            cmd += ["--gui-settings-file", str(gui_settings)]

    controller = make_controller(controller_name, overrides)
    rng = random.Random(seed)
    history = {a: deque(maxlen=max(1, int(config.SMOOTHING_SECONDS / sc.STEP_LENGTH)))
               for a in sc.APPROACHES}
    lane_length = {}
    trace_rows = []
    max_queue = 0
    queue_sum = 0.0
    steps = 0
    phase_changes = 0
    ped_ok = bool(pedestrians)             # set False if this SUMO build rejects a person call
    ped_wait_sum = 0.0
    peak_waiting_peds = 0
    walk_steps = 0
    arm_phase = {a: [p for p, apps in sc.PHASES.items() if a not in apps] for a in sc.WALKABLE_ARMS}

    traci.start(cmd)
    try:
        # which approach and movement each signal index belongs to
        link_info = []
        for group in traci.trafficlight.getControlledLinks(sc.TLS_ID):
            link_info.append(link_movement(group[0][0], group[0][1]) if group else None)
        last_state = None

        while traci.simulation.getMinExpectedNumber() > 0 and traci.simulation.getTime() < max_time:
            # 1) what the "camera" sees: weighted vehicles in the zone before each stop line
            loads, halting = {}, {}
            for approach, edge in sc.EDGE_IN.items():
                load = 0.0
                for veh in traci.edge.getLastStepVehicleIDs(edge):
                    lane = traci.vehicle.getLaneID(veh)
                    if lane not in lane_length:
                        lane_length[lane] = traci.lane.getLength(lane)
                    if lane_length[lane] - traci.vehicle.getLanePosition(veh) > zone_length:
                        continue                                   # outside the zone
                    if miss_rate and rng.random() < miss_rate:
                        continue                                   # a missed detection
                    coco = sc.SIM_TYPE_TO_COCO.get(traci.vehicle.getTypeID(veh), 2)
                    load += config.VEHICLE_WEIGHTS.get(coco, 1.0)
                history[approach].append(load)
                loads[approach] = sum(history[approach]) / len(history[approach])
                halting[approach] = traci.edge.getLastStepHaltingNumber(edge)

            # 1b) pedestrians waiting at a crossing add to the phase that lets them cross
            waiting_peds = 0
            if ped_ok:
                try:
                    waiting = {a: 0 for a in sc.WALKABLE_ARMS}
                    for pid in traci.person.getIDList():
                        if traci.person.getSpeed(pid) < 0.1 and traci.person.getWaitingTime(pid) > 0:
                            arm = pid.split("_")[1] if pid.startswith("pf_") else None
                            if arm in waiting:
                                waiting[arm] += 1
                    waiting_peds = sum(waiting.values())
                    for arm, n in waiting.items():
                        if not n:
                            continue
                        for phase_name in arm_phase[arm]:        # the phase that does not use this arm
                            apps = sc.PHASES[phase_name]
                            for a in apps:
                                loads[a] = loads.get(a, 0.0) + ped_weight * n / len(apps)
                except Exception as exc:                          # keep the vehicle run alive
                    print(f"Pedestrian measurement switched off ({exc}).")
                    ped_ok = False
            peak_waiting_peds = max(peak_waiting_peds, waiting_peds)
            ped_wait_sum += waiting_peds

            # 2) the controller decides
            status = controller.update(loads, sc.STEP_LENGTH)
            phase_changes += sum(1 for e in controller.pop_events() if e["state"] == "GREEN")

            # 3) apply to SUMO and advance one step
            walk = walk_arms(status, crossings)
            walk_steps += 1 if walk else 0
            state = state_string(link_info, status["signals"], crossings, walk)
            if state != last_state:
                traci.trafficlight.setRedYellowGreenState(sc.TLS_ID, state)
                last_state = state
            traci.simulationStep()

            worst = max(halting.values())
            max_queue = max(max_queue, worst)
            queue_sum += sum(halting.values())
            steps += 1
            if trace:
                trace_rows.append([traci.simulation.getTime(), status["phase"], status["state"]]
                                  + [round(loads[a], 2) for a in sc.APPROACHES]
                                  + [halting[a] for a in sc.APPROACHES])

        unfinished = traci.simulation.getMinExpectedNumber()
        sim_time = traci.simulation.getTime()
    finally:
        traci.close()

    delays, waits, travel, stops = read_tripinfo(tripinfo)
    ped_delays, ped_waits = read_personinfo(tripinfo) if pedestrians else ([], [])
    result = {
        "scenario": rou.name.replace(".rou.xml", ""),
        "controller": controller_name,
        "seed": seed,
        "miss_rate": miss_rate,
        "completed": len(delays),
        "unfinished": unfinished,
        "mean_delay_s": round(mean(delays), 2),
        "p95_delay_s": round(percentile(delays, 0.95), 2),
        "max_wait_s": round(max(waits) if waits else 0.0, 2),
        "mean_travel_s": round(mean(travel), 2),
        "mean_stops": round(mean(stops), 2) if stops else None,
        "max_queue_veh": max_queue,
        "mean_queue_veh": round(queue_sum / steps, 2) if steps else 0.0,
        "clear_time_s": round(sim_time, 1),
        "throughput_vph": round(len(delays) / (sim_time / 3600.0), 1) if sim_time else 0.0,
        "phase_changes": phase_changes,
        "pedestrians": bool(pedestrians),
        "peds_completed": len(ped_delays),
        "ped_mean_delay_s": round(mean(ped_delays), 2) if pedestrians else None,
        "ped_p95_delay_s": round(percentile(ped_delays, 0.95), 2) if pedestrians else None,
        "ped_max_wait_s": round(max(ped_waits), 2) if ped_waits else (0.0 if pedestrians else None),
        "ped_peak_waiting": peak_waiting_peds if pedestrians else None,
        "walk_time_share": round(walk_steps / steps, 3) if pedestrians and steps else None,
    }
    if trace:
        header = ["time", "phase", "state"] + [f"{a}_load" for a in sc.APPROACHES] \
                 + [f"{a}_halting" for a in sc.APPROACHES]
        trace_path = out_dir / f"trace_{tag}.csv"
        trace_path.write_text("\n".join(",".join(map(str, r)) for r in [header] + trace_rows) + "\n")
    return result


def parse_overrides(pairs):
    out = {}
    for pair in pairs or []:
        key, _, value = pair.partition("=")
        if not value:
            raise SystemExit(f"--set expects key=value, got '{pair}'")
        out[key.strip()] = float(value)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", default="tidal",
                        help=f"one of {', '.join(make_demand.SCENARIOS)} or a .rou.xml path")
    parser.add_argument("--controller", default="adaptive", choices=CONTROLLERS)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--miss-rate", type=float, default=0.0,
                        help="fraction of vehicles the 'camera' misses (0 to 1)")
    parser.add_argument("--gui", action="store_true", help="watch it in sumo-gui")
    parser.add_argument("--trace", action="store_true", help="also save a per-second CSV")
    parser.add_argument("--set", nargs="*", metavar="KEY=VALUE",
                        help="override controller settings, e.g. demand_threshold=0.3 min_green=6")
    parser.add_argument("--max-time", type=float, default=sc.MAX_TIME)
    parser.add_argument("--no-pedestrians", action="store_true",
                        help="vehicles only (no walkers, no walk phases)")
    parser.add_argument("--ped-weight", type=float, default=None,
                        help=f"load per waiting pedestrian (default {sc.PED_WEIGHT}; 0 = controller ignores them)")
    args = parser.parse_args()

    result = run_one(args.scenario, args.controller, seed=args.seed, miss_rate=args.miss_rate,
                     gui=args.gui, overrides=parse_overrides(args.set), max_time=args.max_time,
                     trace=args.trace, pedestrians=False if args.no_pedestrians else None,
                     ped_weight=args.ped_weight)
    print(json.dumps(result, indent=2))
    if result["unfinished"]:
        print(f"\nWarning: {result['unfinished']} vehicles had not finished when the run stopped.")


if __name__ == "__main__":
    main()
