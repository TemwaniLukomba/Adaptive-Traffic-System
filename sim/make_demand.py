"""Generate demand scenarios (vehicle flows) for the junction.

    python make_demand.py            # writes demand/<scenario>.rou.xml for all scenarios
    python make_demand.py tidal      # just one

A scenario is a list of periods: (start_s, end_s, {approach: vehicles_per_hour}).
Vehicles arrive randomly (a chance each second). Each arrival is a car, minibus,
truck or motorcycle according to MIX, and goes straight, left or right according
to sim_config.TURN_SHARE, so every movement the layout allows (including all the turns)
carries traffic. Scenarios are built from the layout: "the first phase" is north-south at a
crossroads and the main road at a T-junction, and rates scale with the number of lanes. With pedestrians on, every arm also gets walkers crossing it in both
directions (sim_config.PED_RATE per hour, or PED_RATES below for a scenario). The demand
figures are illustrative, not measured: replace them with counts from a real junction.
"""
import sys
from pathlib import Path

import sim_config as sc

DEMAND_DIR = Path(__file__).resolve().parent / "demand"


def _all(v):
    return {a: v for a in sc.APPROACHES}


def _lanes(a):
    return sc.LAYOUT.arm(a).lanes


def _scaled(rates):
    """Demand is written for two lanes; scale it to the lanes the arm actually has."""
    return {a: v * _lanes(a) / 2.0 for a, v in rates.items()}


def _build_scenarios():
    groups = [apps for apps in sc.PHASES.values() if apps != ["ped"]]   # approaches per phase
    first = groups[0]
    second = groups[1] if len(groups) > 1 else groups[0]
    heavy = lambda grp, hi=800, lo=200: _scaled({a: (hi if a in grp else lo) for a in sc.APPROACHES})
    lead = sc.APPROACHES[0]
    side = {a: v for a, v in zip(sc.APPROACHES[1:], [250, 200, 150, 150, 150, 150, 150])}
    balanced = _scaled(_all(450))
    return {
        # even traffic on every approach
        "balanced": [(0, 1800, balanced)],
        # the first phase's approaches (north-south at a crossroads, the main road at a T) are heavy
        "main_peak": [(0, 1800, heavy(first))],
        # the heavy phase flips halfway, then evens out
        "tidal": [(0, 600, heavy(first)), (600, 1200, heavy(second)), (1200, 1800, balanced)],
        # one dominant approach
        "one_sided": [(0, 1800, _scaled({lead: 900, **side}))],
        # ordinary vehicle traffic but a lot of people on foot (see PED_RATES)
        "ped_heavy": [(0, 1800, balanced)],
        # calm, then a five-minute surge on one approach, then calm again
        "surge": [(0, 600, _scaled(_all(300))),
                  (600, 900, _scaled({a: (1000 if a == lead else 300) for a in sc.APPROACHES})),
                  (900, 1800, _scaled(_all(300)))],
    }


SCENARIOS = _build_scenarios()

# pedestrians per hour per arm for scenarios that need more (or fewer) than sim_config.PED_RATE
PED_RATES = {"ped_heavy": 500}

# share of arrivals by vehicle type (a rough local-style mix; adjust to your footage)
MIX = {"car": 0.70, "minibus": 0.14, "truck": 0.08, "motorcycle": 0.08}

# guiShape makes sumo-gui draw a sedan, a van, a truck, a motorcycle and a walking person
# instead of the default triangles (see also gui_settings.xml).
VTYPES = """\
    <vType id="car" vClass="passenger" length="4.5" width="1.8" accel="2.6" decel="4.5" sigma="0.5" maxSpeed="13.89" color="1,0.85,0" guiShape="passenger/sedan"/>
    <vType id="minibus" vClass="bus" length="5.2" width="1.9" accel="1.8" decel="4.0" sigma="0.5" maxSpeed="13.89" color="0,0.6,1" guiShape="passenger/van"/>
    <vType id="truck" vClass="truck" length="10" width="2.5" accel="1.3" decel="4.0" sigma="0.5" maxSpeed="11.11" color="0.6,0.3,0" guiShape="truck"/>
    <vType id="motorcycle" vClass="motorcycle" length="2.2" width="0.8" accel="3.0" decel="5.0" sigma="0.5" maxSpeed="13.89" color="1,0,1" guiShape="motorcycle"/>
"""

PEDTYPE = ('    <vType id="ped" vClass="pedestrian" width="0.6" length="0.3" '
           'minGap="0.3" maxSpeed="1.39" color="0.1,0.8,0.2" guiShape="pedestrian"/>')

# one route per approach and destination arm (12 for a crossroads, 6 for a T-junction, ...)
ROUTES = {
    (approach, dest): sc.route_edges(approach, dest)
    for approach in sc.APPROACHES
    for dest in sc.MOVES[approach]
}


def ped_rate(name):
    return PED_RATES.get(name, sc.PED_RATE)


def person_flows(name, begin, end):
    """Walkers crossing each arm that has a crossing. ids look like pf_north_fwd_0 (arm = 2nd part)."""
    rate = ped_rate(name)
    if rate <= 0:
        return []
    directions = ("fwd", "back") if sc.PED_BOTH_DIRECTIONS else ("fwd",)
    p = rate / len(directions) / 3600.0                      # chance of one walker per second
    lines = []
    for arm in sc.WALKABLE_ARMS:
        letter = sc.ARM_LETTER[arm]
        inbound, outbound = f"{letter}2{sc.TLS_ID}", f"{sc.TLS_ID}2{letter}"
        for d in directions:
            a, b = ((inbound, sc.PED_APPROACH_POS), (outbound, sc.PED_EXIT_POS)) if d == "fwd" \
                else ((outbound, sc.PED_EXIT_POS), (inbound, sc.PED_APPROACH_POS))
            lines.append(
                f'    <personFlow id="pf_{arm}_{d}_{begin}" type="ped" begin="{begin}" '
                f'end="{end}" probability="{p:.5f}">\n'
                f'        <walk from="{a[0]}" to="{b[0]}" departPos="{a[1]:g}" arrivalPos="{b[1]:g}"/>\n'
                f'    </personFlow>')
    return lines


def build(name, pedestrians=None):
    """XML for one scenario. pedestrians=None uses sim_config.PEDESTRIANS."""
    pedestrians = sc.PEDESTRIANS if pedestrians is None else pedestrians
    periods = SCENARIOS[name]
    lines = [
        '<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
        'xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">',
        VTYPES.rstrip("\n"),
    ]
    if pedestrians:
        lines.append(PEDTYPE)
    for (approach, dest), edges in ROUTES.items():
        lines.append(f'    <route id="r_{approach}_{dest}" edges="{edges}"/>')

    n = 0
    for begin, end, rates in sorted(periods, key=lambda p: p[0]):   # flows sorted by start time
        for approach in sc.APPROACHES:
            vph = rates.get(approach, 0)
            for dest, turn_share in sc.movement_shares(approach).items():
                for vtype, share in MIX.items():
                    p = vph * turn_share * share / 3600.0     # chance of one arrival per second
                    if p <= 0:
                        continue
                    if p > 1:
                        raise ValueError(f"{name}: {vph} veh/h is too high for a one-per-second flow")
                    lines.append(
                        f'    <flow id="f{n}_{approach}_{dest}_{vtype}" type="{vtype}" '
                        f'route="r_{approach}_{dest}" begin="{begin}" end="{end}" '
                        f'probability="{p:.5f}" departLane="best" departSpeed="max"/>'
                    )
                    n += 1
        if pedestrians:
            lines += person_flows(name, begin, end)
    lines.append('</routes>')
    return "\n".join(lines) + "\n"


def write(name, pedestrians=None):
    DEMAND_DIR.mkdir(exist_ok=True)
    path = DEMAND_DIR / f"{name}.rou.xml"
    path.write_text(build(name, pedestrians))
    return path


def main():
    names = sys.argv[1:] or list(SCENARIOS)
    for name in names:
        if name not in SCENARIOS:
            sys.exit(f"Unknown scenario '{name}'. Choose from: {', '.join(SCENARIOS)}")
        print("wrote", write(name))


if __name__ == "__main__":
    main()
