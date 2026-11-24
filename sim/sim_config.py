"""Settings for the SUMO junction simulation.

The road itself comes from a LAYOUT (see ../layout.py): a list of arms with compass
bearings, lane counts and one-way flags. Movements, signal phases and give-way rules are
derived from it, so the simulation works for any junction, not just a crossroads.

Choose the road with LAYOUT_SETTING (or the SIM_LAYOUT environment variable):
    "crossroads", "t_junction", "y_junction", "five_way", "shuttle", "road_crossing",
    "one_way_cross", or the path of a site.json written by autocalibrate.py, which simulates
    the junction the camera actually learned (../site.json).
After changing it, rebuild the network:  python build_network.py

These settings are separate from the project's config.py on purpose. Timing constants
(min green, yellow, ...) still come from config.py so the simulated controller behaves
exactly like the prototype's.
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import layout as _layout                                    # noqa: E402

LAYOUT_SETTING = "crossroads"
LANES = 2                         # lanes per direction (presets only)
DRIVE_ON_LEFT = True              # driving side (presets only; Zambia drives on the left)
EDGE_LENGTH = 300.0               # metres from the outer node to the junction (presets)

LAYOUT = _layout.resolve(os.environ.get("SIM_LAYOUT") or LAYOUT_SETTING,
                         lanes=LANES, drive_on_left=DRIVE_ON_LEFT)
for _arm in LAYOUT.arms:
    _arm.length = _arm.length or EDGE_LENGTH

# --- derived from the layout --------------------------------------------------------------
DRIVE_ON_LEFT = LAYOUT.drive_on_left
APPROACHES = LAYOUT.approaches                    # arms vehicles enter from
PHASES = LAYOUT.phases()                          # {"NS": [...], "EW": [...]}, plus "PED" if used
ARM_LETTER = LAYOUT.codes()                       # arm name -> code used in edge ids (N, S, NE ...)
TLS_ID = "C"                                      # the signalised centre node
EDGE_IN = {a: f"{ARM_LETTER[a]}2{TLS_ID}" for a in APPROACHES}
MOVES = {a: {m.dest: m.kind for m in LAYOUT.movements() if m.approach == a} for a in APPROACHES}
YIELD_MOVES = LAYOUT.yielding_movements()         # {(approach, dest)} that give way when green

# Share of each approach's vehicles making each movement (illustrative, not measured).
# If a turn does not exist (a T-junction has no straight-on from the side road) its share
# is spread over the turns that do.
TURN_SHARE = {"straight": 0.70, "left": 0.15, "right": 0.15}

# Simulation
STEP_LENGTH = 1.0                 # seconds per simulation step (also the controller's dt)
ZONE_LENGTH = 50.0                # metres before the stop line that the "camera" sees
MAX_TIME = 5400                   # hard stop in seconds, in case a controller gridlocks

# SUMO vehicle type -> COCO class id, so config.VEHICLE_WEIGHTS applies unchanged.
SIM_TYPE_TO_COCO = {"car": 2, "minibus": 5, "truck": 7, "motorcycle": 3}


def movement_shares(approach):
    return LAYOUT.movement_shares(approach, TURN_SHARE)


def route_edges(approach, dest):
    """SUMO edge list for a movement, for example ('east', 'south') -> 'E2C C2S'."""
    return f"{ARM_LETTER[approach]}2{TLS_ID} {TLS_ID}2{ARM_LETTER[dest]}"


# --- Pedestrians -----------------------------------------------------------------
# build_network.py adds sidewalks and zebra crossings. A crossing is walkable while the
# green phase does NOT use that arm (for example the north and south crossings are walkable
# during the east-west green). Layouts with a pedestrian phase (road_crossing) give walkers
# a phase of their own. Set PEDESTRIANS = False and rebuild for a vehicles-only run.
PEDESTRIANS = True
PED_RATE = 150               # pedestrians per hour crossing each arm (both directions together)
PED_BOTH_DIRECTIONS = True   # False: walkers only cross one way (a simpler routing to debug)
PED_APPROACH_POS = 250.0     # metres from the start of an incoming arm edge where walkers start
PED_EXIT_POS = 40.0          # metres from the start of an outgoing arm edge where they finish
PED_WEIGHT = 0.6             # load a waiting pedestrian adds to the phase that lets them cross

# Arms with a crossing a walker can use. Layouts with three or more arms get a crossing on every
# arm (netconvert places them); two-arm layouts use the layout's own crossing_arms.
CROSSING_ARMS = ([a.name for a in LAYOUT.arms] if len(LAYOUT.arms) >= 3
                 else LAYOUT.crossing_arm_names())
WALKABLE_ARMS = [a for a in CROSSING_ARMS if LAYOUT.arm(a).oneway == "both"]
