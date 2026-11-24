"""Shared settings for the Traffic Jam System prototype.

Edit this file to match your junction and tune the signal timings.
"""

# NOTE: the junction itself (arms, zones, phases, frame size) is now learned from the camera
# by autocalibrate.py and stored in site.json. FRAME_SIZE, APPROACHES and PHASES below are only
# the defaults used by the stand-alone logic test (simulate_controller.py) and by the
# controllers when no phases are passed to them.
FRAME_SIZE = (1280, 720)  # width, height

# Approaches and the signal phases (defaults only, see note above).
# Approaches in the same phase get green together; phases never overlap.
APPROACHES = ["north", "south", "east", "west"]
PHASES = {
    "NS": ["north", "south"],
    "EW": ["east", "west"],
}

# COCO class id -> weight in "passenger car equivalents" (PCU).
# Minibuses are usually detected as bus (5) or car (2); check on your footage.
VEHICLE_WEIGHTS = {
    2: 1.0,   # car
    3: 0.4,   # motorcycle
    5: 2.0,   # bus / minibus
    7: 2.5,   # truck
}

PERSON_WEIGHT = 0.6        # load of one pedestrian waiting in the crossing zone (pedestrian phase)

DETECT_CONF = 0.3          # detection confidence threshold
TARGET_FPS = 10            # frames per second actually processed
SMOOTHING_SECONDS = 1.5    # averaging window for zone loads

# --- Signal timing (seconds) -------------------------------------------------
MIN_GREEN = 8              # a green is never shorter than this
MAX_GREEN = 45             # a green is never longer than this if others wait
YELLOW = 3
ALL_RED = 2                # clearance time between phases

# --- Decision rules ------------------------------------------------------------
DEMAND_THRESHOLD = 0.5     # weighted vehicles needed in a zone to count as "demand"
SWITCH_RATIO = 1.5         # switch early if other phase score > ratio x current score
WAIT_WEIGHT = 0.1          # score = weighted queue + WAIT_WEIGHT x seconds waited
STARVATION_LIMIT = 90      # an approach with demand never waits longer than this

# Fixed-time baseline used for comparison
FIXED_GREEN = 30
