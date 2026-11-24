"""Road layouts: describe any junction by its arms, and everything else is derived.

A road is a list of ARMS around a signalised centre. Each arm has a compass bearing as seen
from the centre (0 = north / top of the camera image, 90 = east, 180 = south, 270 = west),
a number of lanes, and whether traffic can enter it, leave it, or both.

From that, this module works out by geometry:
  * every possible movement (straight / left / right; U-turns are ignored),
  * which pairs of movements conflict, and which are only "give way" conflicts
    (for example a right turn crossing oncoming traffic when driving on the left),
  * the signal phases: the smallest set of groups of approaches that can be green together.

Nothing here knows about video or SUMO, so the same layout drives the camera prototype
(autocalibrate.py, main.py) and the simulation (sim/).

    python layout.py                      # list the presets
    python layout.py t_junction           # arms, movements and phases of a preset
    python layout.py my_site.json         # the same for a calibrated site

Presets: crossroads, t_junction, y_junction, five_way, shuttle (one-lane road works with
alternating flow), road_crossing (two arms plus a pedestrian phase), one_way_cross
(a one-way street meeting a two-way street).
Roundabouts are not covered: they are not controlled by a single signal like these.
"""
import itertools
import json
import sys
from dataclasses import dataclass, field, asdict
from pathlib import Path

DELTA = 8.0            # degrees: entry / exit lanes sit this far either side of the arm axis
STRAIGHT_LIMIT = 35.0  # turn angles within this count as "straight"
UTURN_LIMIT = 155.0    # turn angles beyond this are U-turns (not modelled)

_COMPASS = ["north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest"]
_ABBREV = {"north": "N", "northeast": "NE", "east": "E", "southeast": "SE",
           "south": "S", "southwest": "SW", "west": "W", "northwest": "NW"}


def compass_name(bearing):
    return _COMPASS[int(((bearing % 360) + 22.5) // 45) % 8]


def angle_diff(a, b):
    """Signed difference a - b in degrees, wrapped to (-180, 180]."""
    d = (a - b + 540.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


@dataclass
class Arm:
    name: str
    bearing: float
    lanes: int = 2
    speed_kmh: float = 50.0
    oneway: str = "both"       # "both", "in" (traffic only enters) or "out" (traffic only leaves)
    length: float = 300.0      # metres, used by the simulation


@dataclass
class Movement:
    approach: str
    dest: str
    kind: str                  # "straight", "left" or "right"
    angle: float               # signed turn angle, positive = clockwise (right)


@dataclass
class Layout:
    name: str
    arms: list
    drive_on_left: bool = True
    alternating: bool = False          # every approach gets its own phase (one-lane road works)
    pedestrian_phase: bool = False     # add a phase in which every vehicle approach is red
    crossing_arms: list = None         # arms with a zebra crossing (None = all)
    meta: dict = field(default_factory=dict)

    # ---- arms --------------------------------------------------------------------
    def arm(self, name):
        for a in self.arms:
            if a.name == name:
                return a
        raise KeyError(name)

    @property
    def approaches(self):
        """Arms that traffic enters from, in bearing order starting at north."""
        return [a.name for a in sorted(self.arms, key=lambda a: a.bearing) if a.oneway in ("both", "in")]

    @property
    def exits(self):
        return [a.name for a in self.arms if a.oneway in ("both", "out")]

    def codes(self):
        """Short unique letter codes for arms, used in SUMO edge ids (never digits or 'C')."""
        by_name = {a.name: _ABBREV.get(a.name) for a in self.arms}
        if all(by_name.values()) and len(set(by_name.values())) == len(by_name):
            return by_name
        pool = [c for c in "ABDEFGHIJKLMNOPQRSTUVWXYZ"]
        return {a.name: pool[i] for i, a in enumerate(self.arms)}

    # ---- movements ---------------------------------------------------------------
    def turn_angle(self, a, b):
        """How far a driver entering from arm a turns to leave by arm b (clockwise = right)."""
        heading_in = (self.arm(a).bearing + 180.0) % 360.0
        return angle_diff(self.arm(b).bearing, heading_in)

    def kind(self, a, b):
        ang = self.turn_angle(a, b)
        if abs(ang) <= STRAIGHT_LIMIT:
            return "straight"
        if abs(ang) >= UTURN_LIMIT:
            return "uturn"
        return "right" if ang > 0 else "left"

    def movements(self):
        out = []
        for a in self.approaches:
            for b in self.exits:
                if a == b:
                    continue
                k = self.kind(a, b)
                if k != "uturn":
                    out.append(Movement(a, b, k, self.turn_angle(a, b)))
        return out

    @property
    def crossing_turn(self):
        """The turn that crosses oncoming traffic: right when driving on the left."""
        return "right" if self.drive_on_left else "left"

    # ---- geometry of conflicts -----------------------------------------------------
    def _entry_pos(self, name):
        b = self.arm(name).bearing
        return (b + DELTA) % 360 if self.drive_on_left else (b - DELTA) % 360

    def _exit_pos(self, name):
        b = self.arm(name).bearing
        return (b - DELTA) % 360 if self.drive_on_left else (b + DELTA) % 360

    @staticmethod
    def _between(x, a, b):
        return 0 < (x - a) % 360 < (b - a) % 360

    def relation(self, m1, m2):
        """None (no conflict), ("yield", movement_that_gives_way) or "hard"."""
        if m1.approach == m2.approach:
            return None                                   # they diverge from the same lane
        if m1.dest == m2.dest:                            # they merge onto the same exit
            if abs(m1.angle) == abs(m2.angle):
                return None
            return ("yield", m1 if abs(m1.angle) > abs(m2.angle) else m2)
        a, b = self._entry_pos(m1.approach), self._exit_pos(m1.dest)
        c, d = self._entry_pos(m2.approach), self._exit_pos(m2.dest)
        if self._between(c, a, b) == self._between(d, a, b):
            return None                                   # the paths do not cross
        for turner, other in ((m1, m2), (m2, m1)):        # a crossing turn against oncoming straight
            opposing = abs(abs(angle_diff(self.arm(turner.approach).bearing,
                                          self.arm(other.approach).bearing)) - 180.0) <= STRAIGHT_LIMIT
            if turner.kind == self.crossing_turn and other.kind == "straight" and opposing:
                return ("yield", turner)
        return "hard"

    def _approach_compatible(self, a, b, moves):
        for m1 in (m for m in moves if m.approach == a):
            for m2 in (m for m in moves if m.approach == b):
                if self.relation(m1, m2) == "hard":
                    return False
        return True

    # ---- phases ----------------------------------------------------------------------
    def phase_groups(self):
        """Smallest partition of the approaches into groups that may be green together."""
        apps = self.approaches
        if self.alternating:
            return [[a] for a in apps]
        moves = self.movements()
        ok = {(a, b): self._approach_compatible(a, b, moves) for a in apps for b in apps if a != b}
        best, best_key = None, None
        for parts in _set_partitions(apps):
            if not all(ok[(a, b)] for p in parts for a, b in itertools.combinations(p, 2)):
                continue
            key = (len(parts), -sum(len(p) ** 2 for p in parts))
            if best_key is None or key < best_key:
                best, best_key = parts, key
        groups = [sorted(p, key=lambda n: self.arm(n).bearing) for p in best]
        return sorted(groups, key=lambda g: min(self.arm(n).bearing for n in g))

    def phases(self):
        """{phase name: [approach names]}; names are the arm codes joined, e.g. 'NS', 'EW'."""
        codes = self.codes()
        out = {}
        for g in self.phase_groups():
            name = "".join(codes[n] for n in g)
            while name in out:
                name += "'"
            out[name] = g
        if self.pedestrian_phase:
            out["PED"] = ["ped"]
        return out

    def controller_approaches(self):
        extra = ["ped"] if self.pedestrian_phase else []
        return self.approaches + extra

    def yielding_movements(self):
        """Movements that must give way while their phase is green: {(approach, dest)}."""
        yields = set()
        moves = self.movements()
        for group in self.phase_groups():
            ms = [m for m in moves if m.approach in group]
            for m1, m2 in itertools.combinations(ms, 2):
                rel = self.relation(m1, m2)
                if isinstance(rel, tuple):
                    yields.add((rel[1].approach, rel[1].dest))
        return yields

    def crossing_arm_names(self):
        if self.crossing_arms is not None:
            return [a for a in self.crossing_arms if a in {x.name for x in self.arms}]
        return [a.name for a in self.arms]

    # ---- turn shares for demand ---------------------------------------------------------
    def movement_shares(self, approach, turn_share=None):
        """{dest: share} for one approach; shares of missing turn types are redistributed."""
        turn_share = turn_share or {"straight": 0.70, "left": 0.15, "right": 0.15}
        ms = [m for m in self.movements() if m.approach == approach]
        count = {}
        for m in ms:
            count[m.kind] = count.get(m.kind, 0) + 1
        total = sum(turn_share[k] for k in count)
        return {m.dest: turn_share[m.kind] / count[m.kind] / total for m in ms}

    # ---- saving ---------------------------------------------------------------------------
    def to_dict(self):
        d = asdict(self)
        return d

    @staticmethod
    def from_dict(d):
        d = dict(d)
        d["arms"] = [Arm(**a) for a in d["arms"]]
        return Layout(**d)

    def save(self, path):
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    def describe(self):
        side = "left" if self.drive_on_left else "right"
        lines = [f"{self.name}: {len(self.arms)} arms, driving on the {side}"]
        for a in sorted(self.arms, key=lambda a: a.bearing):
            lines.append(f"  arm {a.name:<10} bearing {a.bearing:5.0f}  {a.lanes} lane(s)  "
                         f"{a.oneway}  code {self.codes()[a.name]}")
        lines.append("  movements:")
        for a in self.approaches:
            parts = [f"{m.kind} -> {m.dest}" for m in self.movements() if m.approach == a]
            lines.append(f"    from {a:<10} " + ", ".join(parts))
        lines.append("  phases (approaches that are green together):")
        for name, apps in self.phases().items():
            lines.append(f"    {name:<6} {', '.join(apps)}")
        y = sorted(self.yielding_movements())
        if y:
            lines.append("  give-way movements while green: " + ", ".join(f"{a}->{d}" for a, d in y))
        return "\n".join(lines)


def _set_partitions(items):
    """All ways to split a list into non-empty groups."""
    items = list(items)
    if not items:
        yield []
        return
    first, rest = items[0], items[1:]
    for parts in _set_partitions(rest):
        yield [[first]] + parts
        for i in range(len(parts)):
            yield parts[:i] + [[first] + parts[i]] + parts[i + 1:]


# ---- presets ---------------------------------------------------------------------------------

def _named(bearings, lanes=2, **kw):
    arms, used = [], {}
    for b in bearings:
        base = compass_name(b)
        used[base] = used.get(base, 0) + 1
        name = base if used[base] == 1 else f"{base}{used[base]}"
        arms.append(Arm(name, float(b) % 360, lanes=lanes, **kw))
    return arms


def preset(kind, lanes=2, drive_on_left=True, rotation=0.0):
    """A ready-made layout. rotation turns the whole junction (degrees, clockwise)."""
    rot = lambda bs: [(b + rotation) % 360 for b in bs]
    if kind == "crossroads":
        return Layout(kind, _named(rot([0, 90, 180, 270]), lanes), drive_on_left)
    if kind == "t_junction":                       # main road east-west, side road to the south
        return Layout(kind, _named(rot([90, 270, 180]), lanes), drive_on_left)
    if kind == "y_junction":
        return Layout(kind, _named(rot([0, 120, 240]), lanes), drive_on_left)
    if kind == "five_way":
        return Layout(kind, _named(rot([0, 72, 144, 216, 288]), lanes), drive_on_left)
    if kind == "shuttle":                          # one lane, two directions, take turns
        return Layout(kind, _named(rot([0, 180]), 1), drive_on_left, alternating=True)
    if kind == "road_crossing":                    # a two-way road with a pedestrian crossing
        lay = Layout(kind, _named(rot([0, 180]), min(lanes, 2)), drive_on_left, pedestrian_phase=True)
        lay.crossing_arms = [lay.arms[0].name]
        return lay
    if kind == "one_way_cross":                    # one-way street (south to north) meets a two-way street
        arms = _named(rot([0, 90, 180, 270]), lanes)
        for a in arms:
            if a.bearing % 360 == (180 + rotation) % 360:
                a.oneway = "in"
            elif a.bearing % 360 == rotation % 360:
                a.oneway = "out"
        return Layout(kind, arms, drive_on_left)
    raise ValueError(f"Unknown layout '{kind}'. Presets: {', '.join(PRESETS)}")


PRESETS = ["crossroads", "t_junction", "y_junction", "five_way", "shuttle",
           "road_crossing", "one_way_cross"]


def resolve(spec, **kw):
    """A preset name, a layout .json, a site.json made by autocalibrate.py, or a Layout."""
    if isinstance(spec, Layout):
        return spec
    path = Path(str(spec))
    if path.suffix == ".json" and path.exists():
        data = json.loads(path.read_text())
        return Layout.from_dict(data["layout"] if "layout" in data else data)
    return preset(str(spec), **kw)


def main():
    if len(sys.argv) < 2:
        print("Presets:", ", ".join(PRESETS))
        print("Show one:  python layout.py t_junction")
        return
    print(resolve(sys.argv[1]).describe())


if __name__ == "__main__":
    main()
