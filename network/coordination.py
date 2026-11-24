"""Neighbour-to-neighbour coordination between signalised nodes.

Every node keeps its own camera, its own controller and its own safety state machine. On top of
that, each node tells its neighbours a few numbers every couple of seconds and listens to theirs.
What it hears only changes how much each of ITS OWN phases is worth (controller.score_adjust).
It can never touch yellow, all-red, minimum green, the starvation limit or the one-phase-at-a-time
rule, so a lost, late or wrong message cannot cause a conflicting green.

Because every node talks to its neighbours, information spreads along the whole network: a queue
that builds at one node changes the scores next door, which changes what arrives at the node after
that, and so on until all nodes are connected. No central computer is needed, and a node that hears
nothing for ttl seconds simply behaves like a stand-alone junction.

What a node tells the neighbour at the other end of a link (a Message):
    queue             vehicles waiting at its own stop line on the arm that faces the neighbour
                      (what its camera sees, in passenger-car equivalents)
    zone_full         True when that queue fills the camera zone, so the real queue is longer
    eta               PCE it has released toward the neighbour that will arrive in 0-10 s, 10-20 s
                      and 20-40 s (it knows when each vehicle left and how long the road takes)
    queued_for_link   PCE waiting at its stop lines that will probably leave toward the neighbour
                      (estimated from the layout's turn shares, because a camera cannot read a
                      driver's intention)
    green_to_link     whether its signal is currently green for traffic heading that way

Two effects on a node's phase scores:
    platoon bonus     a phase is worth more when a group of vehicles is about to arrive on its
                      approaches, so a green is kept or started for them (a green wave)
    spill-back        a phase is worth less when the road it sends traffic onto is nearly full, so
      penalty         vehicles are not released into a jam that would block the junction behind them
The strength of each effect can be scaled per link, for example from Google's congestion data
(../gmaps/priorities.py): congested links are coordinated harder.
"""
import json
from dataclasses import asdict, dataclass, field
from heapq import heappop, heappush

from graph import Network

ETA_BUCKETS = ((0.0, 10.0), (10.0, 20.0), (20.0, 40.0))
FEATURES = ["eta_0_10", "eta_10_20", "eta_20_40", "queued_for_link", "green_to_link",
            "queued_x_green", "my_queue"]


@dataclass
class Message:
    src: str
    dst: str
    src_arm: str               # arm of the sender that faces the receiver
    dst_arm: str               # arm of the receiver that faces the sender
    t: float
    queue: float = 0.0
    zone_full: bool = False
    eta: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    queued_for_link: float = 0.0
    green_to_link: bool = False

    def to_json(self):
        return json.dumps(asdict(self))

    @staticmethod
    def from_json(text):
        return Message(**json.loads(text))


class Bus:
    """In-memory message delivery with delay and loss. A real deployment would replace this with
    MQTT or UDP over the city's network; the Message fields are plain JSON for that reason."""

    def __init__(self, latency=1.0, loss=0.0, rng=None):
        self.latency, self.loss, self.rng = latency, loss, rng
        self._queue = []
        self._n = 0
        self.sent = self.dropped = 0

    def send(self, msg, now):
        self.sent += 1
        if self.loss and self.rng and self.rng.random() < self.loss:
            self.dropped += 1
            return
        self._n += 1
        heappush(self._queue, (now + self.latency, self._n, msg))

    def deliver(self, now):
        out = []
        while self._queue and self._queue[0][0] <= now:
            out.append(heappop(self._queue)[2])
        return out


@dataclass
class CoordParams:
    period: float = 2.0         # seconds between messages
    ttl: float = 15.0           # a message older than this is ignored
    platoon_weight: float = 0.5
    spill_weight: float = 1.5
    fill_start: float = 0.6     # road this full starts to count as blocked
    max_penalty: float = 0.8    # penalty never removes more than this share of a phase's own load
    full_fill: float = 0.9      # fill assumed when the downstream camera zone is full
    arrival_horizon: float = 20.0


def features(msg, my_queue):
    e = msg.eta
    g = 1.0 if msg.green_to_link else 0.0
    return [e[0], e[1], e[2], msg.queued_for_link, g, msg.queued_for_link * g, my_queue]


def heuristic_arrivals(f):
    """Expected PCE arriving in the next 20 s with no learning: what is already within 20 s of
    arriving, plus half the queue that is being released toward us now."""
    return f[0] + f[1] + 0.5 * f[5]


class NodeCoordinator:
    def __init__(self, node_id, net, bus, params=None, predictor=None, link_weight=None,
                 apply=True, record=False):
        self.id, self.net, self.bus = node_id, net, bus
        self.p = params or CoordParams()
        self.predictor = predictor
        self.link_weight = link_weight or (lambda link_id: 1.0)
        self.apply = apply
        self.record = record
        self.samples = []            # (time, link id, feature list) for training the predictor
        self.inbox = {}              # (sender, my arm facing it) -> latest Message
        self.layout = net.nodes[node_id].layout
        self._last_send = -1e9
        self.last = {"bonus": {}, "penalty": {}}

    # ---- sending ---------------------------------------------------------------------------
    def send_updates(self, now, sensors):
        """sensors: {'queue': {arm: PCE seen}, 'zone_cap': {arm: PCE}, 'eta': {link id: [3 floats]},
        'green_to': {link id: bool}, 'queued_for': {link id: PCE}}"""
        if now - self._last_send < self.p.period:
            return
        self._last_send = now
        for arm in self.layout.arms:
            out = self.net.out_link.get((self.id, arm.name))
            inc = self.net.in_link.get((self.id, arm.name))
            link = out or inc
            if link is None:
                continue
            nbr, nbr_arm = (link.dst, link.dst_arm) if out else (link.src, link.src_arm)
            q = sensors["queue"].get(arm.name, 0.0)
            cap = sensors["zone_cap"].get(arm.name, 1e9)
            self.bus.send(Message(
                src=self.id, dst=nbr, src_arm=arm.name, dst_arm=nbr_arm, t=now,
                queue=q, zone_full=q >= 0.9 * cap,
                eta=list(sensors["eta"].get(out.id, [0, 0, 0])) if out else [0, 0, 0],
                queued_for_link=sensors["queued_for"].get(out.id, 0.0) if out else 0.0,
                green_to_link=bool(sensors["green_to"].get(out.id, False)) if out else False), now)

    def receive(self, msg):
        self.inbox[(msg.src, msg.dst_arm)] = msg

    # ---- using what was heard ----------------------------------------------------------------
    def _fresh(self, key, now):
        m = self.inbox.get(key)
        return m if m is not None and now - m.t <= self.p.ttl else None

    def adjustments(self, now, queue, en_route, shares):
        """Phase score adjustments for this node.

        queue: {approach arm: PCE queued}; en_route: {exit arm: PCE this node has released that
        has not yet reached the neighbour}; shares: {(approach, exit arm): turn share}."""
        phases = self.layout.phases()
        bonus = {p: 0.0 for p in phases}
        penalty = {p: 0.0 for p in phases}
        raw = {p: sum(queue.get(a, 0.0) for a in apps) for p, apps in phases.items()}

        for pname, apps in phases.items():
            for a in apps:
                link = self.net.in_link.get((self.id, a))
                if link is not None:                                   # traffic is coming to a
                    m = self._fresh((link.src, a), now)
                    if m is not None:
                        f = features(m, queue.get(a, 0.0))
                        if self.record:
                            self.samples.append((now, link.id, f))
                        est = (self.predictor.predict(f) if self.predictor else heuristic_arrivals(f))
                        bonus[pname] += self.p.platoon_weight * self.link_weight(link.id) * max(0.0, est)
                for dest_arm in self.layout.exits:                     # a sends traffic to dest_arm
                    share = shares.get((a, dest_arm), 0.0)
                    out = self.net.out_link.get((self.id, dest_arm))
                    if not share or out is None:
                        continue
                    m = self._fresh((out.dst, out.src_arm), now)
                    if m is None:
                        continue
                    fill = ((self.p.full_fill if m.zone_full else m.queue / out.capacity)
                            + en_route.get(dest_arm, 0.0) / out.capacity)
                    excess = min(1.0, max(0.0, (fill - self.p.fill_start) / (1.0 - self.p.fill_start)))
                    penalty[pname] += (self.p.spill_weight * self.link_weight(out.id)
                                       * queue.get(a, 0.0) * share * excess)
        adjust = {}
        for p in phases:
            pen = min(penalty[p], self.p.max_penalty * raw[p])
            adjust[p] = bonus[p] - pen
            penalty[p] = pen
        self.last = {"bonus": bonus, "penalty": penalty}
        return adjust if self.apply else {p: 0.0 for p in phases}
