"""A simplified multi-node traffic simulator, used to test the network layer without SUMO.

IMPORTANT: this is NOT SUMO. It is a queue-and-link model written to check the coordination logic
quickly: do messages arrive, do the scores move the way the design says, does coordination help or
hurt when a road fills up. Numbers from it show how the logic behaves in this model. They are not
evidence about real roads, and they should not go into the report as simulation results. For those,
export the network to SUMO and run it there (see README.md, "What is next").

The model, one step per second:
  * Vehicles enter at the network's edge arms at Poisson rates and follow a shortest route through
    the nodes (so the road types can be mixed freely: any layout can be a node).
  * Each approach is a first-in-first-out queue. A queue discharges while its signal is green at
    0.5 passenger-car equivalents per second per lane, half that for a movement that must give way.
  * A discharged vehicle travels the link at free-flow speed, then joins the next node's queue.
  * A link holds only length x lanes / 7.5 m passenger-car equivalents. When it is full the vehicle
    at the head of the queue is blocked, so the queue stays and blocks the vehicles behind it. This
    is spill-back, the thing neighbour information is meant to prevent.
  * Each node's controller sees only what a camera would: the load inside a 60 m zone at each
    stop line (not the whole queue).
  * Pedestrian-phase nodes get pedestrians who wait for the walk phase.
"""
import random
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import config                                                       # noqa: E402
from controller import AdaptiveController, FixedTimeController      # noqa: E402
from coordination import Bus, CoordParams, NodeCoordinator          # noqa: E402
from graph import JAM_SPACING_M                                     # noqa: E402

SAT_FLOW = 0.5            # PCE per second per lane while green
ZONE_M = 60.0             # the camera zone at each stop line
PED_DISCHARGE = 0.5       # pedestrians per second while the walk phase is on
VEHICLE_MIX = ((0.65, 2), (0.18, 5), (0.10, 3), (0.07, 7))      # share, COCO class (car, bus, moto, truck)
MODES = ("fixed", "isolated", "coordinated")


class Veh:
    __slots__ = ("pce", "route", "i", "t0", "qt", "qe")

    def __init__(self, pce, route, t0):
        self.pce, self.route, self.i, self.t0, self.qt, self.qe = pce, route, 0, t0, 0.0, t0


def build_demand(net, entry_rate=400.0, through_boost=1.0):
    """{ (entry, exit): veh/h } with every network-edge arm emitting entry_rate veh/h.

    Entries that can reach an exit 3 or more nodes away are the corridor's through traffic and
    are multiplied by through_boost (1.0 = no extra). Longer trips are weighted higher."""
    demand = {}
    for entry in net.boundary_entries():
        routes = {}
        for ex in net.boundary_exits():
            r = net.route(entry, ex) if ex[0] != entry[0] or ex[1] != entry[1] else None
            if r and not (entry[0] == ex[0] and entry[1] == ex[1]):
                routes[ex] = len(r)
        if not routes:
            continue
        boost = through_boost if max(routes.values()) >= 3 else 1.0
        weights = {ex: 1.0 + n for ex, n in routes.items()}
        total = sum(weights.values())
        for ex, w in weights.items():
            demand[(entry, ex)] = entry_rate * boost * w / total
    return demand


class NetSim:
    def __init__(self, net, mode, demand, seed=1, duration=1800.0, params=None, predictor=None,
                 link_weight=None, msg_loss=0.0, msg_latency=1.0, ped_rate=120.0, record=False,
                 apply_coordination=True, dt=1.0):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        self.net, self.mode, self.dt, self.duration = net, mode, dt, duration
        self.rng = random.Random(seed)
        self.t = 0.0
        self.ped_rate = ped_rate
        self.demand = {k: v for k, v in demand.items() if v > 0}
        self.routes = {k: net.route(*k) for k in self.demand}
        self.layouts = {n: node.layout for n, node in net.nodes.items()}
        self.q = {n: {a: deque() for a in lay.approaches} for n, lay in self.layouts.items()}
        self.qp = {n: {a: 0.0 for a in lay.approaches} for n, lay in self.layouts.items()}
        self.credit = {n: {a: 0.0 for a in lay.approaches} for n, lay in self.layouts.items()}
        self.lanes = {n: {a.name: a.lanes for a in lay.arms} for n, lay in self.layouts.items()}
        self.transit = {lid: deque() for lid in net.links}
        self.tp = {lid: 0.0 for lid in net.links}
        self.arrivals = {lid: [] for lid in net.links}
        self.yields = {n: lay.yielding_movements() for n, lay in self.layouts.items()}
        self.shares = {n: {(a, b): s for a in lay.approaches
                           for b, s in lay.movement_shares(a).items()}
                       for n, lay in self.layouts.items()}
        self.ped_q = {n: deque() for n, lay in self.layouts.items() if lay.pedestrian_phase}
        self.ped_wait = 0.0
        self.peds_total = 0
        make = FixedTimeController if mode == "fixed" else AdaptiveController
        self.ctrl = {n: make(phases=lay.phases()) for n, lay in self.layouts.items()}
        self.bus = Bus(msg_latency, msg_loss, self.rng)
        self.coord = {}
        if mode == "coordinated":
            for n in net.nodes:
                self.coord[n] = NodeCoordinator(n, net, self.bus, params, predictor, link_weight,
                                                apply=apply_coordination, record=record)
        # results
        self.spawned = []
        self.finished = []
        self.blocked_steps = 0
        self.node_steps = 0
        self.max_fill = 0.0
        self.fill_sum = 0.0
        self.fill_n = 0

    # ---- helpers -----------------------------------------------------------------------------
    def zone_cap(self, n, arm):
        return ZONE_M / JAM_SPACING_M * self.lanes[n][arm]

    def link_occupancy(self, link):
        return self.tp[link.id] + (self.qp[link.dst][link.dst_arm])

    def _sample_pce(self):
        r, acc = self.rng.random(), 0.0
        for share, coco in VEHICLE_MIX:
            acc += share
            if r <= acc:
                return config.VEHICLE_WEIGHTS[coco]
        return 1.0

    def _enqueue(self, node, arm, veh):
        veh.qe = self.t
        self.q[node][arm].append(veh)
        self.qp[node][arm] += veh.pce

    # ---- one second ----------------------------------------------------------------------------
    def step(self):
        t, dt = self.t, self.dt
        # 1) arrivals from outside the network
        for key, rate in self.demand.items():
            route = self.routes[key]
            if route and self.rng.random() < rate / 3600.0 * dt:
                veh = Veh(self._sample_pce(), route, t)
                self.spawned.append(veh)
                self._enqueue(route[0][0], route[0][1], veh)
        # 2) vehicles that reached the end of a link
        for lid, dq in self.transit.items():
            link = self.net.links[lid]
            while dq and dq[0][0] <= t:
                _, veh = dq.popleft()
                self.tp[lid] -= veh.pce
                self.arrivals[lid].append((t, veh.pce))
                self._enqueue(link.dst, link.dst_arm, veh)
        # 3) pedestrians
        for n, dq in self.ped_q.items():
            if self.rng.random() < self.ped_rate / 3600.0 * dt:
                dq.append(t)
                self.peds_total += 1
            self.ped_wait += len(dq) * dt
        # 4) each node's controller decides
        for msg in self.bus.deliver(t):
            if msg.dst in self.coord:
                self.coord[msg.dst].receive(msg)
        for n in self.net.nodes:
            lay = self.layouts[n]
            seen = {a: min(self.qp[n][a], self.zone_cap(n, a)) for a in lay.approaches}
            loads = dict(seen)
            if n in self.ped_q:
                loads["ped"] = config.PERSON_WEIGHT * len(self.ped_q[n])
            if n in self.coord:
                co = self.coord[n]
                co.send_updates(t, self._sensors(n, seen))
                en_route = {l.src_arm: self.tp[l.id] for l in self.net.links.values() if l.src == n}
                self.ctrl[n].score_adjust = co.adjustments(t, seen, en_route, self.shares[n])
            self.ctrl[n].update(loads, dt)
        # 5) discharge
        for n in self.net.nodes:
            self._discharge(n)
        self.t += dt

    def _sensors(self, n, seen):
        sig = self.ctrl[n].signals
        lay = self.layouts[n]
        sensors = {"queue": {}, "zone_cap": {}, "eta": {}, "green_to": {}, "queued_for": {}}
        for arm in lay.arms:
            if arm.name in seen:
                sensors["queue"][arm.name] = seen[arm.name]
                sensors["zone_cap"][arm.name] = self.zone_cap(n, arm.name)
        for link in self.net.links.values():
            if link.src != n:
                continue
            buckets = [0.0, 0.0, 0.0]
            for ta, veh in self.transit[link.id]:
                eta = ta - self.t
                for i, (lo, hi) in enumerate((b for b in ((0, 10), (10, 20), (20, 40)))):
                    if lo <= eta < hi:
                        buckets[i] += veh.pce
            sensors["eta"][link.id] = buckets
            sensors["green_to"][link.id] = any(
                sig.get(a) == "green" and self.shares[n].get((a, link.src_arm), 0) > 0
                for a in lay.approaches)
            sensors["queued_for"][link.id] = sum(
                seen[a] * self.shares[n].get((a, link.src_arm), 0.0) for a in lay.approaches)
        return sensors

    def _discharge(self, n):
        lay, sig = self.layouts[n], self.ctrl[n].signals
        any_blocked = False
        for a in lay.approaches:
            dq = self.q[n][a]
            if sig.get(a) != "green" or not dq:
                self.credit[n][a] = 0.0 if sig.get(a) != "green" or not dq else self.credit[n][a]
                continue
            self.credit[n][a] = min(self.credit[n][a] + SAT_FLOW * self.lanes[n][a] * self.dt, 6.0)
            while dq:
                veh = dq[0]
                _, _, out_arm = veh.route[veh.i]
                need = veh.pce * (2.0 if (a, out_arm) in self.yields[n] else 1.0)
                if self.credit[n][a] < need:
                    break
                link = self.net.out_link.get((n, out_arm))
                if link is not None and self.link_occupancy(link) + veh.pce > link.capacity:
                    any_blocked = True                       # spill-back: the exit road is full
                    self.credit[n][a] = min(self.credit[n][a], 1.0)
                    break
                dq.popleft()
                self.qp[n][a] -= veh.pce
                self.credit[n][a] -= need
                veh.qt += self.t - veh.qe
                veh.i += 1
                if link is None:
                    self.finished.append((veh, self.t))
                else:
                    self.transit[link.id].append((self.t + link.free_time, veh))
                    self.tp[link.id] += veh.pce
        if n in self.ped_q and sig.get("ped") == "green":
            budget = PED_DISCHARGE * self.dt
            dq = self.ped_q[n]
            self.credit[n].setdefault("ped", 0.0)
            self.credit[n]["ped"] += budget
            while dq and self.credit[n]["ped"] >= 1.0:
                dq.popleft()
                self.credit[n]["ped"] -= 1.0
        self.node_steps += 1
        self.blocked_steps += 1 if any_blocked else 0

    # ---- running and results -----------------------------------------------------------------
    def run(self):
        while self.t < self.duration:
            self.step()
            if int(self.t) % 10 == 0:
                for link in self.net.links.values():
                    fill = self.link_occupancy(link) / link.capacity
                    self.max_fill = max(self.max_fill, fill)
                    self.fill_sum += fill
                    self.fill_n += 1
        return self.results()

    def results(self):
        done = {id(v) for v, _ in self.finished}
        delays = [v.qt for v, _ in self.finished]
        for v in self.spawned:                                # vehicles still inside count too
            if id(v) not in done:
                delays.append(v.qt + (self.t - v.qe if self.q_has(v) else 0.0))
        travel = [t - v.t0 for v, t in self.finished]
        ordered = sorted(delays)
        n = len(ordered)
        mean = lambda xs: sum(xs) / len(xs) if xs else 0.0
        return {
            "mode": self.mode,
            "spawned": len(self.spawned),
            "completed": len(self.finished),
            "unfinished": len(self.spawned) - len(self.finished),
            "mean_delay_s": round(mean(delays), 2),
            "p95_delay_s": round(ordered[int(0.95 * (n - 1))], 2) if n else 0.0,
            "mean_travel_s": round(mean(travel), 2),
            "throughput_vph": round(len(self.finished) / (self.t / 3600.0), 1),
            "blocked_share": round(self.blocked_steps / max(1, self.node_steps), 4),
            "max_link_fill": round(self.max_fill, 2),
            "mean_link_fill": round(self.fill_sum / max(1, self.fill_n), 3),
            "ped_mean_wait_s": round(self.ped_wait / self.peds_total, 2) if self.peds_total else None,
            "messages_sent": self.bus.sent,
            "messages_dropped": self.bus.dropped,
        }

    def q_has(self, veh):
        n, a, _ = veh.route[min(veh.i, len(veh.route) - 1)]
        return veh in self.q[n][a]

    def training_rows(self, horizon=20.0):
        """(features, PCE that actually arrived in the next `horizon` s) for every recorded sample."""
        X, y = [], []
        for co in self.coord.values():
            for t, lid, f in co.samples:
                if t + horizon > self.t:
                    continue
                got = sum(p for ta, p in self.arrivals[lid] if t < ta <= t + horizon)
                X.append(f)
                y.append(got)
        return X, y
