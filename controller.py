"""Traffic signal controllers for the Traffic Jam System prototype.

AdaptiveController : picks which phase gets green from live lane loads.
FixedTimeController: plain fixed cycle, used as a baseline for comparison.

Both share the same interface:
    status = controller.update(loads, dt)
where `loads` maps approach name -> weighted vehicle load, and `dt` is the
time step in seconds (video time, so it works for recorded footage too).

Safety rules enforced here (they cannot be skipped by the decision logic):
  * only one phase is ever non-red at a time (conflicting phases never overlap)
  * every green lasts at least MIN_GREEN
  * every green is followed by YELLOW, then an ALL_RED clearance
"""
from __future__ import annotations

import config

GREEN, YELLOW, ALL_RED = "GREEN", "YELLOW", "ALL_RED"


class _BaseController:
    def __init__(self, phases, yellow, all_red):
        self.phases = {name: list(apps) for name, apps in phases.items()}
        self.approaches = [a for apps in self.phases.values() for a in apps]
        if len(self.approaches) != len(set(self.approaches)):
            raise ValueError("An approach may belong to only one phase.")
        self.yellow = yellow
        self.all_red = all_red
        self.state = ALL_RED
        self.phase = None          # phase that is (or was last) green
        self.t_in_state = 0.0
        self.clock = 0.0
        self.last_reason = "startup"
        self.events = []

    # -- outputs ---------------------------------------------------------------
    @property
    def signals(self):
        """approach -> 'green' | 'yellow' | 'red'"""
        out = {a: "red" for a in self.approaches}
        if self.phase is not None and self.state in (GREEN, YELLOW):
            colour = "green" if self.state == GREEN else "yellow"
            for a in self.phases[self.phase]:
                out[a] = colour
        return out

    def status(self):
        return {
            "state": self.state,
            "phase": self.phase,
            "t_in_state": self.t_in_state,
            "signals": self.signals,
            "reason": self.last_reason,
        }

    def pop_events(self):
        events, self.events = self.events, []
        return events

    # -- internals ---------------------------------------------------------------
    def _enter(self, state, phase, reason=""):
        self.state, self.phase, self.t_in_state = state, phase, 0.0
        if reason:
            self.last_reason = reason
        self.events.append(
            {"time": self.clock, "state": state, "phase": phase, "reason": reason}
        )


class AdaptiveController(_BaseController):
    def __init__(
        self,
        phases=config.PHASES,
        min_green=config.MIN_GREEN,
        max_green=config.MAX_GREEN,
        yellow=config.YELLOW,
        all_red=config.ALL_RED,
        demand_threshold=config.DEMAND_THRESHOLD,
        switch_ratio=config.SWITCH_RATIO,
        wait_weight=config.WAIT_WEIGHT,
        starvation_limit=config.STARVATION_LIMIT,
    ):
        super().__init__(phases, yellow, all_red)
        self.min_green = min_green
        self.max_green = max_green
        self.demand_threshold = demand_threshold
        self.switch_ratio = switch_ratio
        self.wait_weight = wait_weight
        self.starvation_limit = starvation_limit
        self.wait = {a: 0.0 for a in self.approaches}   # seconds each approach has waited
        # Optional per-phase score adjustment supplied by the network layer (network/coordination.py):
        # a bonus for a platoon about to arrive, a penalty for a full exit road. It only changes
        # which phase is preferred; every safety rule (min green, yellow, all-red, starvation limit,
        # one phase at a time) is untouched. Empty means a stand-alone junction, as before.
        self.score_adjust = {}

    # -- main step ---------------------------------------------------------------
    def update(self, loads, dt):
        self.clock += dt
        self.t_in_state += dt
        demand = {a: loads.get(a, 0.0) >= self.demand_threshold for a in self.approaches}

        # waiting-time bookkeeping: waiting means demand present and not being served
        serving = set()
        if self.phase is not None and self.state in (GREEN, YELLOW):
            serving = set(self.phases[self.phase])
        for a in self.approaches:
            if a in serving or not demand[a]:
                self.wait[a] = 0.0
            else:
                self.wait[a] += dt

        if self.state == ALL_RED:
            if self.t_in_state >= self.all_red:
                nxt, why = self._pick_next(loads, demand)
                self._enter(GREEN, nxt, why)
        elif self.state == GREEN:
            why = self._should_end_green(loads, demand)
            if why:
                self._enter(YELLOW, self.phase, why)
        elif self.state == YELLOW:
            if self.t_in_state >= self.yellow:
                self._enter(ALL_RED, self.phase, "clearance")
        return self.status()

    # -- scoring -----------------------------------------------------------------
    def _has_demand(self, phase, demand):
        return any(demand[a] for a in self.phases[phase])

    def _score(self, phase, loads):
        apps = self.phases[phase]
        load = sum(loads.get(a, 0.0) for a in apps)
        waited = max(self.wait[a] for a in apps)
        return load + self.wait_weight * waited + self.score_adjust.get(phase, 0.0)

    # -- decisions ---------------------------------------------------------------
    def _pick_next(self, loads, demand):
        prev = self.phase
        candidates = [p for p in self.phases if p != prev and self._has_demand(p, demand)]
        if candidates:
            best = max(candidates, key=lambda p: self._score(p, loads))
            return best, f"highest demand (score {self._score(best, loads):.1f})"
        if prev is not None and self._has_demand(prev, demand):
            return prev, "only this phase has demand"
        return (prev or next(iter(self.phases))), "no demand, resting"

    def _should_end_green(self, loads, demand):
        """Return a reason string if the current green should end now, else None."""
        if self.t_in_state < self.min_green:
            return None                                   # minimum green is untouchable
        cur = self.phase
        others = [p for p in self.phases if p != cur and self._has_demand(p, demand)]
        if not others:
            return None                                   # nobody waiting: keep green

        starving = any(
            self.wait[a] >= self.starvation_limit for p in others for a in self.phases[p]
        )
        if starving:
            return "starvation limit reached"
        if self.t_in_state >= self.max_green:
            return "max green reached"
        if not self._has_demand(cur, demand):
            return "queue cleared (gap-out)"
        best_other = max(self._score(p, loads) for p in others)
        if best_other > self.switch_ratio * self._score(cur, loads):
            return "much higher demand on another phase"
        return None


class FixedTimeController(_BaseController):
    """Baseline: cycle through the phases with a constant green time."""

    def __init__(
        self,
        phases=config.PHASES,
        green=config.FIXED_GREEN,
        yellow=config.YELLOW,
        all_red=config.ALL_RED,
    ):
        super().__init__(phases, yellow, all_red)
        self.green = green
        self._order = list(self.phases)
        self._idx = -1

    def update(self, loads, dt):
        self.clock += dt
        self.t_in_state += dt
        if self.state == ALL_RED and self.t_in_state >= self.all_red:
            self._idx = (self._idx + 1) % len(self._order)
            self._enter(GREEN, self._order[self._idx], "fixed plan")
        elif self.state == GREEN and self.t_in_state >= self.green:
            self._enter(YELLOW, self.phase, "fixed plan")
        elif self.state == YELLOW and self.t_in_state >= self.yellow:
            self._enter(ALL_RED, self.phase, "clearance")
        return self.status()
