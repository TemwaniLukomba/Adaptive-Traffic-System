"""Baseline signal controllers to compare against the adaptive controller.

The fixed-time baseline lives in the project's controller.py (FixedTimeController).
This file adds one more, a simple demand-responsive rule.

LongestQueueController
    After the minimum green, switch to whichever phase has the larger load.
    Also switches at the maximum green if the other phase has any demand.
    It is a simplified cousin of max-pressure control: real max-pressure also
    subtracts the queues on the exit roads, which does not matter at a single
    isolated junction. It has no waiting-time or fairness logic, so the
    adaptive controller's extra rules have something to beat.

It has the same interface as the controllers in controller.py:
    controller.update(loads, dt)  and  controller.signals
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config                                                    # noqa: E402
from controller import ALL_RED, GREEN, YELLOW, _BaseController   # noqa: E402


class LongestQueueController(_BaseController):
    def __init__(self, phases, min_green=config.MIN_GREEN, max_green=config.MAX_GREEN,
                 yellow=config.YELLOW, all_red=config.ALL_RED):
        super().__init__(phases, yellow, all_red)
        self.min_green = min_green
        self.max_green = max_green

    def _scores(self, loads):
        return {p: sum(loads.get(a, 0.0) for a in apps) for p, apps in self.phases.items()}

    def update(self, loads, dt):
        self.clock += dt
        self.t_in_state += dt
        scores = self._scores(loads)

        if self.state == ALL_RED:
            if self.t_in_state >= self.all_red:
                best = max(scores, key=scores.get)
                self._enter(GREEN, best, "longest queue")
        elif self.state == GREEN:
            if self.t_in_state >= self.min_green:
                others = {p: s for p, s in scores.items() if p != self.phase}
                best_other = max(others, key=others.get)
                if scores[best_other] > scores[self.phase]:
                    self._enter(YELLOW, self.phase, "other phase has the longer queue")
                elif self.t_in_state >= self.max_green and scores[best_other] > 0:
                    self._enter(YELLOW, self.phase, "max green reached")
        elif self.state == YELLOW:
            if self.t_in_state >= self.yellow:
                self._enter(ALL_RED, self.phase, "clearance")
        return self.status()
