"""Test the signal logic on its own, with synthetic demand (no video, no GPU).

    python simulate_controller.py

Checks the safety rules and compares the adaptive controller with a fixed-time
plan on a scripted 10-minute scenario.
"""
import math

import config
from controller import GREEN, YELLOW, ALL_RED, AdaptiveController, FixedTimeController

DT = 0.5
DURATION = 600


def loads_at(t):
    """Scripted demand (weighted vehicles per approach) over 10 minutes."""
    wiggle = 0.3 * math.sin(t / 7.0)
    if t < 150:      # north-south heavy, east-west light
        base = {"north": 6, "south": 5, "east": 1.5, "west": 1.0}
    elif t < 300:    # east-west heavy
        base = {"north": 1.0, "south": 0.8, "east": 7, "west": 6}
    elif t < 420:    # everything busy
        base = {"north": 5, "south": 5, "east": 5, "west": 5}
    elif t < 500:    # only north has traffic
        base = {"north": 4, "south": 0, "east": 0, "west": 0}
    else:            # empty junction
        base = {"north": 0, "south": 0, "east": 0, "west": 0}
    return {a: max(0.0, v + (wiggle if v else 0.0)) for a, v in base.items()}


def run(controller):
    red_load = 0.0
    max_wait = 0.0
    changes = 0
    steps = int(DURATION / DT)
    for i in range(steps):
        t = i * DT
        loads = loads_at(t)
        status = controller.update(loads, DT)
        sig = status["signals"]

        # safety: at most one phase may be non-red at any moment
        active = {p for p, apps in controller.phases.items()
                  if any(sig[a] != "red" for a in apps)}
        assert len(active) <= 1, f"conflicting phases active at t={t}: {active}"

        for a in controller.approaches:
            if sig[a] == "red":
                red_load += loads[a] * DT
        if hasattr(controller, "wait"):
            max_wait = max(max_wait, max(controller.wait.values()))
        changes += sum(1 for e in controller.events if e["state"] == GREEN)
        controller.events_log = getattr(controller, "events_log", []) + controller.pop_events()
    return red_load, max_wait, changes, controller.events_log


def check_timings(events):
    """Every green >= MIN_GREEN, yellow == YELLOW, all-red >= ALL_RED."""
    problems = []
    for cur, nxt in zip(events, events[1:]):
        span = nxt["time"] - cur["time"]
        if cur["state"] == GREEN and span < config.MIN_GREEN - DT:
            problems.append(f"green too short ({span:.1f}s) at t={cur['time']:.1f}")
        if cur["state"] == YELLOW and abs(span - config.YELLOW) > DT:
            problems.append(f"yellow {span:.1f}s at t={cur['time']:.1f}")
        if cur["state"] == ALL_RED and span < config.ALL_RED - DT:
            problems.append(f"all-red too short ({span:.1f}s) at t={cur['time']:.1f}")
    return problems


def main():
    adaptive = AdaptiveController()
    fixed = FixedTimeController()
    a_red, a_wait, a_changes, a_events = run(adaptive)
    f_red, _, f_changes, _ = run(fixed)

    problems = check_timings(a_events)
    assert not problems, "\n".join(problems)

    print("Safety checks passed: no conflicting greens, min green / yellow / all-red respected.\n")
    print(f"{'':28}{'adaptive':>12}{'fixed-time':>12}")
    print(f"{'weighted vehicle-s on red':28}{a_red:12.0f}{f_red:12.0f}")
    print(f"{'phase changes':28}{a_changes:12d}{f_changes:12d}")
    print(f"{'longest wait (s)':28}{a_wait:12.1f}{'-':>12}")
    if f_red:
        print(f"\nAdaptive spent {100 * (1 - a_red / f_red):.0f}% less demand-weighted time on red.")
    print("\nFirst decisions:")
    for e in a_events[:12]:
        print(f"  t={e['time']:6.1f}s  {e['phase']:>3} -> {e['state']:<7} {e['reason']}")


if __name__ == "__main__":
    main()
