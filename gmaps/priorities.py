"""Turn Google travel-time data into coordination priorities. Offline use only.

Google's Routes API reports, for a route, the travel time with traffic and the travel time with
no traffic. Their ratio is the congestion ratio: 1.0 is free flowing, 2.0 means the trip takes
twice as long. This module reads a saved profile of those ratios (fetch_profile.py) and does two
things:

  1. rank_links(): which links are the most congested, and when, so you know which junctions to
     connect first.
  2. link_weight_fn(): a multiplier for each link and hour that makes the neighbour-aware controller
     coordinate harder on congested links (weight 1.0 when free flowing, up to 2.5).

It is used to set up and tune the system. The live controller does not call Google: the data
update in minutes, the controller decides every second, a connection may be down when the power is
cut, and Google's terms limit how its data may be stored and used (check the current terms before
publishing results). Only the small summary profile.json is kept, not raw Google responses.

profile.json:
    {"source": "google_routes" | "SYNTHETIC", "asof": "2026-11-17",
     "links": {"A.east>B.west": {"ratio_by_hour": {"7": 1.3, "8": 1.9, ...}}}}
"""
import json
from pathlib import Path

MAX_EXTRA = 1.5          # a ratio of 2.5 or more gives the largest multiplier, 2.5


def load_profile(path):
    return json.loads(Path(path).read_text())


def ratio(profile, link_id, hour):
    by_hour = profile.get("links", {}).get(link_id, {}).get("ratio_by_hour", {})
    return float(by_hour.get(str(int(hour)), 1.0))


def multiplier(profile, link_id, hour):
    return 1.0 + min(max(ratio(profile, link_id, hour) - 1.0, 0.0), MAX_EXTRA)


def link_weight_fn(profile, hour):
    """A function link_id -> multiplier, for NodeCoordinator(link_weight=...)."""
    return lambda link_id: multiplier(profile, link_id, hour)


def rank_links(profile):
    """[(link id, worst ratio, hour of the worst ratio)] from most to least congested."""
    rows = []
    for lid, data in profile.get("links", {}).items():
        by_hour = data.get("ratio_by_hour", {})
        if by_hour:
            hour, worst = max(by_hour.items(), key=lambda kv: kv[1])
            rows.append((lid, float(worst), int(hour)))
    return sorted(rows, key=lambda r: -r[1])


def synthetic_profile(link_ids, seed=1):
    """A made-up morning and evening peak, for testing the pipeline with no API key.
    It is labelled SYNTHETIC and says nothing about Lusaka."""
    import random
    rng = random.Random(seed)
    links = {}
    for lid in link_ids:
        base = 1.0 + rng.random() * 0.3
        links[lid] = {"ratio_by_hour": {
            str(h): round(base + (0.9 if h in (7, 8, 17, 18) else 0.3 if h in (9, 16, 19) else 0.0)
                          + rng.random() * 0.2, 2) for h in range(6, 21)}}
    return {"source": "SYNTHETIC", "asof": "n/a", "links": links}


def main():
    import sys
    if len(sys.argv) < 2:
        sys.exit("usage: python priorities.py profile.json")
    profile = load_profile(sys.argv[1])
    print(f"source: {profile.get('source')}  as of {profile.get('asof')}")
    print(f"{'link':28s}{'worst ratio':>12}{'at hour':>9}{'weight':>8}")
    for lid, worst, hour in rank_links(profile):
        print(f"{lid:28s}{worst:>12.2f}{hour:>9d}{multiplier(profile, lid, hour):>8.2f}")


if __name__ == "__main__":
    main()
