"""Fetch typical congestion ratios from Google's Routes API and save profile.json.

    set GOOGLE_MAPS_API_KEY=your-key          (PowerShell: $env:GOOGLE_MAPS_API_KEY="your-key")
    python fetch_profile.py --corridors corridors.json --hours 6-20 --out profile.json
    python fetch_profile.py --corridors corridors.json --dry-run     # print requests, call nothing

For each directed link and each hour it asks for the drive time with traffic (TRAFFIC_AWARE) for
the NEXT occurrence of that hour on a weekday, and compares it with the drive time with no traffic.
ratio = duration / staticDuration. Only the ratios are saved.

Costs money per request and needs the Routes API enabled in Google Cloud. Put a budget alert on the
key, restrict it to the Routes API, and keep the key out of the code and out of the zip.
Read Google's current terms on storing and using its data before you rely on this.
NOTE: written from Google's published request format but not yet run against the live service
from here (no key available), so expect to adjust it on the first call.
"""
import argparse
import datetime as dt
import json
import os
import sys
import urllib.request
from pathlib import Path

URL = "https://routes.googleapis.com/directions/v2:computeRoutes"
FIELDS = "routes.duration,routes.staticDuration,routes.distanceMeters"
LUSAKA_UTC_OFFSET = 2      # Africa/Lusaka is UTC+2, no daylight saving


def next_weekday_at(hour):
    """RFC 3339 UTC time of the next weekday hour (Lusaka time); Google needs a future time."""
    now = dt.datetime.now(dt.timezone.utc).replace(tzinfo=None) + dt.timedelta(hours=LUSAKA_UTC_OFFSET)
    day = now.replace(hour=hour, minute=0, second=0, microsecond=0)
    if day <= now:
        day += dt.timedelta(days=1)
    while day.weekday() >= 5:
        day += dt.timedelta(days=1)
    return (day - dt.timedelta(hours=LUSAKA_UTC_OFFSET)).strftime("%Y-%m-%dT%H:%M:%SZ")


def body(link, hour):
    return {"origin": {"location": {"latLng": {"latitude": link["origin"]["lat"], "longitude": link["origin"]["lng"]}}},
            "destination": {"location": {"latLng": {"latitude": link["destination"]["lat"], "longitude": link["destination"]["lng"]}}},
            "travelMode": "DRIVE", "routingPreference": "TRAFFIC_AWARE",
            "departureTime": next_weekday_at(hour)}


def seconds(text):
    return float(str(text).rstrip("s"))


def call(key, payload):
    req = urllib.request.Request(URL, data=json.dumps(payload).encode(), method="POST", headers={
        "Content-Type": "application/json", "X-Goog-Api-Key": key, "X-Goog-FieldMask": FIELDS})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read())


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--corridors", default="corridors.json")
    p.add_argument("--hours", default="6-20", help="for example 6-20 or 7,8,17,18")
    p.add_argument("--out", default="profile.json")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()
    if "-" in args.hours:
        lo, hi = args.hours.split("-")
        hours = list(range(int(lo), int(hi) + 1))
    else:
        hours = [int(h) for h in args.hours.split(",")]
    links = json.loads(Path(args.corridors).read_text())["links"]
    key = os.environ.get("GOOGLE_MAPS_API_KEY", "")
    if not args.dry_run and not key:
        sys.exit("Set GOOGLE_MAPS_API_KEY first (or use --dry-run).")
    print(f"{len(links)} links x {len(hours)} hours = {len(links) * len(hours)} requests")
    out = {"source": "google_routes", "asof": dt.date.today().isoformat(), "links": {}}
    for lid, link in links.items():
        ratios = {}
        for h in hours:
            payload = body(link, h)
            if args.dry_run:
                print(lid, h, json.dumps(payload)[:140], "...")
                continue
            routes = call(key, payload).get("routes", [])
            if routes:
                r = routes[0]
                ratios[str(h)] = round(seconds(r["duration"]) / max(1.0, seconds(r["staticDuration"])), 2)
        out["links"][lid] = {"ratio_by_hour": ratios}
    if not args.dry_run:
        Path(args.out).write_text(json.dumps(out, indent=2))
        print(f"saved {args.out}")


if __name__ == "__main__":
    main()
