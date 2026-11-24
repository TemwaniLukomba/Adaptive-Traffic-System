"""Learn the junction from the camera itself: no clicking, no typing in zone coordinates.

    python autocalibrate.py data/junction.mp4
    python autocalibrate.py data/junction.mp4 --seconds 180 --pedestrians
    python autocalibrate.py data/junction.mp4 --site my_site.json --drive-side left

It watches the first few minutes of video, follows every vehicle (YOLO + ByteTrack) and then
works out, from where vehicles come from and go to:

  * the junction centre,
  * how many arms there are and their compass bearing (0 = top of the image),
  * which arms are entry-only, exit-only or two-way,
  * which side of the road traffic drives on,
  * the stop line of each approach (where queues form) and a zone covering the queue area,
  * which turning movements are actually used,
  * the signal phases (derived from the geometry by layout.py).

The result is written to site.json plus site_preview.png. main.py runs this by itself when
there is no site.json, so for normal use you never call it directly.

What it cannot know from a camera: lane counts and real distances (set them in site.json if
you need them for the simulation), and whether one lane carries road-works shuttle traffic
(use --alternating). It needs vehicles to actually pass through: a few minutes of normal
traffic is enough; a very quiet junction needs a longer --seconds. Always glance at
site_preview.png once. Every uncertain decision is printed as a warning.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

from layout import Arm, Layout, compass_name, angle_diff

# ------------------------------------------------------------------------------------------
# Pure geometry: tracks in, site description out (numpy only, so it is easy to test)
# ------------------------------------------------------------------------------------------


def bearing_of(dx, dy):
    """Compass bearing of an image vector (x right, y down; up/top of image = north)."""
    return float(np.degrees(np.arctan2(dx, -dy)) % 360.0)


def choose_frame_size(width, height, max_side=1280):
    """Processing size that keeps the video's aspect ratio (no stretching)."""
    scale = min(1.0, max_side / max(width, height))
    even = lambda v: max(2, int(round(v * scale / 2.0)) * 2)
    return even(width), even(height)


def _speeds(xy, t):
    dt = np.maximum(np.diff(t), 1e-6)
    v = np.linalg.norm(np.diff(xy, axis=0), axis=1) / dt
    v = np.r_[v[0], v]
    if len(v) >= 5:                                    # light smoothing
        v = np.convolve(v, np.ones(3) / 3, mode="same")
    return v


def _line_intersection(points, directions, weights=None):
    """Point closest (least squares) to a set of lines. Returns (point, well_conditioned)."""
    A = np.zeros((2, 2))
    b = np.zeros(2)
    for i, (p, d) in enumerate(zip(points, directions)):
        n = np.linalg.norm(d)
        if n < 1e-9:
            continue
        d = d / n
        M = np.eye(2) - np.outer(d, d)
        w = 1.0 if weights is None else weights[i]
        A += w * M
        b += w * M @ p
    if np.linalg.cond(A) > 1e3:
        return None, False
    return np.linalg.solve(A, b), True


def find_centre(tracks, frame_size):
    """Where the paths of vehicles from different arms meet."""
    pts, dirs = [], []
    for tr in tracks:
        xy = tr["xy"]
        q = max(2, len(xy) // 4)
        pts += [xy[0], xy[-1]]
        dirs += [xy[q] - xy[0], xy[-1] - xy[-1 - q]]
    fallback = np.median(np.vstack([tr["xy"] for tr in tracks]), axis=0)
    if len(pts) < 6:
        return fallback, False
    c, ok = _line_intersection(pts, dirs)
    if not ok:
        return fallback, False
    # one robust pass: drop lines that miss the centre by a lot, solve again
    dist = []
    for p, d in zip(pts, dirs):
        n = np.linalg.norm(d)
        dist.append(abs((d[0] * (c - p)[1] - d[1] * (c - p)[0]) / n) if n > 1e-9 else 1e9)
    dist = np.array(dist)
    keep = dist <= max(2.0 * np.median(dist), 8.0)
    if keep.sum() >= 6:
        c2, ok2 = _line_intersection([p for p, k in zip(pts, keep) if k],
                                     [d for d, k in zip(dirs, keep) if k])
        if ok2:
            c = c2
    w, h = frame_size
    if not (-0.1 * w <= c[0] <= 1.1 * w and -0.1 * h <= c[1] <= 1.1 * h):
        return fallback, False
    return c, True


def _cluster_bearings(angles, gap):
    """Split circular angles into clusters wherever there is a gap of at least `gap` degrees."""
    angles = np.asarray(angles, float)
    order = np.argsort(angles)
    a = angles[order]
    gaps = np.diff(np.r_[a, a[0] + 360.0])
    cuts = np.where(gaps >= gap)[0]
    labels_sorted = np.zeros(len(a), int)
    if len(cuts) > 0:
        start = (cuts[-1] + 1) % len(a)               # first element after the last cut
        label = 0
        for step in range(len(a)):
            i = (start + step) % len(a)
            labels_sorted[i] = label
            if i in cuts:
                label += 1
    labels = np.empty(len(a), int)
    labels[order] = labels_sorted
    return labels


def _circular_mean(angles):
    r = np.radians(angles)
    return float(np.degrees(np.arctan2(np.sin(r).mean(), np.cos(r).mean())) % 360.0)


def _hull(points):
    """Convex hull (monotone chain), counter-clockwise, as a list of [x, y]."""
    pts = sorted(set(map(tuple, np.round(points).astype(int))))
    if len(pts) <= 2:
        return [list(p) for p in pts]
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return [list(p) for p in lower[:-1] + upper[:-1]]


def learn_site(tracks, frame_size, people=None, min_arm_tracks=5, gap_deg=28.0,
               zone_frac=0.7, zone_length_px=None, drive_side="auto",
               alternating=False, pedestrian_phase=False, name="auto"):
    """Turn vehicle tracks into a site description.

    tracks: list of {"xy": (n, 2) array of ground points in pixels, "t": (n,) seconds}.
    Returns a dict that autocalibrate writes to site.json (see the keys at the bottom)."""
    warnings = []
    w, h = frame_size
    diag = float(np.hypot(w, h))
    usable = []
    for tr in tracks:
        xy = np.asarray(tr["xy"], float)
        t = np.asarray(tr["t"], float)
        if len(xy) >= 8 and np.linalg.norm(xy[-1] - xy[0]) >= 0.12 * min(w, h):
            usable.append({"xy": xy, "t": t, "v": _speeds(xy, t)})   # moving vehicles only
    if len(usable) < 2 * min_arm_tracks:
        raise ValueError(f"Only {len(usable)} moving vehicle tracks found; need at least "
                         f"{2 * min_arm_tracks}. Use a longer --seconds or a busier clip.")

    centre, centre_ok = find_centre(usable, frame_size)
    if not centre_ok:
        warnings.append("Junction centre was estimated from the average vehicle position "
                        "(paths did not cross clearly); check the preview.")

    ends = []                                           # (track index, role, point)
    for i, tr in enumerate(usable):
        ends.append((i, "in", tr["xy"][0]))
        ends.append((i, "out", tr["xy"][-1]))
    dist = np.array([np.linalg.norm(p - centre) for _, _, p in ends])
    r_cut = 0.5 * float(np.percentile(dist, 75))
    far = [k for k, d in enumerate(dist) if d >= r_cut]
    angles = np.array([bearing_of(*(ends[k][2] - centre)) for k in far])
    labels = _cluster_bearings(angles, gap_deg)

    clusters = []
    for lab in sorted(set(labels)):
        members = [far[j] for j in np.where(labels == lab)[0]]
        if len(members) >= max(min_arm_tracks, 4):
            clusters.append(_circular_mean(angles[labels == lab]))
    if len(clusters) < 2:
        raise ValueError("Could not find two or more arms. The clip may not show a junction "
                         "with traffic from different directions.")
    clusters.sort()
    bearings = clusters

    def arm_of(point):
        d = np.linalg.norm(point - centre)
        if d < r_cut:
            return None
        b = bearing_of(*(point - centre))
        diffs = [abs(angle_diff(b, a)) for a in bearings]
        k = int(np.argmin(diffs))
        return k if diffs[k] <= gap_deg else None

    start_arm = [arm_of(tr["xy"][0]) for tr in usable]
    end_arm = [arm_of(tr["xy"][-1]) for tr in usable]

    # ---- per arm: roles, lateral sides, stop line, zone ---------------------------------------
    names, used = [], {}
    for b in bearings:
        base = compass_name(b)
        used[base] = used.get(base, 0) + 1
        names.append(base if used[base] == 1 else f"{base}{used[base]}")

    votes, vote_weight = 0.0, 0.0
    arms, zones, stop_info, arm_stats = [], {}, {}, {}
    for k, (b, nm) in enumerate(zip(bearings, names)):
        u = np.array([np.sin(np.radians(b)), -np.cos(np.radians(b))])      # outward unit vector
        perp = np.array([np.cos(np.radians(b)), np.sin(np.radians(b))])    # bearing + 90 degrees
        ent = [i for i, a in enumerate(start_arm) if a == k]
        ext = [i for i, a in enumerate(end_arm) if a == k]
        total = len(ent) + len(ext)
        if len(ent) < 0.1 * total:
            oneway = "out"
        elif len(ext) < 0.1 * total:
            oneway = "in"
        else:
            oneway = "both"
        arm_stats[nm] = {"entering": len(ent), "leaving": len(ext)}

        # entering portions: from the start to the closest approach to the centre
        s_all, l_all, v_all = [], [], []
        for i in ent:
            xy = usable[i]["xy"]
            j = int(np.argmin(np.linalg.norm(xy - centre, axis=1)))
            seg = xy[: j + 1] - centre
            s_all.append(seg @ u)
            l_all.append(seg @ perp)
            v_all.append(usable[i]["v"][: j + 1])
        lat_out = []
        for i in ext:
            xy = usable[i]["xy"]
            j = int(np.argmin(np.linalg.norm(xy - centre, axis=1)))
            seg = xy[j:] - centre
            keep = (seg @ u) > 0.3 * r_cut
            lat_out.append((seg @ perp)[keep])
        if s_all:
            s = np.concatenate(s_all)
            lat = np.concatenate(l_all)
            v = np.concatenate(v_all)
            out_l = np.concatenate(lat_out) if lat_out else np.array([])
            far_in = lat[s > 0.3 * r_cut]
            if len(far_in) >= 5 and len(out_l) >= 5:
                wgt = float(min(len(ent), len(ext)))
                votes += wgt * np.sign(np.median(far_in) - np.median(out_l))
                vote_weight += wgt

        if oneway != "out" and len(ent) >= min_arm_tracks:
            pos = s > 0
            s_p90 = float(np.percentile(s[pos], 90))
            moving = v[v > 0.5]
            v_ref = float(np.percentile(moving, 80)) if len(moving) else 1.0
            still = pos & (v < 0.12 * v_ref)
            if still.sum() >= 5:
                s_stop = float(np.percentile(s[still], 8))                 # head of the queue
                stop_info[nm] = "queue head"
            else:
                s_stop = 0.2 * s_p90
                stop_info[nm] = "guessed (no queue seen; vehicles never stopped)"
                warnings.append(f"{nm}: vehicles never queued during calibration, so the stop "
                                "line is a guess. A longer --seconds or a busier clip helps.")
            length = zone_length_px or max(zone_frac * (s_p90 - s_stop), 0.08 * diag)
            s_end = s_stop + length
            sel = (s >= s_stop) & (s <= s_end)
            if sel.sum() < 5:
                sel = s >= s_stop
            l_lo, l_hi = np.percentile(lat[sel], [2, 98])
            pad = 0.15 * (l_hi - l_lo) + 4
            l_lo, l_hi = l_lo - pad, l_hi + pad
            if l_hi - l_lo < 24:
                mid = 0.5 * (l_lo + l_hi)
                l_lo, l_hi = mid - 12, mid + 12
            corners = [centre + s_stop * u + l_lo * perp, centre + s_stop * u + l_hi * perp,
                       centre + s_end * u + l_hi * perp, centre + s_end * u + l_lo * perp]
            zones[nm] = [[int(np.clip(round(x), 0, w - 1)), int(np.clip(round(y), 0, h - 1))]
                         for x, y in corners]
        elif oneway != "out":
            warnings.append(f"{nm}: only {len(ent)} vehicles entered from here; no zone made.")
        arms.append(Arm(nm, round(b, 1), lanes=2, oneway=oneway))

    if drive_side == "auto":
        if vote_weight > 0:
            drive_on_left = bool(votes > 0)
            conf = abs(votes) / vote_weight
            if conf < 0.5:
                warnings.append(f"Drive side is uncertain (confidence {conf:.0%}); "
                                "use --drive-side left or right to set it.")
        else:
            drive_on_left = True
            warnings.append("Drive side could not be measured (no arm showed both entering and "
                            "leaving traffic); assuming left-hand traffic. Use --drive-side to set it.")
    else:
        drive_on_left = bool(drive_side == "left")

    movements = {}
    for sa, ea in zip(start_arm, end_arm):
        if sa is not None and ea is not None and sa != ea:
            key = f"{names[sa]}->{names[ea]}"
            movements[key] = movements.get(key, 0) + 1

    layout = Layout(name, arms, drive_on_left, alternating=alternating,
                    pedestrian_phase=pedestrian_phase)
    entry_arms = [a.name for a in arms if a.oneway != "out"]
    missing = [a for a in entry_arms if a not in zones]
    if missing:
        layout.arms = [a for a in arms if a.name not in missing or a.oneway == "out"]
        warnings.append(f"Dropped arms with too little traffic: {', '.join(missing)}.")

    ped_zone = None
    if pedestrian_phase and people:
        pp = np.vstack([np.asarray(p["xy"], float) for p in people if len(p["xy"])])
        if len(pp) >= 20:
            med = np.median(pp, axis=0)
            near = pp[np.linalg.norm(pp - med, axis=1) <= 2.5 * np.median(np.linalg.norm(pp - med, axis=1)) + 1]
            ped_zone = _hull(near)
        if not ped_zone or len(ped_zone) < 3:
            warnings.append("No pedestrians were seen, so there is no pedestrian zone.")
            ped_zone = None
    elif pedestrian_phase:
        warnings.append("Pedestrian phase requested but no pedestrian tracks were collected.")

    return {
        "frame_size": [int(w), int(h)],
        "layout": layout.to_dict(),
        "zones": zones,
        "ped_zone": ped_zone,
        "centre": [round(float(centre[0]), 1), round(float(centre[1]), 1)],
        "stop_line_source": stop_info,
        "arm_traffic": arm_stats,
        "observed_movements": dict(sorted(movements.items(), key=lambda kv: -kv[1])),
        "tracks_used": len(usable),
        "warnings": warnings,
    }


# ------------------------------------------------------------------------------------------
# Video side: YOLO + ByteTrack to collect tracks, then preview and save
# ------------------------------------------------------------------------------------------

VEHICLE_CLASSES = [2, 3, 5, 7]       # car, motorcycle, bus, truck (COCO)


def collect_tracks(video, model_path, frame_size, seconds, conf=0.3, target_fps=10,
                   people=False):
    import cv2
    from ultralytics import YOLO

    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise SystemExit(f"Could not open {video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    stride = max(1, round(fps / target_fps))
    model = YOLO(model_path)
    classes = VEHICLE_CLASSES + ([0] if people else [])
    veh, ppl = {}, {}
    n = 0
    print(f"Learning the junction from the first {seconds:.0f}s of {video} ...")
    while True:
        ok, frame = cap.read()
        if not ok or n / fps > seconds:
            break
        if n % stride == 0:
            frame = cv2.resize(frame, tuple(frame_size))
            res = model.track(frame, conf=conf, classes=classes, persist=True,
                              tracker="bytetrack.yaml", verbose=False)[0]
            if res.boxes is not None and res.boxes.id is not None:
                for box, tid, cls in zip(res.boxes.xyxy.cpu().numpy(),
                                         res.boxes.id.cpu().numpy().astype(int),
                                         res.boxes.cls.cpu().numpy().astype(int)):
                    foot = ((box[0] + box[2]) / 2.0, box[3])            # where it touches the road
                    store = ppl if cls == 0 else veh
                    store.setdefault(int(tid), []).append((foot[0], foot[1], n / fps))
            if (n // stride) % 100 == 0:
                print(f"  {n / fps:5.0f}s  {len(veh)} vehicles followed", flush=True)
        n += 1
    cap.release()

    def pack(d):
        return [{"xy": np.array([(x, y) for x, y, _ in pts]), "t": np.array([t for _, _, t in pts])}
                for pts in d.values() if len(pts) >= 8]
    return pack(veh), pack(ppl)


def save_preview(video, site, path):
    import cv2
    cap = cv2.VideoCapture(video)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        return
    frame = cv2.resize(frame, tuple(site["frame_size"]))
    layout = Layout.from_dict(site["layout"])
    cx, cy = site["centre"]
    cv2.drawMarker(frame, (int(cx), int(cy)), (0, 255, 255), cv2.MARKER_CROSS, 30, 3)
    for nm, poly in site["zones"].items():
        pts = np.array(poly, np.int32)
        cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
        cv2.putText(frame, nm.upper(), tuple(int(v) for v in pts[2]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (0, 0, 0), 5)
        cv2.putText(frame, nm.upper(), tuple(int(v) for v in pts[2]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (255, 255, 255), 2)
    if site.get("ped_zone"):
        cv2.polylines(frame, [np.array(site["ped_zone"], np.int32)], True, (255, 0, 255), 3)
    cv2.imwrite(str(path), frame)


def summarise(site):
    layout = Layout.from_dict(site["layout"])
    print("\n" + layout.describe())
    print(f"\n  centre at {site['centre']} (pixels), {site['tracks_used']} vehicle tracks used")
    for nm, st in site["arm_traffic"].items():
        src = site["stop_line_source"].get(nm, "-")
        print(f"  {nm:<10} entering {st['entering']:>4}  leaving {st['leaving']:>4}  stop line: {src}")
    if site["observed_movements"]:
        print("  turning movements seen: " + ", ".join(f"{k} x{v}" for k, v in site["observed_movements"].items()))
    for wmsg in site["warnings"]:
        print(f"  WARNING: {wmsg}")


def _plain(obj):
    """json can't store numpy numbers; turn them into plain Python ones."""
    if isinstance(obj, np.generic):
        return obj.item()
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    raise TypeError(f"{type(obj).__name__} is not JSON serializable")


def calibrate_video(video, site_path="site.json", model="yolov8n.pt", seconds=120.0,
                    conf=0.3, max_side=1280, **options):
    """Learn the junction from a video and write site.json. Returns the site dict."""
    import cv2
    cap = cv2.VideoCapture(video)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise SystemExit(f"Could not read {video}")
    frame_size = choose_frame_size(frame.shape[1], frame.shape[0], max_side)
    veh, ppl = collect_tracks(video, model, frame_size, seconds, conf,
                              people=options.get("pedestrian_phase", False))
    try:
        site = learn_site(veh, frame_size, people=ppl, **options)
    except ValueError as exc:
        raise SystemExit(f"Calibration failed: {exc}")
    Path(site_path).write_text(json.dumps(site, indent=2, default=_plain))
    preview = Path(site_path).with_name(Path(site_path).stem + "_preview.png")
    save_preview(video, site, preview)
    summarise(site)
    print(f"\nSaved {site_path} and {preview}. Open the preview once to confirm the zones.")
    return site


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video")
    parser.add_argument("--site", default="site.json")
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--seconds", type=float, default=120.0, help="how much video to learn from")
    parser.add_argument("--conf", type=float, default=0.3)
    parser.add_argument("--drive-side", choices=["auto", "left", "right"], default="auto")
    parser.add_argument("--alternating", action="store_true",
                        help="one-lane road works: every approach gets its own phase")
    parser.add_argument("--pedestrians", action="store_true",
                        help="add a pedestrian phase and learn a waiting zone from people")
    args = parser.parse_args()
    calibrate_video(args.video, args.site, args.model, args.seconds, args.conf,
                    drive_side=args.drive_side, alternating=args.alternating,
                    pedestrian_phase=args.pedestrians)


if __name__ == "__main__":
    sys.exit(main())
