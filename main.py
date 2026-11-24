"""Traffic Jam System prototype: camera -> vehicle loads per approach -> signal decision.

Usage:
    python main.py --video data/junction.mp4
    python main.py --video data/junction.mp4 --save out.mp4 --no-display
    python main.py --video data/junction.mp4 --recalibrate     # learn the junction again

There is nothing to draw or configure by hand. The first run watches the start of the video
and learns the road layout (arms, stop lines, zones, drive side, signal phases) into
site.json; later runs reuse it. See autocalibrate.py and layout.py.

Pipeline per processed frame:
    detect (YOLO) -> track (ByteTrack) -> count weighted vehicles in each zone
    -> smooth -> AdaptiveController decides which phase is green -> draw + log

A fixed-time controller runs in the background on the same footage so the two
can be compared. NOTE: recorded video does not react to the lights, so the
comparison measures decision quality, not real congestion relief.
"""
import argparse
import csv
import json
import sys
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import supervision as sv
from ultralytics import YOLO

import config
from autocalibrate import calibrate_video
from controller import AdaptiveController, FixedTimeController
from layout import Layout

COLOURS = {"green": (0, 200, 0), "yellow": (0, 215, 255), "red": (0, 0, 220)}


def load_site(path):
    """site.json written by autocalibrate.py -> (layout, frame size, zone polygons)."""
    with open(path) as f:
        site = json.load(f)
    layout = Layout.from_dict(site["layout"])
    zones = {name: np.array(poly, dtype=np.int32) for name, poly in site["zones"].items()}
    if site.get("ped_zone"):
        zones["ped"] = np.array(site["ped_zone"], dtype=np.int32)
    needed = layout.controller_approaches()
    missing = [a for a in needed if a not in zones]
    if missing:
        raise SystemExit(f"{path} has no zone for: {', '.join(missing)}. "
                         "Run again with --recalibrate (a longer --calibrate-seconds helps).")
    return layout, tuple(site["frame_size"]), zones


def make_zone(polygon, frame_size):
    """PolygonZone's signature differs between supervision versions."""
    try:
        return sv.PolygonZone(polygon=polygon)
    except TypeError:
        return sv.PolygonZone(polygon=polygon, frame_resolution_wh=frame_size)


def put_text(img, text, org, scale=0.6, colour=(255, 255, 255), thickness=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 2)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, thickness)


def draw_overlay(frame, zones, loads, status, metrics, sim_time):
    signals = status["signals"]
    for name, poly in zones.items():
        colour = COLOURS[signals[name]]
        cv2.polylines(frame, [poly], True, colour, 3)
        x, y = poly[0]
        put_text(frame, f"{name.upper()} {loads[name]:.1f}", (int(x), max(20, int(y) - 12)),
                 0.7, colour, 2)
        cv2.circle(frame, (int(x) - 18, max(20, int(y) - 20)), 10, colour, -1)

    # info panel
    cv2.rectangle(frame, (10, 10), (430, 130), (30, 30, 30), -1)
    put_text(frame, f"Time {sim_time:6.1f}s", (20, 35))
    put_text(frame, f"Phase {status['phase']}  {status['state']}  ({status['t_in_state']:.0f}s)",
             (20, 60), 0.65, (255, 255, 255), 2)
    put_text(frame, f"Why: {status['reason']}", (20, 82), 0.5)
    put_text(frame, f"Red-load-seconds  adaptive {metrics['adaptive']:.0f}", (20, 104), 0.5)
    put_text(frame, f"                  fixed    {metrics['fixed']:.0f}", (20, 122), 0.5)
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--video", required=True)
    parser.add_argument("--site", default="site.json", help="learned junction description")
    parser.add_argument("--recalibrate", action="store_true", help="learn the junction again")
    parser.add_argument("--calibrate-seconds", type=float, default=120.0)
    parser.add_argument("--drive-side", choices=["auto", "left", "right"], default="auto")
    parser.add_argument("--alternating", action="store_true",
                        help="one-lane road works: every approach gets its own phase")
    parser.add_argument("--pedestrians", action="store_true",
                        help="add a pedestrian phase (people are detected too)")
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--conf", type=float, default=config.DETECT_CONF)
    parser.add_argument("--log", default="decisions.csv")
    parser.add_argument("--save", default=None, help="write annotated video to this file")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=None,
                        help="stop after this much video time")
    args = parser.parse_args()

    if args.recalibrate or not Path(args.site).exists():
        calibrate_video(args.video, args.site, args.model, args.calibrate_seconds, args.conf,
                        drive_side=args.drive_side, alternating=args.alternating,
                        pedestrian_phase=args.pedestrians)
    layout, frame_size, zones_px = load_site(args.site)
    phases = layout.phases()
    approaches = layout.controller_approaches()
    zones = {name: make_zone(poly, frame_size) for name, poly in zones_px.items()}
    print("Phases:", ", ".join(f"{p}={'+'.join(a)}" for p, a in phases.items()))

    info = sv.VideoInfo.from_video_path(args.video)
    fps = info.fps or 25
    stride = max(1, round(fps / config.TARGET_FPS))
    dt = stride / fps
    print(f"Video {info.width}x{info.height} @ {fps:.1f} fps -> processing every "
          f"{stride} frame(s), dt={dt:.2f}s")

    model = YOLO(args.model)
    box_annotator = sv.BoxAnnotator()

    adaptive = AdaptiveController(phases=phases)
    fixed = FixedTimeController(phases=phases)
    history = {a: deque(maxlen=max(1, int(config.SMOOTHING_SECONDS / dt)))
               for a in approaches}
    metrics = {"adaptive": 0.0, "fixed": 0.0}   # sum of (weighted vehicles x seconds on red)

    writer = None
    if args.save:
        writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"),
                                 config.TARGET_FPS, frame_size)

    log_file = open(args.log, "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["time_s", "phase", "state", "event_reason"]
                 + [f"{a}_load" for a in approaches]
                 + [f"{a}_signal" for a in approaches]
                 + ["adaptive_red_load_s", "fixed_red_load_s"])

    class_ids = list(config.VEHICLE_WEIGHTS) + ([0] if "ped" in approaches else [])
    sim_time = 0.0
    frames_done = 0
    print("Running. Press q in the video window to stop.")

    try:
        for frame in sv.get_video_frames_generator(args.video, stride=stride):
            sim_time += dt
            if args.max_seconds and sim_time > args.max_seconds:
                break
            frame = cv2.resize(frame, frame_size)

            # 1) detect + 2) track
            result = model.track(frame, conf=args.conf, classes=class_ids, persist=True,
                                 tracker="bytetrack.yaml", verbose=False)[0]
            detections = sv.Detections.from_ultralytics(result)

            # 3) weighted load per zone, smoothed over ~1.5 s
            loads = {}
            for name, zone in zones.items():
                inside = zone.trigger(detections)
                classes = detections.class_id[inside] if len(detections) else []
                if name == "ped":                      # people waiting to cross
                    load = config.PERSON_WEIGHT * sum(1 for c in classes if int(c) == 0)
                else:                                  # vehicles (people never count here)
                    load = sum(config.VEHICLE_WEIGHTS.get(int(c), 1.0) for c in classes if int(c) != 0)
                history[name].append(load)
                loads[name] = sum(history[name]) / len(history[name])

            # 4) decide
            status = adaptive.update(loads, dt)
            fixed_status = fixed.update(loads, dt)
            for event in adaptive.pop_events():
                print(f"[{event['time']:6.1f}s] {event['phase']} -> {event['state']}"
                      f"  ({event['reason']})")
                event_reason = event["reason"]
                break
            else:
                event_reason = ""

            # metric: weighted vehicles waiting on red, per second
            for a in approaches:
                if status["signals"][a] == "red":
                    metrics["adaptive"] += loads[a] * dt
                if fixed_status["signals"][a] == "red":
                    metrics["fixed"] += loads[a] * dt

            log.writerow([f"{sim_time:.1f}", status["phase"], status["state"], event_reason]
                         + [f"{loads[a]:.2f}" for a in approaches]
                         + [status["signals"][a] for a in approaches]
                         + [f"{metrics['adaptive']:.1f}", f"{metrics['fixed']:.1f}"])

            # 5) draw
            frame = box_annotator.annotate(frame, detections)
            frame = draw_overlay(frame, zones_px, loads, status, metrics, sim_time)
            if writer:
                writer.write(frame)
            if not args.no_display:
                cv2.imshow("Traffic Jam System", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            frames_done += 1
    finally:
        log_file.close()
        if writer:
            writer.release()
        cv2.destroyAllWindows()

    print(f"\nProcessed {frames_done} frames ({sim_time:.0f}s of video).")
    print(f"Weighted vehicles x seconds spent on red:  adaptive {metrics['adaptive']:.0f}"
          f"  |  fixed-time {metrics['fixed']:.0f}")
    print(f"Decision log written to {args.log}")


if __name__ == "__main__":
    sys.exit(main())
