"""Build the signalised junction for SUMO from the layout in sim_config.py.

    python build_network.py
    python build_network.py --no-pedestrians
    $env:SIM_LAYOUT="t_junction"; python build_network.py        # PowerShell

Writes the plain-XML description into network/ and runs SUMO's netconvert to
create network/junction.net.xml. The layout (arms, bearings, lanes, one-way flags,
drive side) decides everything: how many edges, where they point, which movements
exist. Edge ids are <arm code>2C into the junction and C2<arm code> out of it
(for example N2C and C2N).

Pedestrians: unless you pass --no-pedestrians (or set sim_config.PEDESTRIANS = False),
netconvert also adds a sidewalk along every edge, zebra crossings and the corner walking
areas. The sidewalk becomes lane 0 of each edge, so the vehicle lanes are numbered 1 and 2.
"""
import json
import math
import subprocess
import sys
from pathlib import Path

import sim_config as sc
from find_sumo import find_binary

NET_DIR = Path(__file__).resolve().parent / "network"


def write_files(with_peds=None):
    NET_DIR.mkdir(exist_ok=True)
    with_peds = sc.PEDESTRIANS if with_peds is None else with_peds
    layout = sc.LAYOUT
    codes = sc.ARM_LETTER

    nodes = ['<nodes>', f'    <node id="{sc.TLS_ID}" x="0" y="0" type="traffic_light"/>']
    for arm in layout.arms:
        x = arm.length * math.sin(math.radians(arm.bearing))
        y = arm.length * math.cos(math.radians(arm.bearing))          # SUMO: y up = north
        nodes.append(f'    <node id="{codes[arm.name]}" x="{x:.2f}" y="{y:.2f}" type="priority"/>')
    nodes.append('</nodes>')

    edges = ['<edges>']
    for arm in layout.arms:
        c = codes[arm.name]
        speed = round(arm.speed_kmh / 3.6, 2)
        if arm.oneway in ("both", "in"):
            edges.append(f'    <edge id="{c}2{sc.TLS_ID}" from="{c}" to="{sc.TLS_ID}" '
                         f'numLanes="{arm.lanes}" speed="{speed}"/>')
        if arm.oneway in ("both", "out"):
            edges.append(f'    <edge id="{sc.TLS_ID}2{c}" from="{sc.TLS_ID}" to="{c}" '
                         f'numLanes="{arm.lanes}" speed="{speed}"/>')
    edges.append('</edges>')

    (NET_DIR / "nodes.nod.xml").write_text("\n".join(nodes) + "\n")
    (NET_DIR / "edges.edg.xml").write_text("\n".join(edges) + "\n")

    # Two-arm layouts: crossings are declared explicitly (netconvert's guess needs 3+ arms).
    con = NET_DIR / "connections.con.xml"
    if with_peds and len(layout.arms) < 3 and sc.CROSSING_ARMS:
        rows = ['<connections>']
        for name in sc.CROSSING_ARMS:
            c = codes[name]
            rows.append(f'    <crossing node="{sc.TLS_ID}" edges="{c}2{sc.TLS_ID} {sc.TLS_ID}2{c}" priority="true"/>')
        rows.append('</connections>')
        con.write_text("\n".join(rows) + "\n")
    elif con.exists():
        con.unlink()
    (NET_DIR / "layout.json").write_text(json.dumps(layout.to_dict(), indent=2))   # run_sim checks this


def main():
    with_peds = sc.PEDESTRIANS and "--no-pedestrians" not in sys.argv
    write_files(with_peds)
    print(sc.LAYOUT.describe())
    netconvert = find_binary("netconvert")
    cmd = [
        netconvert,
        "--node-files", str(NET_DIR / "nodes.nod.xml"),
        "--edge-files", str(NET_DIR / "edges.edg.xml"),
        "--output-file", str(NET_DIR / "junction.net.xml"),
        "--no-turnarounds", "true",
        "--lefthand", "true" if sc.DRIVE_ON_LEFT else "false",
    ]
    if with_peds:
        cmd += ["--sidewalks.guess", "true",
                "--sidewalks.guess.min-speed", "5", "--sidewalks.guess.max-speed", "20",
                "--walkingareas", "true"]
        if len(sc.LAYOUT.arms) >= 3:
            cmd += ["--crossings.guess", "true", "--crossings.guess.speed-threshold", "20"]
        else:
            cmd += ["--connection-files", str(NET_DIR / "connections.con.xml")]
    print("Running:", " ".join(cmd))
    result = subprocess.run(cmd)
    if result.returncode != 0:
        sys.exit("netconvert failed; see its messages above.")
    print(f"Network written to {NET_DIR / 'junction.net.xml'}"
          f" ({'with' if with_peds else 'without'} sidewalks and crossings)")


if __name__ == "__main__":
    main()
