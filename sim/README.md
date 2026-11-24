# sim/: SUMO simulation of the Traffic Jam System

Any junction in [SUMO](https://sumo.dlr.de), where simulated vehicles and pedestrians
react to the signals. The road is chosen in `sim_config.py` (or with `SIM_LAYOUT`): a preset
(`crossroads`, `t_junction`, `y_junction`, `five_way`, `shuttle`, `road_crossing`,
`one_way_cross`) or the `site.json` the camera learned, so you can simulate the junction your
video actually shows. It runs the project's own `controller.py` (unchanged) against baselines, so you can
measure delay, queues and waiting time, which recorded video cannot show.

## Install

```powershell
python -m pip install -r sim/requirements.txt
sumo --version            # should print a version; if not, see "SUMO not found" below
```

## Run (from the project folder)

```powershell
cd sim
python build_network.py                      # creates network/junction.net.xml (re-run after changing the layout)
$env:SIM_LAYOUT="t_junction"; python build_network.py     # PowerShell: pick another road type
$env:SIM_LAYOUT="../site.json"; python build_network.py   # the junction learned from your video
python run_sim.py --scenario tidal --controller adaptive --gui     # watch it
python run_sim.py --scenario ped_heavy --controller adaptive --gui # lots of people crossing
python run_sim.py --scenario tidal --controller fixed              # one run, numbers only
python run_sim.py --scenario tidal --controller adaptive --no-pedestrians   # vehicles only
python compare.py                            # every scenario x controller x 3 seeds
```

To evaluate every road type in one go and get a report ready to paste into a write-up:

```powershell
python study.py --quick          # crossroads + T-junction, 3 scenarios, 1 seed (a few minutes)
python study.py                  # all 7 road types x 6 scenarios x 3 controllers x 3 seeds (hours)
python study.py --miss-rates 0 0.15 --site ../site.json
```

It writes `results/study/STUDY_REPORT.md`, `study_summary.csv` and a chart. A road type that fails
to build is reported and skipped, and finished road types are not redone, so it can be resumed.

Results go to `sim/results/` (`runs.csv`, `summary.csv`, a chart if matplotlib is installed).

## Files

| File | Purpose |
| --- | --- |
| `sim_config.py` | Which road layout to simulate, step length, zone length, pedestrian settings |
| `build_network.py` | Writes the junction as plain XML from the layout (arms at their bearings, lanes, one-way flags) and builds it with `netconvert` |
| `make_demand.py` | Demand scenarios built from the layout, the car / minibus / truck / motorcycle mix, one route per movement |
| `baselines.py` | `LongestQueueController`, a simple demand-responsive baseline |
| `run_sim.py` | One simulation run through TraCI; returns the metrics |
| `compare.py` | Runs the whole matrix on one road type, prints a table, saves CSVs and a chart |
| `study.py` | Runs `compare.py` on every road type and writes one combined report |
| `find_sumo.py` | Finds `sumo`, `sumo-gui` and `netconvert` |
| `gui_settings.xml` | Makes `sumo-gui` draw sedans, vans, trucks, motorcycles and walking people instead of triangles |

## How it connects to the prototype

Each simulated second, `run_sim.py` measures the weighted load in the last 50 m before each
stop line (the same idea as a camera zone, using `config.VEHICLE_WEIGHTS` and the same
smoothing), passes it to the controller, and sets SUMO's signals from the colours it returns.
`--miss-rate 0.15` makes the "camera" miss 15% of vehicles, to test robustness to detection
errors.

The junction uses its own `sim_config.py`, so it works even if `config.py` is set up for
something else (for example the two-approach footage test). Timings (minimum green, yellow,
all-red) still come from `config.py`.

## Scenarios

| Name | Demand (vehicles per hour) |
| --- | --- |
| `balanced` | 450 on every approach |
| `main_peak` | the first phase's approaches 800, the others 200 (north-south at a crossroads, the main road at a T) |
| `tidal` | first phase heavy, then second phase heavy, then balanced |
| `one_sided` | the first approach 900, the rest 150 to 250 |
| `surge` | 300 everywhere with a five-minute surge on the first approach |
| `ped_heavy` | 450 on every approach, but 500 pedestrians per hour on every arm |

Rates are written for two lanes and scale with the number of lanes on each arm.

These numbers are illustrative, not measured in Lusaka. Replace them with counts from a real
junction (a 15 to 30 minute manual count split by vehicle type is enough) in `make_demand.py`.

## Pedestrians

* `build_network.py` adds a sidewalk to every edge (it becomes lane 0, so vehicle lanes are
  numbered 1 and 2), a zebra crossing across each of the four arms, and the corner walking areas.
* Walkers cross each arm in both directions at `PED_RATE` per hour (150 by default, 500 in
  `ped_heavy`). Change `PED_RATE`, `PED_BOTH_DIRECTIONS` and `PED_WEIGHT` in `sim_config.py`.
* **Walk signal:** a crossing shows walk only while the *other* phase is green. For example the
  north and south crossings are open during the east-west green. It closes the moment that green
  turns yellow, so the yellow plus all-red time (5 s) is the clearance for anyone still crossing.
  It never opens during yellow or all-red.
* **Turning vehicles give way:** a vehicle turning onto an arm whose crossing is open gets SUMO
  state `g` (give way), as do right turns that cross oncoming traffic.
* **Pedestrian demand:** each pedestrian waiting at a crossing adds `PED_WEIGHT` (0.6) to the
  phase that would let them cross, so the adaptive controller treats a waiting crowd like
  vehicles. `--ped-weight 0` makes the controller ignore pedestrians, to see what it costs them.
* A network built before this change has no crossings. The run says so and continues without
  pedestrians; rebuild with `python build_network.py`.

## Metrics

Mean delay per vehicle (SUMO's `timeLoss`), 95th percentile delay, longest wait, mean and
maximum queue, throughput, number of phase changes, and unfinished vehicles. With pedestrians
on there are also: `ped_mean_delay_s`, `ped_p95_delay_s`, `ped_max_wait_s`, `peds_completed`,
`ped_peak_waiting` (most people waiting at once) and `walk_time_share` (fraction of the run in
which at least one crossing showed walk). `compare.py` also prints the change against the
fixed-time plan.

## Tuning the adaptive controller

```powershell
python run_sim.py --scenario one_sided --controller adaptive --set demand_threshold=0.3
python run_sim.py --scenario surge --controller adaptive --set min_green=6 max_green=40
```

Worth checking: with `DEMAND_THRESHOLD = 0.5`, a single waiting motorcycle (weight 0.4) does
not count as demand, so it could wait a long time if the other phase stays busy. Compare
`max_wait_s` with a lower threshold.

## Assumptions and limits

* Every movement the layout allows is modelled (straight, left and right, no U-turns). At a
  crossroads that includes all eight turns. Shares are in `sim_config.TURN_SHARE` (70% straight,
  15% left, 15% right, illustrative); a turn that does not exist (a T-junction side road has no
  straight-on) has its share spread over the ones that do.
* Phases come from the geometry (`layout.py`): approaches whose paths never cross share a phase.
  Crossroads give NS and EW, a T-junction gives the through road and the side road, a Y or
  five-way junction gives one phase per approach. Movements that merely cross oncoming traffic
  (right turns when driving on the left, from the layout's drive side) give way (SUMO state
  `g`); there is no protected turn phase.
* `shuttle` takes turns on one lane (an approximation: SUMO still has one lane each way).
  `road_crossing` adds a pedestrian phase in which both vehicle approaches are red. Its crossing
  is declared explicitly in `network/connections.con.xml`, which is the least tested part.
* One-way arms only enter or only leave the junction and get no pedestrian flows.
* Pedestrians use one crossing per arm with a simple rule (walk during the other phase's green).
  There is no pedestrian push button, no flashing-don't-walk countdown, and no jaywalking.
  Motorcycles follow normal lane rules and do not weave between lanes.
* Demand is invented until you replace it with real counts.
* The camera is idealised (it sees every vehicle in the zone) unless you use `--miss-rate`.
* Results show how the controller behaves in simulation, not guaranteed gains on a real road.

## If sumo-gui still shows triangles

`run_sim.py --gui` loads `gui_settings.xml`, and `make_demand.py` gives each vehicle type a
`guiShape` (sedan, van, truck, motorcycle, pedestrian). If your version ignores the settings
file: click the colour-wheel icon (View settings) -> **Vehicles** -> *Show as* -> **simple
shapes**, then **Persons** -> *Show as* -> **simple shapes**. Zoom in on the junction to see them.

## If SUMO is not found

`pip install eclipse-sumo` normally puts `sumo` and `netconvert` on your PATH. Otherwise install
SUMO from https://sumo.dlr.de and add its `bin` folder to PATH, or set `SUMO_HOME`.
