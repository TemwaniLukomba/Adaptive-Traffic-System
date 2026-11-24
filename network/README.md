# Connected signals: a network of nodes that talk to their neighbours

A node is ANY signalised point: crossroads, T or Y junction, five-way junction, one-way street
crossing, a pedestrian crossing in the middle of a road, a road-works shuttle, or a layout learned
from video. Nodes are joined by links between two of their arms, so a corridor or grid can mix road
types freely. `networks/corridor.json` mixes a crossroads, a T-junction, a mid-block pedestrian
crossing and a crossroads. `networks/grid_mixed.json` includes a one-way street.

## Files

| File | What it does |
| --- | --- |
| `graph.py` | The network: nodes (any layout), directed links, routes between edge arms. `python graph.py networks/corridor.json` prints it. |
| `coordination.py` | The neighbour messages, the message bus (delay and loss), and the rules that turn what a node hears into phase-score adjustments. |
| `predictor.py` | A small ridge-regression forecaster: PCE arriving from a neighbour in the next 20 s. |
| `netsim.py` | A SIMPLIFIED multi-node simulator (not SUMO) for testing the logic quickly. |
| `train_predictor.py` | Trains the predictor on the simulator and tests it on unseen seeds and on a different network. |
| `sumo_build.py`, `sumo_demand.py`, `sumo_run.py`, `sumo_study.py` | The SUMO version: build the network, write demand, run it under the same controllers. Untested in SUMO. |
| `study_net.py` | Compares fixed, isolated adaptive, connected, connected + AI, and connected + Google weights. |
| `test_network.py`, `test_sumo_glue.py` | Checks, including that coordination never breaks a signal safety rule, with and without message loss. |
| `../gmaps/` | Offline Google data: `fetch_profile.py` (Routes API), `priorities.py` (rank links, set weights). |

## How the nodes cooperate

Every node keeps its own camera, controller and safety state machine. Every 2 s it sends each
neighbour a small message (queue at the shared stop line, vehicles released toward it and when they
will arrive, queue heading its way, whether its signal is green toward it). What a node hears only
changes the score of its own phases:

* platoon bonus: a phase is worth more when vehicles are about to arrive on its approaches;
* spill-back penalty: a phase is worth less when the road it feeds is nearly full.

Messages cannot touch yellow, all-red, minimum green, the starvation limit or the one-phase-at-a-time
rule (`controller.score_adjust` only changes the score). A message older than 15 s is ignored, so a
node that hears nothing behaves like a stand-alone junction. Information spreads node to node along
the whole network, with no central computer.

## Where AI and Google data fit

* AI: `predictor.py`, a linear model that forecasts arrivals. Small, inspectable, trainable on logs.
  Optional reinforcement learning is a later step, not built.
* Google: used OFFLINE to rank congested links (which junctions to connect first) and to scale
  coordination per link and hour. The live controller never calls Google (minutes-old data, needs
  internet, usage terms). `fetch_profile.py` has NOT been run against the live API from here.

## Commands

    python test_network.py          # checks
    python train_predictor.py       # about 40 s
    python study_net.py --quick     # a few seconds
    python study_net.py             # about 1 minute
    python study_net.py --loss 0.5  # half the messages lost

## What the simplified simulator showed (see results/NETWORK_STUDY.md)

* Connected control cut mean delay by roughly 30% (corridor) and 55% (grid) at the peak setting,
  and made little difference at light and balanced demand (under about 5%, within noise).
* The peak and surge settings are oversaturated in this model (hundreds of vehicles unfinished), so
  the percentages are exaggerated. They show direction, not size.
* Removing one term at a time: the platoon bonus (traffic about to arrive) gave nearly all the gain
  on both networks. The spill-back penalty alone gave a smaller gain on the grid and almost none on
  the corridor. I expected spill-back to matter more, so the model does not yet show why.
* The learned predictor beat the plain rule at forecasting arrivals (MAE 1.8 vs 2.7 on unseen
  seeds, 1.4 vs 2.3 on a different network) but gave no consistent control improvement over the
  plain rule. The Google weights (a SYNTHETIC profile, so they say nothing about Lusaka) helped in
  the grid surge case and made no difference elsewhere.
* Connected control raised pedestrian waiting at the corridor's crossing in some scenarios.
* 3 seeds per cell: differences under about 5% are noise.

A bug was found while building the SUMO export: arms on a one-way link were being treated as
network exits. It is fixed (graph.py) and the figures above are from after the fix.

All of this is a model of a model. It checks the logic. It is not evidence about real roads.

## SUMO version

    python sumo_build.py networks/corridor.json          # writes sumo/corridor/ and runs netconvert
    python sumo_run.py networks/corridor.json --scenario peak --mode isolated
    python sumo_run.py networks/corridor.json --scenario peak --mode coordinated --gui
    python sumo_run.py networks/corridor.json --scenario peak --mode coordinated --ai
    python sumo_study.py --quick                         # fixed vs isolated vs connected in SUMO

`sumo_build.py` places the nodes by bearing and link length, writes nodes, edges and explicit
pedestrian crossings, and records which edge belongs to which node and arm in `map.json`.
`sumo_demand.py` writes vehicle flows and walkers for any route through the network.
`sumo_run.py` drives every traffic light from TraCI with the same controllers and the same neighbour
messages as the simplified simulator. `test_sumo_glue.py` checks the files and the runner against a
stand-in for TraCI.

First real SUMO results (mixed corridor, peak scenario, 3 seeds, mean delay per vehicle): fixed-time
405 s, stand-alone adaptive 337 s, connected 197 s (-42% against stand-alone), connected with the
learned predictor 191 s. Demand is illustrative, the fixed-time baseline is untuned, and only this one
network and scenario have been run in SUMO. The grid network and other scenarios are still to do.

## What is next

1. Run the rest of the SUMO study (grid network, all scenarios, message loss, more seeds) and use the
   SUMO numbers, not the simplified simulator's, in the report.
2. Run `fetch_profile.py` with a real key and real coordinates; replace the synthetic profile.
3. Replace the in-memory bus with MQTT or UDP between real nodes; the Message fields are JSON.
4. Real camera logs to retrain the predictor.
5. Decide whether the learned predictor earns its place; today the plain rule is as good for control.
