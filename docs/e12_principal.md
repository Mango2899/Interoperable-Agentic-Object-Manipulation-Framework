# E12: the principal five-robot experiment, ten runs

Ten independent Gradle/JaCaMo startups of the existing five-robot cell, each one
carried through to the cube standing on support B.

The recorder starts first and waits. You generate or load the scene, start the
lower server and the AgentThings, and explicitly launch each Gradle run. The
recorder never calls a robot endpoint, never starts a lower server and never
connects to CoppeliaSim. The only thing it sends to a robot is the `!deliverCube`
you would otherwise type into the Jason console, and only when you ask it to.

E12 measures the same route discovery as E11, using the same instrumented
`routePlans.asl`, the same JSONL trace and the same monotonic clock, so the two
numbers are comparable. What it adds is the part E11 never timed: the transfer.

## What is measured

| Quantity | From | To |
|---|---|---|
| Startup phases | per-agent goal start | goal end |
| Route discovery | `!findRouteTo` adopted | `routeTotal` asserted at the source |
| **Handover** *k* | giver's first commanded motion to the rendezvous | receiver has retreated holding the cube |
| Delivery | `!deliverCube` adopted at the source | cube released on support B |
| Source pickup | `!ensureCubeHeld` at the source | cube grasped and lifted clear |
| Placement | `!placeCubeAt` at the sink | gripper open and retreated |

The handover interval reproduces the stopwatch definition used for the first
E12 reading. Two refinements over a stopwatch:

* It closes on the **receiver's own** retreat event, not on the giver's belief
  of it, so the message hop back to the giver is not counted as motion. The
  giver-observed value is kept alongside as `giver_observed_ms`, and the
  difference between the two columns is the messaging overhead.
* Every duration is a difference of two `System.nanoTime()` readings taken
  inside the one JaCaMo JVM that all five agents share, so it is immune to
  wall-clock adjustment and comparable across agents.

Each handover is also broken into `handover_plan`, `giver_approach`,
`receiver_approach`, `wait_receiver`, `transfer`, `receiver_retreat_motion` and
`giver_home`, which is what separates the fixed cost of a hop from its motion.

Alongside the timings the recorder captures what the route decision was made of:
every priced edge with its distance and candidate, every edge the pricing
refused with its reason, the full ranked option list the source settles on, and
every rendezvous the cell refused at execution time. `decompose` rebuilds
Eq. (route_cost) from those legs, so the cost table in the thesis is generated
from the run rather than transcribed from the console.

## Running it

All commands run from the WoMAT repository root. The production five-robot
manifests and scene are used unchanged; nothing is copied or regenerated.

Start the scene, the lower server and the five AgentThings as usual. Then, in a
terminal of its own:

```bash
npm run eval:e12
```

This instruments copies of `routePlans.asl`, `handoverPlans.asl` and the five
agent files under `performance_log/e12/plans/`, parse-checks them with Jason's
own parser, writes `e12.jcm`, and waits. The production plans are never touched.
If a plan has been reworded so that an instrumentation anchor no longer matches,
the tool stops here with the anchor it could not find, rather than recording a
run with a missing boundary.

For each of the ten runs, in a second terminal:

```bash
npm run eval:e12:launch
```

Wait for `Route settled:`, then:

```bash
npm run eval:e12:deliver
```

Watch the transfer. When the recorder prints the run's result, stop that Gradle
process (Ctrl+C) and launch the next run. A run slot is exclusive: a failed run
consumes its slot and is kept, so ten launches give ten records whether or not
all ten succeed.

Useful flags: `--runs N` (default 10), `--timeout SECONDS` (default 2400, how
long a run may take before it is recorded as a timeout), `--out DIR` (default
`performance_log/e12`), `--agent robot1_agent` for `deliver`.

To re-derive everything from the recorded runs without re-running the cell:

```bash
npm run eval:e12:analyze
```

## Output

Per run, under `performance_log/e12/runN/`: `events.jsonl` (the raw trace),
`measurement-events.jsonl`, `messages.csv`, `gradle.log`, `inputs.json`
(the manifests and cell descriptor as they stood at launch), `run.json`,
`exit.json` and `result.json`.

Across runs, under `performance_log/e12/`:

* `e12_runs.csv` — one row per run: phase durations, selected route and cost,
  handover total, delivery duration, refused edges and candidates, and the
  issues that made a run `incomplete`.
* `e12_handovers.csv` — one row per handover per run: priced metres, duration,
  giver-observed duration, messaging overhead, the seven sub-phases, and both
  the priced and the executed rendezvous candidate. `candidate_matches_price`
  is false when the cell committed to a different point than the one the route
  was priced at, which is the case Section 5 of the thesis argues about.
* `e12_legs.csv` — every priced and every refused edge, per run.
* `e12_routes.csv` — every route option reported to the source, per run,
  cheapest first, with which one was selected.
* `e12_summary.csv` — mean and sample standard deviation per metric over the
  successful runs, plus `distinct_selected_routes`, which is the route-stability
  answer the thesis asks for.
* `e12_cost_table.tex`, `e12_handover_table.tex`, `e12_figure_timeline.tex` —
  included directly by `thesis.tex` through `\IfFileExists`.

A run counts as `ok` only if all five agents initialized, every startup phase
has both boundaries, a route decision was recorded, every handover closed, the
number of handovers matches the selected route, and the cube was reported placed.
Anything else is `incomplete` with the reasons listed, and `analyze` keeps it in
`e12_runs.csv` while excluding it from the aggregates — failures are never
silently replaced by extra successes.

## What it does not measure

The carrying legs — the motion of an intermediate arm between the rendezvous
where it receives and the one where it gives — are not separately timed. They
fall inside `delivery_ms` and outside the handover intervals, so `unpriced_ms`
(delivery minus the handover intervals) bounds them together with the retreats
and the placement, but does not isolate them. That is the measurement the
discussion of the cost function still wants.

Durations are wall time of the host, as E10's are. The simulator's throughput is
not held constant across runs, so the timings are comparable within a run and
across runs only to the extent that the simulator was configured identically.
Record the scene's speed settings if runs are made on different days.
