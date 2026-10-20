# E11: startup scaling for 3, 4 and 5 robots

Five independent Gradle/JaCaMo startups per cell, fifteen runs altogether.
The recorder starts first and waits. You generate/load the scene, start the
lower server, start the AgentThings, and explicitly launch each Gradle run.
It never calls a robot endpoint, starts a lower server, or connects to CoppeliaSim.
It does not execute a cube transfer.

All commands below run from the WoMAT repository root. The existing five-robot
manifests and scene are kept. The smaller cells use copies under
`generated/experiments/e11/cell3` and `cell4`. Those copies are already prepared.
On a fresh checkout, create them with this **offline** command:

```bash
node src/eval/e11Scaling.js prepare-cells
```

It refuses to overwrite existing copies. It copies models, sampled workspaces,
TDs and plans, and writes generation/lower-server shell scripts. It does not
generate or load a simulator scene.

## Generate and serve the cells (after your current experiment)

Three robots:

```bash
bash generated/experiments/e11/cell3/generate.sh
bash generated/experiments/e11/cell3/lower.sh
```

Four robots:

```bash
bash generated/experiments/e11/cell4/generate.sh
bash generated/experiments/e11/cell4/lower.sh
```

Run only the chosen cell's lower server; its command stays running. These scripts
contain normal `npm run robot:pipeline` and `npm run robot:serve` commands; their
full expanded commands are also in each cell's `setup.json`.

For reference, the complete three-robot generation command is:

```bash
npm run robot:pipeline -- \
  --manifests generated/experiments/e11/cell3/Robot1/manifest.json,generated/experiments/e11/cell3/Robot2/manifest.json,generated/experiments/e11/cell3/Robot3/manifest.json \
  --generate-shared-cell \
  --offsets '0,0,0;1.39,0.19,0;2.19,1.14,0' \
  --orientations '0,0,-15;0,0,50;-90,-155,90' \
  --wall-offsets '0,0,0;0,0,0;-0.42876,-0.25726,0' \
  --sites 'Robot1_grip_support@0.55,0.55,0.08,0,Robot1_grip_cube;Robot3_grip_support_b@2.74,0.84,0.08' \
  --resample ''
```

The four-robot command is:

```bash
npm run robot:pipeline -- \
  --manifests generated/experiments/e11/cell4/Robot1/manifest.json,generated/experiments/e11/cell4/Robot2/manifest.json,generated/experiments/e11/cell4/Robot3/manifest.json,generated/experiments/e11/cell4/Robot4/manifest.json \
  --generate-shared-cell \
  --offsets '0,0,0;1.39,0.19,0;2.19,1.14,0;2.12,-0.24,0' \
  --orientations '0,0,-15;0,0,50;-90,-155,90;-90,35,90' \
  --wall-offsets '0,0,0;0,0,0;-0.42876,-0.25726,0;-0.2,-0.2,0' \
  --sites 'Robot1_grip_support@0.55,0.55,0.08,0,Robot1_grip_cube;Robot4_grip_support_b@2.42,-0.14,0.08' \
  --resample ''
```

`--resample ''` reuses the existing workspace clouds. The pipeline parser now
preserves this empty argument, so it does not accidentally request resampling.
Robot positions, mounting orientations, walls and support A match the current
five-robot descriptor. Interaction pairs follow the existing automatic geometric
discovery; no route or static successor chain is injected.

| Cell | Support B centre (world metres) | Nearest grasp / hover sample |
| --- | --- | --- |
| 3 robots | `(2.74, 0.84, 0.08)` | Robot3: 13.8 mm / 4.6 mm |
| 4 robots | `(2.42, -0.14, 0.08)` | Robot4: 6.4 mm / 2.9 mm |
| 5 robots | current `(4.33, 0.28, 0.08)` | unchanged |

The candidate search used collision-free samples at grasp height 0.1825 m and
hover height 0.3825 m, preferring positions away from other robots' clouds. The
nearest other retained arm's sample was 0.789 m away for cell3 and 0.233 m for
cell4 (minimum over grasp and hover). These distances support choosing candidates;
they do not establish live IK feasibility or collision clearance of the new bench.
Check the live task-site report: B should be reachable by Robot3 / Robot4,
respectively, and outside Robot1's reach. Reused clouds were sampled with the
previous obstacles; this experiment measures scaling with fixed input clouds,
not workspace resampling after moving a support.

For five robots, keep your current scene and use the ordinary manifests:

```bash
npm run robot:serve -- --manifests generated/robots/Robot1/manifest.json,generated/robots/Robot2/manifest.json,generated/robots/Robot3/manifest.json,generated/robots/Robot4/manifest.json,generated/robots/Robot5/manifest.json
```

The smaller-cell generators write their own combined scene/descriptor files;
they leave the five-robot descriptor and original manifest/TD/ASL files intact.
Use the E11 launcher below, which creates a JCM with the correct copied plans.

## Record five runs of a cell

Example for cell3; substitute `4` or `5` in the recorder/launch commands.

1. Generate the cell and start its lower server as above. Keep the cube at A.
2. For **every repetition**, stop the previous Gradle and AgentThing processes.
   To measure fresh interaction-space computation, archive that cell's cached
   pair metadata before starting fresh AgentThings:

   ```bash
   node src/eval/e11Scaling.js reset-cache --cell 3
   ```

   This explicit command moves relevant `space.json` files to
   `performance_log/e11/cache-backups`; it does not contact the lower server or
   simulator. Do not run it during another experiment. Fresh AgentThings are
   required because they also hold an in-memory cache. The lower server can
   stay running for all five startups: no transfer is triggered.
3. Start one AgentThing per robot, each in its own terminal. For cell3:

   ```bash
   npm run robot:agent -- --manifest generated/experiments/e11/cell3/Robot1/manifest.json
   npm run robot:agent -- --manifest generated/experiments/e11/cell3/Robot2/manifest.json
   npm run robot:agent -- --manifest generated/experiments/e11/cell3/Robot3/manifest.json
   ```

   For cell4 use `cell4` paths and start Robot4 too. For cell5 use
   `generated/robots/RobotN/manifest.json` for N=1…5. The generated `setup.json`
   contains the full AgentThing commands for each smaller cell.
4. Start the recorder **before Gradle** in a separate terminal, once per cell:

   ```bash
   npm run eval:e11 -- --cell 3
   ```

5. When ready, explicitly start the measured Gradle run yourself:

   ```bash
   npm run eval:e11:launch -- --cell 3
   ```

   This thin launcher records the timestamp immediately before spawning
   `./gradlew --console=plain -Pheadless run --args='...e11.jcm --log-conf ...'`.
   It passes the trace path to the Java process. Use this command instead of a
   bare `./gradlew`: otherwise there is no exact Gradle launch timestamp and the
   ordinary JCM does not contain the E11 instrumentation.
6. Wait for the recorder to print `Cell 3, run N: ...`. Then stop Gradle with
   Ctrl+C in its launcher terminal. Repeat steps 2, 3 and 5 until five run slots
   are filled. The recorder stays armed across repetitions and exits after five.

Failures consume a run slot and remain in the results. There is no automatic
retry or filtering to five successes. The default timeout is 600 seconds from
Gradle launch; set `--timeout 900` on the recorder if needed. A partial run stays
on disk if the recorder is interrupted; rearming resumes observing that run.
Do not leave a previous cell's AgentThings running when changing cell sizes.

For intentionally warm-cache measurements, omit `reset-cache` consistently and
still restart AgentThings/Gradle. Every result reports observed cache hits and
fresh computations; do not combine cold and warm cohorts as one computation
benchmark. Use a separate directory for a new cohort, passing the same
`--out performance_log/e11_warm` to recorder, launcher and analyzer.
The cache label refers to attempts during each agent's startup interaction-space
phase. Total attempt counts additionally include route-time refreshes, which can
legitimately hit spaces computed earlier in the same cold startup.

The recorder reads the existing `performance_log/interaction-compute.jsonl`
from the byte offset at launch. If your AgentThings use `WOMAT_COMPUTE_LOG`, pass
the matching filename to the launcher with `--compute-log FILE`. Keep that log
in place until the recorder prints the result.

## Measurements and boundaries

### If Robot4/Robot5 appear in a three-robot run

The E11 JCM starts only the selected agents, but the separately started
AgentThings obtain their robot catalogue from their own manifest's shared-cell
descriptor. The original `generated/robots/RobotN/manifest.json` files still
describe five robots. An AgentThing started from those files, or started from a
copy before the smaller cell was generated, therefore scans all five. Missing
workspace signatures conservatively yield `signature_incomplete` neighbours;
they are not evidence that those extra agents were actually started by Gradle.
A route send to a nonexistent agent can then abort the query.

Stop Gradle and the three AgentThings, then restart the AgentThings using exactly
the `generated/experiments/e11/cell3/RobotN/manifest.json` paths shown above.
The generated scene and lower server do not need restarting for this correction.
Each agent's scan should test **two** other robots. Robot1--Robot3 may still be
rejected during edge pricing because its space is below the minimum size; the
intended available path is through Robot2.

Restart the recorder too to load the updated check. The E11 launcher now passes
the expected cell size to the JVM, which checks the actual scanned robot names
and stops a mismatched initialization before cloud exchange or route search.
The recorder immediately labels such a trial incomplete and reports the mismatch.
Keep the misconfigured trials as diagnostic records; for a new five-run cohort,
use `--out performance_log/e11_corrected` on both recorder and launcher.

Java events have epoch timestamps and monotonic timestamps. Phase durations use
the monotonic clock within one JVM; offsets from Gradle launch use epoch time.
The time waiting for you to launch Gradle is excluded.

| Measurement | Start | End |
| --- | --- | --- |
| Neighbour discovery, per agent | immediately before `!discoverNeighbors` | return from that goal, after its report |
| Interaction-space phase, per agent | before `!shareInteractionSpaces` | return from that goal |
| Route discovery | source starts `!findRouteTo` | actual selected-route/no-route decision |
| Gradle startup overhead | launcher spawns Gradle | first phase begins |
| Message count | Gradle launch | recorder observes all agents initialized and the source route decision |

For each phase the cell duration is **latest end minus earliest start** across
its participating agents. Agent intervals can overlap, and route discovery can
start while another arm is still initializing. Do not add these durations or
use them as disjoint segments of a stacked chart. `e11_phases.csv` retains the
individual durations and offsets to show the actual timeline.

The interaction-space phase includes waiting for cloud identities, calculation,
cache reads and rendering, as performed by the existing startup protocol.
`interaction_compute_work_ms` additionally sums the AgentThing computation log's
attempt durations (placement reads plus estimator/cache lookup, before rendering).
It is aggregate work across agents and is not wall-clock phase duration. Attempt
counts distinguish computed, cached and refused pairs; empty geometric overlap
is a recorded refusal, not automatically a failed trial.

Route discovery includes downstream edge pricing and final source pricing.
The initial source edge-pricing pass before the first query is recorded separately
as `initial_edge_pricing`; task-site detection is also recorded separately.
The `routeSettled` belief is asserted before final pricing, so it is deliberately
not used as the endpoint. A source-local goal or missing scan/decision is flagged
as incomplete; `no_route` is retained separately from successful route discovery.
Route success does not mean successful physical delivery.

Messages are successfully dispatched Jason application messages between the
selected robot agents, counted once at the sender per recipient. This covers
workspace-cloud announcements and route queries/replies, including cyclic
refusals. It excludes internal artifact beliefs, HTTP requests, TCP packets,
lower-server traffic and messages to non-robot agents. The current startup
protocol uses direct `.send`; `.broadcast` is not used. The measurement window
closes on the recorder's 250 ms polling interval; routing duration itself uses
the exact source decision marker. Live `events.jsonl` continues until Gradle
stops, while `measurement-events.jsonl` freezes the result's input events.

## Files and analysis

```bash
npm run eval:e11:analyze
```

Results update after each run under `performance_log/e11/`:

- `e11_runs.csv`: one row per run, phase durations, startup overhead, total and
  per-type message counts, cache observations and route outcome.
- `e11_phases.csv`: each agent's phase interval relative to Gradle launch.
- `e11_summary.csv`: means and sample standard deviations for successful route
  runs, plus recorded/successful/expected counts and cache modes.
- `cellN/runR/`: immutable result, copied input manifests/descriptor, frozen
  measurement events, full JVM trace, message CSV, computation records and
  Gradle console log.

Raw events retain all message types and route cost/path. Missing repetitions,
missing timing boundaries, timeouts and no-route outcomes are never written as
zero-duration successes. `launch` refuses a sixth run and never overwrites an
existing run slot.

Offline validation: nine E11 tests (including five synthetic runs through the
passive watcher), Java compilation, parsing of all three JCMs and their copied
plans, and an in-memory Jason send-fanout check. Live simulator validation is
pending. The older `cellSites` and `cellTaskSites` suites have six failing
assertions tied to the old Robot4 goal/robot-local site configuration; the
current production manifests already describe the five-robot cell instead.
