# E10: exclusive and region-based arbitration

Run the five-robot cell's normal BDI delivery with both arbitration modes.
There are no additional independent motions. The agents discover the route;
the giver's final home can overlap with downstream handovers in region mode.
An evaluation copy adds the final receiver's home command after placement in
**both** modes. The production agent plans are not edited.

Start CoppeliaSim with the WebSocket remote API on port 23050. Stop existing
lower Things, Agent Things and the cell MAS: the runner starts and stops its
own complete stack and reloads the saved cell for each trial.

    npm run eval:e10
    npm run eval:e10:analyze
    cd performance_log/e10
    pdflatex -interaction=nonstopmode -halt-on-error e10_figures.tex

The default is ten repetitions per mode, alternating exclusive/region and
region/exclusive order. The runner waits for all five agents and route discovery
before triggering Robot1's deliverCube. Startup and route discovery are outside
the measured task. Existing output is preserved; choose a new --out directory
for a new series. A failed trial is recorded and stops the series for diagnosis
rather than silently retrying until ten successes remain.

    npm run eval:e10 -- --repetitions 1 --out performance_log/e10_pilot

After diagnosing a failure, resume the original schedule without replacing any
failed execution:

    npm run eval:e10 -- --resume --continue-on-failure

Resume checks that the cell and timing settings match the original setup.
The optional --continue-on-failure records a terminal task failure and proceeds
to the next scheduled trial in a freshly loaded scene. Setup failures and
explicit interruption still stop the series. The table reports completed versus
attempted trials as well as the successful duration observations.

## Timing boundaries

Raw events use monotonic host elapsed time, including planning, communication,
arbitration waits, execution and settling. Joint samples also record simulation
time. Both clock versions are exported. The thesis figures use simulation time;
the wall-time figures are supplementary. Simulation time does not include
periods in which the simulator's clock stops advancing, such as synchronous
planning. It must not be interpreted as end-to-end host runtime.

Agent markers are timestamped on receipt from the MAS, including output/IPC
latency. Simulation home endpoints use their exact joint samples. Other events
are bracketed between samples; the following sample supplies their plotted
simulation timestamp and both bounds remain in e10_simulation_runs.jsonl.

- **Delivery:** initial delivery request to the terminal agent confirming
  placement and completing its retreat from the support.
- **Task / all home and idle:** same request to the last participating robot
  completing its final home.
- **Per robot (primary comparison):** the common delivery request to that
  robot's final home confirmation. This measures when each robot becomes
  available again, including the time before it first joins the handover.
- **Participation interval (supplementary):** first entry into a giver or
  receiver handover plan to final home confirmation. An intermediate robot's incoming and outgoing handovers
  form one interval. The first giver's interval includes pickup. Intermediate
  homes while carrying the cube do not count as completion.
- **Home confirmation:** a final-role agent marker after the successful home
  action and transition to idle, plus all six measured joints staying within
  one degree of zero for at least 500 ms. The confirmation time includes this
  settling period. Options: --home-tolerance and --settle (milliseconds).
- Robots omitted by the discovered route have missing values, not zero-duration
  observations. Their CSV rows remain, with participated=0.

Sampling is requested every 100 ms (--interval), plus remote-call latency.
Each sample reads all 30 joint positions in one simulator call. Raw samples
retain the actual times. Timeline bars show participation, **not** physical
motion; overlapping bars do not establish concurrent joint motion.

## Output

Under performance_log/e10/ (or --out):

| File | Content |
| --- | --- |
| e10_setup.json | Timing settings, complete manifests and cell descriptor |
| e10_runs.jsonl | Raw results, timestamps, final joints and failures |
| e10_trials.csv | One row per trial: status, selected route and total durations |
| e10_robots.csv | Per run/robot participation, duration and task-relative start/end |
| e10_summary.csv | Wall-time mean, sample SD and attempted/successful counts |
| e10_simulation_summary.csv | Corresponding simulation-time statistics |
| e10_simulation_runs.jsonl | Derived simulation times with event bounds |
| e10_clocks.csv | Effective simulator throughput per completed trial |
| e10_participation_summary.csv | Durations starting at each robot's own first involvement |
| e10_analysis.json | Explicit timing origin and statistical conventions |
| e10_comparison.json | Wall-time exclusive/region ratio and percentage reduction |
| e10_table.tex | Supplementary wall-time table with observation counts |
| e10_simulation_table.tex | Simulation-time table included by the thesis |
| e10_figure_durations.tex | Delivery, all-home and individual duration comparison |
| e10_figure_timeline.tex | Each mode's lower median successful run |
| e10_figure_simulation_durations.tex | Simulation-time duration figure for the thesis |
| e10_figure_simulation_timeline.tex | Simulation-time participation timeline for the thesis |
| e10_simulation_comparison.json | Ratios and percentage reductions in simulation time |
| e10_figures.tex | Standalone document: simulation figures followed by wall-time figures |
| e10_simulation_durations.pdf, e10_simulation_timeline.pdf | Cropped vector exports for the thesis |
| e10_wall_durations.pdf, e10_wall_timeline.pdf | Cropped supplementary wall-time exports |
| trials/ | Events, joint samples, motion log, runtime logs, individual results |
| agent_sources/ | Exact evaluation agent copies used by this series |

Failed/incomplete trials remain in the raw data but are excluded from duration
means. Single observations have no sample SD. Without successful measurements,
no figures are generated. Do not interpret an absent mode or nonparticipant as
zero. Compare the selected route and refusals in the agent logs before
attributing a duration difference to arbitration.

This version makes no claim about minimum inter-arm distance, a safety
comparison, or independent simultaneous motions. The ordinary safety pipeline
remains enabled in both modes.

The primary figures and table use the common task start for per-robot times.
To regenerate them using individual participation intervals instead:

    npm run eval:e10:analyze -- --robot-origin participation

The raw data always retains both origins. In e10_robots.csv, beginS and homeS
are relative to the common task start; participationS is their difference.
Those three columns are wall seconds. The simulation-time version is available
in e10_simulation_runs.jsonl and e10_simulation_summary.csv.

For the collected series, see performance_log/e10/e10_notes.md. Effective
playback throughput changed during collection. The wall-time means therefore
must not be presented as a comparison under fixed playback conditions.
