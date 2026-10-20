# E10 collection notes

Twenty scheduled trials were attempted, ten per mode. Each mode completed nine.
The selected route was Robot1 → Robot2 → Robot4 → Robot5 throughout. Robot3 did
not participate. Pilot runs and startup diagnostics are in separate directories
and are excluded from this series.

## Clocks and interpretation

The thesis uses simulation-time figures and reports the wall-time results
separately. Confirmed simulated makespan was 108.44 ± 5.80 s for exclusive and
97.68 ± 2.44 s for region-based arbitration (sample SD, nine successes each).
The difference is 9.9%. This does **not** establish an end-to-end wall-time gain.

The corresponding wall-time means were 65.15 ± 20.27 s and 68.76 ± 19.85 s.
Effective simulator throughput changed: most earlier successful trials advanced
about 1.3–1.4 simulated seconds per wall second; from repetition eight it was
about 2.4–2.7. The cause of this throughput change was not established during
collection. A late-series scene inspection reported realtime=false,
realtimeModifier=1, speedModifier=0, stepsPerRendering=1 and a time step of
approximately 0.05 s. This inspection does not establish the settings of earlier
trials.

Region repetition five also had a 31.6 s gap between sampler responses and
took 104.46 wall seconds. Robot4's home action remained outstanding for about
42.3 s. The evidence does not isolate the cause of that whole interval.
The run is retained in all successful-run statistics.

Simulation home timestamps come directly from the joint samples used to confirm
completion. The delivery and first-involvement markers arrive between samples;
their following simulation sample is plotted and their bracketing bounds are
retained. All endpoints include the original 500 ms wall-time home confirmation
window, whose simulation-time length depends on playback throughput.

The primary per-robot figures use the common delivery request as time zero.
Individual participation intervals are retained separately. Zero participation
by Robot3 is represented as missing observations, not a measured zero duration.

## Failed trials and continuation

- Exclusive repetition four: Robot5's placement planner reported
  pick_hover_unreachable, with Robot5_wall named in its diagnostics. Its agent
  repeatedly entered the placement failure handler. The runner was interrupted
  after the terminal failure was identified. Its raw result therefore says
  "Evaluation interrupted"; the underlying cause is preserved in Robot5.log
  and agents.log in trials/exclusive_4/.
- Region repetition nine: Robot4's final home returned motion_execution_failed,
  with simple_joint_path_collision_detected in the planner message. The runner
  detected the terminal failure and stopped the trial. This is a collision-check
  abort, not a measurement of minimum separation or proof of physical contact.

After exclusive repetition four, terminal-failure detection and resume support
were added to the evaluation harness. The original evaluation agent copies,
cell configuration and successful timing rules were retained. The schedule
resumed at exclusive repetition five and kept the failed trial; it did not add
a replacement success. The resumption is logged in e10_resumptions.jsonl.
Raw records and failed-trial logs have not been rewritten.

See e10_audit.json for endpoint validation and sample intervals, e10_clocks.csv
for both clocks per trial, and e10_analysis.json for analysis conventions.
