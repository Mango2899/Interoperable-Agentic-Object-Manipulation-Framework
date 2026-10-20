# E13: the alternative route, measured two ways

E12 times a delivery that goes to plan. E13 times the one that does not.

The five-robot cell has two routes from the loaded support to the empty one,
and the search finds both every time: `Robot1-Robot2-Robot4-Robot5` at 4.050 m
and `Robot1-Robot2-Robot3-Robot5` at 4.237 m. E13 takes Robot4's agent away and
measures what the cell does about it. No obstacle is added: the absence of the
agent is the only thing the experiment perturbs.

Robot2 is the arm that matters, because it is the only one with a choice to
make. It collected both answers during the search — Robot4 at 2.855 m and
Robot3 at 3.042 m — and it is holding the cube when Robot4 turns out to be
unusable. So the recovery is one arm re-deciding locally over answers it already
has, with no message to anybody, which is the pay-off of having accumulated the
costs upstream instead of collecting paths at the origin.

## The two arms

Five runs each, and the pair is the result. "The cube went via Robot3" is the
same sentence for both, and they are not the same experiment.

| | `cold` — agent absent | `kill` — agent killed |
|---|---|---|
| Robot4's AgentThing | running | running, then stopped |
| Robot4's Jason agent | never in the MAS | started, then loses its layer |
| Route discovered | only the dearer one | the cheap one |
| How Robot4 leaves the search | unaskable: the send fails | answers, is chosen, then fails |
| Reroute | none | one, at Robot2 |
| Discarded motion | none | one approach plus a posture reset |

The cold arm is what knowing in advance costs: the cheap route is never found,
and the delivery that follows is an ordinary E12 delivery down the dearer route.
The kill arm is what not knowing costs: discovery is fast, and the bill arrives
later as a wasted approach, a detection deadline, a reset, and the dearer route
from Robot2 onwards.

**How the cold arm leaves the search matters, and it is cheap.** Jason *fails*
`.send` when the receiver is not in the MAS, so an arm whose agent never started
cannot be asked at all. `routePlans.asl` treats that as what it is — no route
runs through an arm that is not there to answer — and records a "no" on its
behalf, which is what keeps the neighbours further down the same list getting
asked. So the cold arm's discovery ends on a complete tally in about the time an
undisturbed one does, rather than waiting out the 240 s budget. The recorder
still distinguishes `complete_tally` from `deadline`, because a deadline there
would mean something else had gone wrong.

That behaviour was added for this experiment and was a real bug before it: Robot1
asks its neighbours in the order `[robot4,robot3,robot2]`, the inline `.send` to
the absent Robot4 failed first, and the failure took the whole search down at the
origin — `Route discovery did not complete`, with Robot3 and Robot2 never asked.
`routeSearch.test.js` now checks over 400 random topologies that an unaskable arm
costs exactly what deleting it would, and that every arm on a list still gets
asked whatever position the unaskable one holds.

**A third state, which the production `.jcm` produces and this experiment does
not.** An arm can be *in* the MAS with its own layer dead — `robot4_agent`
reasoning, its AgentThing stopped. That arm can be sent to, so it used to be sent
to, and it then spent three separate timeouts (30 s neighbour scan, 20 s task
sites, then 30 s per requester waiting on a `taskSitesReady` that could no longer
be asserted) arriving at an answer it already had. It now finds out at once, says
so once by broadcast, answers `noRoute` immediately if asked before the
announcement lands, and is skipped entirely afterwards. Nothing about it names a
robot: it is whichever arm finds its own layer silent. So a production run with a
stopped AgentThing now settles about as fast as this experiment's cold arm, and
`routeSearch.test.js` asserts that all three states reach the same route at the
same price.

**In the cold arm Robot4's AgentThing stays up.** Only the reasoning agent is
missing from the `.jcm`. Every other arm still finds Robot4 by reach and still
prices the edge to it, so the route the cold arm fails to find is a route that
genuinely exists. Taking the AgentThing down as well would make it a four-robot
cell, which is a different experiment — and the neighbour scan would abort on a
cell mismatch rather than measure anything.

## What is measured

Route discovery is measured exactly as in E11 and E12 — same instrumented
`routePlans.asl`, same JSONL trace, same monotonic clock — so the three numbers
are comparable. E13 adds how the search *ended*, and the reroute.

| Quantity | From | To |
|---|---|---|
| Route discovery | `!findRouteTo` adopted | `routeTotal` asserted at the source |
| **Convergence kind** | — | `complete_tally` or `deadline`, with the silent branches named |
| **Arms that could not be asked** | — | named by whoever tried to ask them |
| **Detection** | giver's first commanded motion to the doomed rendezvous | the arm concludes the partner is unfit |
| **Decision** | `!rerouteAround` adopted | the replacement `nextHop` asserted |
| Reset | `!prepareToForwardCube` at the reroute | upright with the cube |
| Recovery | partner judged unfit | giver's first motion of the replacement hop |
| Delivery | `!deliverCube` adopted | cube released on support B |

Two of those need saying out loud.

**Detection is bimodal, and is never averaged across its two modes.** An arm
that answers "I cannot take it" is a message hop. An arm that says nothing costs
the whole 120 s readiness deadline. Both are real failures and both reroute; the
trace records which, and `e13_reroute_table.tex` gives them separate rows with
their own *n*.

Expect the **explicit refusal** in this cell. Stopping Robot4's AgentThing takes
its websocket down, and `wotSend` now fails an operation it could not send rather
than logging and returning — so `robot4_agent` learns at once, and `-!receiveCube`
tells the giver *before* it tries to tidy up through the layer it has just lost.
Previously the tidy-up came first, failed, and took the notification with it, so
the giver waited out its whole readiness deadline for an arm that had already
worked out it could not come. If a run still records `timeout`, something else
went quiet and the row will say so.

**The decision is the contribution, and it is timed on its own.** Everything
else in a recovery is motion or teardown. `reroute_decide` brackets the local
re-decision alone, and `messages_during` counts what crossed it — expected to be
nil, and reported as a number rather than asserted in a comment.
`messages_teardown` counts the two tells that happen *outside* it (the arm being
left behind is told the handover is off; the arm being handed to instead is told
which arm is out) so that the headline claim is not flattered by hiding them.

**The arms are compared on `discovery + delivery`, not on wall-clock.** The
end-to-end number contains the operator's own pause between the route settling
and `!deliverCube`, and in the kill arm that pause is where the AgentThing is
stopped, so it is longer on purpose. It is kept in the data as the raw fact and
is never the comparison.

## Running it

All commands from the WoMAT repository root. The production five-robot
manifests and scene are used unchanged; nothing is regenerated. Start the scene,
the shared lower server and **all five** AgentThings as usual — including
Robot4's, in both arms.

```bash
npm run eval:e13
```

This instruments copies of `routePlans.asl`, `handoverPlans.asl` and the five
agent files under `performance_log/e13/plans/`, writes `e13_cold.jcm` and
`e13_kill.jcm`, parse-checks everything with Jason's own parser, and waits. The
production plans are never touched. If a plan has been reworded so that an
instrumentation anchor no longer matches, it stops here naming the anchor rather
than recording a run with a missing boundary.

Runs 1–5 are the cold arm and runs 6–10 the kill arm; the slot decides, so the
two arms cannot drift apart in count. `npm run eval:e13:status` says what is
still owed, which slots are free, and which trials have been discarded.

A trial that was aborted — Gradle stopped before the cube was placed, a launch
was cancelled, the cell was restarted by hand — is recorded as `incomplete` and
still holds its slot, and because the arm follows the slot number that costs a
measurement of that arm. Free the slot without losing the trace:

```bash
node src/eval/e13Rerouting.js discard --run 4          # or --run 1,2,4
```

The whole directory moves to `performance_log/e13/discarded/` and the slot
becomes available again. The next `launch` takes the lowest free slot, so the
repeat lands in the same arm as the run it replaces. A run whose status is `ok`
is refused unless `--force` is given, so a typo cannot throw away a measurement.

### A cold run (1–5)

```bash
npm run eval:e13:launch     # starts Gradle with the four-agent MAS
```

Wait for `Route settled:`. Along the way each arm that has Robot4 as a
neighbour prints `Cannot put the question to robot4_agent`, which is the cold
arm's intervention showing up in the transcript. The search settles on a complete
tally, not on its deadline — an absent agent is refused instantly rather than
waited for. Then:

```bash
npm run eval:e13:deliver
```

### A kill run (6–10)

```bash
npm run eval:e13:launch     # starts Gradle with all five agents
```

Wait for `Route settled: hand to robot2 ... via [robot2,robot4,robot5]`. The
cheap route has to be the one that settles, or the run has measured the cold
arm's question under the kill arm's label — which `summarize` reports as an issue
rather than averaging in.

Now stop Robot4's AgentThing yourself (`Ctrl-C` in its terminal), and record
that you did:

```bash
npm run eval:e13:killed     # writes a timestamp; it signals nothing
npm run eval:e13:deliver
```

`killed` checks the timestamp against the route decision and warns if no route
has settled yet. Leave the shared lower server alone — it serves all five robots
on separate ports, so stopping it would take the whole cell down rather than one
arm.

Whether Robot4's AgentThing was still up is recorded per run as
`failed_robot_priced_by`: with it up, the other arms still build the shared space
with Robot4 and still price the edge to it, so the cost table has both routes in
full. With it down they each pay an estimator timeout in startup instead and the
Robot4 column comes out blank. Keeping it up in the cold arm is therefore worth
the terminal.

Watch the transfer. Robot2 will approach the Robot4 rendezvous, wait, judge the
partner unfit, strike the hop, come back upright with the cube and hand it to
Robot3 instead. When the recorder prints the run's result, stop that Gradle
process before launching the next run.

## Outputs

Written to `performance_log/e13/` and rewritten after every run:

| File | Contents |
|---|---|
| `e13_runs.csv` | one row per run: arm, discovery, how it converged, which arms could not be asked, planned and executed route with both costs, the penalty, discarded motion, delivery |
| `e13_reroutes.csv` | one row per reroute: where, which arm failed, how it was detected, detect/decide/reset/recover, both message counts, the metres given up |
| `e13_handovers.csv` | every handover interval including the abandoned one, with its sub-phases |
| `e13_routes.csv` | both routes as the cell priced them, flagged `planned` and `executed` |
| `e13_legs.csv` | every priced edge and every edge the pricing refused |
| `e13_summary.csv` | mean and sample sd **per arm**, because every headline number here is a comparison |
| `e13_arm_table.tex` | the two-column comparison |
| `e13_reroute_table.tex` | the failure broken into the part that is free and the parts that are not |
| `e13_cost_table.tex` | both routes decomposed by Eq. (route_cost), with the executed column marked per arm |
| `e13_figure_budget.tex` | where each arm's time goes, as two stacked bars |
| `e13_recovery_table.tex` | the two postural recoveries of the last hop, grouped by outcome |
| `e13_figure_delivery.tex` | every run's delivery decomposed, in the order they were flown |

`resummarize` rebuilds every `result.json` from the stored traces, so a
correction to the analysis applies to runs already recorded instead of needing
the cell run again. The same happens by itself when the recorder meets a result
written before a field was added: `result.json` carries the schema it was written
against, and a result from an older version is rebuilt from its own trace before
the tables are written. A result whose trace has been deleted is named and left
out of the analysis rather than stopping it.

A trial is written, and its line printed, before the experiment-wide tables are
rebuilt, and a failure in that rebuild is reported without discarding the run or
stopping the recorder. The run costs minutes of simulation and an operator's
attention; the tables can be rebuilt at any time with
`npm run eval:e13:analyze`.

## What the ten runs showed

All ten placed the cube, all ten executed `Robot1-Robot2-Robot3-Robot5`.

- **Discovery.** cold `0.15 ± 0.00` s over 22 messages, one route reported; kill
  `4.36 ± 0.11` s over 41, two reported. Neither arm ended on a deadline in any
  run. Knowing in advance turns out to be cheaper in discovery too, by about
  thirty times, because an arm that cannot be asked is refused instantly.
- **Reroute.** 5/5 in the kill arm, detected by explicit refusal at
  `3.21 ± 0.02` s — never by the 120 s readiness deadline. Re-decided in
  `1.34 ± 0.43` ms with no query. Penalty `0.18676 ± 0.00000` m, which is
  exactly E12's margin between the two routes, reproduced during execution.
  Discarded motion `8.50 ± 0.08` s.
- **End to end**, over the seven runs that placed the cube directly: delivery
  `72.80 ± 2.95` s (cold) against `79.53 ± 0.84` s (kill); discovery plus
  delivery `72.94 ± 2.95` s against `83.93 ± 0.82` s. Failing after the decision
  costs about 11 s more than failing before it.
- **The tail is not the reroute.** Three runs exceeded 100 s, two of them in the
  cold arm, and all three are long in the placement phase alone: 7 runs placed
  directly in `10.2 ± 0.7` s, two needed one recovery (`70.4 ± 5.6` s) and one
  needed two (`95.7` s). In those three, Robot3's home motion inflates from
  `7.19 ± 0.18` s to `73.03 ± 14.63` s and overlaps the placement almost exactly
  — the retreating giver and the placing receiver each wait for a region the
  other holds, and the placement reset is what breaks it.

### One defect these runs found

When Robot4's layer stops, `-!receiveCube` tells the giver and then calls
`setHandoverApproach` on the layer that has just died. The call fails, so the
handler never reaches its `.abolish(handoverSession(...))`, and the arm retries
several hundred times a second — which is the whole of the kill arm's
`276.4 ± 0.9` messages against the cold arm's `49.4 ± 0.9`. The `cannotServe`
backstop in `routePlans.asl` bounds it, and fired in all five kill runs at
exactly 200 dropped calls:

```
[Robot4] lower layer still not connected; 200 request(s) dropped so far, by request: {setHandoverApproach=200}
```

The transfer is unaffected — the kill arm's handover durations match the cold
arm's — but the ordering inside that handler is worth fixing: the tidy-up
through the layer belongs after the session has been abolished, not before.

## What the numbers will and will not support

The claim E13 supports is that the recovery is **local, free, and optimal among
the routes still open to the arm holding the cube**. It re-decides from answers
it already holds, so the decision costs no messages and takes milliseconds; the
replacement is the cheapest of what is left; and the executed cost is exactly
the number the cell had already quoted for that route.

It is not a claim of a new global optimum. The cube is in Robot2's gripper by
the time anything is known, so a cheaper Robot4-free route through some other
part of the cell would be out of reach even if one existed. And the local strike
is sufficient rather than merely convenient: the answers an arm collected come
only from the arms it asked, so the arm that refused cannot reappear on the
replacement route — which `routeSearch.test.js` checks over 400 random
topologies, and is why no message has to be sent to make the recovery safe.
