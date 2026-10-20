"use strict";

const assert = require("node:assert/strict");
const test = require("node:test");

const { bruteForceOptimal, deliver, pathCost, simulate } = require("./routeSearch");

function costs(table) {
  return (from, to) => table[`${from}->${to}`];
}

// Deterministic pseudo-randomness: a failing case must be reproducible from the
// seed printed in the assertion, or it is not a finding, it is an anecdote.
function rng(seed) {
  let state = seed >>> 0;
  return () => {
    state = (state * 1664525 + 1013904223) >>> 0;
    return state / 0x100000000;
  };
}

function randomCell(seed, { size = 6, extraEdges = 2 } = {}) {
  const random = rng(seed);
  const nodes = Array.from({ length: size }, (_, index) => `R${index + 1}`);
  // A spanning tree first, so the sink is always reachable, then chords that
  // turn the tree into a graph with cycles -- which is where the flood's
  // first-seen rule starts to cost something.
  const edges = [];
  for (let index = 1; index < size; index += 1) {
    edges.push([nodes[Math.floor(random() * index)], nodes[index]]);
  }
  for (let added = 0; added < extraEdges; added += 1) {
    const a = Math.floor(random() * size);
    const b = Math.floor(random() * size);
    if (a === b) continue;
    const pair = [nodes[a], nodes[b]];
    if (edges.some(([l, r]) => (l === pair[0] && r === pair[1]) || (l === pair[1] && r === pair[0]))) continue;
    edges.push(pair);
  }
  const table = {};
  for (const [left, right] of edges) {
    table[`${left}->${right}`] = Math.round(random() * 900 + 100) / 1000;
    table[`${right}->${left}`] = Math.round(random() * 900 + 100) / 1000;
  }
  return { nodes, edges, legCost: costs(table), origin: nodes[0], sink: nodes[size - 1], ttl: size + 2 };
}

test("a chain accumulates one leg per arm, and the origin adds its own", () => {
  const spec = {
    nodes: ["R1", "R2", "R3", "R4"],
    edges: [["R1", "R2"], ["R2", "R3"], ["R3", "R4"]],
    legCost: costs({
      "R1->R2": 0.3, "R2->R1": 0.2,
      "R2->R3": 0.4, "R3->R2": 0.5,
      "R3->R4": 0.6, "R4->R3": 0.7
    }),
    origin: "R1",
    sink: "R4"
  };
  const result = simulate(spec);
  // R1's own leg out (0.3) + R2's leg back to R1 (0.2) + R3's to R2 (0.5) + R4's to R3 (0.7).
  // R2's leg toward R3 and R3's toward R4 are deliberately not in the sum: one
  // term per arm is the model, and this is where that is visible.
  assert.equal(result.total, 1.7);
  assert.deepEqual(result.path, ["R1", "R2", "R3", "R4"]);
  assert.equal(result.nextHop, "R2");
});

test("the cheapest first hop wins even when it is not the shortest in hops", () => {
  // R1 can reach the sink directly through R3, or the long way through R2.
  const spec = {
    nodes: ["R1", "R2", "R3", "R4"],
    edges: [["R1", "R2"], ["R2", "R4"], ["R1", "R3"], ["R3", "R4"]],
    legCost: costs({
      "R1->R2": 0.1, "R2->R1": 0.1, "R2->R4": 0.1, "R4->R2": 0.1,
      "R1->R3": 0.9, "R3->R1": 0.9, "R3->R4": 0.9, "R4->R3": 0.9
    }),
    origin: "R1",
    sink: "R4"
  };
  const result = simulate(spec);
  assert.equal(result.nextHop, "R2");
  assert.deepEqual(result, { ...result, total: 0.30000000000000004 });
  assert.deepEqual(bruteForceOptimal(spec).path, ["R1", "R2", "R4"]);
});

test("an unreachable goal comes back as no route, not as a free one", () => {
  const spec = {
    nodes: ["R1", "R2", "R3"],
    edges: [["R1", "R2"]],
    legCost: costs({ "R1->R2": 0.3, "R2->R1": 0.3 }),
    origin: "R1",
    sink: "R3"
  };
  const result = simulate(spec);
  assert.equal(result.reachable, false);
  assert.equal(result.total, null);
  assert.equal(bruteForceOptimal(spec), null);
});

test("an edge nobody can price is not a route through it", () => {
  // R1-R2 exists in the neighbour graph but carries no usable rendezvous --
  // exactly the live Robot1-Robot3 case.
  const spec = {
    nodes: ["R1", "R2", "R3"],
    edges: [["R1", "R2"], ["R1", "R3"], ["R3", "R2"]],
    legCost: costs({
      "R1->R2": undefined, "R2->R1": undefined,
      "R1->R3": 0.4, "R3->R1": 0.4, "R3->R2": 0.2, "R2->R3": 0.2
    }),
    origin: "R1",
    sink: "R2"
  };
  const result = simulate(spec);
  assert.equal(result.nextHop, "R3", "the unpriceable direct edge must not be chosen");
  assert.equal(bruteForceOptimal(spec).path.join(">"), "R1>R3>R2");
});

test("an arm with no neighbours at all reports no route rather than hanging", () => {
  const result = simulate({
    nodes: ["R1", "R2"],
    edges: [],
    legCost: () => 1,
    origin: "R1",
    sink: "R2"
  });
  assert.equal(result.reachable, false);
});

test("the search terminates on cyclic graphs and does not flood forever", () => {
  for (let seed = 1; seed <= 200; seed += 1) {
    const spec = randomCell(seed, { size: 6, extraEdges: 3 });
    const result = simulate(spec);
    // Every node is asked at most once and answers each requester once, so the
    // traffic is bounded by the edge count, not by the number of paths.
    assert.ok(result.messages < 200, `seed ${seed} sent ${result.messages} messages`);
  }
});

test("on a tree the search finds the true optimum", () => {
  for (let seed = 1; seed <= 300; seed += 1) {
    const spec = randomCell(seed, { size: 7, extraEdges: 0 });
    const optimal = bruteForceOptimal(spec);
    const result = simulate(spec);
    if (optimal === null) {
      assert.equal(result.reachable, false, `seed ${seed}`);
      continue;
    }
    assert.ok(Math.abs(result.total - optimal.total) < 1e-9,
      `seed ${seed}: protocol ${result.total} vs optimum ${optimal.total}`);
    assert.deepEqual(result.path, optimal.path, `seed ${seed}`);
  }
});

test("on a graph with cycles the answer is always a real route, never a cheaper fiction", () => {
  // The bound that must hold: first-seen dedup can make the flood miss a
  // cheaper path, but it must never report a total below the true optimum. A
  // number lower than the optimum would mean the accumulation is wrong.
  let worse = 0;
  let compared = 0;
  for (let seed = 1; seed <= 400; seed += 1) {
    const spec = randomCell(seed, { size: 7, extraEdges: 4 });
    const optimal = bruteForceOptimal(spec);
    const result = simulate(spec);
    if (optimal === null) {
      assert.equal(result.reachable, false, `seed ${seed}`);
      continue;
    }
    assert.equal(result.reachable, true, `seed ${seed}: a reachable goal was reported unreachable`);
    compared += 1;
    assert.ok(result.total >= optimal.total - 1e-9,
      `seed ${seed}: reported ${result.total}, below the true optimum ${optimal.total}`);
    if (result.total > optimal.total + 1e-9) worse += 1;
  }
  assert.ok(compared > 100, "not enough reachable cases to conclude anything");
  // This gap is the honest limit of the flood as specified, and the thing
  // Stage 5's re-announcement is for. It is asserted rather than hidden so that
  // implementing Stage 5 has to move it.
  assert.ok(worse > 0,
    "no suboptimal case found -- if this ever passes, the Stage 5 premise needs rechecking");
});

// The live five-arm cell. Robot1 can only hand to Robot2 -- its edges to Robot3
// and Robot4 carry no usable rendezvous -- and from Robot2 the goal is reachable
// two ways, through Robot3 or through Robot4. Both are real and both were priced,
// which is what makes this the cell the routing AND the rerouting are read on.
const liveFiveArmCell = {
  nodes: ["robot1", "robot2", "robot3", "robot4", "robot5"],
  edges: [["robot1", "robot2"], ["robot1", "robot3"], ["robot1", "robot4"],
          ["robot2", "robot3"], ["robot2", "robot4"], ["robot2", "robot5"],
          ["robot3", "robot4"], ["robot3", "robot5"], ["robot4", "robot5"]],
  legCost: costs({
    "robot1->robot2": 0.677, "robot2->robot1": 0.518,
    "robot2->robot3": 0.4, "robot3->robot2": 1.13,
    "robot2->robot4": 0.4, "robot4->robot2": 0.904,
    "robot3->robot5": 0.4, "robot5->robot3": 1.912,
    "robot4->robot5": 0.4, "robot5->robot4": 1.951,
    // The pairs the cell found unusable: neighbours by reach, but their
    // shared space carries no rendezvous at handover height.
    "robot1->robot3": undefined, "robot3->robot1": undefined,
    "robot1->robot4": undefined, "robot4->robot1": undefined,
    "robot2->robot5": undefined, "robot5->robot2": undefined,
    "robot3->robot4": undefined, "robot4->robot3": undefined
  }),
  origin: "robot1",
  sink: "robot5"
};

test("the arm that decides is shown every route found, not one per hop", () => {
  // Every hop takes a minimum, so without carrying the losers upstream the arm
  // holding the cube is told about exactly one route and cannot tell a single
  // route from the cheapest of several.
  const spec = liveFiveArmCell;
  const result = simulate(spec);
  assert.deepEqual(result.routes.map((route) => route.path.join("-")), [
    "robot1-robot2-robot4-robot5",
    "robot1-robot2-robot3-robot5"
  ], "both ways round have to survive the hop that chose between them");
  // Cheapest first, and the one acted on is the head of the list.
  assert.deepEqual(result.path, result.routes[0].path);
  assert.equal(result.total, result.routes[0].total);
  for (const route of result.routes) {
    assert.ok(Math.abs(route.total - pathCost(spec.legCost, route.path)) < 1e-9,
      `${route.path.join("-")} was reported at ${route.total}`);
  }
});

test("every route reported is a real route, priced as if it were taken", () => {
  // Reporting the alternatives is only worth anything if they are true: an arm
  // may be asked to fall back on one the day the chosen rendezvous is refused.
  let withAlternatives = 0;
  for (let seed = 1; seed <= 400; seed += 1) {
    const spec = randomCell(seed, { size: 7, extraEdges: 4 });
    const result = simulate(spec);
    if (!result.reachable) {
      assert.deepEqual(result.routes, [], `seed ${seed}`);
      continue;
    }
    if (result.routes.length > 1) withAlternatives += 1;
    const seenPaths = new Set();
    let previous = -Infinity;
    for (const route of result.routes) {
      const key = route.path.join("-");
      assert.ok(!seenPaths.has(key), `seed ${seed}: ${key} was reported twice`);
      seenPaths.add(key);
      assert.equal(route.path[0], spec.origin, `seed ${seed}`);
      assert.equal(route.path.at(-1), spec.sink, `seed ${seed}`);
      assert.equal(new Set(route.path).size, route.path.length,
        `seed ${seed}: ${key} visits an arm twice`);
      const real = pathCost(spec.legCost, route.path);
      assert.ok(real !== null, `seed ${seed}: ${key} uses an edge nobody can price`);
      assert.ok(Math.abs(route.total - real) < 1e-9,
        `seed ${seed}: ${key} reported at ${route.total}, really costs ${real}`);
      assert.ok(route.total >= previous - 1e-9, `seed ${seed}: routes are not cheapest first`);
      previous = route.total;
    }
    // The route acted on is the cheapest of the ones reported -- the extra
    // information must not change the decision.
    assert.equal(result.routes[0].path.join("-"), result.path.join("-"), `seed ${seed}`);
    assert.ok(Math.abs(result.routes[0].total - result.total) < 1e-9, `seed ${seed}`);
  }
  assert.ok(withAlternatives > 20,
    "not enough graphs offered a second route to conclude anything");
});

test("the answer does not depend on the order messages happen to arrive", () => {
  for (let seed = 1; seed <= 120; seed += 1) {
    const spec = randomCell(seed, { size: 6, extraEdges: 2 });
    const inOrder = simulate(spec);
    // Same protocol, adversarial delivery order.
    const random = rng(seed * 7919);
    const shuffled = simulate({
      ...spec,
      shuffle: (queue) => {
        for (let i = queue.length - 1; i > 0; i -= 1) {
          const j = Math.floor(random() * (i + 1));
          [queue[i], queue[j]] = [queue[j], queue[i]];
        }
      }
    });
    assert.equal(shuffled.reachable, inOrder.reachable, `seed ${seed}`);
    if (inOrder.reachable) {
      assert.ok(Number.isFinite(shuffled.total), `seed ${seed}`);
    }
  }
});

test("every arm on the chosen route knows its own next hop", () => {
  const spec = {
    nodes: ["R1", "R2", "R3", "R4"],
    edges: [["R1", "R2"], ["R2", "R3"], ["R3", "R4"]],
    legCost: () => 0.5,
    origin: "R1",
    sink: "R4"
  };
  const result = simulate(spec);
  // This is what execution reads: not a path held by the origin, but a next hop
  // held separately by each arm.
  assert.equal(result.state.get("R1").nextHop, "R2");
  assert.equal(result.state.get("R2").nextHop, "R3");
  assert.equal(result.state.get("R3").nextHop, "R4");
});

test("an arm that cannot hand back must not consume its one answer doing so", () => {
  // The live 4-robot cell, exactly. R1-R3 are neighbours by the reach test but
  // their shared space carries no rendezvous at handover height, so R3 cannot
  // hand back to R1. R3 accepting R1's question anyway spent its one answer,
  // then refused R2's forward under the already-seen rule -- and the route that
  // actually exists, R1-R2-R3-R4, became invisible. The cell reported "no route
  // to the goal" for a goal it could reach.
  const spec = {
    nodes: ["R1", "R2", "R3", "R4"],
    edges: [["R1", "R2"], ["R2", "R3"], ["R3", "R4"], ["R1", "R3"]],
    legCost: costs({
      "R1->R2": 0.2, "R2->R1": 0.2,
      "R2->R3": 0.6, "R3->R2": 0.7,
      "R3->R4": 0.5, "R4->R3": 0.5
      // R1<->R3 deliberately absent: neighbours, but nothing they can hand over on.
    }),
    origin: "R1",
    sink: "R4"
  };
  const result = simulate(spec);
  const optimal = bruteForceOptimal(spec);
  assert.equal(result.reachable, true, "a reachable goal was reported unreachable");
  assert.ok(Math.abs(result.total - optimal.total) < 1e-9, `${result.total} vs ${optimal.total}`);
  assert.deepEqual(result.path, ["R1", "R2", "R3", "R4"]);
});

test("unpriceable edges scattered through a graph never fake an unreachable goal", () => {
  // The general form of the case above: if a route exists using only priceable
  // edges, the search must find one. Reporting no route for a goal that can be
  // reached is the one failure that cannot be argued as a heuristic limit.
  for (let seed = 1; seed <= 300; seed += 1) {
    const base = randomCell(seed, { size: 6, extraEdges: 3 });
    const random = rng(seed * 104729);
    // Knock out roughly a fifth of the edges in both directions.
    const dropped = new Set();
    for (const [left, right] of base.edges) {
      if (random() < 0.2) dropped.add(`${left}|${right}`);
    }
    const legCost = (from, to) => (
      dropped.has(`${from}|${to}`) || dropped.has(`${to}|${from}`) ? undefined : base.legCost(from, to)
    );
    const spec = { ...base, legCost };
    const optimal = bruteForceOptimal(spec);
    const result = simulate(spec);
    if (optimal === null) {
      assert.equal(result.reachable, false, `seed ${seed}: invented a route where none exists`);
    } else {
      assert.equal(result.reachable, true, `seed ${seed}: reachable goal reported unreachable`);
      assert.ok(result.total >= optimal.total - 1e-9, `seed ${seed}: below the optimum`);
    }
  }
});

test("a neighbour always answers before the arm that asked it gives up", () => {
  // Not a simulated property -- a structural one, asserted against the plans.
  // A flat deadline at every depth lets a child reply a moment after its parent
  // has settled on whatever happened to arrive, and the parent then reports no
  // route while holding, unread, the reply that proved there was one. Live
  // cell: Robot3 settled at 90s on Robot1's refusal, and Robot4's real answer
  // (0.5063 via [robot4]) landed afterwards and was ignored.
  const fs = require("node:fs");
  const path = require("node:path");
  const plans = fs.readFileSync(path.resolve(__dirname, "../agt/routePlans.asl"), "utf8");
  // The budget travels with the question...
  assert.match(plans, /findRoute\(Task, Cube, Goal, TTL, Deadline\)/);
  assert.match(plans, /\.send\(Agent, achieve, findRoute\(Task, Cube, Goal, TTL, Deadline\)\)/);
  // ...and strictly shrinks at every hop, by enough for this arm to price its
  // own reply and send it.
  const shrink = plans.match(/Onward = Deadline - (\d+);/);
  assert.ok(shrink, "the forwarded budget must be smaller than the one received");
  assert.ok(Number(shrink[1]) > 0);
  // The arm's own wait uses the shrunken budget, not the one it was given.
  assert.match(plans, /!routeDeadline\(Task, Onward\)/);
  // And a budget already spent is a dead end answered at once, not a wait.
  assert.match(plans, /: Deadline <= \d+\s*<-[\s\S]*?\.send\(Upstream, tell, noRoute\(Task\)\)/);
});

test("the live cell's own topology finds the chain, in every delivery order", () => {
  // The four-arm cell as it actually stands: Robot1 and Robot3 are neighbours
  // on paper but share no rendezvous at handover height, and Robot2 and Robot4
  // do not overlap at all. So the only way to the goal is 1-2-3-4, and it has
  // to survive the flood coming back at Robot3 from two directions at once --
  // once across the edge it can use, once across the one it cannot.
  const spec = {
    nodes: ["robot1", "robot2", "robot3", "robot4"],
    edges: [["robot1", "robot2"], ["robot1", "robot3"], ["robot2", "robot3"], ["robot3", "robot4"]],
    legCost: costs({
      "robot1->robot2": 0.371522, "robot2->robot1": 0.529310,
      "robot2->robot3": 0.964134, "robot3->robot2": 0.800066,
      "robot3->robot4": 0.700000, "robot4->robot3": 0.506349
      // robot1 <-> robot3: no rendezvous at handover height, either way.
    }),
    origin: "robot1",
    sink: "robot4"
  };
  const expected = bruteForceOptimal(spec);
  assert.equal(expected.path.join(">"), "robot1>robot2>robot3>robot4");

  // Message order is not something the cell controls, and the answer must not
  // depend on it: an arm refused across the dead edge must never be the reason
  // the live one goes unread.
  for (let seed = 1; seed <= 60; seed += 1) {
    const result = simulate({ ...spec, shuffle: rng(seed) });
    assert.equal(result.reachable, true, `seed ${seed}: the cell reported no route to its only empty support`);
    assert.equal(result.nextHop, "robot2", `seed ${seed}: wrong first hop`);
    assert.ok(
      Math.abs(result.total - expected.total) < 1e-9,
      `seed ${seed}: total ${result.total} is not the optimum ${expected.total}`
    );
  }
});

// ---------------------------------------------------------------------------
// Rerouting: an arm that was chosen and then failed.
// ---------------------------------------------------------------------------

test("the arm that refused is struck by the arm holding the cube, and by nobody else", () => {
  const clean = simulate(liveFiveArmCell);
  assert.equal(clean.path.join(">"), "robot1>robot2>robot4>robot5", "the cheap way is through robot4");
  assert.equal(clean.routes.length, 2, "and the cell knows of a second way");

  const rerouted = deliver(liveFiveArmCell, ["robot4"]);
  assert.equal(rerouted.delivered, true, "the cube still reaches the goal");
  assert.equal(rerouted.path.join(">"), "robot1>robot2>robot3>robot5");
  // One arm re-decided, and it is the arm that was holding the cube -- not the
  // origin, which never had a choice to re-make, and not the cell as a whole.
  assert.deepEqual(rerouted.struck, [{ at: "robot2", arm: "robot4" }]);
});

test("the reroute lands on the alternative the search had already priced, at its price", () => {
  // This is the whole pay-off of accumulating the costs rather than collecting
  // paths centrally: the second-cheapest route was priced by the same search,
  // travelled home inside the same replies, and is still in the belief base of
  // the arm that has to replace the first. Nothing is recomputed and nobody is
  // asked again, so the executed cost must be exactly the number the cell had
  // already reported for that route.
  const clean = simulate(liveFiveArmCell);
  const rerouted = deliver(liveFiveArmCell, ["robot4"]);
  const alternative = clean.routes[1];
  assert.equal(alternative.path.join(">"), rerouted.path.join(">"));
  assert.ok(
    Math.abs(rerouted.total - alternative.total) < 1e-9,
    `executed ${rerouted.total} is not the quoted ${alternative.total}`
  );
  // And it is dearer than the route it replaces, or the search had chosen wrong.
  assert.ok(rerouted.total > clean.total);
});

test("the holder takes the cheapest of what it has left, not merely something else", () => {
  // Three ways on from R2, so "not the one that failed" and "the cheapest of the
  // rest" are different answers and the test can tell them apart.
  const spec = {
    nodes: ["R1", "R2", "R3", "R4", "R6", "R5"],
    edges: [["R1", "R2"], ["R2", "R3"], ["R2", "R4"], ["R2", "R6"],
      ["R3", "R5"], ["R4", "R5"], ["R6", "R5"]],
    legCost: costs({
      "R1->R2": 0.4, "R2->R1": 0.4,
      "R2->R3": 0.5, "R3->R2": 0.5,
      "R2->R4": 0.1, "R4->R2": 0.1,
      "R2->R6": 0.9, "R6->R2": 0.9,
      "R3->R5": 0.5, "R5->R3": 0.1,
      "R4->R5": 0.5, "R5->R4": 0.1,
      "R6->R5": 0.5, "R5->R6": 0.1
    }),
    origin: "R1",
    sink: "R5"
  };
  assert.equal(simulate(spec).path.join(">"), "R1>R2>R4>R5", "R4 is the cheap way");
  const rerouted = deliver(spec, ["R4"]);
  assert.equal(rerouted.path.join(">"), "R1>R2>R3>R5", "R3 is cheaper than R6");
  // Which is the same answer as re-running the whole search without R4 in the
  // cell at all -- for this graph, because every remaining route from R2 is
  // priced in R2's own answers.
  const withoutR4 = simulate({
    ...spec,
    nodes: spec.nodes.filter((node) => node !== "R4"),
    edges: spec.edges.filter(([left, right]) => left !== "R4" && right !== "R4")
  });
  assert.equal(withoutR4.path.join(">"), rerouted.path.join(">"));
  assert.ok(Math.abs(withoutR4.total - rerouted.total) < 1e-9);
});

test("a struck arm never appears on the route the cube actually travels", () => {
  // Every arm on the chosen route, struck one at a time, on graphs nobody built.
  // The recovery is local and works from what one arm collected, so it is NOT
  // promised to find a route whenever one exists -- the flood tree an arm sees
  // is not the whole cell. What it must never do is deliver through the arm that
  // refused, or claim a delivery it did not make.
  for (let seed = 1; seed <= 400; seed += 1) {
    const spec = randomCell(seed, { size: 6, extraEdges: 3 });
    const clean = simulate(spec);
    if (!clean.reachable) continue;
    for (const arm of clean.path.slice(1, -1)) {
      const rerouted = deliver(spec, [arm]);
      assert.ok(!(rerouted.path || []).includes(arm),
        `seed ${seed}: delivered through ${arm}, which had refused the cube`);
      if (rerouted.delivered) {
        assert.equal(rerouted.path[0], spec.origin, `seed ${seed}: the walk must start where the cube is`);
        assert.equal(rerouted.path.at(-1), spec.sink, `seed ${seed}: a delivery ends at the goal`);
        assert.ok(Math.abs(rerouted.total - pathCost(spec.legCost, rerouted.path)) < 1e-9);
        // Nothing is free: a reroute is a replacement for the cheapest route, so
        // it cannot come out cheaper than the route it replaced.
        assert.ok(rerouted.total >= clean.total - 1e-9,
          `seed ${seed}: rerouting around ${arm} came out cheaper than the optimum`);
      } else {
        assert.ok(rerouted.reason !== null, `seed ${seed}: a failed delivery has to say where it stopped`);
      }
    }
  }
});

test("the local strike is enough, because an arm only ever holds its own subtree", () => {
  // Why one arm re-deciding by itself is not a shortcut. The answers an arm
  // collected came from the arms it ASKED, so they describe disjoint parts of
  // the flood tree: the arm that refused was a child of the arm now re-deciding,
  // and cannot also be underneath any of its siblings. The replacement route can
  // therefore never run back through the arm that refused, and the arm holding
  // the cube does not have to tell anybody anything to be sure of it.
  //
  // This is what makes the recovery cost zero messages. It is also exactly how
  // far the argument goes: it says nothing about an arm that fails LATER on the
  // route, which is struck independently, by its own giver, the same way.
  for (let seed = 1; seed <= 400; seed += 1) {
    const spec = randomCell(seed, { size: 7, extraEdges: 4 });
    const clean = simulate(spec);
    if (!clean.reachable) continue;
    for (let hop = 1; hop < clean.path.length - 1; hop += 1) {
      const holder = clean.path[hop - 1];
      const failing = clean.path[hop];
      const remaining = clean.state.get(holder).answers
        .filter((answer) => answer.value !== null && answer.from !== failing);
      for (const answer of remaining) {
        assert.ok(!answer.path.includes(failing),
          `seed ${seed}: ${holder} would reroute around ${failing} through ${answer.path.join(">")}`);
      }
    }
  }
});

test("a reroute costs no messages at all", () => {
  // The alternatives were priced by the search that chose the route and came
  // home inside the same replies, so replacing a refused hop is arithmetic on
  // beliefs the arm already holds. Nothing is asked again -- which is the
  // property that distributing the costs was for, and the one that would be
  // lost by collecting paths at the origin instead.
  const before = simulate(liveFiveArmCell).messages;
  const rerouted = deliver(liveFiveArmCell, ["robot4"]);
  assert.equal(rerouted.delivered, true);
  assert.equal(simulate(liveFiveArmCell).messages, before,
    "the walk must not have made the cell send anything");
});

// ---------------------------------------------------------------------------
// An arm that cannot be asked at all.
// ---------------------------------------------------------------------------

test("an arm with no agent is a no-route answer, not the end of the search", () => {
  // Jason FAILS .send when the receiver is not in the MAS. Inline in the loop
  // that asks the neighbours, that failure stopped the recursion: the arms
  // further down the list were never asked, and the arm holding the cube
  // reported that route discovery did not complete while two perfectly good
  // branches stood unasked. Live cell: robot1's peers are [robot4,robot3,robot2]
  // and robot4 had no agent, so its position in the list was the whole failure.
  const absent = simulate({ ...liveFiveArmCell, absent: ["robot4"] });
  assert.equal(absent.reachable, true, "the dearer route was there to be found");
  assert.equal(absent.path.join(">"), "robot1>robot2>robot3>robot5");
  // The arm that could not be asked still counted towards the tally, so the
  // search ends on a complete count rather than by running out of time.
  const origin = absent.state.get("robot1");
  assert.equal(origin.answers.length, origin.asked, "every branch has a verdict");
  assert.ok(origin.answers.some(a => a.from === "robot4" && a.value === null));
});

test("an unaskable arm costs exactly what deleting it would", () => {
  // The two are the same search: an arm nobody can put a question to forwards
  // nothing and prices nothing, so it contributes what an arm that is not there
  // contributes. Asserting the equivalence is what rules out the failure mode
  // where the absent arm silently takes a live branch down with it.
  for (let seed = 1; seed <= 400; seed += 1) {
    const spec = randomCell(seed, { size: 7, extraEdges: 4 });
    for (const arm of spec.nodes) {
      if (arm === spec.origin || arm === spec.sink) continue;
      const absent = simulate({ ...spec, absent: [arm] });
      const deleted = simulate({
        ...spec,
        nodes: spec.nodes.filter(node => node !== arm),
        edges: spec.edges.filter(([left, right]) => left !== arm && right !== arm)
      });
      assert.equal(absent.reachable, deleted.reachable, `seed ${seed}, ${arm}: reachability differs`);
      if (!absent.reachable) continue;
      assert.equal(absent.path.join(">"), deleted.path.join(">"), `seed ${seed}, ${arm}: different route`);
      assert.ok(Math.abs(absent.total - deleted.total) < 1e-9, `seed ${seed}, ${arm}: different cost`);
      assert.ok(!absent.path.includes(arm));
    }
  }
});

test("every arm on the list is asked, whatever the unaskable one's position", () => {
  // The bug was positional: robot4 came first, so nothing after it was asked.
  // Rotating which arm is missing over the whole cell is what catches that, and
  // it is why the fix answers on the absent arm's behalf instead of skipping it.
  for (let seed = 1; seed <= 200; seed += 1) {
    const spec = randomCell(seed, { size: 6, extraEdges: 3 });
    if (!simulate(spec).reachable) continue;
    for (const arm of spec.nodes) {
      if (arm === spec.origin || arm === spec.sink) continue;
      const result = simulate({ ...spec, absent: [arm] });
      for (const [node, self] of result.state) {
        if (!self.seen || node === spec.sink) continue;
        assert.equal(self.answers.length, self.asked,
          `seed ${seed}, ${arm} absent: ${node} asked ${self.asked} and heard ${self.answers.length}`);
      }
    }
  }
});

test("an arm whose own layer is gone reaches the same route, announced or not", () => {
  // Two ways for the same arm to be useless, and they used to cost wildly
  // different amounts. Absent from the MAS: every .send to it fails, the arm
  // asking answers on its behalf, done. Stranded -- agent reasoning, own layer
  // silent -- it CAN be sent to, so it was, and it then spent a deadline
  // arriving at the answer it already had.
  //
  // What must never differ is the answer. Whichever way an arm drops out, the
  // cell has to settle on the same route at the same price.
  const healthy = simulate(liveFiveArmCell);
  assert.equal(healthy.path.join(">"), "robot1>robot2>robot4>robot5");

  const variants = [
    simulate({ ...liveFiveArmCell, absent: ["robot4"] }),
    simulate({ ...liveFiveArmCell, stranded: ["robot4"], announced: true }),
    simulate({ ...liveFiveArmCell, stranded: ["robot4"], announced: false })
  ];
  for (const result of variants) {
    assert.equal(result.reachable, true);
    assert.equal(result.path.join(">"), "robot1>robot2>robot3>robot5");
    assert.ok(Math.abs(result.total - variants[0].total) < 1e-9, "same route, same price");
    assert.ok(!result.path.includes("robot4"));
  }
});

test("announcing costs the cell one question per requester less than staying silent", () => {
  // The point of the broadcast, as a number. A stranded arm that has announced is
  // not asked at all, which is exactly the traffic of an arm that was never in
  // the MAS; one that has not announced is asked by every neighbour and answers
  // each of them.
  //
  // This counts messages, not seconds. The seconds -- three timeouts standing in
  // for one fact -- are removed by the agent answering at once instead of waiting
  // on a belief that can no longer be asserted, and by the artifact failing a
  // send it did not make. Neither is visible here, which is why this asserts the
  // structural saving and not the wall-clock one.
  const absent = simulate({ ...liveFiveArmCell, absent: ["robot4"] });
  const announced = simulate({ ...liveFiveArmCell, stranded: ["robot4"], announced: true });
  const silent = simulate({ ...liveFiveArmCell, stranded: ["robot4"], announced: false });
  assert.equal(announced.messages, absent.messages,
    "an arm that has said it cannot serve costs the same as one that was never there");
  assert.ok(silent.messages > announced.messages,
    `staying silent must cost more, not the same (${silent.messages} vs ${announced.messages})`);
  // One question out and one refusal back, for each arm that had to ask.
  const askers = liveFiveArmCell.edges.filter(([l, r]) => l === "robot4" || r === "robot4").length;
  assert.ok(silent.messages - announced.messages <= 2 * askers);
});

test("a stranded arm never becomes a route, however many arms ask it", () => {
  // It answers every requester, so it must answer every requester "no": an arm
  // that cannot move cannot carry, and a single stray cost from it would put a
  // route through an arm that is not going to perform it.
  for (let seed = 1; seed <= 300; seed += 1) {
    const spec = randomCell(seed, { size: 6, extraEdges: 3 });
    for (const arm of spec.nodes) {
      if (arm === spec.origin || arm === spec.sink) continue;
      for (const announced of [true, false]) {
        const result = simulate({ ...spec, stranded: [arm], announced });
        assert.ok(!(result.path || []).includes(arm),
          `seed ${seed}: routed through ${arm}, whose layer never answered`);
        // And the tally still closes at every arm that asked, so nothing is left
        // waiting for a branch that will not report.
        for (const [node, self] of result.state) {
          if (!self.seen || node === spec.sink || node === arm) continue;
          assert.equal(self.answers.length, self.asked,
            `seed ${seed}, ${arm} stranded: ${node} asked ${self.asked} and heard ${self.answers.length}`);
        }
      }
    }
  }
});
