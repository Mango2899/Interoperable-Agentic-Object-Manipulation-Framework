"use strict";

// An executable model of the route search the agents run.
//
// The protocol lives in src/agt/routePlans.asl, where it belongs: the agents
// are the contribution. But an ASL plan cannot be run against a thousand random
// topologies in a second, and "it worked in the cell" is not evidence that a
// distributed search is correct -- it is evidence about one graph. This module
// is the same protocol as a deterministic message simulator, so its answers can
// be checked against a brute-force optimum on graphs nobody had to build.
//
// It is a model, not the implementation. If the two disagree, one of them is
// wrong and the tests that compare them are how anyone finds out.

// One arm's own leg on one edge: what the handover across it costs the arm at
// `node`, measured from wherever that arm's journey starts. Null means the pair
// has no rendezvous this arm could use, which the cell produces for real
// whenever a shared space is empty at handover height.
function edgeLeg(legCost, node, neighbour) {
  const value = typeof legCost === "function" ? legCost(node, neighbour) : legCost?.[`${node}->${neighbour}`];
  return Number.isFinite(value) ? value : null;
}

function adjacency(nodes, edges) {
  const map = new Map(nodes.map((node) => [node, []]));
  for (const [left, right] of edges) {
    if (!map.has(left) || !map.has(right)) throw new Error(`edge names an unknown node: ${left}-${right}`);
    map.get(left).push(right);
    map.get(right).push(left);
  }
  // Deterministic order: a simulator whose answer depends on Map insertion
  // order cannot be compared against anything.
  for (const list of map.values()) list.sort();
  return map;
}

// The protocol, message for message.
//
// Delivery is FIFO and one message is processed at a time. Real agents run
// concurrently, so this fixes an order the cell does not guarantee -- which is
// the point: a protocol whose answer depends on delivery order is a protocol
// with a bug, and the randomised tests vary the order to look for one.
// `absent` names arms that are neighbours by reach but have no reasoning agent:
// present in the cell, discovered by everyone, and impossible to put a question
// to. Jason fails .send to an agent that is not in the MAS, so the arm doing the
// asking answers on its behalf -- which is the only way the arms further down
// its own list still get asked.
//
// `stranded` names the other half of the same problem, and the expensive half: an
// arm whose agent IS reasoning but whose own layer never answered. It can be sent
// to, so it used to be sent to, and it then spent a deadline arriving at the
// answer it already had. Modelled as a node that answers "no" immediately and
// forwards nothing -- which is what it does once it announces itself. `announced`
// false models the arm not having said so yet, and is what the message count
// shows the cost of.
function simulate({ nodes, edges, legCost, origin, sink, ttl = 8, shuffle = null,
  absent = [], stranded = [], announced = true }) {
  const unaskable = new Set(absent);
  const strandedArms = new Set(stranded);
  // An arm that has announced is not asked at all; one that has not is asked and
  // answers "no" itself, which costs one question and one reply per requester.
  if (announced) for (const arm of strandedArms) unaskable.add(arm);
  const neighbours = adjacency(nodes, edges);
  if (!neighbours.has(origin)) throw new Error(`unknown origin ${origin}`);
  if (!neighbours.has(sink)) throw new Error(`unknown sink ${sink}`);

  const state = new Map(nodes.map((node) => [node, {
    seen: false,
    requesters: [],
    asked: 0,
    answers: [],
    settled: false,
    nextHop: null,
    value: null,
    // Every route to the goal this node knows of, its own answer excluded:
    // each neighbour's verdict plus whatever that neighbour reported from
    // further out. It rides upstream inside the reply, so it can never arrive
    // after the answer that completes the tally.
    options: []
  }]));

  const queue = [];
  let delivered = 0;
  const send = (to, message) => queue.push({ ...message, to });

  // An arm that cannot be asked is counted as asked and answered "no" at once.
  // Counting it is what keeps the tally reachable: it was on the list before
  // anyone tried to reach it, so leaving it without a verdict would mean the
  // search could only ever end by running out of time.
  const ask = (from, to, hops) => {
    if (!unaskable.has(to)) return send(to, { kind: "findRoute", from, ttl: hops });
    return send(from, { kind: "noRoute", from: to });
  };

  const originState = state.get(origin);
  originState.seen = true;
  originState.asked = neighbours.get(origin).length;
  for (const neighbour of neighbours.get(origin)) ask(origin, neighbour, ttl);
  if (originState.asked === 0) originState.settled = true;

  const settle = (node) => {
    const self = state.get(node);
    if (self.settled) return;
    self.settled = true;
    const usable = self.answers.filter((answer) => answer.value !== null);

    if (node === origin) {
      // The origin's own first leg is the one term no accumulated total
      // contains, because it has nobody upstream to have reported it to.
      const price = (answer) => {
        const own = edgeLeg(legCost, origin, answer.from);
        return own === null ? null : { ...answer, total: own + answer.value };
      };
      const totals = usable.map(price).filter(Boolean);
      if (totals.length === 0) return;
      totals.sort((a, b) => a.total - b.total || a.from.localeCompare(b.from));
      self.nextHop = totals[0].from;
      self.value = totals[0].total;
      self.path = [origin, ...totals[0].path];
      // The decision is made on the neighbours' own verdicts alone, and it is
      // the same decision either way: an alternative reported through a
      // neighbour is never cheaper than that neighbour's verdict. The rest is
      // what the cell found and did not choose.
      const alternatives = self.options.map(price).filter(Boolean);
      self.routes = [...totals, ...alternatives]
        .map((entry) => ({ total: entry.total, path: [origin, ...entry.path] }))
        .sort((a, b) => a.total - b.total || a.path.join().localeCompare(b.path.join()));
      return;
    }

    if (usable.length === 0) {
      for (const requester of self.requesters) send(requester, { kind: "noRoute", from: node });
      return;
    }
    usable.sort((a, b) => a.value - b.value || a.from.localeCompare(b.from));
    self.nextHop = usable[0].from;
    self.value = usable[0].value;
    const path = [node, ...usable[0].path];
    // What this arm acts on is the minimum; what it reports is everything, so
    // the choice is not quietly made for the arm upstream. Every neighbour's
    // own verdict plus every alternative they reported, minus the one being
    // sent as the answer, each with this arm joined to its path.
    const others = [
      // usable is sorted, so its head is the answer being sent: drop it, and
      // everything else through here is an alternative.
      ...usable.slice(1).map((answer) => ({ value: answer.value, path: answer.path })),
      ...self.options
    ].map((option) => ({ value: option.value, path: [node, ...option.path] }));
    // A different number to each requester: the leg is priced on the edge the
    // reply travels across, and only the downstream part is shared. The leg is
    // the same for every route through here, so adding it leaves their order
    // alone and the answer stays the cheapest.
    for (const requester of self.requesters) {
      const own = edgeLeg(legCost, node, requester);
      if (own === null) {
        send(requester, { kind: "noRoute", from: node });
        continue;
      }
      send(requester, {
        kind: "routeCost",
        from: node,
        value: own + usable[0].value,
        path,
        options: others.map((option) => ({ value: own + option.value, path: option.path }))
      });
    }
  };

  const record = (node, answer, options = []) => {
    const self = state.get(node);
    // Counted by name: one neighbour is one verdict, however many replies it
    // sends. Counting in bulk let a neighbour that answered twice stand in for
    // one that had not answered at all -- and the branch still in flight was
    // always the slow one, since a refusal is instant while a real leg costs
    // the arm a rendezvous ranking first.
    if (self.answers.some((seen) => seen.from === answer.from)) return;
    self.answers.push(answer);
    // Tagged with the neighbour it arrived through, because that is the edge
    // this arm would pay to use it -- the option's own value stops at that
    // neighbour, exactly as the verdict's does.
    self.options.push(...options.map((option) => ({ ...option, from: answer.from })));
    if (self.answers.length >= self.asked) settle(node);
  };

  while (queue.length > 0) {
    if (shuffle) shuffle(queue);
    const message = queue.shift();
    delivered += 1;
    if (delivered > 100000) throw new Error("route search did not terminate");
    const node = message.to;
    const self = state.get(node);

    if (message.kind === "findRoute") {
      // The goal is checked BEFORE the already-seen rule, and deliberately so.
      // A sink never forwards, so answering every requester cannot loop -- and
      // if it answered only the first, then a first request arriving across an
      // edge it cannot price would spend its one answer on a "no" that was
      // never about reachability, and poison every other way to it.
      if (node === sink) {
        self.seen = true;
        self.requesters.push(message.from);
        self.settled = true;
        self.value = 0;
        const own = edgeLeg(legCost, node, message.from);
        if (own === null) send(message.from, { kind: "noRoute", from: node });
        else send(message.from, { kind: "routeCost", from: node, value: own, path: [node], options: [] });
        continue;
      }
      // Its own layer never answered, so it knows of no goal and cannot move:
      // "no route through me" is the whole of what it has to say, and saying it
      // at once is the difference between milliseconds and a deadline.
      if (strandedArms.has(node) || self.seen || message.ttl <= 0) {
        self.seen = true;
        send(message.from, { kind: "noRoute", from: node });
        continue;
      }
      // Can I even hand back to whoever asked? If not, no route from them runs
      // through me -- and that is a fact about our shared edge, not about the
      // goal. Answering "no" is right; spending my one answer to do it is not,
      // because an arm I COULD have served may ask next and get refused by the
      // dedup rule. This is the same mistake as the goal-holder's, one hop out:
      // a refusal that was never about reachability silencing a real route.
      if (edgeLeg(legCost, node, message.from) === null) {
        send(message.from, { kind: "noRoute", from: node });
        continue;
      }
      self.seen = true;
      self.requesters.push(message.from);
      const onward = neighbours.get(node).filter((other) => other !== message.from);
      if (onward.length === 0) {
        self.settled = true;
        send(message.from, { kind: "noRoute", from: node });
        continue;
      }
      self.asked = onward.length;
      for (const other of onward) ask(node, other, message.ttl - 1);
      continue;
    }

    if (message.kind === "routeCost") {
      record(node, { from: message.from, value: message.value, path: message.path }, message.options);
      continue;
    }
    record(node, { from: message.from, value: null, path: [] });
  }

  const chosen = state.get(origin);
  return {
    messages: delivered,
    nextHop: chosen.nextHop,
    total: chosen.value,
    path: chosen.path || null,
    // Every route the search found, cheapest first, not only the one taken.
    routes: chosen.routes || [],
    reachable: chosen.nextHop !== null,
    state
  };
}

// What a route costs, under the one-leg-per-arm model: the origin's own leg onto
// its first hop, plus, at every step, the leg paid by the arm being handed TO.
// Shared by the walk and the brute force so a comparison of the two cannot drift
// on the definition of the thing being compared.
function pathCost(legCost, path) {
  let total = edgeLeg(legCost, path[0], path[1]);
  if (total === null) return null;
  for (let index = 1; index < path.length; index += 1) {
    const inbound = edgeLeg(legCost, path[index], path[index - 1]);
    if (inbound === null) return null;
    total += inbound;
  }
  return total;
}

// Carrying the cube along the route, with some arms refusing to take it.
//
// The search settles in seconds; the transfer takes minutes, and an arm can be
// settled on and still turn out to be unusable by the time the cube reaches the
// arm before it. This walks the nextHop chain the search left behind and applies
// the recovery the agents apply: the holder strikes the arm that refused, takes
// the next minimum over the answers IT collected, and hands the cube on. No arm
// is asked anything again -- every alternative was priced by the same search and
// travelled home inside the same replies.
//
// `failed` is the set of arms whose agent will not take the cube. It is given to
// the walk rather than to `simulate`, and deliberately so: an arm that was
// already gone during the search would simply never have answered, and the
// cheap route would never have been chosen. The interesting case -- the one the
// experiment runs -- is an arm that answered, was chosen, and then failed.
function deliver(spec, failed = []) {
  const unfit = new Set(failed);
  const { state, nextHop: firstHop } = simulate(spec);
  if (firstHop === null) return { delivered: false, path: null, struck: [], total: null, reason: "no route at all" };

  const struck = [];
  const walked = [spec.origin];
  let holder = spec.origin;

  // Following the answers can only descend the flood tree -- an arm's answers
  // come from the arms it asked, and the sink is a leaf -- so this terminates
  // without a visited set, and striking a hop only moves the walk to another
  // child of the same node. The cap is there to turn a model that stops being
  // true of the plans into a failing test rather than a hang.
  while (holder !== spec.sink) {
    if (walked.length > spec.nodes.length) throw new Error("delivery walk did not descend");
    const self = state.get(holder);
    const open = self.answers
      .filter((answer) => answer.value !== null && !unfit.has(answer.from))
      .map((answer) => {
        // The origin is the one arm that also pays its own first leg, because it
        // had nobody upstream to report that leg to.
        const own = holder === spec.origin ? edgeLeg(spec.legCost, holder, answer.from) : 0;
        return own === null ? null : { from: answer.from, total: own + answer.value };
      })
      .filter(Boolean)
      .sort((a, b) => a.total - b.total || a.from.localeCompare(b.from));

    // The hop the route named is unusable, so it is struck rather than
    // forgotten: which arm refused, and where, is the whole record of a reroute.
    if (self.nextHop !== null && unfit.has(self.nextHop)) {
      struck.push({ at: holder, arm: self.nextHop });
    }
    if (open.length === 0) {
      return { delivered: false, path: walked, struck, total: null, reason: `${holder} has no way on left` };
    }
    holder = open[0].from;
    walked.push(holder);
  }

  return { delivered: true, path: walked, struck, total: pathCost(spec.legCost, walked), reason: null };
}

// What the cheapest route actually costs, by looking at all of them.
//
// Under the one-leg-per-arm model a path v0..vk costs leg(v0,v1) plus, for each
// step, the leg paid by the arm being handed TO. Enumerating simple paths is
// exponential and deliberately so: this is the answer the protocol is graded
// against, and it must be obviously right rather than fast.
function bruteForceOptimal({ nodes, edges, legCost, origin, sink }) {
  const neighbours = adjacency(nodes, edges);
  let best = null;

  const walk = (node, visited, path, accumulated) => {
    if (node === sink) {
      if (best === null || accumulated < best.total) best = { total: accumulated, path: [...path] };
      return;
    }
    for (const next of neighbours.get(node)) {
      if (visited.has(next)) continue;
      const downstreamLeg = edgeLeg(legCost, next, node);
      if (downstreamLeg === null) continue;
      // The origin also pays its own leg on its first hop.
      const ownLeg = path.length === 1 ? edgeLeg(legCost, node, next) : 0;
      if (ownLeg === null) continue;
      visited.add(next);
      path.push(next);
      walk(next, visited, path, accumulated + downstreamLeg + ownLeg);
      path.pop();
      visited.delete(next);
    }
  };

  walk(origin, new Set([origin]), [origin], 0);
  return best;
}

module.exports = { adjacency, bruteForceOptimal, deliver, edgeLeg, pathCost, simulate };
