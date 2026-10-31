// harness.js — loads the browser scripts into a Node VM context so the exact same code runs headless.
const fs = require('fs');
const path = require('path');
const vm = require('vm');

function load() {
  const ctx = { performance: { now: () => Number(process.hrtime.bigint()) / 1e6 }, Math, console };
  vm.createContext(ctx);
  for (const f of ['geometry.js', 'world.js', 'pathfinder.js', 'steering.js', 'scheduler.js']) {
    const src = fs.readFileSync(path.join(__dirname, '..', 'js', f), 'utf8');
    // Top-level class/const declarations are lexical; re-export them onto the context.
    vm.runInContext(src + '\n;(' + exportsFor(src) + ')();', ctx, { filename: f });
  }
  return ctx;
}
function exportsFor(src) {
  const names = [...src.matchAll(/^(?:class|function|const)\s+([A-Za-z_$][\w$]*)/gm)].map((m) => m[1]);
  return '() => {' + names.map((n) => `globalThis.${n} = ${n};`).join('') + '}';
}

/**
 * Ground truth: is there ANY collision-free route for a disc of radius r from p to the goal?
 * Fine 3 px grid, disc tested exactly against the polygons. Independent of the planner's nav grid.
 */
function solvable(C, world, p, r, margin = 0.2, cs = 3) {
  const cols = Math.floor(world.width / cs), rows = Math.floor(world.height / cs);
  const free = new Uint8Array(cols * rows);
  for (let y = 0; y < rows; y++) for (let x = 0; x < cols; x++) free[y * cols + x] = world.discFree((x + 0.5) * cs, (y + 0.5) * cs, r + margin) ? 1 : 0;
  const near = (q) => { // nearest free fine cell within 2r
    let best = -1, bd = Infinity;
    const cx = Math.floor(q.x / cs), cy = Math.floor(q.y / cs), R = Math.ceil((2 * r) / cs);
    for (let dy = -R; dy <= R; dy++) for (let dx = -R; dx <= R; dx++) {
      const x = cx + dx, y = cy + dy; if (x < 0 || y < 0 || x >= cols || y >= rows) continue;
      const i = y * cols + x; if (!free[i]) continue;
      const d = dx * dx + dy * dy; if (d < bd) { bd = d; best = i; }
    }
    return best;
  };
  const s = near(p), g = near(world.goal);
  if (s < 0 || g < 0) return false;
  const seen = new Uint8Array(cols * rows), q = [s]; seen[s] = 1;
  while (q.length) {
    const i = q.pop(); if (i === g) return true;
    const x = i % cols, y = (i - x) / cols;
    for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      const nx = x + dx, ny = y + dy; if (nx < 0 || ny < 0 || nx >= cols || ny >= rows) continue;
      const j = ny * cols + nx; if (free[j] && !seen[j]) { seen[j] = 1; q.push(j); }
    }
  }
  return false;
}

/** One mission. drops: list of {at: simSeconds, dist: px ahead of agent}. */
function runMission(C, opts) {
  const { seed, density, drops = [], maxTime = 150, radius = 9, allowSealing = false, planBudget = 50, buildBudget = 3 } = opts;
  const world = new C.World(960, 640, 10);
  const agent = new C.Agent(radius);
  const astar = new C.AStar();
  const ctl = new C.Controller(world, agent, astar);
  if (!world.generate(seed, density, radius)) return { seed, gen: false };
  // PARITY=1 removes the only wall-clock dependence (sliced background build) so JS and C++ can be diffed.
  if (process.env.PARITY) world.fine.buildStep(Infinity);
  ctl.reset();
  ctl.startMission();
  let di = 0, placed = 0, rejected = 0, sealedAt = null;
  let maxPen = 0;
  const frameCosts = opts.frameCosts; // optional: per-frame cost of (plan or background slice) + tick
  const now = () => Number(process.hrtime.bigint()) / 1e6;
  while (!ctl.isTerminal && ctl.simTime < maxTime) {
    const f0 = frameCosts ? now() : 0;
    if (ctl.isPlanning) ctl.plan(planBudget);
    else if (!world.fine.ready) world.fine.buildStep(buildBudget); // mirrors the frame loop's background slice
    ctl.tick(1 / 60);
    if (frameCosts) frameCosts.push(now() - f0);
    if (di < drops.length && ctl.simTime >= drops[di].at) {
      const d = drops[di++];
      const px = agent.pos.x + Math.cos(agent.heading) * d.dist, py = agent.pos.y + Math.sin(agent.heading) * d.dist;
      if (C.V.dist({ x: px, y: py }, world.goal) > 50 && px > 20 && py > 20 && px < world.width - 20 && py < world.height - 20) {
        const res = world.addObstacle(px, py, radius, agent.pos, { allowSealing });
        if (res.poly) { placed++; if (res.sealed && sealedAt === null) sealedAt = ctl.simTime; ctl.onWorldChanged(); } else rejected++;
      }
    }
  }
  const res = { seed, density, state: ctl.state, t: +ctl.simTime.toFixed(1), placed, rejected, replans: ctl.stats.replans, recov: ctl.stats.recoveries, coll: ctl.stats.collisions, reason: ctl.log[0] && ctl.log[0].reason };
  if (ctl.state !== 'ARRIVED') {
    res.solvable = solvable(C, world, agent.pos, radius);
    // Contract check: is there a route that leaves at least FINE_GUARANTEE px each side? (1 px oracle)
    if (opts.checkContract) res.solvableAtGuarantee = solvable(C, world, agent.pos, radius, C.FINE_GUARANTEE, 1);
  }
  res.sealed = sealedAt !== null;
  if (sealedAt !== null && ctl.state === 'FAILED') res.failLatency = ctl.simTime - sealedAt;
  if (opts.keepWorld) { res.world = world; res.agentPos = { ...agent.pos }; }
  return res;
}

module.exports = { load, runMission, solvable };
