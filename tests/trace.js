// trace.js — prints per-tick controller internals in a time window, for the "16 random drops" scenario.
// Usage: node tests/trace.js <seed> <density> <fromSeconds> <toSeconds>
const { load } = require('./harness');
const C = load();
const [seed, density, t0, t1] = [parseInt(process.argv[2] || '1', 10), parseInt(process.argv[3] || '40', 10), parseFloat(process.argv[4] || 0), parseFloat(process.argv[5] || 5)];
function rng(seed) { let a = seed >>> 0; return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
const r = rng(seed * 131); const drops = Array.from({ length: 16 }, (_, i) => ({ at: 0.3 + i * 0.8 + r() * 0.5, dist: 20 + r() * 200 }));
const world = new C.World(960, 640, 10), agent = new C.Agent(9), ctl = new C.Controller(world, agent, new C.AStar());
world.generate(seed, density, 9); ctl.reset(); ctl.startMission(); let di = 0, printedPath = false;
while (!ctl.isTerminal && ctl.simTime < t1) {
  if (ctl.isPlanning) ctl.plan(50);
  else if (!world.fine.ready) world.fine.buildStep(3);
  ctl.tick(1 / 60);
  if (di < drops.length && ctl.simTime >= drops[di].at) { const d = drops[di++]; const px = agent.pos.x + Math.cos(agent.heading) * d.dist, py = agent.pos.y + Math.sin(agent.heading) * d.dist; if (C.V.dist({ x: px, y: py }, world.goal) > 50 && px > 20 && py > 20 && px < 940 && py < 620) { const q = world.addObstacle(px, py, 9, agent.pos); if (q.poly) ctl.onWorldChanged(); } }
  if (ctl.simTime >= t0 && ctl.state === 'FOLLOWING') {
    if (!printedPath) { printedPath = true; console.log('path', ctl.path.slice(0, 8).map((p) => `${p.x.toFixed(0)},${p.y.toFixed(0)}`).join(' ')); }
    if (Math.round(ctl.simTime * 60) % 4 === 0) console.log(`${ctl.simTime.toFixed(2)} pos ${agent.pos.x.toFixed(1)},${agent.pos.y.toFixed(1)} hd ${agent.heading.toFixed(2)} sp ${agent.speed.toFixed(1)} seg ${ctl.segIdx} tgt ${ctl.target.x.toFixed(0)},${ctl.target.y.toFixed(0)} rem ${ctl.remaining.toFixed(1)} trust ${ctl.trustPlan} wh ${ctl.whiskers.map((w) => w.t.toFixed(2)).join('/')} coll ${ctl.stats.collisions}`);
  }
}
