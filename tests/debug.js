// debug.js — replay one scenario seed and dump the transition log and the agent's situation at the end.
const { load, solvable } = require('./harness');
const C = load();
const [seed, density, mode] = [parseInt(process.argv[2]), parseInt(process.argv[3]), process.argv[4]];
function rng(seed) { let a = seed >>> 0; return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
const dropsFor = {
  d45: () => [2, 6, 10, 14].map((at) => ({ at, dist: 45 })),
  r8: (s) => { const r = rng(s * 977); return Array.from({ length: 8 }, (_, i) => ({ at: 0.5 + i * 1.5 + r(), dist: 30 + r() * 150 })); },
  r16: (s) => { const r = rng(s * 131); return Array.from({ length: 16 }, (_, i) => ({ at: 0.3 + i * 0.8 + r() * 0.5, dist: 20 + r() * 200 })); },
}[mode] || (() => []);
const world = new C.World(960, 640, 10), agent = new C.Agent(9), astar = new C.AStar(), ctl = new C.Controller(world, agent, astar);
world.generate(seed, density, 9); ctl.reset(); ctl.startMission();
const drops = dropsFor(seed); let di = 0; const events = [];
while (!ctl.isTerminal && ctl.simTime < 150) {
  if (ctl.isPlanning) ctl.plan(50);
  else if (!world.fine.ready) world.fine.buildStep(3);
  ctl.tick(1 / 60);
  if (di < drops.length && ctl.simTime >= drops[di].at) {
    const d = drops[di++];
    const px = agent.pos.x + Math.cos(agent.heading) * d.dist, py = agent.pos.y + Math.sin(agent.heading) * d.dist;
    if (C.V.dist({ x: px, y: py }, world.goal) > 50 && px > 20 && py > 20 && px < 940 && py < 620) {
      const r = world.addObstacle(px, py, 9, agent.pos);
      events.push(`${ctl.simTime.toFixed(2)} drop at ${px.toFixed(0)},${py.toFixed(0)} agent ${agent.pos.x.toFixed(0)},${agent.pos.y.toFixed(0)} -> ${r.poly ? 'placed' : r.rejected}`);
      if (r.poly) { ctl.onWorldChanged(); events.push(`      after: fine connected ${world.fine.connected(agent.pos, world.goal, 9)} oracle ${solvable(C, world, agent.pos, 9)}`); }
    }
  }
}
console.log(events.join('\n'));
console.log(ctl.log.slice().reverse().map((l) => `${l.t.toFixed(2)} ${l.from}->${l.to}: ${l.reason}`).join('\n'));
const a = agent.pos, f = world.fine;
const i = f.cellIndexAt(a.x, a.y);
console.log('agent', a.x.toFixed(1), a.y.toFixed(1), 'heading', agent.heading.toFixed(2), 'goal', world.goal);
console.log('agent fine cell blocked?', f.blocked[i], ' nearestFree(LOS) ->', f.nearestFreeCell(a.x, a.y, 9), ' nearestFree(maxDist 200) ->', f.nearestFreeCell(a.x, a.y, 9, 200));
const ns = world.nearestSurface(a.x, a.y, 40); console.log('clearance to nearest surface', ns && ns.d.toFixed(2));
console.log('fine connected from agent:', f.connected(a, world.goal, 9), ' oracle solvable:', solvable(C, world, a, 9));
// ASCII map of fine grid around the agent (A=agent, #=blocked, .=free)
const R = 14, c0 = i % f.cols, r0 = Math.floor(i / f.cols); let out = '';
for (let r = r0 - R; r <= r0 + R; r++) { for (let c = c0 - R * 2; c <= c0 + R * 2; c++) { if (r < 0 || c < 0 || r >= f.rows || c >= f.cols) { out += ' '; continue; } out += (r === r0 && c === c0) ? 'A' : f.blocked[r * f.cols + c] ? '#' : '.'; } out += '\n'; }
console.log(out);
