// bench.js — JS counterpart of cpp/src/bench.cpp. Same seeds, same operations, same statistics.
const { load, runMission } = require('./harness');
const C = load();
const now = () => Number(process.hrtime.bigint()) / 1e6;
function report(label, v) {
  v.sort((a, b) => a - b);
  const at = (q) => v[Math.min(v.length - 1, Math.floor(q * v.length))];
  console.log(`${label.padEnd(34)} median ${at(0.5).toFixed(4).padStart(9)} ms   p99 ${at(0.99).toFixed(4).padStart(9)} ms   max ${v[v.length - 1].toFixed(4).padStart(9)} ms   (n=${v.length})`);
}
function rng(seed) { let a = seed >>> 0; return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }

// JIT warm-up so steady-state code is measured, as a long-running page would be.
for (let s = 1000; s < 1010; s++) runMission(C, { seed: s, density: 36, drops: [2, 6, 10, 14].map((at) => ({ at, dist: 95 })) });

const gen = [], fineBuild = [], add = [], fineAstar = [], coarseAstar = [];
for (let seed = 1; seed <= 60; seed++) {
  const w = new C.World(960, 640, 10);
  let t = now(); w.generate(seed, 36, 9); gen.push(now() - t);
  t = now(); w.fine.buildStep(Infinity); fineBuild.push(now() - t);
  for (let k = 0; k < 5; k++) { t = now(); w.addObstacle(100 + k * 170, 320, 9, w.start); add.push(now() - t); }
  const a = new C.AStar();
  t = now(); a.start(w.nav, w.nav.seedCells(w.start.x, w.start.y, 9), w.nav.seedCells(w.goal.x, w.goal.y, 9)); while (a.step(1e9) === 'running'); coarseAstar.push(now() - t);
  t = now(); a.start(w.fine, w.fine.seedCells(w.start.x, w.start.y, 9), w.fine.seedCells(w.goal.x, w.goal.y, 9)); while (a.step(1e9) === 'running'); fineAstar.push(now() - t);
}
console.log('== operations (60 worlds, density 36)');
report('generate (coarse grid)', gen);
report('fine grid full build', fineBuild);
report('addObstacle + validation', add);
report('coarse A* incl. seeding (unsliced)', coarseAstar);
report('fine A* incl. seeding (unsliced)', fineAstar);

console.log('== per-frame cost with app-like 2 ms slices: plan or build slice + tick (100 missions each)');
const scen = [
  { name: 'static d=36', density: 36, drops: () => [] },
  { name: '16 random drops d=40', density: 40, drops: (s) => { const r = rng(s * 131); return Array.from({ length: 16 }, (_, i) => ({ at: 0.3 + i * 0.8 + r() * 0.5, dist: 20 + r() * 200 })); } },
];
for (const sc of scen) {
  const frames = [];
  const t0 = now();
  for (let s = 1; s <= 100; s++) runMission(C, { seed: s, density: sc.density, drops: sc.drops(s), frameCosts: frames, planBudget: 2, buildBudget: 2 });
  const total = now() - t0;
  report(sc.name, frames);
  console.log(`${''.padEnd(34)} ${(frames.length / (total / 1000)).toFixed(0)} simulated frames per wall-clock second`);
}
