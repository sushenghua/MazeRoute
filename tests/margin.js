// margin.js — for missions that FAILED with sealing allowed, find the widest side clearance at which
// B is still reachable (1 px oracle, binary search). Tells "planner missed a real gap" apart from
// "gap narrower than the agent's safety margin".
const { load, runMission, solvable } = require('./harness');
const C = load();
function rng(seed) { let a = seed >>> 0; return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
const scen = {
  s45: { density: 36, drops: () => [2, 6, 10, 14].map((at) => ({ at, dist: 45 })) },
  s16: { density: 40, drops: (s) => { const r = rng(s * 131); return Array.from({ length: 16 }, (_, i) => ({ at: 0.3 + i * 0.8 + r() * 0.5, dist: 20 + r() * 200 })); } },
};
const [name, seedsArg] = [process.argv[2], process.argv[3]];
const sc = scen[name];
for (const seed of seedsArg.split(',').map(Number)) {
  const r = runMission(C, { seed, density: sc.density, drops: sc.drops(seed), allowSealing: true, keepWorld: true });
  if (r.state === 'ARRIVED') { console.log(seed, 'ARRIVED'); continue; }
  let lo = -0.5, hi = 4; // margin in px beyond the agent radius
  if (!solvable(C, r.world, r.agentPos, 9, lo, 1)) { console.log(seed, r.state, 'unreachable even with zero margin'); continue; }
  for (let k = 0; k < 7; k++) { const mid = (lo + hi) / 2; if (solvable(C, r.world, r.agentPos, 9, mid, 1)) lo = mid; else hi = mid; }
  console.log(seed, r.state, `widest passable side margin ≈ ${lo.toFixed(2)} px`);
}
