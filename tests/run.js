// run.js — stress suite. Usage: node tests/run.js [seedsPerScenario] [firstSeed]
const { load, runMission } = require('./harness');
const C = load();
const N = parseInt(process.argv[2] || '100', 10);
const S0 = parseInt(process.argv[3] || '1', 10);
const ONLY = process.argv[4] && process.argv[4] !== '--dump' ? process.argv[4].split(',').map(Number) : null; // optional explicit seed list
const dumpAt = process.argv.indexOf('--dump');
const DUMP = dumpAt > 0 ? require('fs').openSync(process.argv[dumpAt + 1], 'w') : null; // per-mission outcomes, same format as cpp/suite

function rng(seed) { let a = seed >>> 0; return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }

const scenarios = [
  { name: 'static d=12', density: 12 },
  { name: 'static d=22', density: 22 },
  { name: 'static d=36', density: 36 },
  { name: 'static d=40', density: 40 },
  { name: '4 drops @95px d=36', density: 36, drops: (s) => [2, 6, 10, 14].map((at) => ({ at, dist: 95 })) },
  { name: '4 drops @45px d=36', density: 36, drops: (s) => [2, 6, 10, 14].map((at) => ({ at, dist: 45 })) },
  { name: '8 random drops d=30', density: 30, drops: (s) => { const r = rng(s * 977); return Array.from({ length: 8 }, (_, i) => ({ at: 0.5 + i * 1.5 + r(), dist: 30 + r() * 150 })); } },
  { name: '16 random drops d=40', density: 40, drops: (s) => { const r = rng(s * 131); return Array.from({ length: 16 }, (_, i) => ({ at: 0.3 + i * 0.8 + r() * 0.5, dist: 20 + r() * 200 })); } },
  // With sealing allowed, FAILED is the correct outcome when B really is unreachable. These runs
  // pass only if every non-arrival is a prompt FAILED in a world the oracle confirms is unwinnable.
  { name: 'SEAL 4 drops @45px d=36', density: 36, allowSealing: true, drops: (s) => [2, 6, 10, 14].map((at) => ({ at, dist: 45 })) },
  { name: 'SEAL 16 random d=40', density: 40, allowSealing: true, drops: (s) => { const r = rng(s * 131); return Array.from({ length: 16 }, (_, i) => ({ at: 0.3 + i * 0.8 + r() * 0.5, dist: 20 + r() * 200 })); } },
];

let totalFail = 0, totalRuns = 0;
const t0 = Date.now();
for (const sc of scenarios) {
  const rows = [];
  for (const s of ONLY || Array.from({ length: N }, (_, k) => S0 + k)) rows.push(runMission(C, { seed: s, density: sc.density, allowSealing: !!sc.allowSealing, checkContract: !!sc.allowSealing, drops: sc.drops ? sc.drops(s) : [] }));
  const ok = rows.filter((r) => r.gen !== false);
  if (DUMP) for (const r of ok) require('fs').writeSync(DUMP, `${sc.name}|${r.seed}|${r.state}|${r.t.toFixed(1)}|${r.placed}|${r.rejected}\n`);
  const notArrived = ok.filter((r) => r.state !== 'ARRIVED');
  // An honest FAILED (only possible with sealing allowed): the machine said FAILED and the 1 px oracle
  // confirms no route leaves the guaranteed clearance (FINE_GUARANTEE px each side). Gaps tighter than
  // that may legitimately be treated as closed.
  const honest = sc.allowSealing ? notArrived.filter((r) => r.state === 'FAILED' && r.solvableAtGuarantee === false) : [];
  const fails = notArrived.filter((r) => !honest.includes(r));
  totalFail += fails.length; totalRuns += ok.length;
  const sum = (k) => ok.reduce((a, r) => a + (r[k] || 0), 0);
  console.log(`${sc.name.padEnd(22)} runs ${ok.length}  arrived ${ok.length - notArrived.length}  FAIL ${fails.length}  zeroColl ${ok.filter((r) => !r.coll).length}  recov ${sum('recov')}  replans ${sum('replans')}  placed ${sum('placed')} rejected ${sum('rejected')}  avgT ${(sum('t') / ok.length).toFixed(1)}`);
  for (const f of fails.slice(0, 12)) console.log(`    seed ${f.seed}: ${f.state} t=${f.t} solvable=${f.solvable}${f.solvableAtGuarantee !== undefined ? ` atGuarantee=${f.solvableAtGuarantee}` : ''} placed=${f.placed} recov=${f.recov} — ${f.reason}`);
  if (sc.allowSealing) {
    const lat = honest.map((r) => r.failLatency).filter((x) => x !== undefined).sort((a, b) => a - b);
    const sealedRuns = ok.filter((r) => r.sealed);
    console.log(`${''.padEnd(22)} sealed runs ${sealedRuns.length}  arrived anyway ${sealedRuns.filter((r) => r.state === 'ARRIVED').length}  honest FAILED ${honest.length}  seal->FAILED latency median ${lat.length ? lat[lat.length >> 1].toFixed(1) : '-'} s  max ${lat.length ? lat[lat.length - 1].toFixed(1) : '-'} s`);
  }
}
console.log(`\nTOTAL runs ${totalRuns}  failures ${totalFail}  (${((Date.now() - t0) / 1000).toFixed(1)} s)`);
process.exitCode = totalFail ? 1 : 0;
