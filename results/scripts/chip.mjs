// chip.mjs <runDir> — record everything measured on the ESP32-S3, through the bridge (esp32/bridge).
//
// Needs the bridge running (node esp32/bridge/bridge.js) and Node 22+ (global WebSocket). Writes:
//   esp32/info.json             chip, clock, number type, ESP-IDF and compiler, frame budgets, heap
//   esp32/bench.json            bench <n>: operation timings
//   esp32/profile.json          profile <n>: drop-validation phases and search start-up costs
//   esp32/realtime.json         one entry per real-time mission: drops, transitions, summary
//   esp32/frames/<id>.csv       every frame of every real-time mission as the chip timed it
//   esp32/suite.json            suite rows and total (with live slice budgets)
//   esp32/suite-missions.jsonl  one line per suite mission
//   esp32/raw.jsonl             every message received, for anything not summarised above
import { mkdirSync, writeFileSync, appendFileSync } from 'node:fs';
import { join } from 'node:path';

const RUN = process.argv[2];
if (!RUN) { console.error('usage: node chip.mjs <runDir>'); process.exit(2); }
const URL = process.env.BRIDGE_URL || 'ws://127.0.0.1:8766/ws';
const BENCH_WORLDS = Number(process.env.CHIP_BENCH_WORLDS || 20);
const SUITE_SEEDS = Number(process.env.CHIP_SUITE_SEEDS || 30);
const SLICE_MS = Number(process.env.CHIP_SLICE_MS || 6);
const OUT = join(RUN, 'esp32');
mkdirSync(join(OUT, 'frames'), { recursive: true });

const log = (...a) => console.log(`[${new Date().toTimeString().slice(0, 8)}] chip:`, ...a);
const ws = new WebSocket(URL);
const listeners = new Set();
ws.onmessage = (e) => {
  appendFileSync(join(OUT, 'raw.jsonl'), e.data + '\n');
  const m = JSON.parse(e.data);
  for (const f of [...listeners]) f(m);
};
await new Promise((ok, fail) => { ws.onopen = ok; ws.onerror = () => fail(new Error(`cannot reach the bridge at ${URL}`)); });

const send = (cmd) => ws.send(cmd);
/** Resolve with the first message for which pred() is true (and collect every message meanwhile). */
function until(pred, timeoutMs, onEach) {
  return new Promise((ok, fail) => {
    const timer = setTimeout(() => { listeners.delete(f); fail(new Error('timed out')); }, timeoutMs);
    const f = (m) => {
      if (m.k === 'hello') { clearTimeout(timer); listeners.delete(f); fail(new Error('the chip rebooted (crash or watchdog); see the bridge log')); return; }
      onEach?.(m);
      if (pred(m)) { clearTimeout(timer); listeners.delete(f); ok(m); }
    };
    listeners.add(f);
  });
}
const save = (name, data) => writeFileSync(join(OUT, name), JSON.stringify(data, null, 2) + '\n');
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

await until((m) => m.k === 'bridge' && m.connected, 5000);
send(`budget ${SLICE_MS} ${SLICE_MS} ${SLICE_MS - 1}`);

// 1) Identity.
send('info');
const info = await until((m) => m.k === 'info', 5000);
save('info.json', info);
log(`${info.chip} rev ${info.rev}, ${info.mhz} MHz, ${info.real}, ESP-IDF ${info.idf}`);

// 2) Operation benchmark and drop-validation profile (unsliced: total work per operation).
log(`bench ${BENCH_WORLDS}`);
send(`bench ${BENCH_WORLDS}`);
save('bench.json', await until((m) => m.k === 'bench', 20 * 60e3));
log(`profile ${BENCH_WORLDS}`);
send(`profile ${BENCH_WORLDS}`);
save('profile.json', await until((m) => m.k === 'profile', 20 * 60e3));

// 3) Real-time missions, paced by the chip at 60 Hz exactly as in the demo. Fixed seeds so they can be re-run.
//    Static missions first, then missions with obstacles dropped ahead of the moving agent.
const missions = [
  ...[201, 202, 203, 204].map((seed) => ({ id: `static-${seed}`, seed, density: 36, drops: [] })),
  ...Array.from({ length: 12 }, (_, i) => ({
    id: `drops-${101 + i}`, seed: 101 + i, density: i % 2 ? 40 : 30,
    drops: [1.0, 2.2, 3.4, 4.6, 5.8].map((at) => ({ at, dist: [45, 70, 95][i % 3] })),
  })),
];
const realtime = [];
for (const ms of missions.slice(0, Number(process.env.CHIP_MISSIONS || missions.length))) {
  send(`gen ${ms.seed} ${ms.density}`);
  const world = await until((m) => m.k === 'world', 10e3);
  await sleep(300);
  const rec = { ...ms, genMs: world.genMs, polygons: world.polys.length, start: world.start, goal: world.goal, dropResults: [], transitions: [], paths: [] };
  const rows = ['t,state,x,y,heading,speed,dropMs,planMs,tickMs,frameMs'];
  let next = 0, last = null;
  send('start');
  const done = await until((m) => m.k === 'done', 120e3, (m) => {
    if (m.k === 's' && m.st !== 'IDLE') {
      last = m;
      rows.push([m.t, m.st, m.x, m.y, m.h, m.v, m.drop, m.plan, m.tick, m.frame].join(','));
      if (next < ms.drops.length && m.t >= ms.drops[next].at && m.st !== 'ARRIVED' && m.st !== 'FAILED') {
        const d = ms.drops[next++];
        const x = m.x + Math.cos(m.h) * d.dist, y = m.y + Math.sin(m.h) * d.dist;
        if (x > 20 && y > 20 && x < 940 && y < 620) send(`drop ${x.toFixed(1)} ${y.toFixed(1)} 0`);
      }
    }
    if (m.k === 'drop') rec.dropResults.push({ ok: m.ok, reason: m.reason, sealed: m.sealed, workMs: m.ms, slices: m.slices, frames: m.frames, atT: last?.t });
    if (m.k === 'tr') rec.transitions.push({ t: m.t, from: m.from, to: m.to, reason: m.reason });
    if (m.k === 'path') rec.paths.push({ tier: m.tier, nodes: m.nodes, slices: m.slices, planMs: m.planMs, points: m.path.length / 2 });
  });
  rec.summary = done;
  writeFileSync(join(OUT, 'frames', `${ms.id}.csv`), rows.join('\n') + '\n');
  realtime.push(rec);
  log(`${ms.id}: ${done.state} ${done.t}s, frames over 16.7 ms ${done.overBudget}/${done.frames}, worst ${done.worstFrame} ms, drops ${rec.dropResults.filter((d) => d.ok).length} ok / ${rec.dropResults.filter((d) => !d.ok).length} rejected`);
  save('realtime.json', realtime);
  await sleep(500);
}

// 4) The test suite on the chip, with the live slice budgets.
log(`suite ${SUITE_SEEDS} 1 ${SLICE_MS}`);
const rows = [];
send(`suite ${SUITE_SEEDS} 1 ${SLICE_MS}`);
const total = await until((m) => m.k === 'suite', 3 * 3600e3, (m) => {
  if (m.k === 'suiteMission') appendFileSync(join(OUT, 'suite-missions.jsonl'), JSON.stringify(m) + '\n');
  if (m.k === 'suiteRow') { rows.push(m); log(`  ${m.scenario}: ${m.arrived}/${m.runs} arrived, ${m.honestFailed} honest FAILED, ${m.fail} fail`); }
});
save('suite.json', { sliceMs: SLICE_MS, seedsPerScenario: SUITE_SEEDS, rows, total });
log(`suite: ${total.runs} missions, ${total.fail} failures, ${total.seconds} s`);

send('info');
save('info-after.json', await until((m) => m.k === 'info', 5000));
ws.close();
log('chip done');
