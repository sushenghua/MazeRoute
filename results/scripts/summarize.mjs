// summarize.mjs <runDir> — turn one run's raw outputs into summary.json and SUMMARY.md.
// Only reads what mac.sh and chip.mjs wrote; every number in the summary can be traced to a raw file.
import { existsSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const RUN = process.argv[2];
if (!RUN) { console.error('usage: node summarize.mjs <runDir>'); process.exit(2); }
const read = (...p) => (existsSync(join(RUN, ...p)) ? readFileSync(join(RUN, ...p), 'utf8') : null);
const readJson = (...p) => { const t = read(...p); return t ? JSON.parse(t) : null; };
const files = (...p) => (existsSync(join(RUN, ...p)) ? readdirSync(join(RUN, ...p)) : []);

// ---------------- parsers for the text outputs ----------------

const SCEN = /^(.+?)\s+runs (\d+)\s+arrived (\d+)\s+FAIL (\d+)\s+zeroColl (\d+)\s+recov (\d+)\s+replans (\d+)\s+placed (\d+) rejected (\d+)\s+avgT ([\d.]+)/;
const SEALED = /sealed runs (\d+)\s+arrived anyway (\d+)\s+honest FAILED (\d+)\s+seal->FAILED latency median ([\d.-]+) s\s+max ([\d.-]+) s/;
const TOTAL = /^TOTAL runs (\d+)\s+failures (\d+)\s+\(([\d.]+) s\)/;

/** Merge every suite_*.txt in a folder: per-scenario sums and the total. */
function suite(...dir) {
  const names = files(...dir).filter((f) => /^suite_\d+\.txt$/.test(f)).sort((a, b) => parseInt(a.slice(6)) - parseInt(b.slice(6)));
  if (!names.length) return null;
  const by = new Map();
  const total = { runs: 0, failures: 0, seconds: 0, ranges: names.map((n) => Number(n.match(/\d+/)[0])) };
  const failures = [];
  for (const n of names) {
    let cur = null;
    for (const line of read(...dir, n).split('\n')) {
      let m;
      if ((m = line.match(SCEN))) {
        const [, name, runs, arrived, fail, zeroColl, recov, replans, placed, rejected, avgT] = m;
        cur = by.get(name) || { scenario: name, runs: 0, arrived: 0, fail: 0, zeroColl: 0, recov: 0, replans: 0, placed: 0, rejected: 0, sumT: 0 };
        for (const [k, v] of Object.entries({ runs, arrived, fail, zeroColl, recov, replans, placed, rejected })) cur[k] += Number(v);
        cur.sumT += Number(avgT) * Number(runs);
        by.set(name, cur);
      } else if ((m = line.match(SEALED)) && cur) {
        cur.sealedRuns = (cur.sealedRuns || 0) + Number(m[1]);
        cur.sealedArrivedAnyway = (cur.sealedArrivedAnyway || 0) + Number(m[2]);
        cur.honestFailed = (cur.honestFailed || 0) + Number(m[3]);
        cur.sealToFailedMaxS = Math.max(cur.sealToFailedMaxS || 0, Number(m[5]) || 0);
      } else if ((m = line.match(TOTAL))) {
        total.runs += Number(m[1]); total.failures += Number(m[2]); total.seconds += Number(m[3]);
      } else if (/^\s+seed \d+:/.test(line)) failures.push(`${cur?.scenario}: ${line.trim()}`);
    }
  }
  const scenarios = [...by.values()].map(({ sumT, ...s }) => ({ ...s, meanTimeS: +(sumT / s.runs).toFixed(2) }));
  return { total: { ...total, seconds: +total.seconds.toFixed(1) }, scenarios, failures };
}

/** "label   median X ms   p99 Y ms   max Z ms   (n=N)" lines, grouped under "== section" headers. */
function bench(...p) {
  const t = read(...p);
  if (!t) return null;
  const out = { sections: [] };
  let sec = null, lastRow = null;
  for (const line of t.split('\n')) {
    let m;
    if (line.startsWith('== ')) { sec = { title: line.slice(3), rows: [] }; out.sections.push(sec); continue; }
    if ((m = line.match(/^(.+?)\s+median\s+([\d.]+) ms\s+(p\d+)\s+([\d.]+) ms\s+max\s+([\d.]+) ms(?:\s+\(n=(\d+)\))?/))) {
      if (!sec) { sec = { title: '', rows: [] }; out.sections.push(sec); }
      lastRow = { label: m[1].trim(), median: +m[2], [m[3]]: +m[4], max: +m[5], n: m[6] ? +m[6] : undefined };
      sec.rows.push(lastRow);
    } else if ((m = line.match(/([\d.]+) simulated frames per wall-clock second/)) && lastRow) lastRow.simFramesPerSecond = +m[1];
  }
  return out;
}

const pct = (sorted, q) => sorted[Math.min(sorted.length - 1, Math.floor(q * sorted.length))];
function dist(values) {
  const v = [...values].sort((a, b) => a - b);
  if (!v.length) return null;
  return { n: v.length, median: +pct(v, 0.5).toFixed(3), p90: +pct(v, 0.9).toFixed(3), p99: +pct(v, 0.99).toFixed(3), max: +v[v.length - 1].toFixed(3), mean: +(v.reduce((a, b) => a + b, 0) / v.length).toFixed(3) };
}

// ---------------- gather ----------------

const env = readJson('environment.json');
const js = { suite: suite('js'), bench: bench('js', 'bench.txt'), perf: bench('js', 'perf.txt') };
const cpp = { suite: suite('cpp'), bench: bench('cpp', 'bench.txt') };
const hostFloat = { bench: bench('host-float', 'bench.txt'), suites: {} };
for (const d of files('host-float')) if (!d.endsWith('.txt')) hostFloat.suites[d] = suite('host-float', d);

const chip = {
  info: readJson('esp32', 'info.json'),
  bench: readJson('esp32', 'bench.json'),
  profile: readJson('esp32', 'profile.json'),
  suite: readJson('esp32', 'suite.json'),
  realtime: readJson('esp32', 'realtime.json'),
};
if (chip.realtime) {
  const frames = { static: [], drops: [] };
  for (const f of files('esp32', 'frames')) {
    const kind = f.startsWith('static') ? 'static' : 'drops';
    for (const line of read('esp32', 'frames', f).split('\n').slice(1)) if (line) frames[kind].push(Number(line.split(',')[9]));
  }
  const all = [...frames.static, ...frames.drops];
  const drops = chip.realtime.flatMap((m) => m.dropResults);
  const ok = drops.filter((d) => d.ok);
  chip.realtimeSummary = {
    missions: chip.realtime.length,
    arrived: chip.realtime.filter((m) => m.summary.state === 'ARRIVED').length,
    frames: all.length,
    framesOverBudget: all.filter((x) => x > 1000 / 60).length,
    frameMs: { all: dist(all), static: dist(frames.static), withDrops: dist(frames.drops) },
    drops: {
      sent: drops.length, accepted: ok.length, rejected: drops.length - ok.length,
      rejectReasons: Object.entries(drops.filter((d) => !d.ok).reduce((a, d) => ((a[d.reason] = (a[d.reason] || 0) + 1), a), {})).map(([reason, n]) => ({ reason, n })),
      validationFrames: dist(ok.map((d) => d.frames)), validationWorkMs: dist(ok.map((d) => d.workMs)),
    },
    contactPushes: chip.realtime.reduce((a, m) => a + m.summary.coll, 0),  // resolveDisc push-outs, per frame and obstacle
    replans: chip.realtime.reduce((a, m) => a + m.summary.replans, 0),
    psramPeakMB: +Math.max(...chip.realtime.map((m) => (m.summary.heap.psramTotal - m.summary.heap.psramMin) / 1048576)).toFixed(2),
    internalFreeMinKB: Math.round(Math.min(...chip.realtime.map((m) => m.summary.heap.intMin)) / 1024),
  };
}

// Same five operations across environments (JS and C++ benches share labels; the chip names them its own way).
const OPS = [
  ['generate', 'Generate arena + coarse grid', 'generate (coarse grid)', 'generate + coarse grid'],
  ['fineBuild', 'Fine grid full build (480×320)', 'fine grid full build', 'fine grid full build'],
  ['drop', 'Obstacle drop + validation', 'addObstacle + validation', 'drop validation (all slices)'],
  ['coarseAstar', 'Coarse A* incl. seeding', 'coarse A* incl. seeding (unsliced)', 'coarse A* (unsliced)'],
  ['fineAstar', 'Fine A* incl. seeding', 'fine A* incl. seeding (unsliced)', 'fine A* (unsliced)'],
];
const row = (b, label) => b?.sections.flatMap((s) => s.rows).find((r) => r.label === label);
const operations = OPS.map(([key, title, hostLabel, chipLabel]) => ({
  key, title,
  js: row(js.bench, hostLabel)?.median, cpp: row(cpp.bench, hostLabel)?.median, hostFloat: row(hostFloat.bench, hostLabel)?.median,
  esp32: chip.bench?.rows.find((r) => r.label === chipLabel)?.median,
}));

const summary = { run: RUN.split('/').pop(), environment: env, js, cpp, hostFloat, esp32: chip, operations };
writeFileSync(join(RUN, 'summary.json'), JSON.stringify(summary, null, 2) + '\n');

// ---------------- SUMMARY.md ----------------

const f = (x, d = 2) => (x === undefined || x === null ? '–' : typeof x === 'number' ? (x >= 100 ? x.toFixed(0) : x.toFixed(d)) : x);
const md = [];
md.push(`# Results: ${summary.run}`, '');
if (env) {
  md.push(`Recorded ${env.recordedAt} at commit \`${env.git.commit.slice(0, 7)}\`${env.git.dirty ? ' (uncommitted changes)' : ''}.`, '');
  md.push(`- Host: ${env.host.model}, ${env.host.cpu}, ${env.host.cores} cores, ${env.host.memoryGB} GB, ${env.host.os}`);
  md.push(`- Tools: Node ${env.toolchains.node}; ${env.toolchains.cxx}; ESP-IDF ${env.toolchains.espIdf}; ${env.toolchains.xtensaGcc}`);
}
if (chip.info) md.push(`- Board: ${chip.info.chip} rev ${chip.info.rev}, ${chip.info.cores} cores @ ${chip.info.mhz} MHz, ${chip.info.flash / 1048576} MB flash, ${(chip.info.heap.psramTotal / 1048576).toFixed(0)} MB PSRAM; core in \`${chip.info.real}\`; budgets per 16.7 ms frame: plan ${chip.info.budgetMs.plan} ms, build ${chip.info.budgetMs.build} ms, drop ${chip.info.budgetMs.drop} ms`);
md.push('');

md.push('## Correctness: zero-failure suites', '');
md.push('Each mission must reach B, or, when sealing drops are allowed, end in FAILED in a world an independent oracle confirms is unwinnable.', '');
md.push('| Implementation | Where | Slicing | Missions | Failures | Time |', '| --- | --- | --- | ---: | ---: | ---: |');
if (js.suite) md.push(`| JavaScript (browser code) | Node on host | 50 ms | ${js.suite.total.runs} | ${js.suite.total.failures} | ${js.suite.total.seconds} s |`);
if (cpp.suite) md.push(`| C++23, double | host | 50 ms | ${cpp.suite.total.runs} | ${cpp.suite.total.failures} | ${cpp.suite.total.seconds} s |`);
for (const [b, s] of Object.entries(hostFloat.suites)) if (s) md.push(`| C++23, float (MCU core) | host | ${b === 'unsliced' ? '50 ms' : `${b} ms (${b === '0.06' ? '≈ chip' : 'stress'})`} | ${s.total.runs} | ${s.total.failures} | ${s.total.seconds} s |`);
if (chip.suite) md.push(`| C++23, float (MCU core) | **ESP32-S3** | ${chip.suite.sliceMs} ms, as live | ${chip.suite.total.runs} | ${chip.suite.total.fail} | ${chip.suite.total.seconds} s |`);
md.push('');

const scenTable = (title, s) => {
  if (!s) return;
  md.push(`### ${title}`, '', '| Scenario | Runs | Arrived | Honest FAILED | Fail | Zero-collision runs | Drops placed / rejected | Mean time |', '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |');
  for (const r of s.scenarios) md.push(`| ${r.scenario} | ${r.runs} | ${r.arrived} | ${r.honestFailed ?? 0} | ${r.fail} | ${r.zeroColl} | ${r.placed} / ${r.rejected} | ${r.meanTimeS} s |`);
  md.push('');
};
scenTable(`JavaScript, ${js.suite?.total.runs ?? 0} missions`, js.suite);
scenTable(`C++23 double, ${cpp.suite?.total.runs ?? 0} missions`, cpp.suite);
scenTable(`Float core at chip-like slicing (0.06 ms), ${hostFloat.suites['0.06']?.total.runs ?? 0} missions`, hostFloat.suites['0.06']);
if (chip.suite) {
  md.push(`### ESP32-S3, ${chip.suite.total.runs} missions`, '', '| Scenario | Runs | Arrived | Honest FAILED | Fail |', '| --- | ---: | ---: | ---: | ---: |');
  for (const r of chip.suite.rows) md.push(`| ${r.scenario} | ${r.runs} | ${r.arrived} | ${r.honestFailed} | ${r.fail} |`);
  md.push('');
}

md.push('## Performance: the same operations everywhere (median ms, unsliced total work)', '');
md.push('| Operation | JavaScript (Node) | C++ double | C++ float | ESP32-S3 | ESP32 ÷ C++ double |', '| --- | ---: | ---: | ---: | ---: | ---: |');
for (const o of operations) md.push(`| ${o.title} | ${f(o.js)} | ${f(o.cpp, 3)} | ${f(o.hostFloat, 3)} | ${f(o.esp32)} | ${o.esp32 && o.cpp ? `${Math.round(o.esp32 / o.cpp)}×` : '–'} |`);
md.push('', 'On the ESP32-S3, drop validation is the sliced `DropTask` (all slices added up); on the host it is the synchronous `addObstacle`.', '');

md.push('## Frame cost (ms per 16.7 ms frame)', '');
md.push('| Implementation | Scenario | Median | p99 | Max | Note |', '| --- | --- | ---: | ---: | ---: | --- |');
for (const [name, b] of [['JavaScript', js.bench], ['C++ double', cpp.bench], ['C++ float', hostFloat.bench]]) {
  const sec = b?.sections.find((s) => s.title.startsWith('per-frame'));
  for (const r of sec?.rows || []) md.push(`| ${name} (host) | ${r.label} | ${f(r.median, 3)} | ${f(r.p99, 3)} | ${f(r.max, 3)} | 100 missions, 2 ms slices${r.simFramesPerSecond ? `, ${r.simFramesPerSecond} sim frames/s` : ''} |`);
}
const rs = chip.realtimeSummary;
if (rs) {
  const nStatic = chip.realtime.filter((m) => !m.drops.length).length, nDrops = chip.realtime.length - nStatic;
  md.push(`| **ESP32-S3 (real time)** | ${nStatic} static missions | ${f(rs.frameMs.static.median, 3)} | ${f(rs.frameMs.static.p99, 3)} | ${f(rs.frameMs.static.max, 3)} | ${rs.frameMs.static.n} frames |`);
  md.push(`| **ESP32-S3 (real time)** | ${nDrops} missions with drops | ${f(rs.frameMs.withDrops.median, 3)} | ${f(rs.frameMs.withDrops.p99, 3)} | ${f(rs.frameMs.withDrops.max, 3)} | ${rs.frameMs.withDrops.n} frames |`);
  md.push('');
  md.push('## ESP32-S3 real-time missions', '');
  md.push(`${rs.arrived} of ${rs.missions} missions arrived. **${rs.framesOverBudget} of ${rs.frames} frames** exceeded 16.7 ms (worst ${f(rs.frameMs.all.max, 2)} ms).`);
  md.push(`Drops: ${rs.drops.accepted} accepted, ${rs.drops.rejected} rejected (${rs.drops.rejectReasons.map((r) => `${r.n}× ${r.reason}`).join(', ') || 'none'}). Validation took a median of ${rs.drops.validationFrames?.median ?? '–'} frames (max ${rs.drops.validationFrames?.max ?? '–'}) and ${f(rs.drops.validationWorkMs?.median)} ms of work (max ${f(rs.drops.validationWorkMs?.max)}).`);
  md.push(`Memory: PSRAM peak ${rs.psramPeakMB} MB of 8 MB, internal RAM never below ${rs.internalFreeMinKB} KB free.`, '');
  md.push('| Mission | Density | Drops ahead | Result | Time | Frames > 16.7 ms | Worst frame | Mean frame | Drops ok / rejected | Replans | Contact pushes |', '| --- | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |');
  for (const m of chip.realtime) {
    const s = m.summary;
    md.push(`| ${m.id} | ${m.density} | ${m.drops[0] ? `${m.drops[0].dist} px` : '–'} | ${s.state} | ${s.t} s | ${s.overBudget} / ${s.frames} | ${f(s.worstFrame)} ms | ${f(s.avgFrame)} ms | ${m.dropResults.filter((d) => d.ok).length} / ${m.dropResults.filter((d) => !d.ok).length} | ${s.replans} | ${s.coll} |`);
  }
  md.push('');
}
if (chip.profile) {
  md.push('## ESP32-S3: drop-validation phases and search start-up (unsliced, ms)', '', '| Step | Median | p90 | Max |', '| --- | ---: | ---: | ---: |');
  for (const r of chip.profile.rows) md.push(`| ${r.label} | ${f(r.median)} | ${f(r.p90)} | ${f(r.max)} |`);
  md.push('');
}
writeFileSync(join(RUN, 'SUMMARY.md'), md.join('\n'));
console.log(`wrote ${join(RUN, 'summary.json')} and SUMMARY.md`);
