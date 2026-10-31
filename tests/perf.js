// perf.js — cost of the operations that must fit inside a frame or a click handler.
const { load } = require('./harness');
const C = load();
const now = () => Number(process.hrtime.bigint()) / 1e6;
const gen = [], fineBuild = [], add = [], fineAstar = [], sliceMax = [];
for (let seed = 1; seed <= 60; seed++) {
  const w = new C.World(960, 640, 10);
  let t = now(); w.generate(seed, 36, 9); gen.push(now() - t);
  const slices = []; while (!w.fine.ready) { const t1 = now(); w.fine.buildStep(2); slices.push(now() - t1); } sliceMax.push(Math.max(...slices));
  t = now(); new C.NavGrid(w, w.fineCellSize, 9 + w.fineMargin); fineBuild.push(now() - t);
  for (let k = 0; k < 5; k++) { t = now(); w.addObstacle(100 + k * 170, 320, 9, w.start); add.push(now() - t); }
  const a = new C.AStar();
  a.start(w.fine, w.fine.seedCells(w.start.x, w.start.y, 9), w.fine.seedCells(w.goal.x, w.goal.y, 9));
  t = now(); while (a.step(1000) === 'running'); fineAstar.push(now() - t);
}
const st = (a) => { a.sort((x, y) => x - y); return `median ${a[a.length >> 1].toFixed(1)} ms   p95 ${a[Math.floor(a.length * 0.95)].toFixed(1)} ms   max ${a[a.length - 1].toFixed(1)} ms`; };
console.log('generate (coarse grid only)', st(gen));
console.log('fine build, worst 2ms slice', st(sliceMax));
console.log('fine grid full build      ', st(fineBuild));
console.log('addObstacle (validate)    ', st(add));
console.log('fine-grid A* (unsliced)   ', st(fineAstar));
