// app.js — draws what the ESP32-S3 reports. No simulation runs in the browser: every position, path,
// state transition and timing below arrives from the chip as one JSON object per WebSocket message.
(() => {
  'use strict';

  const STATE_COLORS = {
    IDLE: '#8b93a7', PLANNING: '#f6c453', FOLLOWING: '#4fd1c5', RECOVERING: '#f97316',
    REPLANNING: '#f6c453', ARRIVED: '#22c55e', FAILED: '#ef4444',
  };
  const TIERS = ['coarse', 'fine', 'fine via pixel escape'];
  const FRAME_MS = 1000 / 60;

  const $ = (id) => document.getElementById(id);
  const canvas = $('arena'), ctx = canvas.getContext('2d');
  const budgetCanvas = $('budget'), bctx = budgetCanvas.getContext('2d');
  const W = canvas.width, H = canvas.height;

  const ui = {
    conn: $('conn'), generate: $('btn-generate'), start: $('btn-start'), reset: $('btn-reset'),
    density: $('density'), densityVal: $('density-val'), planCap: $('plan-cap'), planCapVal: $('plan-cap-val'),
    allowSealing: $('allow-sealing'), showRaw: $('show-raw'), showTrail: $('show-trail'),
    badge: $('state-badge'), stats: $('stats'), log: $('log'), toast: $('toast'),
    bench: $('bench').tBodies[0], benchBtn: $('btn-bench'),
    suite: $('suite').tBodies[0], suiteBtn: $('btn-suite'), suiteN: $('suite-n'), suiteTotal: $('suite-total'), stop: $('btn-stop'),
  };

  // Everything the chip has told us.
  const chip = {
    info: null, world: null, s: null, path: [], raw: [], log: [], trail: [], frames: [],
    lastPath: null, summary: null, lastDrop: null, busy: null,
  };

  // ---------------- connection ----------------

  let ws = null;
  function connect() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onmessage = (ev) => onMessage(JSON.parse(ev.data));
    ws.onclose = () => { setConn(false, 'bridge offline'); setTimeout(connect, 1000); };
  }
  function send(cmd) { if (ws && ws.readyState === 1) ws.send(cmd); }
  function setConn(on, text) { ui.conn.textContent = text; ui.conn.className = `pill ${on ? 'on' : 'off'}`; }

  const pairs = (flat) => { const out = []; for (let i = 0; i + 1 < flat.length; i += 2) out.push({ x: flat[i], y: flat[i + 1] }); return out; };

  function onMessage(m) {
    switch (m.k) {
      case 'bridge':
        setConn(m.connected, m.connected ? `ESP32-S3 on ${m.port.replace('/dev/cu.', '')}` : 'serial closed');
        if (m.connected) send('info');
        break;
      case 'hello': toast('chip rebooted'); chip.world = null; send('info'); break;
      case 'cleared':  // the chip freed the arena to run a benchmark or the test suite
        chip.world = null; chip.s = null; chip.path = []; chip.raw = []; chip.trail = [];
        ui.start.disabled = true;
        break;
      case 'info':
        chip.info = m;
        setConn(true, `ESP32-S3 · ${m.mhz} MHz · ${m.real} · PSRAM ${(m.heap.psramTotal / 1048576).toFixed(0)} MB`);
        break;
      case 'world':
        chip.world = { ...m, start: { x: m.start[0], y: m.start[1] }, goal: { x: m.goal[0], y: m.goal[1] }, polys: m.polys.map(pairs) };
        chip.s = null; chip.path = []; chip.raw = []; chip.log = []; chip.trail = []; chip.frames = []; chip.summary = null; chip.lastPath = null;
        chip.genMs = m.genMs; chip.heap = m.heap;
        ui.start.disabled = false;
        break;
      case 's':
        chip.s = m;
        if (m.st !== 'IDLE') {
          const last = chip.trail[chip.trail.length - 1];
          if (!last || Math.hypot(last.x - m.x, last.y - m.y) > 2) chip.trail.push({ x: m.x, y: m.y });
        }
        chip.frames.push({ drop: m.drop || 0, plan: m.plan, tick: m.tick });
        if (chip.frames.length > 240) chip.frames.shift();
        if (m.st === 'IDLE') { chip.path = []; chip.raw = []; }
        break;
      case 'tr':
        chip.log.unshift(m);
        if (chip.log.length > 40) chip.log.pop();
        break;
      case 'path':
        chip.path = pairs(m.path); chip.raw = pairs(m.raw);
        if (chip.path.length) chip.lastPath = m;
        break;
      case 'done': chip.summary = m; chip.heap = m.heap; break;
      case 'drop':
        chip.lastDrop = m;
        if (m.ok) {
          chip.world.polys.push(pairs(m.poly));
          toast(`${m.sealed ? 'B is now sealed off · ' : ''}validated on chip: ${m.ms.toFixed(1)} ms of work over ${m.frames} frame${m.frames === 1 ? '' : 's'}`, m.sealed);
        } else toast(`rejected on chip: ${m.reason}${m.frames ? ` (after ${m.frames} frames)` : ''}`);
        break;
      case 'benchProgress': ui.bench.innerHTML = `<tr><td colspan="4" class="muted">running on chip… ${m.done}/${m.of} worlds</td></tr>`; break;
      case 'bench':
        chip.busy = null;
        ui.bench.innerHTML = m.rows.map((r) => `<tr><td>${r.label}</td><td>${fmtMs(r.median)}</td><td>${fmtMs(r.p90)}</td><td>${fmtMs(r.max)}</td></tr>`).join('') +
          `<tr><td class="muted" colspan="4">PSRAM high-water ${((m.heap.psramTotal - m.heap.psramMin) / 1048576).toFixed(2)} MB · internal RAM free ${(m.heap.intFree / 1024).toFixed(0)} KB</td></tr>`;
        break;
      case 'suiteRow': {
        if (ui.suite.querySelector('.muted')) ui.suite.innerHTML = '';
        const tr = document.createElement('tr');
        tr.innerHTML = `<td>${m.scenario}</td><td>${m.runs}</td><td>${m.arrived}</td><td>${m.honestFailed}</td><td class="${m.fail ? 'bad' : 'good'}">${m.fail}</td>`;
        ui.suite.appendChild(tr);
        ui.suiteTotal.textContent = `running on chip… ${m.elapsed.toFixed(0)} s`;
        break;
      }
      case 'suiteFail': console.warn('suite failure', m); break;
      case 'suite':
        chip.busy = null;
        ui.suiteTotal.innerHTML = `<b class="${m.fail ? 'bad' : 'good'}">${m.runs} missions, ${m.fail} failures</b> in ${m.seconds.toFixed(0)} s on the chip${m.aborted ? ' (stopped early)' : ''}`;
        break;
      case 'error': toast(`chip: ${m.msg}`); break;
    }
  }

  const fmtMs = (v) => (v >= 100 ? v.toFixed(0) : v >= 10 ? v.toFixed(1) : v.toFixed(2)) + ' ms';

  // ---------------- controls ----------------

  ui.generate.onclick = () => send(`gen ${(Math.random() * 1e9) >>> 0} ${ui.density.value}`);
  ui.start.onclick = () => { chip.trail = []; chip.log = []; chip.summary = null; chip.frames = []; send('start'); };
  ui.reset.onclick = () => { chip.trail = []; chip.log = []; chip.summary = null; send('reset'); };
  ui.density.oninput = () => { ui.densityVal.textContent = ui.density.value; };
  ui.planCap.oninput = () => { ui.planCapVal.textContent = `${ui.planCap.value} ms`; send(`budget ${ui.planCap.value} ${ui.planCap.value} ${Math.max(1, ui.planCap.value - 1)}`); };
  ui.benchBtn.onclick = () => { chip.busy = 'bench'; ui.bench.innerHTML = '<tr><td colspan="4" class="muted">starting…</td></tr>'; send('bench 20'); };
  ui.suiteBtn.onclick = () => {
    chip.busy = 'suite';
    ui.suite.innerHTML = '<tr><td colspan="5" class="muted">starting…</td></tr>';
    ui.suiteTotal.textContent = '';
    send(`suite ${Math.max(1, Math.min(300, ui.suiteN.value | 0))} 1 ${ui.planCap.value}`);  // same slices as live
  };
  ui.stop.onclick = () => send('stop');
  canvas.addEventListener('click', (e) => {
    if (!chip.world) return;
    const r = canvas.getBoundingClientRect();
    const x = ((e.clientX - r.left) * W) / r.width, y = ((e.clientY - r.top) * H) / r.height;
    send(`drop ${x.toFixed(1)} ${y.toFixed(1)} ${ui.allowSealing.checked ? 1 : 0}`);
  });

  let toastTimer = 0;
  function toast(text, warn) {
    ui.toast.textContent = text;
    ui.toast.style.background = warn ? '#ef4444' : '#f97316';
    ui.toast.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => ui.toast.classList.remove('show'), 2600);
  }

  // ---------------- drawing ----------------

  function render() {
    ctx.fillStyle = '#0f131a';
    ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = 'rgba(255,255,255,0.03)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = 0; x <= W; x += 40) { ctx.moveTo(x + 0.5, 0); ctx.lineTo(x + 0.5, H); }
    for (let y = 0; y <= H; y += 40) { ctx.moveTo(0, y + 0.5); ctx.lineTo(W, y + 0.5); }
    ctx.stroke();

    const w = chip.world;
    if (!w) {
      ctx.fillStyle = '#8b93a7'; ctx.font = '15px system-ui, sans-serif'; ctx.textAlign = 'center';
      ctx.fillText('Press Generate: the ESP32-S3 builds the arena and sends it here.', W / 2, H / 2);
      return;
    }
    for (const p of w.polys) {
      ctx.beginPath();
      ctx.moveTo(p[0].x, p[0].y);
      for (let i = 1; i < p.length; i++) ctx.lineTo(p[i].x, p[i].y);
      ctx.closePath();
      ctx.fillStyle = '#3b4252'; ctx.fill();
      ctx.strokeStyle = '#5c6a85'; ctx.lineWidth = 1.5; ctx.stroke();
    }
    if (ui.showTrail.checked && chip.trail.length > 1) drawPolyline(chip.trail, 'rgba(79,209,197,0.35)', 2, []);
    if (ui.showRaw.checked && chip.raw.length > 1) drawPolyline(chip.raw, 'rgba(148,163,184,0.45)', 1, []);
    if (chip.path.length > 1) drawPolyline(chip.path, '#38bdf8', 2, [8, 6]);
    const s = chip.s;
    if (s && s.st === 'FOLLOWING') { ctx.fillStyle = s.tv ? '#38bdf8' : '#f97316'; ctx.beginPath(); ctx.arc(s.tx, s.ty, 3.5, 0, Math.PI * 2); ctx.fill(); }

    marker(w.start, '#22c55e', 'A');
    marker(w.goal, '#ef4444', 'B');
    ctx.strokeStyle = 'rgba(239,68,68,0.6)'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(w.goal.x, w.goal.y, w.r + 3, 0, Math.PI * 2); ctx.stroke();

    const a = s && s.st !== 'IDLE' ? s : { x: w.start.x, y: w.start.y, h: 0, st: 'IDLE' };
    const r = w.r;
    ctx.save();
    ctx.translate(a.x, a.y);
    ctx.rotate(a.h);
    ctx.beginPath(); ctx.arc(0, 0, r, 0, Math.PI * 2); ctx.fillStyle = 'rgba(255,255,255,0.08)'; ctx.fill();
    ctx.beginPath();
    ctx.moveTo(r + 2, 0); ctx.lineTo(-r * 0.7, r * 0.75); ctx.lineTo(-r * 0.3, 0); ctx.lineTo(-r * 0.7, -r * 0.75); ctx.closePath();
    ctx.fillStyle = STATE_COLORS[a.st]; ctx.fill();
    ctx.strokeStyle = '#0f131a'; ctx.lineWidth = 1; ctx.stroke();
    ctx.restore();
  }

  function drawPolyline(pts, color, width, dash) {
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
    ctx.setLineDash(dash); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); ctx.setLineDash([]);
  }
  function marker(p, color, label) {
    ctx.fillStyle = color; ctx.beginPath(); ctx.arc(p.x, p.y, 6, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#fff'; ctx.font = 'bold 13px system-ui, sans-serif'; ctx.textAlign = 'center';
    ctx.fillText(label, p.x, p.y - 12);
  }

  function renderBudget() {
    const w = budgetCanvas.width, h = budgetCanvas.height;
    bctx.fillStyle = '#0f131a'; bctx.fillRect(0, 0, w, h);
    const scale = h / 25;  // 25 ms full height
    const bw = w / 240;
    chip.frames.forEach((f, i) => {
      let y = h;
      for (const [v, c] of [[f.drop, '#f472b6'], [f.plan, '#f6c453'], [f.tick, '#4fd1c5']]) {
        const hh = Math.min(h, v * scale); y -= hh;
        bctx.fillStyle = c; bctx.fillRect(i * bw, y, Math.max(1, bw - 0.3), hh);
      }
    });
    const ty = h - FRAME_MS * scale;
    bctx.strokeStyle = '#ef4444'; bctx.setLineDash([4, 3]); bctx.beginPath(); bctx.moveTo(0, ty + 0.5); bctx.lineTo(w, ty + 0.5); bctx.stroke(); bctx.setLineDash([]);
    bctx.fillStyle = '#ef4444'; bctx.font = '10px system-ui, sans-serif'; bctx.textAlign = 'right';
    bctx.fillText('16.7 ms', w - 3, ty - 3);
  }

  let lastStatsKey = '';
  function renderStats() {
    const s = chip.s, w = chip.world, sum = chip.summary, lp = chip.lastPath;
    const st = s ? s.st : 'IDLE';
    ui.badge.textContent = st;
    ui.badge.style.background = STATE_COLORS[st];
    const fine = w ? w.fine[1] : 320;
    const rows = [
      ['sim time', s ? `${s.t.toFixed(1)} s` : '–'],
      ['speed', s ? `${s.v.toFixed(0)} px/s` : '–'],
      ['remaining path', s && st !== 'IDLE' ? `${s.remaining.toFixed(0)} px` : '–'],
      ['last plan', lp ? `${TIERS[lp.tier]} grid · ${lp.nodes} nodes · ${lp.slices} slice${lp.slices > 1 ? 's' : ''} · ${lp.planMs.toFixed(1)} ms` : '–'],
      ['replans / recoveries / collisions', s ? `${s.replans} / ${s.recov} / ${s.coll}` : '–'],
      ['fine grid (background build)', s ? (s.fineRows >= fine ? 'ready' : `${Math.round((100 * s.fineRows) / fine)} %`) : '–'],
      ['this frame: drop + plan + tick', s ? `${(s.drop || 0).toFixed(2)} + ${s.plan.toFixed(2)} + ${s.tick.toFixed(2)} ms` : '–'],
      ['generate on chip', w ? `${w.genMs.toFixed(1)} ms` : '–'],
    ];
    if (sum) {
      rows.push(['mission frames over 16.7 ms', `${sum.overBudget} of ${sum.frames}`]);
      rows.push(['worst / mean frame', `${sum.worstFrame.toFixed(2)} / ${sum.avgFrame.toFixed(2)} ms`]);
    }
    if (chip.heap) rows.push(['PSRAM used (peak)', `${((chip.heap.psramTotal - chip.heap.psramMin) / 1048576).toFixed(2)} MB`]);
    const key = JSON.stringify(rows);
    if (key !== lastStatsKey) {
      lastStatsKey = key;
      ui.stats.innerHTML = rows.map(([k, v]) => `<div class="row"><span>${k}</span><b>${v}</b></div>`).join('');
    }
    const logKey = chip.log.length + (chip.log[0] ? chip.log[0].t + chip.log[0].to : '');
    if (logKey !== renderStats.logKey) {
      renderStats.logKey = logKey;
      ui.log.innerHTML = chip.log.map((l) => `<li><span class="t">${l.t.toFixed(1)}s</span> <span class="s" style="color:${STATE_COLORS[l.from]}">${l.from}</span> → <span class="s" style="color:${STATE_COLORS[l.to]}">${l.to}</span><br><em>${l.reason}</em></li>`).join('');
    }
  }

  function frame() {
    render();
    renderBudget();
    renderStats();
    requestAnimationFrame(frame);
  }

  window.chip = chip;  // for inspection from devtools
  connect();
  requestAnimationFrame(frame);
})();
