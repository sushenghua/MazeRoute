// main.js — wires UI, render loop and the frame-budget scheduler together.

(function () {
  const W = 960, H = 640, CELL = 10, AGENT_R = 9;
  const canvas = document.getElementById('arena');
  const ctx = canvas.getContext('2d');
  const budgetCanvas = document.getElementById('budget');
  const bctx = budgetCanvas.getContext('2d');

  const world = new World(W, H, CELL);
  const agent = new Agent(AGENT_R);
  const astar = new AStar();
  const ctl = new Controller(world, agent, astar);
  const budget = new FrameBudget(1000 / 60);
  const clock = new FixedClock(1 / 60, 5);

  const $ = (id) => document.getElementById(id);
  const ui = {
    generate: $('btn-generate'), start: $('btn-start'), reset: $('btn-reset'),
    density: $('density'), densityVal: $('density-val'),
    speed: $('speed'), speedVal: $('speed-val'),
    planCap: $('plan-cap'), planCapVal: $('plan-cap-val'),
    showGrid: $('show-grid'), showSearch: $('show-search'), showWhiskers: $('show-whiskers'), showRaw: $('show-raw'),
    allowSealing: $('allow-sealing'),
    stateBadge: $('state-badge'), stats: $('stats'), log: $('log'), seed: $('seed'), toast: $('toast'),
  };

  let generated = false;
  let lastTime = performance.now();

  // ---------------- UI ----------------

  function generate() {
    const seed = (Math.random() * 1e9) >>> 0;
    const ok = world.generate(seed, parseInt(ui.density.value, 10), AGENT_R);
    ui.seed.textContent = ok ? `seed ${seed}` : 'generation failed';
    generated = ok;
    ctl.reset();
    astar.status = 'idle';
    ui.start.disabled = !ok;
    ui.start.textContent = 'Start';
  }

  ui.generate.addEventListener('click', generate);
  ui.start.addEventListener('click', () => { if (generated) { ctl.startMission(); ui.start.textContent = 'Restart'; } });
  ui.reset.addEventListener('click', () => { ctl.reset(); astar.status = 'idle'; ui.start.textContent = 'Start'; });
  ui.density.addEventListener('input', () => (ui.densityVal.textContent = ui.density.value));
  ui.speed.addEventListener('input', () => (ui.speedVal.textContent = `${parseFloat(ui.speed.value).toFixed(2)}×`));
  ui.planCap.addEventListener('input', () => (ui.planCapVal.textContent = `${parseFloat(ui.planCap.value).toFixed(2)} ms`));

  // Click in the arena while a mission is running to drop a new obstacle in the agent's way.
  canvas.addEventListener('click', (e) => {
    if (!generated) return;
    const rect = canvas.getBoundingClientRect();
    const x = ((e.clientX - rect.left) / rect.width) * W, y = ((e.clientY - rect.top) / rect.height) * H;
    const res = world.addObstacle(x, y, AGENT_R, agent.pos, { allowSealing: ui.allowSealing.checked });
    if (res.poly) {
      ctl.onWorldChanged();
      if (res.sealed) showToast('Obstacle placed: B is now sealed off from the agent');
    } else showToast(`Obstacle rejected: ${res.rejected}`);
  });

  let toastTimer = 0;
  function showToast(msg) {
    ui.toast.textContent = msg;
    ui.toast.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => ui.toast.classList.remove('show'), 1800);
  }

  // ---------------- frame loop ----------------

  function frame(now) {
    const frameDt = Math.min((now - lastTime) / 1000, 0.25);
    lastTime = now;
    budget.begin();

    // 1) Simulation: deterministic fixed steps, bounded per frame.
    clock.advance(frameDt, parseFloat(ui.speed.value), (dt) => ctl.tick(dt));
    budget.mark('sim');

    // 2) Planning: resumable A* gets only the slack left in the frame (capped by the slider).
    // Background work (fine-grid rasterisation) uses the same slack when no search is running.
    const slice = budget.planBudget(parseFloat(ui.planCap.value));
    if (ctl.isPlanning) ctl.plan(slice);
    else if (world.fine && !world.fine.ready) world.fine.buildStep(slice);
    budget.mark('plan');

    // 3) Render.
    render();
    budget.mark('render');
    budget.end(frameDt);
    renderBudget();
    if ((budget.frames % 6) === 0) renderStats();
    requestAnimationFrame(frame);
  }

  // ---------------- drawing ----------------

  function render() {
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = '#0f131a';
    ctx.fillRect(0, 0, W, H);

    // Faint grid
    ctx.strokeStyle = 'rgba(255,255,255,0.03)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = 0; x <= W; x += 40) { ctx.moveTo(x + 0.5, 0); ctx.lineTo(x + 0.5, H); }
    for (let y = 0; y <= H; y += 40) { ctx.moveTo(0, y + 0.5); ctx.lineTo(W, y + 0.5); }
    ctx.stroke();

    // Grid overlays are rasterised into a 1-pixel-per-cell image and scaled up in one draw call:
    // the fine grid has ~68k cells, far too many for per-cell fillRect inside a frame budget.
    if (ui.showGrid.checked && world.nav) {
      const g = astar.grid === world.fine && world.fine.ready ? world.fine : world.nav;
      drawCells(g, (i) => (g.blocked[i] ? 0x22ef4444 : 0));
    }
    if (ui.showSearch.checked && astar.status !== 'idle' && astar.grid) {
      const closed = astar.closed, open = astar.inOpen;
      drawCells(astar.grid, (i) => (closed[i] ? 0x3360a5fa : open[i] ? 0x88facc15 : 0));
    }

    // Obstacles
    for (const p of world.polys) {
      ctx.beginPath();
      ctx.moveTo(p.pts[0].x, p.pts[0].y);
      for (let i = 1; i < p.n; i++) ctx.lineTo(p.pts[i].x, p.pts[i].y);
      ctx.closePath();
      ctx.fillStyle = '#3b4252';
      ctx.fill();
      ctx.strokeStyle = '#5c6a85';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // Trail
    if (agent.trail.length > 1) {
      ctx.beginPath();
      ctx.moveTo(agent.trail[0].x, agent.trail[0].y);
      for (const t of agent.trail) ctx.lineTo(t.x, t.y);
      ctx.strokeStyle = 'rgba(79,209,197,0.35)';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Paths
    if (ui.showRaw.checked && ctl.rawPath.length > 1) drawPolyline(ctl.rawPath, 'rgba(148,163,184,0.35)', 1, []);
    if (ctl.path.length > 1) drawPolyline(ctl.path, '#38bdf8', 2, [8, 6]);
    if (ctl.target && ctl.state === S.FOLLOWING) { ctx.fillStyle = '#38bdf8'; ctx.beginPath(); ctx.arc(ctl.target.x, ctl.target.y, 3.5, 0, Math.PI * 2); ctx.fill(); }

    // A and B
    if (world.start) {
      marker(world.start, '#22c55e', 'A');
      marker(world.goal, '#ef4444', 'B');
      ctx.strokeStyle = 'rgba(239,68,68,0.6)'; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(world.goal.x, world.goal.y, AGENT_R + 3, 0, Math.PI * 2); ctx.stroke();
    }

    // Whiskers
    if (ui.showWhiskers.checked && ctl.state === S.FOLLOWING) {
      for (const w of ctl.whiskers) {
        const hx = agent.pos.x + (w.end.x - agent.pos.x) * w.t, hy = agent.pos.y + (w.end.y - agent.pos.y) * w.t;
        ctx.beginPath(); ctx.moveTo(agent.pos.x, agent.pos.y); ctx.lineTo(hx, hy);
        ctx.strokeStyle = w.hit ? 'rgba(249,115,22,0.9)' : 'rgba(255,255,255,0.25)'; ctx.lineWidth = 1; ctx.stroke();
        if (w.hit) { ctx.fillStyle = '#f97316'; ctx.beginPath(); ctx.arc(hx, hy, 2.5, 0, Math.PI * 2); ctx.fill(); }
      }
    }

    // Agent (directional triangle)
    if (world.start) {
      const a = agent, r = a.radius;
      ctx.save();
      ctx.translate(a.pos.x, a.pos.y);
      ctx.rotate(a.heading);
      ctx.beginPath();
      ctx.arc(0, 0, r, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(255,255,255,0.08)';
      ctx.fill();
      ctx.beginPath();
      ctx.moveTo(r + 2, 0); ctx.lineTo(-r * 0.7, r * 0.75); ctx.lineTo(-r * 0.3, 0); ctx.lineTo(-r * 0.7, -r * 0.75); ctx.closePath();
      ctx.fillStyle = STATE_COLORS[ctl.state];
      ctx.fill();
      ctx.strokeStyle = '#0f131a'; ctx.lineWidth = 1; ctx.stroke();
      ctx.restore();
    }
  }

  const overlay = document.createElement('canvas');
  const octx = overlay.getContext('2d');
  /** colorOf(i) returns 0xAARRGGBB or 0 for transparent. */
  function drawCells(grid, colorOf) {
    if (overlay.width !== grid.cols || overlay.height !== grid.rows) { overlay.width = grid.cols; overlay.height = grid.rows; }
    const img = octx.createImageData(grid.cols, grid.rows), d = img.data;
    for (let i = 0, n = grid.cols * grid.rows; i < n; i++) {
      const c = colorOf(i);
      if (!c) continue;
      const o = i * 4;
      d[o] = (c >>> 16) & 255; d[o + 1] = (c >>> 8) & 255; d[o + 2] = c & 255; d[o + 3] = c >>> 24;
    }
    octx.putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(overlay, 0, 0, grid.cols * grid.cellSize, grid.rows * grid.cellSize);
    ctx.imageSmoothingEnabled = true;
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
    bctx.clearRect(0, 0, w, h);
    bctx.fillStyle = '#0f131a'; bctx.fillRect(0, 0, w, h);
    const scale = h / 25; // 25 ms full height
    const bw = w / budget.maxHistory;
    budget.history.forEach((f, i) => {
      let y = h;
      for (const [k, c] of [['sim', '#4fd1c5'], ['plan', '#f6c453'], ['render', '#a78bfa']]) {
        const hh = f[k] * scale; y -= hh;
        bctx.fillStyle = c; bctx.fillRect(i * bw, y, Math.max(1, bw - 0.5), hh);
      }
    });
    const ty = h - budget.target * scale;
    bctx.strokeStyle = '#ef4444'; bctx.setLineDash([4, 3]); bctx.beginPath(); bctx.moveTo(0, ty + 0.5); bctx.lineTo(w, ty + 0.5); bctx.stroke(); bctx.setLineDash([]);
    bctx.fillStyle = '#ef4444'; bctx.font = '10px system-ui, sans-serif'; bctx.textAlign = 'right';
    bctx.fillText('16.7 ms', w - 3, ty - 3);
  }

  function renderStats() {
    ui.stateBadge.textContent = ctl.state;
    ui.stateBadge.style.background = STATE_COLORS[ctl.state];
    const e = budget.ema, st = ctl.stats;
    const rows = [
      ['time in state', `${ctl.timeInState.toFixed(1)} s / ${isFinite(ctl.deadline) ? ctl.deadline.toFixed(1) + ' s' : '∞'}`],
      ['sim time', `${ctl.simTime.toFixed(1)} s`],
      ['speed', `${Math.abs(agent.speed).toFixed(0)} px/s${agent.speed < -1 ? ' (reverse)' : ''}`],
      ['remaining path', `${ctl.remaining.toFixed(0)} px`],
      ['path points', `${ctl.rawPath.length} raw → ${ctl.path.length} smoothed`],
      ['A*', `${['coarse', 'fine', 'fine (escape)'][ctl.planLevel]} grid · ${st.planNodes} nodes · ${st.planSlices} slices · ${st.planMs.toFixed(2)} ms`],
      ['replans / recoveries', `${st.replans} / ${st.recoveries}`],
      ['collision corrections', `${st.collisions}`],
      ['stuck streak / escalation', `${ctl.stuckStreak} / ${st.escalation}`],
      ['frame (ema)', `sim ${e.sim.toFixed(2)} · plan ${e.plan.toFixed(2)} · render ${e.render.toFixed(2)} · total ${e.total.toFixed(2)} ms`],
      ['fps / over budget', `${budget.fps.toFixed(0)} · ${budget.overBudget} of ${budget.frames} frames`],
      ['dropped sim time', `${clock.droppedSeconds.toFixed(2)} s`],
      ['obstacles', `${world.polys.length} polygons`],
      ['nav grids', world.nav ? `coarse ${world.nav.cols}×${world.nav.rows} · fine ${world.fine.cols}×${world.fine.rows}${world.fine.ready ? '' : ` (building ${Math.round(100 * world.fine.builtRows / world.fine.rows)}%)`}` : '-'],
    ];
    ui.stats.innerHTML = rows.map(([k, v]) => `<div class="row"><span>${k}</span><b>${v}</b></div>`).join('');
    ui.log.innerHTML = ctl.log.map((l) => `<li><span class="t">${l.t.toFixed(1)}s</span> <span class="s" style="color:${STATE_COLORS[l.from]}">${l.from}</span> → <span class="s" style="color:${STATE_COLORS[l.to]}">${l.to}</span><br><em>${l.reason}</em></li>`).join('');
  }

  // Expose for debugging / automated checks.
  window.demo = { world, agent, astar, ctl, budget, clock, generate };

  generate();
  requestAnimationFrame(frame);
})();
