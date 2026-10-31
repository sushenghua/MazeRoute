// pathfinder.js — time-sliced A* on the nav grid + line-of-sight path smoothing.
// The search is resumable: step(budgetMs) expands nodes until the budget is spent, then
// yields to the frame. This is what lets planning share a 16 ms frame with simulation + render.

class MinHeap {
  constructor(score) { this.a = []; this.score = score; }
  get size() { return this.a.length; }
  push(v) { const a = this.a; a.push(v); let i = a.length - 1;
    while (i > 0) { const p = (i - 1) >> 1; if (this.score(a[p]) <= this.score(a[i])) break; [a[p], a[i]] = [a[i], a[p]]; i = p; } }
  pop() { const a = this.a; const top = a[0]; const last = a.pop();
    if (a.length) { a[0] = last; let i = 0;
      for (;;) { const l = 2 * i + 1, r = l + 1; let m = i;
        if (l < a.length && this.score(a[l]) < this.score(a[m])) m = l;
        if (r < a.length && this.score(a[r]) < this.score(a[m])) m = r;
        if (m === i) break; [a[m], a[i]] = [a[i], a[m]]; i = m; } }
    return top; }
  clear() { this.a.length = 0; }
}

class AStar {
  constructor() {
    this.grid = null;
    this.heap = new MinHeap((i) => this.f[i]);
    this.status = 'idle';
    this.expanded = 0;
    this.msSpent = 0;
    this.slices = 0;
  }

  /**
   * Begin a search on `grid` from any of `starts` to any of `goals` (arrays of {i, d}, d in px).
   * Multi-source / multi-goal keeps one bad seed cell from failing the whole search.
   * Buffers are reallocated only when the grid size changes.
   */
  start(grid, starts, goals) {
    this.grid = grid;
    const n = grid.cols * grid.rows;
    if (!this.g || this.g.length !== n) {
      this.g = new Float32Array(n); this.f = new Float32Array(n); this.parent = new Int32Array(n);
      this.closed = new Uint8Array(n); this.inOpen = new Uint8Array(n); this.isGoal = new Uint8Array(n);
    }
    this.g.fill(Infinity); this.f.fill(Infinity); this.parent.fill(-1);
    this.closed.fill(0); this.inOpen.fill(0); this.isGoal.fill(0); this.heap.clear();
    this.expanded = 0; this.msSpent = 0; this.slices = 0; this.reached = -1;
    if (!starts.length || !goals.length) { this.status = 'failed'; return; }
    this.goalIdx = goals[0].i;
    // Heuristic targets goals[0]; subtracting the spread of the goal set keeps it admissible.
    this.goalSpread = 0;
    for (const gl of goals) { this.isGoal[gl.i] = 1; this.goalSpread = Math.max(this.goalSpread, this.octile(gl.i, this.goalIdx)); }
    for (const st of starts) {
      const g0 = st.d / grid.cellSize;
      if (g0 < this.g[st.i]) { this.g[st.i] = g0; this.f[st.i] = g0 + this.h(st.i); this.heap.push(st.i); this.inOpen[st.i] = 1; }
    }
    this.status = 'running';
  }

  octile(a, b) {
    const cols = this.grid.cols;
    const dx = Math.abs((a % cols) - (b % cols));
    const dy = Math.abs(Math.floor(a / cols) - Math.floor(b / cols));
    return (dx + dy) + (Math.SQRT2 - 2) * Math.min(dx, dy);
  }

  h(i) { return Math.max(0, this.octile(i, this.goalIdx) - this.goalSpread); }

  /** Expand nodes until done or budgetMs elapsed. */
  step(budgetMs) {
    if (this.status !== 'running') return this.status;
    const t0 = performance.now();
    const w = this.grid, cols = w.cols, rows = w.rows, blocked = w.blocked;
    let n = 0;
    this.slices++;
    while (this.heap.size) {
      if ((n & 31) === 0 && performance.now() - t0 > budgetMs) { this.msSpent += performance.now() - t0; return this.status; }
      const cur = this.heap.pop();
      if (this.closed[cur]) continue; // stale heap entry
      this.closed[cur] = 1; this.inOpen[cur] = 0;
      this.expanded++; n++;
      if (this.isGoal[cur]) { this.reached = cur; this.status = 'found'; break; }
      const c = cur % cols, r = (cur - c) / cols;
      for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) {
        if (!dr && !dc) continue;
        const nr = r + dr, nc = c + dc;
        if (nr < 0 || nc < 0 || nr >= rows || nc >= cols) continue;
        const ni = nr * cols + nc;
        if (blocked[ni] || this.closed[ni]) continue;
        // No corner cutting: a diagonal needs both orthogonal neighbours free.
        if (dr && dc && (blocked[r * cols + nc] || blocked[nr * cols + c])) continue;
        const ng = this.g[cur] + (dr && dc ? Math.SQRT2 : 1);
        if (ng < this.g[ni]) {
          this.g[ni] = ng; this.f[ni] = ng + this.h(ni) * 1.001; this.parent[ni] = cur;
          this.heap.push(ni); this.inOpen[ni] = 1;
        }
      }
    }
    if (this.status === 'running') this.status = 'failed';
    this.msSpent += performance.now() - t0;
    return this.status;
  }

  /** Reconstruct the cell path (list of cell centres), goal last. */
  path() {
    const out = [];
    for (let i = this.reached; i !== -1; i = this.parent[i]) out.push(this.grid.cellCenter(i));
    return out.reverse();
  }
}

/**
 * Forward string-pulling: from each anchor, keep extending the straight segment while the swept
 * corridor of radius r stays clear, and start a new anchor at the last point that was still visible.
 * This needs one sweep test per path point. The earlier "farthest visible point first" version was
 * quadratic and cost hundreds of milliseconds on long fine-grid paths, blowing the frame budget.
 */
function smoothPath(points, world, r) {
  points = dropCollinear(points);
  if (points.length <= 2) return points.slice();
  const out = [points[0]];
  let i = 0, j = 1;
  while (j < points.length - 1) {
    const a = points[i], b = points[j + 1];
    if (world.segmentFree(a.x, a.y, b.x, b.y, r)) { j++; continue; }
    out.push(points[j]);
    i = j; j = i + 1;
  }
  out.push(points[points.length - 1]);
  return out;
}

/**
 * Last-resort escape (planning tier 2). The agent's body can be pushed into spots with less clearance
 * than any grid cell represents (e.g. pressed into a narrow neck), and from there no grid cell may be
 * reachable in a straight line. This floods the agent's real free space at 1 px resolution inside a
 * window around it, testing each pixel against the polygons only when the search reaches it, and stops
 * at the first pixel that lies in a fine-grid cell already connected to the goal.
 *
 * Resumable like A*: step(budgetMs) works through two phases (flood the goal's grid component, then
 * flood pixels) and yields when the budget is spent. status: 'running' | 'found' | 'failed';
 * when found, `result` is {path: [points from the agent], cell, length}.
 */
class EscapeSearch {
  constructor(world, grid, from, r, goalSeedIds, window = 160) {
    this.world = world; this.grid = grid; this.from = { x: from.x, y: from.y }; this.r = r;
    this.goalComp = new Uint8Array(grid.cols * grid.rows);
    this.stack = [];
    for (const g of goalSeedIds) if (!this.goalComp[g]) { this.goalComp[g] = 1; this.stack.push(g); }
    this.window = window;
    this.size = 2 * window + 1;
    this.x0 = Math.round(from.x) - window; this.y0 = Math.round(from.y) - window;
    this.state = new Uint8Array(this.size * this.size);
    this.prev = new Int32Array(this.size * this.size).fill(-1);
    this.startK = window * this.size + window;
    this.state[this.startK] = 1;
    this.queue = [this.startK]; this.qi = 0;
    this.phase = 0; // 0 = goal component flood, 1 = pixel flood
    this.status = goalSeedIds.length ? 'running' : 'failed';
    this.result = null;
  }

  step(budgetMs) {
    if (this.status !== 'running') return this.status;
    const t0 = performance.now();
    const g = this.grid, cols = g.cols, rows = g.rows, blocked = g.blocked, comp = this.goalComp;
    let n = 0;
    while (this.phase === 0) {
      if (!this.stack.length) { this.phase = 1; break; }
      if ((++n & 255) === 0 && performance.now() - t0 > budgetMs) return this.status;
      const i = this.stack.pop(), c = i % cols, rr = (i - c) / cols;
      for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) {
        if (!dr && !dc) continue;
        const nr = rr + dr, nc = c + dc;
        if (nr < 0 || nc < 0 || nr >= rows || nc >= cols) continue;
        const j = nr * cols + nc;
        if (blocked[j] || comp[j]) continue;
        if (dr && dc && (blocked[rr * cols + nc] || blocked[nr * cols + c])) continue;
        comp[j] = 1; this.stack.push(j);
      }
    }
    const size = this.size, x0 = this.x0, y0 = this.y0, st = this.state, prev = this.prev, q = this.queue;
    // Reduced clearance only right around the start, so an agent already touching a wall can get
    // moving; everywhere else it needs the fine grid's own margin, so the escape only finds routes the
    // grid would accept but samples too coarsely to see. With less clearance than that,
    // validation once approved a route the agent physically could not drive.
    const w = this.window;
    const isFree = (k) => {
      if (!st[k]) {
        const kx = k % size, ky = Math.floor(k / size), near = (kx - w) ** 2 + (ky - w) ** 2 <= 16;
        st[k] = this.world.discFree(x0 + kx, y0 + ky, near ? this.r - 0.3 : this.r + this.world.fineMargin) ? 1 : 2;
      }
      return st[k] === 1;
    };
    while (this.qi < q.length) {
      if ((++n & 63) === 0 && performance.now() - t0 > budgetMs) return this.status;
      const k = q[this.qi++], px = x0 + (k % size), py = y0 + Math.floor(k / size);
      const cell = g.cellIndexAt(px, py);
      if (k !== this.startK && comp[cell] && !blocked[cell]) {
        const path = [];
        for (let m = k; m !== -1; m = prev[m]) path.push({ x: x0 + (m % size), y: y0 + Math.floor(m / size) });
        path.reverse();
        path[0] = { x: this.from.x, y: this.from.y };
        this.result = { path, cell, length: pathLength(path) };
        return (this.status = 'found');
      }
      const kx = k % size, ky = Math.floor(k / size);
      for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const nx = kx + dx, ny = ky + dy;
        if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;
        const j = ny * size + nx;
        if (prev[j] !== -1 || j === this.startK || !isFree(j)) continue;
        prev[j] = k; q.push(j);
      }
    }
    return (this.status = 'failed');
  }
}

/**
 * Keep only the points where a grid path changes direction. A straight run of free cells is already a
 * clear segment, so this is lossless, and it shrinks long fine-grid paths several-fold before smoothing.
 */
function dropCollinear(points) {
  if (points.length <= 2) return points.slice();
  const out = [points[0]];
  for (let i = 1; i < points.length - 1; i++) {
    const a = out[out.length - 1], b = points[i], c = points[i + 1];
    const cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    const dot = (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y);
    if (Math.abs(cross) > 1e-9 || dot <= 0) out.push(b);
  }
  out.push(points[points.length - 1]);
  return out;
}

/** Run an escape search to completion (used by obstacle validation, which needs an answer now). */
function escapeRoute(world, grid, from, r, goalSeedIds, window = 160) {
  const e = new EscapeSearch(world, grid, from, r, goalSeedIds, window);
  return e.step(Infinity) === 'found' ? e.result : null;
}

function pathLength(pts) {
  let l = 0;
  for (let i = 1; i < pts.length; i++) l += V.dist(pts[i - 1], pts[i]);
  return l;
}
