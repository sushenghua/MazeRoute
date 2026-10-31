// world.js — procedural arena with irregular obstacles, a spatial hash for broadphase,
// and a navigation grid inflated by the agent radius so A* plans for a disc, not a point.

function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Side clearance (px) beyond the agent's radius that the fine planner is guaranteed to find. */
const FINE_GUARANTEE = 2.5;


class World {
  constructor(width, height, cellSize) {
    this.width = width;
    this.height = height;
    this.cellSize = cellSize;   // coarse planning grid
    // Fallback grid. With 2 px cells and a 1.1 px margin it is guaranteed to find any gap that leaves
    // at least FINE_GUARANTEE px of clearance on each side of the agent (strip of cell centres with
    // enough clearance is then at least one cell diagonal wide), while every grid edge keeps the
    // body within ~0.3 px of contact.
    this.fineCellSize = 2;
    this.fineMargin = 1.1;
    this.polys = [];
    this.start = null;
    this.goal = null;
    this.seed = 0;
    this.hashSize = 80; // spatial hash bucket size in px
    this.hashCols = Math.ceil(width / this.hashSize);
    this.hashRows = Math.ceil(height / this.hashSize);
    this.hash = [];
    this.nav = null;   // coarse grid: comfortable margin, fast A*, smooth paths
    this.fine = null;  // fine grid: tight margin, used when the coarse grid finds no route
    this.version = 0;
  }

  // ---------------- generation ----------------

  generate(seed, density, agentRadius) {
    this.seed = seed;
    const rng = mulberry32(seed);
    const diag = Math.hypot(this.width, this.height);
    for (let attempt = 0; attempt < 40; attempt++) {
      this.polys = [];
      const nBlobs = Math.round(density * 0.6), nBars = Math.round(density * 0.4);
      for (let i = 0; i < nBlobs; i++) {
        const r = 28 + rng() * 55;
        this.polys.push(makeBlob(rng, r + rng() * (this.width - 2 * r), r + rng() * (this.height - 2 * r), r, 6 + Math.floor(rng() * 7)));
      }
      for (let i = 0; i < nBars; i++) {
        const len = 90 + rng() * 220;
        const ang = rng() < 0.5 ? (rng() - 0.5) * 0.5 : Math.PI / 2 + (rng() - 0.5) * 0.5; // mostly axis-ish, like a maze
        this.polys.push(makeBar(rng, 60 + rng() * (this.width - 120), 60 + rng() * (this.height - 120), len, 7 + rng() * 8, ang + (rng() - 0.5) * 0.4));
      }
      this.rebuildHash();
      this.nav = new NavGrid(this, this.cellSize, agentRadius + 5);

      // Pick A and B far apart in free cells that are mutually reachable.
      const free = [];
      const nav = this.nav;
      for (let i = 0; i < nav.blocked.length; i++) if (!nav.blocked[i]) free.push(i);
      if (free.length < 50) continue;
      let ok = false;
      for (let tries = 0; tries < 60 && !ok; tries++) {
        const a = free[Math.floor(rng() * free.length)];
        const b = free[Math.floor(rng() * free.length)];
        const pa = nav.cellCenter(a), pb = nav.cellCenter(b);
        if (V.dist(pa, pb) < diag * 0.55) continue;
        if (nav.reachable(a, b)) {
          this.start = pa; this.goal = pb; ok = true;
        }
      }
      if (ok) {
        this.agentRadius = agentRadius;
        // The fine grid costs tens of ms to rasterise, so it is built in time slices by the frame
        // loop (see main.js) rather than inside this click handler.
        this.fine = new NavGrid(this, this.fineCellSize, agentRadius + this.fineMargin, true);
        this.version++;
        return true;
      }
      density = Math.max(6, density - 2); // ease off and try again
    }
    return false;
  }

  /**
   * Add an obstacle at runtime (user click). Drops that would overlap the agent or cover B are always
   * rejected. A drop that would cut B off from the agent (at the agent's true size) is rejected too,
   * unless `allowSealing` is set: then it is placed and reported as `sealed`, and the controller is
   * expected to end in FAILED with a reason. Returns {poly, sealed} or {rejected: reason}.
   */
  addObstacle(x, y, agentRadius, agentPos, { allowSealing = false } = {}) {
    const rng = mulberry32((x * 7919 + y * 104729 + this.version * 31) | 0);
    const p = makeBlob(rng, x, y, 26 + rng() * 22, 6 + Math.floor(rng() * 5));
    const from = agentPos || this.start;
    if (p.circlePenetration(from.x, from.y, agentRadius + 4)) return { rejected: 'would overlap the agent' };
    if (p.circlePenetration(this.goal.x, this.goal.y, agentRadius + 4)) return { rejected: 'would cover B' };
    if (!this.fine.ready) this.fine.buildStep(Infinity); // rare: click within the first few frames
    this.polys.push(p);
    this.onGeometryChanged(p);
    let sealed = !this.fine.gridConnected(from, this.goal, agentRadius);
    if (sealed) {
      // No grid route is left, only possibly a pixel-level squeeze (the planner's escape tier). Trust that
      // only if the agent was already off the grid before this drop. A drop that removes the last grid route
      // seals B: the agent can get stuck in the squeeze, back out of it, and lose it (seed 2589).
      this.polys.pop();
      this.onGeometryChanged(p);
      const hadGridRoute = this.fine.gridConnected(from, this.goal, agentRadius);
      this.polys.push(p);
      this.onGeometryChanged(p);
      sealed = hadGridRoute || !this.fine.connected(from, this.goal, agentRadius);
    }
    if (sealed && !allowSealing) {
      this.polys.pop();
      this.onGeometryChanged(p);
      return { rejected: 'would seal B off from the agent' };
    }
    this.version++;
    return { poly: p, sealed };
  }

  /** Incrementally refresh broadphase and both nav grids around a changed polygon. */
  onGeometryChanged(p) {
    this.rebuildHash();
    for (const g of [this.nav, this.fine]) g.updateRegion(p.minX, p.minY, p.maxX, p.maxY);
  }

  // ---------------- spatial hash (broadphase) ----------------

  rebuildHash() {
    this.hash = new Array(this.hashCols * this.hashRows);
    for (let i = 0; i < this.hash.length; i++) this.hash[i] = [];
    this.polys.forEach((p, idx) => {
      const x0 = clamp(Math.floor(p.minX / this.hashSize), 0, this.hashCols - 1);
      const x1 = clamp(Math.floor(p.maxX / this.hashSize), 0, this.hashCols - 1);
      const y0 = clamp(Math.floor(p.minY / this.hashSize), 0, this.hashRows - 1);
      const y1 = clamp(Math.floor(p.maxY / this.hashSize), 0, this.hashRows - 1);
      for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) this.hash[y * this.hashCols + x].push(idx);
    });
  }

  /** Polygons whose AABB-bucket overlaps the query box. De-duplicated via stamp. */
  query(minX, minY, maxX, maxY) {
    const out = [];
    const x0 = clamp(Math.floor(minX / this.hashSize), 0, this.hashCols - 1);
    const x1 = clamp(Math.floor(maxX / this.hashSize), 0, this.hashCols - 1);
    const y0 = clamp(Math.floor(minY / this.hashSize), 0, this.hashRows - 1);
    const y1 = clamp(Math.floor(maxY / this.hashSize), 0, this.hashRows - 1);
    this._stamp = (this._stamp || 0) + 1;
    for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
      for (const idx of this.hash[y * this.hashCols + x]) {
        const p = this.polys[idx];
        if (p._stamp === this._stamp) continue;
        p._stamp = this._stamp;
        if (p.maxX < minX || p.minX > maxX || p.maxY < minY || p.minY > maxY) continue;
        out.push(p);
      }
    }
    return out;
  }

  // ---------------- queries ----------------

  /** True if a disc of radius r at (x,y) is free of obstacles and inside the arena. */
  discFree(x, y, r) {
    if (x - r < 0 || y - r < 0 || x + r > this.width || y + r > this.height) return false;
    for (const p of this.query(x - r, y - r, x + r, y + r)) if (p.circlePenetration(x, y, r)) return false;
    return true;
  }

  /** True if a disc of radius r can sweep from a to b without touching anything. */
  segmentFree(ax, ay, bx, by, r) {
    const m = r;
    if (Math.min(ax, bx) - m < 0 || Math.min(ay, by) - m < 0 || Math.max(ax, bx) + m > this.width || Math.max(ay, by) + m > this.height) return false;
    const polys = this.query(Math.min(ax, bx) - r, Math.min(ay, by) - r, Math.max(ax, bx) + r, Math.max(ay, by) + r);
    for (const p of polys) if (p.segmentDistance(ax, ay, bx, by).d < r) return false;
    return true;
  }

  /** Earliest t in [0,1] where a disc sweeping a→b touches something (1 = clear). */
  sweep(ax, ay, bx, by, r) {
    let t = 1;
    const polys = this.query(Math.min(ax, bx) - r, Math.min(ay, by) - r, Math.max(ax, bx) + r, Math.max(ay, by) + r);
    for (const p of polys) { const h = p.sweepHit(ax, ay, bx, by, r); if (h < t) t = h; }
    // Arena walls.
    const dx = bx - ax, dy = by - ay;
    const wall = (lim, a, d) => (d !== 0 ? (lim - a) / d : Infinity);
    for (const tw of [wall(r, ax, dx), wall(this.width - r, ax, dx), wall(r, ay, dy), wall(this.height - r, ay, dy)]) {
      if (tw >= 0 && tw < t) t = tw;
    }
    return t;
  }

  /** Resolve a disc out of all overlapping obstacles and arena walls. Returns number of corrections. */
  resolveDisc(pos, r) {
    let hits = 0;
    for (let iter = 0; iter < 3; iter++) {
      let moved = false;
      for (const p of this.query(pos.x - r, pos.y - r, pos.x + r, pos.y + r)) {
        const pen = p.circlePenetration(pos.x, pos.y, r);
        if (pen) { pos.x += pen.x; pos.y += pen.y; moved = true; hits++; }
      }
      if (pos.x < r) { pos.x = r; moved = true; }
      if (pos.y < r) { pos.y = r; moved = true; }
      if (pos.x > this.width - r) { pos.x = this.width - r; moved = true; }
      if (pos.y > this.height - r) { pos.y = this.height - r; moved = true; }
      if (!moved) break;
    }
    return hits;
  }

  /** Nearest obstacle surface to a point within `range`. Returns {d, nx, ny} or null. */
  nearestSurface(x, y, range) {
    let best = null;
    for (const p of this.query(x - range, y - range, x + range, y + range)) {
      const e = p.nearestEdgePoint(x, y);
      const inside = p.contains(x, y);
      const d = inside ? -e.d : e.d;
      if (!best || d < best.d) {
        let nx = x - e.qx, ny = y - e.qy; const l = Math.hypot(nx, ny) || 1; nx /= l; ny /= l;
        if (inside) { nx = -nx; ny = -ny; }
        best = { d, nx, ny };
      }
    }
    // Arena walls count as surfaces too.
    const walls = [[x, 1, 0], [this.width - x, -1, 0], [y, 0, 1], [this.height - y, 0, -1]];
    for (const [d, nx, ny] of walls) if (d < range && (!best || d < best.d)) best = { d, nx, ny };
    return best && best.d < range ? best : null;
  }
}

// ---------------- navigation grid ----------------

/**
 * Occupancy grid for one agent size. A cell is blocked when a disc of radius `inflation` centred on
 * it would touch an obstacle, so A* can treat the agent as a point. Updates are local (only cells
 * near a changed polygon are re-tested), which keeps runtime edits cheap even on the fine grid.
 */
class NavGrid {
  constructor(world, cellSize, inflation, deferred = false) {
    this.world = world;
    this.cellSize = cellSize;
    this.inflation = inflation;
    this.cols = Math.floor(world.width / cellSize);
    this.rows = Math.floor(world.height / cellSize);
    this.blocked = new Uint8Array(this.cols * this.rows);
    this.builtRows = 0;
    this.ready = false;
    if (!deferred) this.buildStep(Infinity);
  }

  /** Time-sliced full build: rasterise bands of rows until `budgetMs` is spent. Returns `ready`. */
  buildStep(budgetMs) {
    const t0 = performance.now(), band = 4, cs = this.cellSize;
    while (this.builtRows < this.rows) {
      const r1 = Math.min(this.rows, this.builtRows + band);
      // updateRegion pads by the inflation; clamp the rows it rewrites to this band only.
      this._rowClamp = [this.builtRows, r1 - 1];
      this.updateRegion(0, this.builtRows * cs + this.inflation + cs, this.world.width, (r1 - 1) * cs - this.inflation - cs);
      this._rowClamp = null;
      this.builtRows = r1;
      if (performance.now() - t0 > budgetMs) break;
    }
    this.ready = this.builtRows >= this.rows;
    return this.ready;
  }

  /**
   * Re-test cells near a changed area. Iterates per polygon over only the cells its inflated
   * bounding box covers (no per-cell broadphase queries), which keeps a full 3 px rebuild cheap.
   */
  updateRegion(minX, minY, maxX, maxY) {
    const cs = this.cellSize, inf = this.inflation, m = inf + cs, W = this.world.width, H = this.world.height;
    const c0 = clamp(Math.floor((minX - m) / cs), 0, this.cols - 1), c1 = clamp(Math.ceil((maxX + m) / cs), 0, this.cols - 1);
    let r0 = clamp(Math.floor((minY - m) / cs), 0, this.rows - 1), r1 = clamp(Math.ceil((maxY + m) / cs), 0, this.rows - 1);
    if (this._rowClamp) { r0 = this._rowClamp[0]; r1 = this._rowClamp[1]; }
    const b = this.blocked, cols = this.cols;
    for (let r = r0; r <= r1; r++) {
      const y = (r + 0.5) * cs;
      for (let c = c0; c <= c1; c++) {
        const x = (c + 0.5) * cs;
        b[r * cols + c] = x < inf || y < inf || x > W - inf || y > H - inf ? 1 : 0;
      }
    }
    for (const p of this.world.polys) {
      if (p.maxX + inf < c0 * cs || p.minX - inf > (c1 + 1) * cs || p.maxY + inf < r0 * cs || p.minY - inf > (r1 + 1) * cs) continue;
      const pc0 = Math.max(c0, Math.floor((p.minX - inf) / cs)), pc1 = Math.min(c1, Math.ceil((p.maxX + inf) / cs));
      const pr0 = Math.max(r0, Math.floor((p.minY - inf) / cs)), pr1 = Math.min(r1, Math.ceil((p.maxY + inf) / cs));
      for (let r = pr0; r <= pr1; r++) {
        const y = (r + 0.5) * cs;
        for (let c = pc0; c <= pc1; c++) {
          const i = r * cols + c;
          if (!b[i] && p.circlePenetration((c + 0.5) * cs, y, inf)) b[i] = 1;
        }
      }
    }
  }

  cellIndexAt(x, y) {
    const c = clamp(Math.floor(x / this.cellSize), 0, this.cols - 1);
    const r = clamp(Math.floor(y / this.cellSize), 0, this.rows - 1);
    return r * this.cols + c;
  }
  cellCenter(i) {
    return { x: ((i % this.cols) + 0.5) * this.cellSize, y: (Math.floor(i / this.cols) + 0.5) * this.cellSize };
  }

  /**
   * Free cells the agent can drive to in a straight line from (x, y), nearest first, each with its
   * distance in px. Two traps this avoids:
   *  - the nearest free cell can lie behind a thin wall (hence the line-of-sight sweep);
   *  - the nearest visible cell can be a tiny isolated island of free cells in a pocket corner.
   *    Returning every visible cell within a body-width band lets the searches seed from all of
   *    them, so one dead island cannot hide the real exit.
   */
  seedCells(x, y, agentRadius, maxDist = 60) {
    // Sweep the agent's full body (minus a hair so a disc already touching a wall still counts):
    // a thinner sweep can "see" through a gap the agent cannot fit through, which once let a
    // validated obstacle drop seal the agent in.
    // ...and no looser than the grid itself when the agent has room: with a thinner sweep a seed can
    // be "seen" through a neck the grid would never traverse, so validation approves a world whose
    // only exit needs the agent to squeeze through. Sweep = current clearance minus a hair, clamped
    // between (radius - 0.25) and (radius + fine margin).
    const w = this.world, cs = this.cellSize;
    const surf = w.nearestSurface(x, y, agentRadius + w.fineMargin + 1);
    const clear = surf ? surf.d : agentRadius + w.fineMargin;
    const sweepR = clamp(clear - 0.05, agentRadius - 0.25, agentRadius + w.fineMargin);
    const i0 = this.cellIndexAt(x, y);
    const c0 = i0 % this.cols, r0 = Math.floor(i0 / this.cols);
    const maxRing = Math.ceil(maxDist / cs), band = Math.ceil((2 * agentRadius + 6) / cs);
    let lastRing = maxRing;
    const out = [];
    for (let ring = 0; ring <= lastRing; ring++) {
      for (let dr = -ring; dr <= ring; dr++) for (let dc = -ring; dc <= ring; dc++) {
        if (Math.abs(dr) !== ring && Math.abs(dc) !== ring) continue;
        const r = r0 + dr, c = c0 + dc;
        if (r < 0 || c < 0 || r >= this.rows || c >= this.cols) continue;
        const j = r * this.cols + c;
        if (this.blocked[j]) continue;
        const p = this.cellCenter(j);
        if (!w.segmentFree(x, y, p.x, p.y, sweepR)) continue;
        out.push({ i: j, d: Math.hypot(p.x - x, p.y - y) });
      }
      if (out.length && lastRing === maxRing) lastRing = Math.min(maxRing, ring + band);
    }
    return out.sort((a, b) => a.d - b.d);
  }

  nearestFreeCell(x, y, agentRadius, maxDist = 60) {
    const s = this.seedCells(x, y, agentRadius, maxDist);
    return s.length ? s[0].i : -1;
  }

  /** Multi-source 8-connected flood fill with the same no-corner-cutting rule A* uses. */
  reachableMulti(starts, goals) {
    if (!starts.length || !goals.length) return false;
    const cols = this.cols, rows = this.rows, blocked = this.blocked;
    const seen = new Uint8Array(blocked.length), goal = new Uint8Array(blocked.length);
    for (const g of goals) goal[g] = 1;
    const q = [];
    for (const s of starts) if (!seen[s]) { seen[s] = 1; q.push(s); }
    while (q.length) {
      const i = q.pop();
      if (goal[i]) return true;
      const c = i % cols, r = (i - c) / cols;
      for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) {
        if (!dr && !dc) continue;
        const nr = r + dr, nc = c + dc;
        if (nr < 0 || nc < 0 || nr >= rows || nc >= cols) continue;
        const j = nr * cols + nc;
        if (blocked[j] || seen[j]) continue;
        if (dr && dc && (blocked[r * cols + nc] || blocked[nr * cols + c])) continue;
        seen[j] = 1; q.push(j);
      }
    }
    return false;
  }

  reachable(a, b) {
    if (a < 0 || b < 0 || this.blocked[a] || this.blocked[b]) return false;
    return this.reachableMulti([a], [b]);
  }

  /** Can a disc of `agentRadius` get from point `from` to point `to` on this grid? */
  /** A route on this grid between the seed cells of two points (no pixel-level escape). */
  gridConnected(from, to, agentRadius) {
    return this.reachableMulti(this.seedCells(from.x, from.y, agentRadius).map((c) => c.i), this.seedCells(to.x, to.y, agentRadius).map((c) => c.i));
  }

  connected(from, to, agentRadius) {
    const goalIds = this.seedCells(to.x, to.y, agentRadius).map((c) => c.i);
    if (this.reachableMulti(this.seedCells(from.x, from.y, agentRadius).map((c) => c.i), goalIds)) return true;
    // Same last resort the planner uses (tier 2), so validation never assumes more than the planner can do.
    return escapeRoute(this.world, this, from, agentRadius, goalIds) !== null;
  }
}
