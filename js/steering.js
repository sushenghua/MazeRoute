// steering.js — a directional (car-like) agent and the controller state machine that drives it.
//
// Liveness guarantees (why the machine cannot get stuck):
//  1. Every state has a deadline. If a state overstays, onTimeout() forces a transition.
//  2. FOLLOWING is watched by a progress monitor: remaining path distance must shrink; if it
//     does not within STUCK_SECONDS the machine moves to RECOVERING.
//  3. Every cycle (FOLLOWING → RECOVERING → REPLANNING → FOLLOWING) bumps a "stuck streak"
//     counter that resets only on real progress. The streak escalates (switch to the fine
//     grid, then declare FAILED), so the cycle is bounded. ARRIVED and FAILED are terminal.
//  Planning is three-tier: the coarse grid (wide margin) first, then the fine grid (tight margin),
//  then a pixel-level escape from the agent's exact spot onto the fine grid. "No path" is only
//  reported when none of them finds a route.
//  4. Planning is time-sliced and resumable, so it never blocks a frame; its own deadline
//     covers pathological grids.

const S = Object.freeze({
  IDLE: 'IDLE', PLANNING: 'PLANNING', FOLLOWING: 'FOLLOWING', RECOVERING: 'RECOVERING',
  REPLANNING: 'REPLANNING', ARRIVED: 'ARRIVED', FAILED: 'FAILED',
});

const STATE_COLORS = {
  IDLE: '#8b93a7', PLANNING: '#f6c453', FOLLOWING: '#4fd1c5', RECOVERING: '#f97316',
  REPLANNING: '#f6c453', ARRIVED: '#22c55e', FAILED: '#ef4444',
};

class Agent {
  constructor(radius) {
    this.radius = radius;
    this.pos = { x: 0, y: 0 };
    this.heading = 0;      // radians, direction the nose points
    this.speed = 0;        // px/s along heading (negative = reversing)
    this.maxSpeed = 150;
    this.maxAccel = 320;
    this.maxTurn = 3.6;    // rad/s
    this.trail = [];
  }
  place(p, heading) { this.pos = { x: p.x, y: p.y }; this.heading = heading; this.speed = 0; this.trail = []; }
}

class Controller {
  constructor(world, agent, astar) {
    this.world = world;
    this.agent = agent;
    this.astar = astar;
    this.STUCK_SECONDS = 1.2;
    this.MIN_PROGRESS = 8; // px per window
    this.reset();

    // State table. maxTime may be a function so deadlines can depend on the path.
    this.states = {
      [S.IDLE]: { maxTime: Infinity, enter() {}, tick() {}, onTimeout() {} },
      [S.PLANNING]: {
        maxTime: 20,
        enter() { this.beginPlan(); },
        tick(dt) { this.agent.speed = this.approach(this.agent.speed, 0, this.agent.maxAccel * dt); },
        plan(budget) { this.stepPlan(budget); },
        onTimeout() { this.go(S.FAILED, 'planner deadline exceeded'); },
      },
      [S.FOLLOWING]: {
        maxTime: () => 6 + pathLength(this.path) / (this.agent.maxSpeed * 0.25),
        enter() {
          // A replan triggered while following must not reset the progress clock, otherwise a
          // follow → replan → follow cycle could run forever without moving the agent.
          if (!this.keepWatchdog) this.lastProgressTime = this.simTime;
          this.keepWatchdog = false;
          this.bestRemaining = Infinity; this.anchorRemaining = Infinity; this.frame = 0;
        },
        tick(dt) { this.follow(dt); },
        onTimeout() { this.go(S.RECOVERING, 'follow deadline exceeded'); },
      },
      [S.RECOVERING]: {
        maxTime: 0.6,
        enter() {
          this.stats.recoveries++;
          this.stuckStreak++;
          this.remainingAtRecovery = this.remaining;
          this.recoverSide = this.lastLateral >= 0 ? 1 : -1;
        },
        tick(dt) {
          // Back away only if the space behind is clear; otherwise pivot in place. Rotate toward
          // the path so the replan starts facing the right way.
          const a = this.agent, w = this.world;
          const back = V.fromAngle(a.heading + Math.PI, a.radius + 6);
          const rearClear = w.sweep(a.pos.x, a.pos.y, a.pos.x + back.x, a.pos.y + back.y, a.radius * 0.95) >= 1;
          const aim = this.onPath && V.dist(this.onPath, a.pos) > 2 ? V.sub(this.onPath, a.pos) : V.fromAngle(a.heading + this.recoverSide);
          const dh = wrapAngle(V.angle(aim) - a.heading);
          a.heading = wrapAngle(a.heading + clamp(dh, -a.maxTurn * 0.6 * dt, a.maxTurn * 0.6 * dt));
          this.move(dt, null, rearClear ? -a.maxSpeed * 0.3 : 0);
        },
        onTimeout() {
          if (this.stuckStreak >= 7) { this.go(S.FAILED, 'no progress after 7 recoveries'); return; }
          if (this.stuckStreak >= 3 && !this.preferFine) {
            this.preferFine = true;
            this.stats.escalation++;
            this.go(S.REPLANNING, 'escalation: plan on the fine grid from now on');
            return;
          }
          this.go(S.REPLANNING, 'recovery manoeuvre done');
        },
      },
      [S.REPLANNING]: {
        maxTime: 20,
        enter() { this.stats.replans++; this.beginPlan(); },
        tick(dt) { this.agent.speed = this.approach(this.agent.speed, 0, this.agent.maxAccel * dt); },
        plan(budget) { this.stepPlan(budget); },
        onTimeout() { this.go(S.FAILED, 'planner deadline exceeded'); },
      },
      [S.ARRIVED]: { maxTime: Infinity, enter() { this.agent.speed = 0; }, tick() {}, onTimeout() {} },
      // The old route is no longer valid (that is why we failed), so stop drawing it.
      [S.FAILED]: { maxTime: Infinity, enter() { this.agent.speed = 0; this.path = []; this.rawPath = []; this.target = null; }, tick() {}, onTimeout() {} },
    };
  }

  reset() {
    this.state = S.IDLE;
    this.timeInState = 0;
    this.deadline = Infinity;
    this.simTime = 0;
    this.path = []; this.rawPath = []; this.cum = []; this.segIdx = 0;
    this.target = null; this.targetVisible = false; this.cornerSpeed = Infinity; this.onPath = null; this.remaining = 0; this.bestRemaining = Infinity;
    this.lastProgressTime = 0; this.remainingAtRecovery = Infinity;
    this.stuckStreak = 0; this.escape = null; this.escapePrefix = null; this.pendingGrid = false; this.keepWatchdog = false; this.preferFine = false; this.planLevel = 0; this.lastLateral = 0; this.brake = 0; this.frame = 0;
    this.whiskers = []; this.avoidLateral = { x: 0, y: 0 }; this.avoidNormal = { x: 0, y: 0 }; this.trustPlan = true;
    this.log = [];
    this.stats = { replans: 0, recoveries: 0, collisions: 0, escalation: 0, stuckEvents: 0, planNodes: 0, planMs: 0, planSlices: 0 };
    if (this.world.start) this.agent.place(this.world.start, 0);
  }

  // ---------------- state machine core ----------------

  go(next, reason) {
    this.log.unshift({ t: this.simTime, from: this.state, to: next, reason });
    if (this.log.length > 14) this.log.pop();
    this.state = next;
    this.timeInState = 0;
    const st = this.states[next];
    st.enter.call(this);
    const mt = typeof st.maxTime === 'function' ? st.maxTime() : st.maxTime;
    this.deadline = mt;
  }

  startMission() {
    if (this.state !== S.IDLE) this.reset();
    this.go(S.PLANNING, 'start requested');
  }

  get isTerminal() { return this.state === S.ARRIVED || this.state === S.FAILED; }
  get isPlanning() { return this.state === S.PLANNING || this.state === S.REPLANNING; }

  /** Fixed-timestep simulation tick. */
  tick(dt) {
    this.simTime += dt;
    this.timeInState += dt;
    const st = this.states[this.state];
    st.tick.call(this, dt);
    if (this.timeInState > this.deadline) {
      this.timeInState = 0; // guard against onTimeout re-firing if it keeps the same state
      st.onTimeout.call(this);
    }
  }

  /** Once-per-frame planning slice; budgetMs comes from the frame scheduler. */
  plan(budgetMs) {
    const st = this.states[this.state];
    if (st.plan) st.plan.call(this, budgetMs);
  }

  /** Called by the app when obstacles change at runtime. */
  onWorldChanged() {
    // Replan on every change, not only when the path is outright blocked: a drop beside the path can
    // narrow its corridor below what the planner considers passable without blocking a thin sweep, and
    // following it then can lead the agent through a neck into a region the planner cannot route out of.
    if (this.state === S.FOLLOWING) { this.keepWatchdog = true; this.go(S.REPLANNING, this.pathAheadClear(this.path.length) ? 'world changed: replanning' : 'world changed: path blocked'); }
    else if (this.isPlanning) this.beginPlan(); // the grid changed under a running search: restart it
  }

  // ---------------- planning ----------------

  /**
   * Start (or restart) a search at the current tier: 0 = coarse grid, 1 = fine grid,
   * 2 = pixel-level escape from the agent's exact spot onto the fine grid, then fine A* (last resort).
   */
  beginPlan(level = this.preferFine ? 1 : 0) {
    const w = this.world, r = this.agent.radius;
    this.planLevel = level;
    const grid = level === 0 ? w.nav : w.fine;
    // The fine grid may still be rasterising in the background; stepPlan finishes it first.
    if (!grid.ready) { this.pendingGrid = true; return; }
    this.pendingGrid = false;
    this.escapePrefix = null;
    this.escape = null;
    const goals = grid.seedCells(w.goal.x, w.goal.y, r);
    if (level === 2) {
      // Resumable: stepPlan() advances it in the frame's planning slice, then hands over to A*.
      this.escape = new EscapeSearch(w, grid, this.agent.pos, r, goals.map((g) => g.i));
      this.escapeGoals = goals;
      return;
    }
    this.astar.start(grid, grid.seedCells(this.agent.pos.x, this.agent.pos.y, r), goals);
  }

  stepPlan(budgetMs) {
    if (this.pendingGrid) {
      if (!this.world.fine.buildStep(budgetMs)) return;
      this.beginPlan(this.planLevel);
      return;
    }
    if (this.escape) {
      const e = this.escape;
      if (e.step(budgetMs) === 'running') return;
      this.escape = null;
      // Found: A* continues from the cell the escape reached. Not found: an empty search fails at once.
      if (e.status === 'found') {
        this.escapePrefix = e.result.path;
        this.astar.start(this.world.fine, [{ i: e.result.cell, d: e.result.length }], this.escapeGoals);
      } else this.astar.start(this.world.fine, [], this.escapeGoals);
      return;
    }
    const status = this.astar.step(budgetMs);
    this.stats.planNodes = this.astar.expanded;
    this.stats.planMs = this.astar.msSpent;
    this.stats.planSlices = this.astar.slices;
    if (status === 'found') {
      const raw = this.astar.path();
      if (this.escapePrefix) raw.unshift(...this.escapePrefix);
      else raw.unshift({ x: this.agent.pos.x, y: this.agent.pos.y });
      raw.push({ x: this.world.goal.x, y: this.world.goal.y });
      this.rawPath = raw;
      this.setPath(smoothPath(raw, this.world, this.agent.radius + 3));
      const tier = ['coarse', 'fine', 'fine via pixel escape'][this.planLevel];
      this.go(S.FOLLOWING, `path found on ${tier} grid: ${this.astar.expanded} nodes in ${this.astar.slices} slice${this.astar.slices > 1 ? 's' : ''}`);
    } else if (status === 'failed') {
      if (this.planLevel < 2) {
        const next = this.planLevel + 1;
        const why = ['', 'coarse grid found no route, retrying on fine grid', 'no grid route from here, trying a pixel-level escape'][next];
        this.log.unshift({ t: this.simTime, from: this.state, to: this.state, reason: why });
        this.beginPlan(next);
      } else {
        this.go(S.FAILED, 'no route exists at the agent\'s size');
      }
    }
  }

  setPath(pts) {
    this.path = pts;
    this.cum = [0];
    for (let i = 1; i < pts.length; i++) this.cum.push(this.cum[i - 1] + V.dist(pts[i - 1], pts[i]));
    this.segIdx = 0;
    this.remaining = this.cum[this.cum.length - 1];
  }

  // ---------------- following ----------------

  /** Locate agent on the path (monotonic search), compute remaining distance and lookahead target. */
  trackPath() {
    const p = this.path, a = this.agent, n = p.length - 1;
    let best = null, bestK = this.segIdx;
    for (let k = this.segIdx; k <= Math.min(this.segIdx + 2, n - 1); k++) {
      const r = distPointSeg(a.pos.x, a.pos.y, p[k].x, p[k].y, p[k + 1].x, p[k + 1].y);
      if (!best || r.d < best.d) { best = r; bestK = k; }
    }
    this.segIdx = bestK;
    const segLen = this.cum[bestK + 1] - this.cum[bestK];
    const along = this.cum[bestK] + best.t * segLen;
    const total = this.cum[n];
    this.remaining = total - along + best.d;
    this.onPath = { x: best.qx, y: best.qy };

    // Visible look-ahead: walk back from the nominal look-ahead distance toward the agent's own
    // projection until the straight line to that point is collision-free for the agent's body.
    // Pursuit then never aims through a corner, even after the agent has drifted off the path.
    const lookahead = 34 + Math.max(0, a.speed) * 0.22;
    this.target = null;
    for (const sweepR of [a.radius + 2.5, a.radius * 0.95]) {
      for (let s = Math.min(along + lookahead, total); s >= along - 0.01; s -= 4) {
        const pt = this.pointAt(s, bestK);
        if (this.world.segmentFree(a.pos.x, a.pos.y, pt.x, pt.y, sweepR)) { this.target = pt; break; }
      }
      if (this.target) break;
    }
    this.targetVisible = !!this.target;
    if (!this.target) this.target = this.onPath;

    // Corner speed limit: the sharper the next turn, the slower the agent must arrive at it.
    this.cornerSpeed = a.maxSpeed;
    let dist = this.cum[bestK + 1] - along;
    for (let k = bestK + 1; k < n && dist < 160; k++) {
      const d1 = V.norm(V.sub(p[k], p[k - 1])), d2 = V.norm(V.sub(p[k + 1], p[k]));
      const turn = Math.acos(clamp(V.dot(d1, d2), -1, 1));
      const vc = a.maxSpeed * clamp(1 - turn / 2.0, 0.12, 1);
      this.cornerSpeed = Math.min(this.cornerSpeed, Math.sqrt(vc * vc + 2 * a.maxAccel * 0.7 * Math.max(0, dist - 4)));
      dist += this.cum[k + 1] - this.cum[k];
    }
  }

  /** Point at arc length s along the path, searching forward from segment k0. */
  pointAt(s, k0) {
    const p = this.path, n = p.length - 1;
    if (s >= this.cum[n]) return p[n];
    let k = k0;
    while (k < n - 1 && this.cum[k + 1] < s) k++;
    const f = clamp((s - this.cum[k]) / ((this.cum[k + 1] - this.cum[k]) || 1), 0, 1);
    return { x: p[k].x + (p[k + 1].x - p[k].x) * f, y: p[k].y + (p[k + 1].y - p[k].y) * f };
  }

  /** Check the next few path segments for obstacles that appeared after planning. */
  pathAheadClear(count) {
    const p = this.path, r = this.agent.radius;
    for (let k = this.segIdx; k < Math.min(this.segIdx + count, p.length - 1); k++) {
      const a = k === this.segIdx ? (this.onPath || this.agent.pos) : p[k];
      if (!this.world.segmentFree(a.x, a.y, p[k + 1].x, p[k + 1].y, r * 0.75)) return false;
    }
    return true;
  }

  /** Whisker-based local avoidance: returns a lateral/normal steering offset (world space). */
  computeAvoidance() {
    const a = this.agent, w = this.world;
    const L = 26 + Math.max(0, a.speed) * 0.3;
    const side = V.perp(V.fromAngle(a.heading));
    const defs = [[0, L], [0.55, L * 0.7], [-0.55, L * 0.7]];
    this.whiskers = defs.map(([ang, len]) => {
      const d = V.fromAngle(a.heading + ang, len);
      const end = { x: a.pos.x + d.x, y: a.pos.y + d.y };
      const t = w.sweep(a.pos.x, a.pos.y, end.x, end.y, a.radius * 0.9);
      return { end, t, hit: t < 1, len };
    });
    const [c, l, r] = this.whiskers.map((wk) => wk.t);
    let lat = 0, brake = 0;
    if (c < 1) { lat += (l > r ? 1 : -1) * (1 - c) * 1.6; brake += 1 - c; }
    if (l < 1) lat -= (1 - l) * 1.0;
    if (r < 1) lat += (1 - r) * 1.0;
    this.avoidLateral = V.scale(side, lat);
    this.avoidNormal = { x: 0, y: 0 };
    const ns = w.nearestSurface(a.pos.x, a.pos.y, a.radius + 8);
    if (ns) { const k = (1 - ns.d / (a.radius + 8)) * 0.8; this.avoidNormal = { x: ns.nx * k, y: ns.ny * k }; }
    this.lastLateral = lat;
    this.brake = clamp(brake, 0, 0.6);
    return V.add(this.avoidLateral, this.avoidNormal);
  }

  follow(dt) {
    const a = this.agent, goal = this.world.goal;
    this.frame++;
    this.trackPath();

    const dGoal = V.dist(a.pos, goal);
    if (dGoal < a.radius + 3) { this.go(S.ARRIVED, `reached B after ${this.simTime.toFixed(1)} s`); return; }

    this.computeAvoidance();
    const toTarget = V.norm(V.sub(this.target, a.pos));
    // Local avoidance vs. global plan: whiskers are a heuristic and can contradict the path at a
    // corner (they say "turn away from the wall", the path says "round the corner"). If the straight
    // line to the target is verifiably clear, trust the plan and keep only a gentle surface repulsion;
    // otherwise (e.g. a freshly dropped obstacle) let the whiskers take over.
    // trackPath() already verified the line to the target is clear, so a visible target is followed
    // as-is; whiskers only steer when no part of the path ahead is visible.
    this.trustPlan = this.targetVisible;
    let desired, brake;
    if (this.targetVisible) { desired = V.add(toTarget, V.scale(this.avoidNormal, 0.3)); brake = 0; }
    else { desired = V.add(V.add(toTarget, this.avoidLateral), this.avoidNormal); brake = this.brake; }
    const arrive = clamp(dGoal / 70, 0.2, 1);
    this.move(dt, desired, Math.min(a.maxSpeed * (1 - brake) * arrive, this.cornerSpeed));

    // Progress watchdog: remaining distance must drop by at least MIN_PROGRESS within each window,
    // otherwise creeping along a wall at 2 px/s would count as progress forever.
    if (this.remaining < this.anchorRemaining - this.MIN_PROGRESS) { this.anchorRemaining = this.remaining; this.lastProgressTime = this.simTime; }
    if (this.remaining < this.remainingAtRecovery - 60) this.stuckStreak = 0; // real progress since last recovery
    if (this.simTime - this.lastProgressTime > this.STUCK_SECONDS) {
      this.stats.stuckEvents++;
      this.go(S.RECOVERING, `no progress for ${this.STUCK_SECONDS} s`);
      return;
    }
    // Periodic path validation (handles obstacles added at runtime).
    if ((this.frame % 10) === 0 && !this.pathAheadClear(3)) { this.keepWatchdog = true; this.go(S.REPLANNING, 'path ahead blocked'); }
  }

  // ---------------- motion model ----------------

  approach(v, target, maxDelta) { return v + clamp(target - v, -maxDelta, maxDelta); }

  /** Turn-rate-limited heading, acceleration-limited speed, then collision resolution. */
  move(dt, desiredDir, speedTarget) {
    const a = this.agent;
    if (desiredDir && (desiredDir.x || desiredDir.y)) {
      const dh = wrapAngle(V.angle(desiredDir) - a.heading);
      a.heading = wrapAngle(a.heading + clamp(dh, -a.maxTurn * dt, a.maxTurn * dt));
      // Slow down to turn, and pivot in place (zero forward speed) when the error is large:
      // creeping forward while badly misaligned is what drives an agent into walls.
      const align = Math.cos(dh);
      speedTarget *= align < 0.5 ? 0 : align * align;
    }
    a.speed = this.approach(a.speed, speedTarget, a.maxAccel * dt);
    a.pos.x += Math.cos(a.heading) * a.speed * dt;
    a.pos.y += Math.sin(a.heading) * a.speed * dt;
    const hits = this.world.resolveDisc(a.pos, a.radius);
    if (hits) { this.stats.collisions += hits; a.speed *= 0.6; }
    if ((this.frame & 1) === 0) { a.trail.push({ x: a.pos.x, y: a.pos.y }); if (a.trail.length > 600) a.trail.shift(); }
  }
}
