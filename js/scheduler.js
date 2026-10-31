// scheduler.js — fixed-timestep simulation clock plus a per-frame time budget.
// The budget measures each phase (sim / plan / render), keeps smoothed estimates, and hands the
// planner whatever time is left after reserving for simulation and rendering.

class FrameBudget {
  constructor(targetMs) {
    this.target = targetMs;
    this.ema = { sim: 1, plan: 0, render: 2, total: 4 };
    this.history = [];
    this.maxHistory = 150;
    this.frames = 0;
    this.overBudget = 0;
    this.fps = 0;
    this._fpsAcc = 0; this._fpsCount = 0;
  }
  begin() { this.t0 = performance.now(); this.last = this.t0; this.cur = { sim: 0, plan: 0, render: 0, total: 0 }; }
  mark(name) { const n = performance.now(); this.cur[name] += n - this.last; this.last = n; }
  end(frameDt) {
    this.cur.total = performance.now() - this.t0;
    for (const k of Object.keys(this.cur)) this.ema[k] += (this.cur[k] - this.ema[k]) * 0.1;
    this.history.push(this.cur);
    if (this.history.length > this.maxHistory) this.history.shift();
    this.frames++;
    if (this.cur.total > this.target) this.overBudget++;
    this._fpsAcc += frameDt; this._fpsCount++;
    if (this._fpsAcc >= 0.5) { this.fps = this._fpsCount / this._fpsAcc; this._fpsAcc = 0; this._fpsCount = 0; }
  }
  elapsed() { return performance.now() - this.t0; }
  /** Time the planner may use this frame: user cap, minus what render is expected to need. */
  planBudget(userCapMs) {
    const reserve = this.ema.render + 0.5;
    const left = this.target - this.elapsed() - reserve;
    return clamp(Math.min(userCapMs, left), 0.15, userCapMs);
  }
}

class FixedClock {
  constructor(stepSeconds, maxSteps) {
    this.dt = stepSeconds;
    this.maxSteps = maxSteps;
    this.acc = 0;
    this.droppedSeconds = 0;
  }
  /** Run fn(dt) as many fixed steps as the elapsed time allows; drop the excess (spiral-of-death guard). */
  advance(frameSeconds, scale, fn) {
    this.acc += Math.min(frameSeconds, 0.1) * scale;
    let steps = 0;
    while (this.acc >= this.dt && steps < this.maxSteps) { fn(this.dt); this.acc -= this.dt; steps++; }
    if (this.acc >= this.dt) { this.droppedSeconds += this.acc; this.acc = 0; }
    return steps;
  }
}
