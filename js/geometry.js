// geometry.js — vectors, polygons, and circle/segment-vs-polygon collision queries.
// Everything here is pure and allocation-light so it can run thousands of times per frame.

const V = {
  add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y }),
  sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y }),
  scale: (a, s) => ({ x: a.x * s, y: a.y * s }),
  dot: (a, b) => a.x * b.x + a.y * b.y,
  cross: (a, b) => a.x * b.y - a.y * b.x,
  len: (a) => Math.hypot(a.x, a.y),
  dist: (a, b) => Math.hypot(a.x - b.x, a.y - b.y),
  norm: (a) => { const l = Math.hypot(a.x, a.y) || 1; return { x: a.x / l, y: a.y / l }; },
  fromAngle: (t, l = 1) => ({ x: Math.cos(t) * l, y: Math.sin(t) * l }),
  angle: (a) => Math.atan2(a.y, a.x),
  perp: (a) => ({ x: -a.y, y: a.x }),
};

function wrapAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

/** Closest point on segment ab to p. Returns {d, t, qx, qy}. */
function distPointSeg(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay;
  const l2 = dx * dx + dy * dy;
  let t = l2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const qx = ax + dx * t, qy = ay + dy * t;
  return { d: Math.hypot(px - qx, py - qy), t, qx, qy };
}

/** Parametric intersection of segment p→p2 with q→q2. Returns t along p→p2 or -1. */
function segSegIntersect(px, py, p2x, p2y, qx, qy, q2x, q2y) {
  const rx = p2x - px, ry = p2y - py;
  const sx = q2x - qx, sy = q2y - qy;
  const denom = rx * sy - ry * sx;
  if (Math.abs(denom) < 1e-9) return -1;
  const qpx = qx - px, qpy = qy - py;
  const t = (qpx * sy - qpy * sx) / denom;
  const u = (qpx * ry - qpy * rx) / denom;
  if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return t;
  return -1;
}

/** Minimum distance between two segments, plus the parameter along ab of the closest approach. */
function segSegDist(ax, ay, bx, by, cx, cy, dx, dy) {
  const ti = segSegIntersect(ax, ay, bx, by, cx, cy, dx, dy);
  if (ti >= 0) return { d: 0, t: ti };
  let best = distPointSeg(cx, cy, ax, ay, bx, by);
  let r = { d: best.d, t: best.t };
  best = distPointSeg(dx, dy, ax, ay, bx, by);
  if (best.d < r.d) r = { d: best.d, t: best.t };
  best = distPointSeg(ax, ay, cx, cy, dx, dy);
  if (best.d < r.d) r = { d: best.d, t: 0 };
  best = distPointSeg(bx, by, cx, cy, dx, dy);
  if (best.d < r.d) r = { d: best.d, t: 1 };
  return r;
}

class Polygon {
  constructor(pts) {
    this.pts = pts;
    this.n = pts.length;
    this.minX = Infinity; this.minY = Infinity; this.maxX = -Infinity; this.maxY = -Infinity;
    for (const p of pts) {
      if (p.x < this.minX) this.minX = p.x; if (p.x > this.maxX) this.maxX = p.x;
      if (p.y < this.minY) this.minY = p.y; if (p.y > this.maxY) this.maxY = p.y;
    }
    // Ensure counter-clockwise (screen coords: y down) so edge normals are consistent.
    let area = 0;
    for (let i = 0; i < this.n; i++) {
      const a = pts[i], b = pts[(i + 1) % this.n];
      area += a.x * b.y - b.x * a.y;
    }
    if (area < 0) this.pts.reverse();
  }

  contains(px, py) {
    // Ray casting, robust for concave shapes.
    let inside = false;
    const p = this.pts;
    for (let i = 0, j = this.n - 1; i < this.n; j = i++) {
      const yi = p[i].y, yj = p[j].y;
      if ((yi > py) !== (yj > py)) {
        const x = p[j].x + ((py - yj) * (p[i].x - p[j].x)) / (yi - yj);
        if (px < x) inside = !inside;
      }
    }
    return inside;
  }

  /** Nearest point on the polygon boundary. */
  nearestEdgePoint(px, py) {
    let best = null;
    const p = this.pts;
    for (let i = 0; i < this.n; i++) {
      const a = p[i], b = p[(i + 1) % this.n];
      const r = distPointSeg(px, py, a.x, a.y, b.x, b.y);
      if (!best || r.d < best.d) best = r;
    }
    return best;
  }

  /** Signed distance: negative when inside. */
  signedDistance(px, py) {
    const e = this.nearestEdgePoint(px, py);
    return this.contains(px, py) ? -e.d : e.d;
  }

  /**
   * Circle vs polygon. Returns null or a push-out vector {x,y} that resolves the overlap.
   */
  circlePenetration(cx, cy, r) {
    if (cx + r < this.minX || cx - r > this.maxX || cy + r < this.minY || cy - r > this.maxY) return null;
    const e = this.nearestEdgePoint(cx, cy);
    if (this.contains(cx, cy)) {
      // Centre is inside: push out through the nearest edge by (distance + radius).
      let nx = cx - e.qx, ny = cy - e.qy;
      const l = Math.hypot(nx, ny);
      if (l < 1e-6) { nx = 1; ny = 0; } else { nx = -nx / l; ny = -ny / l; }
      // nx,ny now points from centre toward the edge => outward.
      return { x: nx * (e.d + r), y: ny * (e.d + r), depth: e.d + r };
    }
    if (e.d < r) {
      const l = e.d || 1e-6;
      return { x: ((cx - e.qx) / l) * (r - e.d), y: ((cy - e.qy) / l) * (r - e.d), depth: r - e.d };
    }
    return null;
  }

  /** Minimum distance from segment ab to the polygon (0 if touching/crossing/inside). */
  segmentDistance(ax, ay, bx, by) {
    if (this.contains(ax, ay) || this.contains(bx, by)) return { d: 0, t: this.contains(ax, ay) ? 0 : 1 };
    let r = { d: Infinity, t: 1 };
    const p = this.pts;
    for (let i = 0; i < this.n; i++) {
      const c = p[i], d = p[(i + 1) % this.n];
      const s = segSegDist(ax, ay, bx, by, c.x, c.y, d.x, d.y);
      if (s.d < r.d) r = s;
    }
    return r;
  }

  /** First parameter t along a→b where a disc of radius r would touch the polygon, or 1 if clear. */
  sweepHit(ax, ay, bx, by, r) {
    const s = this.segmentDistance(ax, ay, bx, by);
    return s.d < r ? s.t : 1;
  }
}

/** Build an irregular blob polygon: jittered radii on jittered angles. */
function makeBlob(rng, cx, cy, radius, nVerts) {
  const pts = [];
  const base = (Math.PI * 2) / nVerts;
  for (let i = 0; i < nVerts; i++) {
    const ang = i * base + (rng() - 0.5) * base * 0.8;
    const rad = radius * (0.35 + 0.65 * rng());
    pts.push({ x: cx + Math.cos(ang) * rad, y: cy + Math.sin(ang) * rad });
  }
  return new Polygon(pts);
}

/** Build a long, slightly bent "wall" with jittered edges so no two look alike. */
function makeBar(rng, cx, cy, length, thick, angle) {
  const segs = 3;
  const left = [], right = [];
  const dir = V.fromAngle(angle), n = V.perp(dir);
  let bend = (rng() - 0.5) * 0.9;
  for (let i = 0; i <= segs; i++) {
    const f = i / segs - 0.5;
    const along = f * length;
    const off = bend * Math.sin(f * Math.PI) * thick * 2;
    const t = thick * (0.6 + 0.8 * rng());
    const px = cx + dir.x * along + n.x * off, py = cy + dir.y * along + n.y * off;
    left.push({ x: px + n.x * t, y: py + n.y * t });
    right.push({ x: px - n.x * t, y: py - n.y * t });
  }
  return new Polygon(left.concat(right.reverse()));
}
