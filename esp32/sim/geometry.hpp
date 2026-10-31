// geometry.hpp — MCU variant of cpp/src/geometry.hpp. Identical algorithms, but every quantity is `real`,
// which is float by default: the ESP32-S3 FPU is single precision and doubles run in software.
// Build with -DSIM_REAL_DOUBLE to get the original double-precision behaviour.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

namespace sim {

#ifdef SIM_REAL_DOUBLE
using real = double;
#else
using real = float;
#endif

inline constexpr real kInf = std::numeric_limits<real>::infinity();
inline constexpr real kPi = std::numbers::pi_v<real>;
inline constexpr real kSqrt2 = std::numbers::sqrt2_v<real>;

/** sqrt(x² + y²). std::hypot guards against overflow at a large cost on an MCU; these values never come close. */
inline real length(real x, real y) { return std::sqrt(x * x + y * y); }

struct Vec2 { real x = 0, y = 0; };
inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, real s) { return {a.x * s, a.y * s}; }
inline real dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline real dist(Vec2 a, Vec2 b) { return length(a.x - b.x, a.y - b.y); }
inline Vec2 norm(Vec2 a) { real l = length(a.x, a.y); if (l == 0 || std::isnan(l)) l = 1; return {a.x / l, a.y / l}; }
inline Vec2 fromAngle(real t, real l = 1) { return {std::cos(t) * l, std::sin(t) * l}; }
inline real angleOf(Vec2 a) { return std::atan2(a.y, a.x); }
inline Vec2 perp(Vec2 a) { return {-a.y, a.x}; }

/** JS-style clamp (no precondition that lo <= hi, unlike std::clamp). */
inline real clampd(real v, real lo, real hi) { return v < lo ? lo : v > hi ? hi : v; }
inline int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

inline real wrapAngle(real a) {
  while (a > kPi) a -= 2 * kPi;
  while (a < -kPi) a += 2 * kPi;
  return a;
}

/** JS ToInt32 (what `x | 0` does to a real). */
inline std::int32_t toInt32(double v) {
  if (!std::isfinite(v)) return 0;
  double t = std::trunc(v);
  double m = std::fmod(t, 4294967296.0);
  if (m < 0) m += 4294967296.0;
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(m));
}

/** mulberry32, bit-identical to the JS version. */
struct Mulberry32 {
  std::uint32_t a;
  explicit Mulberry32(std::uint32_t seed) : a(seed) {}
  // Stays double so the value is exact and strictly below 1 (a float would round up to 1.0).
  double operator()() {
    a += 0x6D2B79F5u;
    std::uint32_t t = a;
    t = (t ^ (t >> 15)) * (t | 1u);
    t ^= t + (t ^ (t >> 7)) * (t | 61u);
    return static_cast<double>(t ^ (t >> 14)) / 4294967296.0;
  }
};

struct PointSeg { real d, t, qx, qy; };
struct DistT { real d, t; };

inline PointSeg distPointSeg(real px, real py, real ax, real ay, real bx, real by) {
  const real dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
  real t = l2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const real qx = ax + dx * t, qy = ay + dy * t;
  return {length(px - qx, py - qy), t, qx, qy};
}

inline real segSegIntersect(real px, real py, real p2x, real p2y, real qx, real qy, real q2x, real q2y) {
  const real rx = p2x - px, ry = p2y - py, sx = q2x - qx, sy = q2y - qy;
  const real denom = rx * sy - ry * sx;
  if (std::abs(denom) < 1e-9f) return -1;
  const real qpx = qx - px, qpy = qy - py;
  const real t = (qpx * sy - qpy * sx) / denom, u = (qpx * ry - qpy * rx) / denom;
  return (t >= 0 && t <= 1 && u >= 0 && u <= 1) ? t : -1;
}

inline DistT segSegDist(real ax, real ay, real bx, real by, real cx, real cy, real dx, real dy) {
  const real ti = segSegIntersect(ax, ay, bx, by, cx, cy, dx, dy);
  if (ti >= 0) return {0, ti};
  auto b = distPointSeg(cx, cy, ax, ay, bx, by);
  DistT r{b.d, b.t};
  b = distPointSeg(dx, dy, ax, ay, bx, by);
  if (b.d < r.d) r = {b.d, b.t};
  b = distPointSeg(ax, ay, cx, cy, dx, dy);
  if (b.d < r.d) r = {b.d, 0};
  b = distPointSeg(bx, by, cx, cy, dx, dy);
  if (b.d < r.d) r = {b.d, 1};
  return r;
}

struct Polygon {
  std::vector<Vec2> pts;
  real minX = kInf, minY = kInf, maxX = -kInf, maxY = -kInf;
  mutable std::uint32_t stamp = 0;  // broadphase de-duplication

  explicit Polygon(std::vector<Vec2> p) : pts(std::move(p)) {
    for (auto q : pts) {
      if (q.x < minX) minX = q.x;
      if (q.x > maxX) maxX = q.x;
      if (q.y < minY) minY = q.y;
      if (q.y > maxY) maxY = q.y;
    }
    real area = 0;
    const auto n = pts.size();
    for (std::size_t i = 0; i < n; i++) { auto a = pts[i], b = pts[(i + 1) % n]; area += a.x * b.y - b.x * a.y; }
    if (area < 0) std::ranges::reverse(pts);
  }

  bool contains(real px, real py) const {
    bool inside = false;
    const auto n = pts.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
      const real yi = pts[i].y, yj = pts[j].y;
      if ((yi > py) != (yj > py)) {
        const real x = pts[j].x + ((py - yj) * (pts[i].x - pts[j].x)) / (yi - yj);
        if (px < x) inside = !inside;
      }
    }
    return inside;
  }

  PointSeg nearestEdgePoint(real px, real py) const {
    PointSeg best{kInf, 0, 0, 0};
    bool have = false;
    const auto n = pts.size();
    for (std::size_t i = 0; i < n; i++) {
      auto a = pts[i], b = pts[(i + 1) % n];
      auto r = distPointSeg(px, py, a.x, a.y, b.x, b.y);
      if (!have || r.d < best.d) { best = r; have = true; }
    }
    return best;
  }

  /** Push-out vector resolving a disc overlap, or nullopt if the disc is clear. */
  std::optional<Vec2> circlePenetration(real cx, real cy, real r) const {
    if (cx + r < minX || cx - r > maxX || cy + r < minY || cy - r > maxY) return std::nullopt;
    const auto e = nearestEdgePoint(cx, cy);
    if (contains(cx, cy)) {
      real nx = cx - e.qx, ny = cy - e.qy;
      const real l = length(nx, ny);
      if (l < 1e-6f) { nx = 1; ny = 0; } else { nx = -nx / l; ny = -ny / l; }
      return Vec2{nx * (e.d + r), ny * (e.d + r)};
    }
    if (e.d < r) {
      const real l = e.d != 0 ? e.d : 1e-6f;
      return Vec2{((cx - e.qx) / l) * (r - e.d), ((cy - e.qy) / l) * (r - e.d)};
    }
    return std::nullopt;
  }

  DistT segmentDistance(real ax, real ay, real bx, real by) const {
    const bool ca = contains(ax, ay);
    if (ca || contains(bx, by)) return {0, ca ? 0.0f : 1.0f};
    DistT r{kInf, 1};
    const auto n = pts.size();
    for (std::size_t i = 0; i < n; i++) {
      auto c = pts[i], d = pts[(i + 1) % n];
      auto s = segSegDist(ax, ay, bx, by, c.x, c.y, d.x, d.y);
      if (s.d < r.d) r = s;
    }
    return r;
  }

  real sweepHit(real ax, real ay, real bx, real by, real r) const {
    const auto s = segmentDistance(ax, ay, bx, by);
    return s.d < r ? s.t : 1;
  }
};

inline Polygon makeBlob(Mulberry32& rng, real cx, real cy, real radius, int nVerts) {
  std::vector<Vec2> pts;
  pts.reserve(nVerts);
  const real base = (kPi * 2) / nVerts;
  for (int i = 0; i < nVerts; i++) {
    const real ang = i * base + (rng() - 0.5f) * base * 0.8f;
    const real rad = radius * (0.35f + 0.65f * rng());
    pts.push_back({cx + std::cos(ang) * rad, cy + std::sin(ang) * rad});
  }
  return Polygon(std::move(pts));
}

inline Polygon makeBar(Mulberry32& rng, real cx, real cy, real length, real thick, real angle) {
  constexpr int segs = 3;
  std::vector<Vec2> left, right;
  const Vec2 dir = fromAngle(angle), n = perp(dir);
  const real bend = (rng() - 0.5f) * 0.9f;
  for (int i = 0; i <= segs; i++) {
    const real f = static_cast<real>(i) / segs - 0.5f;
    const real along = f * length;
    const real off = bend * std::sin(f * kPi) * thick * 2;
    const real t = thick * (0.6f + 0.8f * rng());
    const real px = cx + dir.x * along + n.x * off, py = cy + dir.y * along + n.y * off;
    left.push_back({px + n.x * t, py + n.y * t});
    right.push_back({px - n.x * t, py - n.y * t});
  }
  std::ranges::reverse(right);
  left.insert(left.end(), right.begin(), right.end());
  return Polygon(std::move(left));
}

}  // namespace sim
