// geometry.hpp — C++23 port of js/geometry.js: vectors, polygons, circle/segment-vs-polygon queries.
// Kept operation-for-operation identical to the JS so both produce the same worlds and decisions.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

namespace sim {

inline constexpr double kInf = std::numeric_limits<double>::infinity();
inline constexpr double kPi = std::numbers::pi;

struct Vec2 { double x = 0, y = 0; };
inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double dist(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }
inline Vec2 norm(Vec2 a) { double l = std::hypot(a.x, a.y); if (l == 0 || std::isnan(l)) l = 1; return {a.x / l, a.y / l}; }
inline Vec2 fromAngle(double t, double l = 1) { return {std::cos(t) * l, std::sin(t) * l}; }
inline double angleOf(Vec2 a) { return std::atan2(a.y, a.x); }
inline Vec2 perp(Vec2 a) { return {-a.y, a.x}; }

/** JS-style clamp (no precondition that lo <= hi, unlike std::clamp). */
inline double clampd(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }
inline int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

inline double wrapAngle(double a) {
  while (a > kPi) a -= 2 * kPi;
  while (a < -kPi) a += 2 * kPi;
  return a;
}

/** JS ToInt32 (what `x | 0` does to a double). */
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
  double operator()() {
    a += 0x6D2B79F5u;
    std::uint32_t t = a;
    t = (t ^ (t >> 15)) * (t | 1u);
    t ^= t + (t ^ (t >> 7)) * (t | 61u);
    return static_cast<double>(t ^ (t >> 14)) / 4294967296.0;
  }
};

struct PointSeg { double d, t, qx, qy; };
struct DistT { double d, t; };

inline PointSeg distPointSeg(double px, double py, double ax, double ay, double bx, double by) {
  const double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
  double t = l2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const double qx = ax + dx * t, qy = ay + dy * t;
  return {std::hypot(px - qx, py - qy), t, qx, qy};
}

inline double segSegIntersect(double px, double py, double p2x, double p2y, double qx, double qy, double q2x, double q2y) {
  const double rx = p2x - px, ry = p2y - py, sx = q2x - qx, sy = q2y - qy;
  const double denom = rx * sy - ry * sx;
  if (std::abs(denom) < 1e-9) return -1;
  const double qpx = qx - px, qpy = qy - py;
  const double t = (qpx * sy - qpy * sx) / denom, u = (qpx * ry - qpy * rx) / denom;
  return (t >= 0 && t <= 1 && u >= 0 && u <= 1) ? t : -1;
}

inline DistT segSegDist(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
  const double ti = segSegIntersect(ax, ay, bx, by, cx, cy, dx, dy);
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
  double minX = kInf, minY = kInf, maxX = -kInf, maxY = -kInf;
  mutable std::uint32_t stamp = 0;  // broadphase de-duplication

  explicit Polygon(std::vector<Vec2> p) : pts(std::move(p)) {
    for (auto q : pts) {
      if (q.x < minX) minX = q.x; if (q.x > maxX) maxX = q.x;
      if (q.y < minY) minY = q.y; if (q.y > maxY) maxY = q.y;
    }
    double area = 0;
    const auto n = pts.size();
    for (std::size_t i = 0; i < n; i++) { auto a = pts[i], b = pts[(i + 1) % n]; area += a.x * b.y - b.x * a.y; }
    if (area < 0) std::ranges::reverse(pts);
  }

  bool contains(double px, double py) const {
    bool inside = false;
    const auto n = pts.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
      const double yi = pts[i].y, yj = pts[j].y;
      if ((yi > py) != (yj > py)) {
        const double x = pts[j].x + ((py - yj) * (pts[i].x - pts[j].x)) / (yi - yj);
        if (px < x) inside = !inside;
      }
    }
    return inside;
  }

  PointSeg nearestEdgePoint(double px, double py) const {
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
  std::optional<Vec2> circlePenetration(double cx, double cy, double r) const {
    if (cx + r < minX || cx - r > maxX || cy + r < minY || cy - r > maxY) return std::nullopt;
    const auto e = nearestEdgePoint(cx, cy);
    if (contains(cx, cy)) {
      double nx = cx - e.qx, ny = cy - e.qy;
      const double l = std::hypot(nx, ny);
      if (l < 1e-6) { nx = 1; ny = 0; } else { nx = -nx / l; ny = -ny / l; }
      return Vec2{nx * (e.d + r), ny * (e.d + r)};
    }
    if (e.d < r) {
      const double l = e.d != 0 ? e.d : 1e-6;
      return Vec2{((cx - e.qx) / l) * (r - e.d), ((cy - e.qy) / l) * (r - e.d)};
    }
    return std::nullopt;
  }

  DistT segmentDistance(double ax, double ay, double bx, double by) const {
    const bool ca = contains(ax, ay);
    if (ca || contains(bx, by)) return {0, ca ? 0.0 : 1.0};
    DistT r{kInf, 1};
    const auto n = pts.size();
    for (std::size_t i = 0; i < n; i++) {
      auto c = pts[i], d = pts[(i + 1) % n];
      auto s = segSegDist(ax, ay, bx, by, c.x, c.y, d.x, d.y);
      if (s.d < r.d) r = s;
    }
    return r;
  }

  double sweepHit(double ax, double ay, double bx, double by, double r) const {
    const auto s = segmentDistance(ax, ay, bx, by);
    return s.d < r ? s.t : 1;
  }
};

inline Polygon makeBlob(Mulberry32& rng, double cx, double cy, double radius, int nVerts) {
  std::vector<Vec2> pts;
  pts.reserve(nVerts);
  const double base = (kPi * 2) / nVerts;
  for (int i = 0; i < nVerts; i++) {
    const double ang = i * base + (rng() - 0.5) * base * 0.8;
    const double rad = radius * (0.35 + 0.65 * rng());
    pts.push_back({cx + std::cos(ang) * rad, cy + std::sin(ang) * rad});
  }
  return Polygon(std::move(pts));
}

inline Polygon makeBar(Mulberry32& rng, double cx, double cy, double length, double thick, double angle) {
  constexpr int segs = 3;
  std::vector<Vec2> left, right;
  const Vec2 dir = fromAngle(angle), n = perp(dir);
  const double bend = (rng() - 0.5) * 0.9;
  for (int i = 0; i <= segs; i++) {
    const double f = static_cast<double>(i) / segs - 0.5;
    const double along = f * length;
    const double off = bend * std::sin(f * kPi) * thick * 2;
    const double t = thick * (0.6 + 0.8 * rng());
    const double px = cx + dir.x * along + n.x * off, py = cy + dir.y * along + n.y * off;
    left.push_back({px + n.x * t, py + n.y * t});
    right.push_back({px - n.x * t, py - n.y * t});
  }
  std::ranges::reverse(right);
  left.insert(left.end(), right.begin(), right.end());
  return Polygon(std::move(left));
}

}  // namespace sim
