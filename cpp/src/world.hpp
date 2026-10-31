// world.hpp — C++23 port of js/world.js: arena generation, spatial hash, two nav grids.
#pragma once
#include <chrono>
#include <expected>
#include <memory>
#include <optional>
#include <string_view>
#include "geometry.hpp"

namespace sim {

inline double nowMs() {
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

/** Side clearance (px) beyond the agent's radius that the fine planner is guaranteed to find. */
inline constexpr double kFineGuarantee = 2.5;

class World;

struct Seed { int i; double d; };

/** Occupancy grid for one agent size; see js/world.js NavGrid for the design notes. */
class NavGrid {
 public:
  const World* world;
  double cellSize, inflation;
  int cols, rows;
  std::vector<std::uint8_t> blocked;
  int builtRows = 0;
  bool ready = false;

  NavGrid(const World& w, double cs, double inf, bool deferred = false);
  bool buildStep(double budgetMs);
  void updateRegion(double minX, double minY, double maxX, double maxY);
  int cellIndexAt(double x, double y) const {
    const int c = clampi(static_cast<int>(std::floor(x / cellSize)), 0, cols - 1);
    const int r = clampi(static_cast<int>(std::floor(y / cellSize)), 0, rows - 1);
    return r * cols + c;
  }
  Vec2 cellCenter(int i) const { return {((i % cols) + 0.5) * cellSize, ((i / cols) + 0.5) * cellSize}; }
  std::vector<Seed> seedCells(double x, double y, double agentRadius, double maxDist = 60) const;
  bool reachableMulti(const std::vector<int>& starts, const std::vector<int>& goals) const;
  bool reachable(int a, int b) const {
    if (a < 0 || b < 0 || blocked[a] || blocked[b]) return false;
    return reachableMulti({a}, {b});
  }
  bool connected(Vec2 from, Vec2 to, double agentRadius) const;
  /** A route on this grid between the seed cells of two points (no pixel-level escape). */
  bool gridConnected(Vec2 from, Vec2 to, double agentRadius) const {
    std::vector<int> s, g;
    for (auto& c : seedCells(from.x, from.y, agentRadius)) s.push_back(c.i);
    for (auto& c : seedCells(to.x, to.y, agentRadius)) g.push_back(c.i);
    return reachableMulti(s, g);
  }

 private:
  std::optional<std::pair<int, int>> rowClamp_;
};

struct Placed { std::size_t index; bool sealed; };

class World {
 public:
  double width, height, cellSize;
  double fineCellSize = 2, fineMargin = 1.1;
  std::vector<Polygon> polys;
  Vec2 start{}, goal{};
  std::uint32_t seed = 0;
  double hashSize = 80;
  int hashCols, hashRows;
  std::vector<std::vector<int>> hash;
  std::unique_ptr<NavGrid> nav, fine;
  int version = 0;
  double agentRadius = 0;

  World(double w, double h, double cs)
      : width(w), height(h), cellSize(cs),
        hashCols(static_cast<int>(std::ceil(w / 80))), hashRows(static_cast<int>(std::ceil(h / 80))) {}

  bool generate(std::uint32_t s, int density, double agentR) {
    seed = s;
    Mulberry32 rng(s);
    const double diag = std::hypot(width, height);
    for (int attempt = 0; attempt < 40; attempt++) {
      polys.clear();
      const int nBlobs = static_cast<int>(std::floor(density * 0.6 + 0.5)), nBars = static_cast<int>(std::floor(density * 0.4 + 0.5));
      for (int i = 0; i < nBlobs; i++) {
        // JS evaluates call arguments left to right; C++ does not, so draw them in order first.
        const double r = 28 + rng() * 55;
        const double cx = r + rng() * (width - 2 * r);
        const double cy = r + rng() * (height - 2 * r);
        const int nv = 6 + static_cast<int>(std::floor(rng() * 7));
        polys.push_back(makeBlob(rng, cx, cy, r, nv));
      }
      for (int i = 0; i < nBars; i++) {
        const double len = 90 + rng() * 220;
        const double ang = rng() < 0.5 ? (rng() - 0.5) * 0.5 : kPi / 2 + (rng() - 0.5) * 0.5;
        const double cx = 60 + rng() * (width - 120);
        const double cy = 60 + rng() * (height - 120);
        const double thick = 7 + rng() * 8;
        const double a2 = ang + (rng() - 0.5) * 0.4;
        polys.push_back(makeBar(rng, cx, cy, len, thick, a2));
      }
      rebuildHash();
      nav = std::make_unique<NavGrid>(*this, cellSize, agentR + 5);
      std::vector<int> freeCells;
      for (int i = 0; i < static_cast<int>(nav->blocked.size()); i++) if (!nav->blocked[i]) freeCells.push_back(i);
      if (freeCells.size() < 50) continue;
      bool ok = false;
      for (int tries = 0; tries < 60 && !ok; tries++) {
        const int a = freeCells[static_cast<std::size_t>(std::floor(rng() * freeCells.size()))];
        const int b = freeCells[static_cast<std::size_t>(std::floor(rng() * freeCells.size()))];
        const Vec2 pa = nav->cellCenter(a), pb = nav->cellCenter(b);
        if (dist(pa, pb) < diag * 0.55) continue;
        if (nav->reachable(a, b)) { start = pa; goal = pb; ok = true; }
      }
      if (ok) {
        agentRadius = agentR;
        fine = std::make_unique<NavGrid>(*this, fineCellSize, agentR + fineMargin, true);
        version++;
        return true;
      }
      density = std::max(6, density - 2);
    }
    return false;
  }

  /** Validated runtime obstacle drop; see js/world.js addObstacle. */
  std::expected<Placed, std::string_view> addObstacle(double x, double y, double agentR, Vec2 from, bool allowSealing = false) {
    Mulberry32 rng(static_cast<std::uint32_t>(toInt32(x * 7919 + y * 104729 + version * 31)));
    const double radius = 26 + rng() * 22;
    const int nv = 6 + static_cast<int>(std::floor(rng() * 5));
    Polygon p = makeBlob(rng, x, y, radius, nv);
    if (p.circlePenetration(from.x, from.y, agentR + 4)) return std::unexpected("would overlap the agent");
    if (p.circlePenetration(goal.x, goal.y, agentR + 4)) return std::unexpected("would cover B");
    if (!fine->ready) fine->buildStep(kInf);
    const double bx0 = p.minX, by0 = p.minY, bx1 = p.maxX, by1 = p.maxY;
    polys.push_back(std::move(p));
    onGeometryChanged(bx0, by0, bx1, by1);
    bool sealed = !fine->gridConnected(from, goal, agentR);
    if (sealed) {
      // No grid route is left, only possibly a pixel-level squeeze (the planner's escape tier). Trust that
      // only if the agent was already off the grid before this drop; see js/world.js addObstacle.
      Polygon dropped = std::move(polys.back());
      polys.pop_back();
      onGeometryChanged(bx0, by0, bx1, by1);
      const bool hadGridRoute = fine->gridConnected(from, goal, agentR);
      polys.push_back(std::move(dropped));
      onGeometryChanged(bx0, by0, bx1, by1);
      sealed = hadGridRoute || !fine->connected(from, goal, agentR);
    }
    if (sealed && !allowSealing) {
      polys.pop_back();
      onGeometryChanged(bx0, by0, bx1, by1);
      return std::unexpected("would seal B off from the agent");
    }
    version++;
    return Placed{polys.size() - 1, sealed};
  }

  void onGeometryChanged(double minX, double minY, double maxX, double maxY) {
    rebuildHash();
    nav->updateRegion(minX, minY, maxX, maxY);
    fine->updateRegion(minX, minY, maxX, maxY);
  }

  void rebuildHash() {
    hash.assign(static_cast<std::size_t>(hashCols * hashRows), {});
    for (int idx = 0; idx < static_cast<int>(polys.size()); idx++) {
      const auto& p = polys[idx];
      const int x0 = clampi(static_cast<int>(std::floor(p.minX / hashSize)), 0, hashCols - 1);
      const int x1 = clampi(static_cast<int>(std::floor(p.maxX / hashSize)), 0, hashCols - 1);
      const int y0 = clampi(static_cast<int>(std::floor(p.minY / hashSize)), 0, hashRows - 1);
      const int y1 = clampi(static_cast<int>(std::floor(p.maxY / hashSize)), 0, hashRows - 1);
      for (int y = y0; y <= y1; y++) for (int x = x0; x <= x1; x++) hash[y * hashCols + x].push_back(idx);
    }
  }

  /**
   * Visit polygons whose AABB overlaps the box, in the same order as the JS query(). The visitor
   * returns true to stop early. No allocation, unlike the JS version which builds an array.
   */
  template <class F>
  void forEachNear(double minX, double minY, double maxX, double maxY, F&& f) const {
    const int x0 = clampi(static_cast<int>(std::floor(minX / hashSize)), 0, hashCols - 1);
    const int x1 = clampi(static_cast<int>(std::floor(maxX / hashSize)), 0, hashCols - 1);
    const int y0 = clampi(static_cast<int>(std::floor(minY / hashSize)), 0, hashRows - 1);
    const int y1 = clampi(static_cast<int>(std::floor(maxY / hashSize)), 0, hashRows - 1);
    const std::uint32_t s = ++stamp_;
    for (int y = y0; y <= y1; y++) for (int x = x0; x <= x1; x++) {
      for (int idx : hash[y * hashCols + x]) {
        const auto& p = polys[idx];
        if (p.stamp == s) continue;
        p.stamp = s;
        if (p.maxX < minX || p.minX > maxX || p.maxY < minY || p.minY > maxY) continue;
        if (f(p)) return;
      }
    }
  }

  bool discFree(double x, double y, double r) const {
    if (x - r < 0 || y - r < 0 || x + r > width || y + r > height) return false;
    bool hit = false;
    forEachNear(x - r, y - r, x + r, y + r, [&](const Polygon& p) { return hit = p.circlePenetration(x, y, r).has_value(); });
    return !hit;
  }

  bool segmentFree(double ax, double ay, double bx, double by, double r) const {
    if (std::min(ax, bx) - r < 0 || std::min(ay, by) - r < 0 || std::max(ax, bx) + r > width || std::max(ay, by) + r > height) return false;
    bool hit = false;
    forEachNear(std::min(ax, bx) - r, std::min(ay, by) - r, std::max(ax, bx) + r, std::max(ay, by) + r,
                [&](const Polygon& p) { return hit = p.segmentDistance(ax, ay, bx, by).d < r; });
    return !hit;
  }

  double sweep(double ax, double ay, double bx, double by, double r) const {
    double t = 1;
    forEachNear(std::min(ax, bx) - r, std::min(ay, by) - r, std::max(ax, bx) + r, std::max(ay, by) + r, [&](const Polygon& p) {
      const double h = p.sweepHit(ax, ay, bx, by, r);
      if (h < t) t = h;
      return false;
    });
    const double dx = bx - ax, dy = by - ay;
    auto wall = [](double lim, double a, double d) { return d != 0 ? (lim - a) / d : kInf; };
    for (double tw : {wall(r, ax, dx), wall(width - r, ax, dx), wall(r, ay, dy), wall(height - r, ay, dy)})
      if (tw >= 0 && tw < t) t = tw;
    return t;
  }

  int resolveDisc(Vec2& pos, double r) const {
    int hits = 0;
    for (int iter = 0; iter < 3; iter++) {
      bool moved = false;
      // Collect first, then apply, exactly like the JS (its query() returns an array up front).
      scratch_.clear();
      forEachNear(pos.x - r, pos.y - r, pos.x + r, pos.y + r, [&](const Polygon& p) { scratch_.push_back(&p); return false; });
      for (const Polygon* p : scratch_) {
        if (auto pen = p->circlePenetration(pos.x, pos.y, r)) { pos.x += pen->x; pos.y += pen->y; moved = true; hits++; }
      }
      if (pos.x < r) { pos.x = r; moved = true; }
      if (pos.y < r) { pos.y = r; moved = true; }
      if (pos.x > width - r) { pos.x = width - r; moved = true; }
      if (pos.y > height - r) { pos.y = height - r; moved = true; }
      if (!moved) break;
    }
    return hits;
  }

  struct Surface { double d, nx, ny; };
  std::optional<Surface> nearestSurface(double x, double y, double range) const {
    std::optional<Surface> best;
    forEachNear(x - range, y - range, x + range, y + range, [&](const Polygon& p) {
      const auto e = p.nearestEdgePoint(x, y);
      const bool inside = p.contains(x, y);
      const double d = inside ? -e.d : e.d;
      if (!best || d < best->d) {
        double nx = x - e.qx, ny = y - e.qy, l = std::hypot(nx, ny);
        if (l == 0) l = 1;
        nx /= l; ny /= l;
        if (inside) { nx = -nx; ny = -ny; }
        best = Surface{d, nx, ny};
      }
      return false;
    });
    const Surface walls[] = {{x, 1, 0}, {width - x, -1, 0}, {y, 0, 1}, {height - y, 0, -1}};
    for (auto w : walls) if (w.d < range && (!best || w.d < best->d)) best = w;
    if (best && best->d < range) return best;
    return std::nullopt;
  }

 private:
  mutable std::uint32_t stamp_ = 0;
  mutable std::vector<const Polygon*> scratch_;
};

// ---------------- NavGrid ----------------

inline NavGrid::NavGrid(const World& w, double cs, double inf, bool deferred)
    : world(&w), cellSize(cs), inflation(inf),
      cols(static_cast<int>(std::floor(w.width / cs))), rows(static_cast<int>(std::floor(w.height / cs))),
      blocked(static_cast<std::size_t>(cols * rows), 0) {
  if (!deferred) buildStep(kInf);
}

inline bool NavGrid::buildStep(double budgetMs) {
  const double t0 = nowMs();
  constexpr int band = 4;
  const double cs = cellSize;
  while (builtRows < rows) {
    const int r1 = std::min(rows, builtRows + band);
    rowClamp_ = std::pair{builtRows, r1 - 1};
    updateRegion(0, builtRows * cs + inflation + cs, world->width, (r1 - 1) * cs - inflation - cs);
    rowClamp_.reset();
    builtRows = r1;
    if (nowMs() - t0 > budgetMs) break;
  }
  ready = builtRows >= rows;
  return ready;
}

inline void NavGrid::updateRegion(double minX, double minY, double maxX, double maxY) {
  const double cs = cellSize, inf = inflation, m = inf + cs, W = world->width, H = world->height;
  const int c0 = clampi(static_cast<int>(std::floor((minX - m) / cs)), 0, cols - 1);
  const int c1 = clampi(static_cast<int>(std::ceil((maxX + m) / cs)), 0, cols - 1);
  int r0 = clampi(static_cast<int>(std::floor((minY - m) / cs)), 0, rows - 1);
  int r1 = clampi(static_cast<int>(std::ceil((maxY + m) / cs)), 0, rows - 1);
  if (rowClamp_) { r0 = rowClamp_->first; r1 = rowClamp_->second; }
  auto* b = blocked.data();
  for (int r = r0; r <= r1; r++) {
    const double y = (r + 0.5) * cs;
    for (int c = c0; c <= c1; c++) {
      const double x = (c + 0.5) * cs;
      b[r * cols + c] = (x < inf || y < inf || x > W - inf || y > H - inf) ? 1 : 0;
    }
  }
  for (const auto& p : world->polys) {
    if (p.maxX + inf < c0 * cs || p.minX - inf > (c1 + 1) * cs || p.maxY + inf < r0 * cs || p.minY - inf > (r1 + 1) * cs) continue;
    const int pc0 = std::max(c0, static_cast<int>(std::floor((p.minX - inf) / cs)));
    const int pc1 = std::min(c1, static_cast<int>(std::ceil((p.maxX + inf) / cs)));
    const int pr0 = std::max(r0, static_cast<int>(std::floor((p.minY - inf) / cs)));
    const int pr1 = std::min(r1, static_cast<int>(std::ceil((p.maxY + inf) / cs)));
    for (int r = pr0; r <= pr1; r++) {
      const double y = (r + 0.5) * cs;
      for (int c = pc0; c <= pc1; c++) {
        const int i = r * cols + c;
        if (!b[i] && p.circlePenetration((c + 0.5) * cs, y, inf)) b[i] = 1;
      }
    }
  }
}

inline std::vector<Seed> NavGrid::seedCells(double x, double y, double agentR, double maxDist) const {
  const double cs = cellSize;
  const auto surf = world->nearestSurface(x, y, agentR + world->fineMargin + 1);
  const double clear = surf ? surf->d : agentR + world->fineMargin;
  const double sweepR = clampd(clear - 0.05, agentR - 0.25, agentR + world->fineMargin);
  const int i0 = cellIndexAt(x, y);
  const int c0 = i0 % cols, r0 = i0 / cols;
  const int maxRing = static_cast<int>(std::ceil(maxDist / cs)), band = static_cast<int>(std::ceil((2 * agentR + 6) / cs));
  int lastRing = maxRing;
  std::vector<Seed> out;
  for (int ring = 0; ring <= lastRing; ring++) {
    for (int dr = -ring; dr <= ring; dr++) for (int dc = -ring; dc <= ring; dc++) {
      if (std::abs(dr) != ring && std::abs(dc) != ring) continue;
      const int r = r0 + dr, c = c0 + dc;
      if (r < 0 || c < 0 || r >= rows || c >= cols) continue;
      const int j = r * cols + c;
      if (blocked[j]) continue;
      const Vec2 p = cellCenter(j);
      if (!world->segmentFree(x, y, p.x, p.y, sweepR)) continue;
      out.push_back({j, std::hypot(p.x - x, p.y - y)});
    }
    if (!out.empty() && lastRing == maxRing) lastRing = std::min(maxRing, ring + band);
  }
  std::ranges::stable_sort(out, {}, &Seed::d);  // JS Array.sort is stable
  return out;
}

inline bool NavGrid::reachableMulti(const std::vector<int>& starts, const std::vector<int>& goals) const {
  if (starts.empty() || goals.empty()) return false;
  std::vector<std::uint8_t> seen(blocked.size(), 0), goal(blocked.size(), 0);
  for (int g : goals) goal[g] = 1;
  std::vector<int> q;
  for (int s : starts) if (!seen[s]) { seen[s] = 1; q.push_back(s); }
  while (!q.empty()) {
    const int i = q.back(); q.pop_back();
    if (goal[i]) return true;
    const int c = i % cols, r = i / cols;
    for (int dr = -1; dr <= 1; dr++) for (int dc = -1; dc <= 1; dc++) {
      if (!dr && !dc) continue;
      const int nr = r + dr, nc = c + dc;
      if (nr < 0 || nc < 0 || nr >= rows || nc >= cols) continue;
      const int j = nr * cols + nc;
      if (blocked[j] || seen[j]) continue;
      if (dr && dc && (blocked[r * cols + nc] || blocked[nr * cols + c])) continue;
      seen[j] = 1; q.push_back(j);
    }
  }
  return false;
}

// connected() is defined in pathfinder.hpp: its last resort is the planner's escapeRoute().

}  // namespace sim
