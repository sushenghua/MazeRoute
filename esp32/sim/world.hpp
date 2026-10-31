// world.hpp — MCU (float) variant of cpp/src/world.hpp; see geometry.hpp for the `real` type.
// Original: C++23 port of js/world.js: arena generation, spatial hash, two nav grids.
#pragma once
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>
#include "geometry.hpp"

namespace sim {

#ifndef ESP_PLATFORM
/**
 * Host only: SIM_FAKE_CLOCK="<seed>:<mean us>[:<stall chance>:<stall us>]" replaces the wall clock with one
 * that advances a random 0..2x mean per call, plus, with the given chance per call, a stall (an OS preemption).
 * Slice boundaries then depend only on the seed, so a timing-dependent failure seen once under SLICE_MS can be
 * reproduced, and many interleavings searched for one. The harness reseeds it per mission, so a mission
 * replays the same way alone as inside a full suite run.
 *
 * SIM_CLOCK_RECORD=<dir> keeps the wall clock but records every reading of a mission, and the harness writes
 * them to <dir> when the mission fails; SIM_CLOCK_REPLAY=<file> plays such a recording back, which repeats
 * that mission's slicing exactly.
 */
struct FakeClock {
  bool on = false;
  std::uint64_t seed = 0, s = 0;
  double meanMs = 0, stallP = 0, stallMs = 0, t = 0;
  const char* recordDir = nullptr;
  std::vector<double> tape;  // recorded or replayed readings
  std::size_t pos = 0;
  bool replay = false;
  static FakeClock& get() {
    static FakeClock c = [] {
      FakeClock f;
      unsigned long long seed = 0; double us = 0, p = 0, stallUs = 0;
      if (const char* e = std::getenv("SIM_FAKE_CLOCK"); e && std::sscanf(e, "%llu:%lf:%lf:%lf", &seed, &us, &p, &stallUs) >= 2) {
        f.on = true; f.seed = seed; f.meanMs = us / 1000; f.stallP = p; f.stallMs = stallUs / 1000;
      }
      f.recordDir = std::getenv("SIM_CLOCK_RECORD");
      if (const char* e = std::getenv("SIM_CLOCK_REPLAY")) {
        if (std::FILE* fp = std::fopen(e, "rb")) {
          double v;
          while (std::fread(&v, sizeof v, 1, fp) == 1) f.tape.push_back(v);
          std::fclose(fp);
          f.on = f.replay = true;
        }
      }
      f.reseed(0);
      return f;
    }();
    return c;
  }
  void beginMission(std::uint64_t k) { reseed(k); pos = 0; if (recordDir) tape.clear(); }
  void save(const std::string& name) const {
    if (!recordDir) return;
    if (std::FILE* fp = std::fopen((std::string(recordDir) + "/" + name).c_str(), "wb")) {
      std::fwrite(tape.data(), sizeof(double), tape.size(), fp);
      std::fclose(fp);
    }
  }
  void reseed(std::uint64_t k) { s = (seed * 0x9E3779B97F4A7C15ull) ^ (k * 0xD1B54A32D192ED03ull) ^ 1; for (int i = 0; i < 8; i++) next(); }
  double next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return static_cast<double>(s >> 11) / 9007199254740992.0; }
  double now() {
    if (replay) return pos < tape.size() ? tape[pos++] : (tape.empty() ? 0 : tape.back() + 1e6);
    t += meanMs * 2 * next();
    if (stallP > 0 && next() < stallP) t += stallMs;
    return t;
  }
};
#endif

inline double nowMs() {
#ifndef ESP_PLATFORM
  auto& c = FakeClock::get();
  if (c.on) return c.now();
#endif
  using namespace std::chrono;
  const double t = duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
#ifndef ESP_PLATFORM
  if (c.recordDir) c.tape.push_back(t);
#endif
  return t;
}

/** Side clearance (px) beyond the agent's radius that the fine planner is guaranteed to find. */
inline constexpr real kFineGuarantee = 2.5f;

class World;

struct Seed { int i; real d; };

/** Occupancy grid for one agent size; see js/world.js NavGrid for the design notes. */
class NavGrid {
 public:
  const World* world;
  real cellSize, inflation;
  int cols, rows;
  std::vector<std::uint8_t> blocked;
  int builtRows = 0;
  bool ready = false;

  NavGrid(const World& w, real cs, real inf, bool deferred = false);
  bool buildStep(double budgetMs);
  void updateRegion(real minX, real minY, real maxX, real maxY);
  int cellIndexAt(real x, real y) const {
    const int c = clampi(static_cast<int>(std::floor(x / cellSize)), 0, cols - 1);
    const int r = clampi(static_cast<int>(std::floor(y / cellSize)), 0, rows - 1);
    return r * cols + c;
  }
  Vec2 cellCenter(int i) const { return {((i % cols) + 0.5f) * cellSize, ((i / cols) + 0.5f) * cellSize}; }
  std::vector<Seed> seedCells(real x, real y, real agentRadius, real maxDist = 60) const;
  bool reachableMulti(const std::vector<int>& starts, const std::vector<int>& goals) const;
  bool reachable(int a, int b) const {
    if (a < 0 || b < 0 || blocked[a] || blocked[b]) return false;
    return reachableMulti({a}, {b});
  }
  bool connected(Vec2 from, Vec2 to, real agentRadius) const;
  /** A route on this grid between the seed cells of two points (no pixel-level escape). */
  bool gridConnected(Vec2 from, Vec2 to, real agentRadius) const {
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
  real width, height, cellSize;
  real fineCellSize = 2, fineMargin = 1.1f;
  std::vector<Polygon> polys;
  Vec2 start{}, goal{};
  std::uint32_t seed = 0;
  real hashSize = 80;
  int hashCols, hashRows;
  std::vector<std::vector<int>> hash;
  std::unique_ptr<NavGrid> nav, fine;
  int version = 0;
  /** A not-yet-placed obstacle that queries treat as present; see DropTask (drop.hpp). Null otherwise. */
  const Polygon* overlay = nullptr;
  real agentRadius = 0;

  World(real w, real h, real cs)
      : width(w), height(h), cellSize(cs),
        hashCols(static_cast<int>(std::ceil(w / 80))), hashRows(static_cast<int>(std::ceil(h / 80))) {}

  bool generate(std::uint32_t s, int density, real agentR) {
    seed = s;
    Mulberry32 rng(s);
    const real diag = std::hypot(width, height);
    for (int attempt = 0; attempt < 40; attempt++) {
      polys.clear();
      const int nBlobs = static_cast<int>(std::floor(density * 0.6f + 0.5f)), nBars = static_cast<int>(std::floor(density * 0.4f + 0.5f));
      for (int i = 0; i < nBlobs; i++) {
        // JS evaluates call arguments left to right; C++ does not, so draw them in order first.
        const real r = 28 + rng() * 55;
        const real cx = r + rng() * (width - 2 * r);
        const real cy = r + rng() * (height - 2 * r);
        const int nv = 6 + static_cast<int>(std::floor(rng() * 7));
        polys.push_back(makeBlob(rng, cx, cy, r, nv));
      }
      for (int i = 0; i < nBars; i++) {
        const real len = 90 + rng() * 220;
        const real ang = rng() < 0.5f ? (rng() - 0.5f) * 0.5f : kPi / 2 + (rng() - 0.5f) * 0.5f;
        const real cx = 60 + rng() * (width - 120);
        const real cy = 60 + rng() * (height - 120);
        const real thick = 7 + rng() * 8;
        const real a2 = ang + (rng() - 0.5f) * 0.4f;
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
        if (dist(pa, pb) < diag * 0.55f) continue;
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
  std::expected<Placed, std::string_view> addObstacle(real x, real y, real agentR, Vec2 from, bool allowSealing = false) {
    Polygon p = dropPolygon(x, y);
    if (p.circlePenetration(from.x, from.y, agentR + 4)) return std::unexpected("would overlap the agent");
    if (p.circlePenetration(goal.x, goal.y, agentR + 4)) return std::unexpected("would cover B");
    if (!fine->ready) fine->buildStep(kInf);
    const real bx0 = p.minX, by0 = p.minY, bx1 = p.maxX, by1 = p.maxY;
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

  /** The obstacle a drop at (x, y) creates; deterministic in the position and the world version. */
  Polygon dropPolygon(real x, real y) const {
    Mulberry32 rng(static_cast<std::uint32_t>(toInt32(x * 7919 + y * 104729 + version * 31)));
    const real radius = 26 + rng() * 22;
    const int nv = 6 + static_cast<int>(std::floor(rng() * 5));
    return makeBlob(rng, x, y, radius, nv);
  }

  /**
   * Place an obstacle that DropTask already validated. `newlyBlocked` are the fine cells it blocks; adding a
   * polygon only ever blocks cells, so this equals fine->updateRegion over its box without the cost.
   */
  std::size_t commitObstacle(Polygon p, const std::vector<int>& newlyBlocked) {
    const real bx0 = p.minX, by0 = p.minY, bx1 = p.maxX, by1 = p.maxY;
    polys.push_back(std::move(p));
    rebuildHash();
    nav->updateRegion(bx0, by0, bx1, by1);
    for (int i : newlyBlocked) fine->blocked[i] = 1;
    version++;
    return polys.size() - 1;
  }

  void onGeometryChanged(real minX, real minY, real maxX, real maxY) {
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
  void forEachNear(real minX, real minY, real maxX, real maxY, F&& f) const {
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
    if (const Polygon* p = overlay; p && !(p->maxX < minX || p->minX > maxX || p->maxY < minY || p->minY > maxY)) f(*p);
  }

  bool discFree(real x, real y, real r) const {
    if (x - r < 0 || y - r < 0 || x + r > width || y + r > height) return false;
    bool hit = false;
    forEachNear(x - r, y - r, x + r, y + r, [&](const Polygon& p) { return hit = p.circlePenetration(x, y, r).has_value(); });
    return !hit;
  }

  bool segmentFree(real ax, real ay, real bx, real by, real r) const {
    if (std::min(ax, bx) - r < 0 || std::min(ay, by) - r < 0 || std::max(ax, bx) + r > width || std::max(ay, by) + r > height) return false;
    bool hit = false;
    forEachNear(std::min(ax, bx) - r, std::min(ay, by) - r, std::max(ax, bx) + r, std::max(ay, by) + r,
                [&](const Polygon& p) { return hit = p.segmentDistance(ax, ay, bx, by).d < r; });
    return !hit;
  }

  real sweep(real ax, real ay, real bx, real by, real r) const {
    real t = 1;
    forEachNear(std::min(ax, bx) - r, std::min(ay, by) - r, std::max(ax, bx) + r, std::max(ay, by) + r, [&](const Polygon& p) {
      const real h = p.sweepHit(ax, ay, bx, by, r);
      if (h < t) t = h;
      return false;
    });
    const real dx = bx - ax, dy = by - ay;
    auto wall = [](real lim, real a, real d) { return d != 0 ? (lim - a) / d : kInf; };
    for (real tw : {wall(r, ax, dx), wall(width - r, ax, dx), wall(r, ay, dy), wall(height - r, ay, dy)})
      if (tw >= 0 && tw < t) t = tw;
    return t;
  }

  int resolveDisc(Vec2& pos, real r) const {
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

  struct Surface { real d, nx, ny; };
  std::optional<Surface> nearestSurface(real x, real y, real range) const {
    std::optional<Surface> best;
    forEachNear(x - range, y - range, x + range, y + range, [&](const Polygon& p) {
      const auto e = p.nearestEdgePoint(x, y);
      const bool inside = p.contains(x, y);
      const real d = inside ? -e.d : e.d;
      if (!best || d < best->d) {
        real nx = x - e.qx, ny = y - e.qy, l = length(nx, ny);
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

inline NavGrid::NavGrid(const World& w, real cs, real inf, bool deferred)
    : world(&w), cellSize(cs), inflation(inf),
      cols(static_cast<int>(std::floor(w.width / cs))), rows(static_cast<int>(std::floor(w.height / cs))),
      blocked(static_cast<std::size_t>(cols * rows), 0) {
  if (!deferred) buildStep(kInf);
}

inline bool NavGrid::buildStep(double budgetMs) {
  const double t0 = nowMs();
  constexpr int band = 1;  // one row per budget check: a 4-row band of the 2 px grid is ~20 ms on an ESP32-S3
  const real cs = cellSize;
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

inline void NavGrid::updateRegion(real minX, real minY, real maxX, real maxY) {
  const real cs = cellSize, inf = inflation, m = inf + cs, W = world->width, H = world->height;
  const int c0 = clampi(static_cast<int>(std::floor((minX - m) / cs)), 0, cols - 1);
  const int c1 = clampi(static_cast<int>(std::ceil((maxX + m) / cs)), 0, cols - 1);
  int r0 = clampi(static_cast<int>(std::floor((minY - m) / cs)), 0, rows - 1);
  int r1 = clampi(static_cast<int>(std::ceil((maxY + m) / cs)), 0, rows - 1);
  if (rowClamp_) { r0 = rowClamp_->first; r1 = rowClamp_->second; }
  auto* b = blocked.data();
  for (int r = r0; r <= r1; r++) {
    const real y = (r + 0.5f) * cs;
    for (int c = c0; c <= c1; c++) {
      const real x = (c + 0.5f) * cs;
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
      const real y = (r + 0.5f) * cs;
      for (int c = pc0; c <= pc1; c++) {
        const int i = r * cols + c;
        if (!b[i] && p.circlePenetration((c + 0.5f) * cs, y, inf)) b[i] = 1;
      }
    }
  }
}

/**
 * Resumable NavGrid::seedCells: the same rings in the same order with the same result, but step() stops
 * when its time budget runs out. Each candidate cell costs a line-of-sight sweep, so seeding on the 2 px
 * grid takes 16-42 ms on an ESP32-S3; sliced, it fits the frame.
 */
class SeedSearch {
 public:
  void start(const NavGrid& g, real x, real y, real agentR, real maxDist = 60) {
    grid_ = &g; x_ = x; y_ = y;
    const World& w = *g.world;
    const auto surf = w.nearestSurface(x, y, agentR + w.fineMargin + 1);
    const real clear = surf ? surf->d : agentR + w.fineMargin;
    sweepR_ = clampd(clear - 0.05f, agentR - 0.25f, agentR + w.fineMargin);
    const int i0 = g.cellIndexAt(x, y);
    c0_ = i0 % g.cols; r0_ = i0 / g.cols;
    maxRing_ = lastRing_ = static_cast<int>(std::ceil(maxDist / g.cellSize));
    band_ = static_cast<int>(std::ceil((2 * agentR + 6) / g.cellSize));
    ring_ = dr_ = dc_ = 0;
    out_.clear();
    done_ = false;
  }

  /** Returns true once finished; result() is then sorted by distance. */
  bool step(double budgetMs) {
    if (done_) return true;
    const double t0 = nowMs();
    const NavGrid& g = *grid_;
    int tested = 0;
    while (ring_ <= lastRing_) {
      while (dr_ <= ring_) {
        while (dc_ <= ring_) {
          const int dc = dc_++;
          if (std::abs(dr_) != ring_ && std::abs(dc) != ring_) { dc_ = ring_; continue; }  // interior: jump to the right edge
          const int r = r0_ + dr_, c = c0_ + dc;
          if (r < 0 || c < 0 || r >= g.rows || c >= g.cols) continue;
          const int j = r * g.cols + c;
          if (g.blocked[j]) continue;
          const Vec2 p = g.cellCenter(j);
          if (g.world->segmentFree(x_, y_, p.x, p.y, sweepR_)) out_.push_back({j, length(p.x - x_, p.y - y_)});
          if ((++tested & 3) == 0 && nowMs() - t0 > budgetMs) return false;
        }
        dr_++; dc_ = -ring_;
      }
      if (!out_.empty() && lastRing_ == maxRing_) lastRing_ = std::min(maxRing_, ring_ + band_);
      ring_++; dr_ = -ring_; dc_ = -ring_;
    }
    std::ranges::stable_sort(out_, {}, &Seed::d);  // JS Array.sort is stable
    done_ = true;
    return true;
  }

  bool done() const { return done_; }
  const std::vector<Seed>& result() const { return out_; }

 private:
  const NavGrid* grid_ = nullptr;
  real x_ = 0, y_ = 0, sweepR_ = 0;
  int c0_ = 0, r0_ = 0, maxRing_ = 0, lastRing_ = 0, band_ = 0, ring_ = 0, dr_ = 0, dc_ = 0;
  std::vector<Seed> out_;
  bool done_ = true;
};

inline std::vector<Seed> NavGrid::seedCells(real x, real y, real agentR, real maxDist) const {
  SeedSearch s;
  s.start(*this, x, y, agentR, maxDist);
  s.step(kInf);
  return s.result();
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
