// pathfinder.hpp — MCU (float) variant of cpp/src/pathfinder.hpp; see geometry.hpp for the `real` type.
// Original: C++23 port of js/pathfinder.js: resumable multi-source A* + path smoothing.
#pragma once
#include "world.hpp"

namespace sim {

enum class SearchStatus { Idle, Running, Found, Failed };

/**
 * Per-cell 8-bit labels that clear in O(1): each entry carries the generation that wrote it, and reset()
 * just starts a new generation. Clearing a 480x320 array in PSRAM costs 2-10 ms on an ESP32-S3 (the A*
 * arrays together took 60-120 ms), which is more than a frame slice, so searches must not clear eagerly.
 */
class Labels {
 public:
  void reset(std::size_t n) {
    reserve(n);
    if (++gen_ >= (1u << 24)) { std::ranges::fill(v_, 0u); gen_ = 1; }  // wraps after 16M searches
  }
  /** Grow-only: allocating (and zeroing) 600 KB of PSRAM is itself a 10-30 ms stall, so do it up front. */
  void reserve(std::size_t n) { if (v_.size() < n) { v_.assign(n, 0); gen_ = 0; } }
  std::uint8_t get(std::size_t i) const { const auto x = v_[i]; return (x >> 8) == gen_ ? static_cast<std::uint8_t>(x) : 0; }
  void set(std::size_t i, std::uint8_t value) { v_[i] = gen_ << 8 | value; }

 private:
  std::vector<std::uint32_t> v_;
  std::uint32_t gen_ = 0;
};

class AStar {
 public:
  const NavGrid* grid = nullptr;
  SearchStatus status = SearchStatus::Idle;
  long expanded = 0;
  double msSpent = 0;
  int slices = 0;

  void start(const NavGrid& gr, const std::vector<Seed>& starts, const std::vector<Seed>& goals) {
    grid = &gr;
    const std::size_t n = static_cast<std::size_t>(gr.cols * gr.rows);
    reserve(n);
    // Float32 costs, exactly like the JS Float32Array, so ties break the same way. Cells are initialised
    // lazily (touch()) instead of filling ~2 MB here: that fill alone was 60-120 ms on an ESP32-S3.
    flags_.reset(n);
    heap_.clear();
    expanded = 0; msSpent = 0; slices = 0; reached_ = -1;
    if (starts.empty() || goals.empty()) { status = SearchStatus::Failed; return; }
    goalIdx_ = goals[0].i;
    goalSpread_ = 0;
    for (auto& gl : goals) { touch(gl.i); flags_.set(gl.i, flags_.get(gl.i) | kGoal); goalSpread_ = std::max(goalSpread_, octile(gl.i, goalIdx_)); }
    for (auto& st : starts) {
      const real g0 = st.d / gr.cellSize;
      touch(st.i);
      if (g0 < g_[st.i]) { g_[st.i] = static_cast<float>(g0); f_[st.i] = static_cast<float>(g0 + h(st.i)); push(st.i); }
    }
    status = SearchStatus::Running;
  }

  /** Grow-only storage, shared by the coarse and fine grids (shrinking would reallocate on every switch). */
  void reserve(std::size_t n) {
    if (g_.size() < n) { g_.resize(n); f_.resize(n); parent_.resize(n); }
    flags_.reserve(n);
  }

  SearchStatus step(double budgetMs) {
    if (status != SearchStatus::Running) return status;
    const double t0 = nowMs();
    const int cols = grid->cols, rows = grid->rows;
    const auto* blocked = grid->blocked.data();
    long n = 0;
    slices++;
    while (!heap_.empty()) {
      if ((n & 31) == 0 && nowMs() - t0 > budgetMs) { msSpent += nowMs() - t0; return status; }
      const int cur = pop();
      const std::uint8_t fl = flags_.get(cur);
      if (fl & kClosed) continue;
      flags_.set(cur, fl | kClosed);
      expanded++; n++;
      if (fl & kGoal) { reached_ = cur; status = SearchStatus::Found; break; }
      const int c = cur % cols, r = cur / cols;
      for (int dr = -1; dr <= 1; dr++) for (int dc = -1; dc <= 1; dc++) {
        if (!dr && !dc) continue;
        const int nr = r + dr, nc = c + dc;
        if (nr < 0 || nc < 0 || nr >= rows || nc >= cols) continue;
        const int ni = nr * cols + nc;
        if (blocked[ni]) continue;
        touch(ni);
        if (flags_.get(ni) & kClosed) continue;
        if (dr && dc && (blocked[r * cols + nc] || blocked[nr * cols + c])) continue;
        const real ng = static_cast<real>(g_[cur]) + ((dr && dc) ? kSqrt2 : 1.0f);
        if (ng < g_[ni]) {
          g_[ni] = static_cast<float>(ng);
          f_[ni] = static_cast<float>(ng + h(ni) * 1.001f);
          parent_[ni] = cur;
          push(ni);
        }
      }
    }
    if (status == SearchStatus::Running) status = SearchStatus::Failed;
    msSpent += nowMs() - t0;
    return status;
  }

  std::vector<Vec2> path() const {
    std::vector<Vec2> out;
    for (int i = reached_; i != -1; i = parent_[i]) out.push_back(grid->cellCenter(i));
    std::ranges::reverse(out);
    return out;
  }

 private:
  std::vector<float> g_, f_;
  std::vector<int> parent_;
  Labels flags_;  // kSeen once g/f/parent hold this search's values
  static constexpr std::uint8_t kSeen = 1, kClosed = 2, kGoal = 4;

  void touch(int i) {
    if (flags_.get(i) & kSeen) return;
    g_[i] = f_[i] = std::numeric_limits<float>::infinity();
    parent_[i] = -1;
    flags_.set(i, kSeen);
  }
  std::vector<int> heap_;
  int goalIdx_ = 0, reached_ = -1;
  real goalSpread_ = 0;

  real octile(int a, int b) const {
    const int cols = grid->cols;
    const real dx = std::abs(a % cols - b % cols), dy = std::abs(a / cols - b / cols);
    return (dx + dy) + (kSqrt2 - 2) * std::min(dx, dy);
  }
  real h(int i) const { return std::max(real(0), octile(i, goalIdx_) - goalSpread_); }

  // Binary heap with the same sift rules as the JS MinHeap (so equal keys pop in the same order).
  void push(int v) {
    heap_.push_back(v);
    std::size_t i = heap_.size() - 1;
    while (i > 0) {
      const std::size_t p = (i - 1) >> 1;
      if (f_[heap_[p]] <= f_[heap_[i]]) break;
      std::swap(heap_[p], heap_[i]); i = p;
    }
  }
  int pop() {
    const int top = heap_[0];
    const int last = heap_.back(); heap_.pop_back();
    if (!heap_.empty()) {
      heap_[0] = last;
      std::size_t i = 0;
      for (;;) {
        const std::size_t l = 2 * i + 1, r = l + 1;
        std::size_t m = i;
        if (l < heap_.size() && f_[heap_[l]] < f_[heap_[m]]) m = l;
        if (r < heap_.size() && f_[heap_[r]] < f_[heap_[m]]) m = r;
        if (m == i) break;
        std::swap(heap_[m], heap_[i]); i = m;
      }
    }
    return top;
  }
};

/** Keep only direction changes; see js/pathfinder.js dropCollinear. */
inline std::vector<Vec2> dropCollinear(const std::vector<Vec2>& p) {
  if (p.size() <= 2) return p;
  std::vector<Vec2> out{p[0]};
  for (std::size_t i = 1; i + 1 < p.size(); i++) {
    const Vec2 a = out.back(), b = p[i], c = p[i + 1];
    const real cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    const real dt = (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y);
    if (std::abs(cross) > 1e-9f || dt <= 0) out.push_back(b);
  }
  out.push_back(p.back());
  return out;
}

/**
 * Forward string-pulling, one sweep test per point; see js/pathfinder.js smoothPath. Resumable: a long fine-grid
 * path is hundreds of sweeps, which on the chip is more than a frame, so the planner runs it a slice at a time.
 */
class PathSmoother {
 public:
  void start(const std::vector<Vec2>& input, const World& w, real r) {
    w_ = &w; r_ = r;
    pts_ = dropCollinear(input);
    out_.clear();
    if (pts_.size() > 2) out_.push_back(pts_[0]);
    i_ = 0; j_ = 1; done_ = false;
  }
  /** Run sweeps until done (true) or the budget is spent (false). */
  bool step(double budgetMs) {
    if (pts_.size() <= 2 || done_) return true;
    const double t0 = nowMs();
    while (j_ < pts_.size() - 1) {
      if (w_->segmentFree(pts_[i_].x, pts_[i_].y, pts_[j_ + 1].x, pts_[j_ + 1].y, r_)) j_++;
      else { out_.push_back(pts_[j_]); i_ = j_; j_ = i_ + 1; }
      if (nowMs() - t0 >= budgetMs) return j_ >= pts_.size() - 1 && finish();
    }
    return finish();
  }
  std::vector<Vec2> take() { return pts_.size() <= 2 ? std::move(pts_) : std::move(out_); }

 private:
  bool finish() { out_.push_back(pts_.back()); return done_ = true; }
  const World* w_ = nullptr;
  real r_ = 0;
  std::vector<Vec2> pts_, out_;
  std::size_t i_ = 0, j_ = 1;
  bool done_ = false;
};

inline std::vector<Vec2> smoothPath(const std::vector<Vec2>& input, const World& w, real r) {
  PathSmoother s;
  s.start(input, w, r);
  s.step(kInf);
  return s.take();
}

inline real pathLength(const std::vector<Vec2>& p) {
  real l = 0;
  for (std::size_t i = 1; i < p.size(); i++) l += dist(p[i - 1], p[i]);
  return l;
}

struct Escape { std::vector<Vec2> path; int cell; real length; };

/** Resumable flood of the grid component that contains the given cells (8-connected, no corner cutting). */
class ComponentFlood {
 public:
  void start(const NavGrid& g, const std::vector<int>& seeds) {
    grid_ = &g;
    value_ = 1;
    comp_.reset(static_cast<std::size_t>(g.cols * g.rows));
    stack_.clear();
    for (int i : seeds) if (!comp_.get(i)) { comp_.set(i, 1); stack_.push_back(i); }
    done_ = false;
  }
  bool step(double budgetMs) {
    if (done_) return true;
    const double t0 = nowMs();
    const NavGrid& g = *grid_;
    long n = 0;
    while (!stack_.empty()) {
      if ((++n & 1023) == 0 && nowMs() - t0 > budgetMs) return false;
      const int i = stack_.back(); stack_.pop_back();
      const int c = i % g.cols, rr = i / g.cols;
      for (int dr = -1; dr <= 1; dr++) for (int dc = -1; dc <= 1; dc++) {
        if (!dr && !dc) continue;
        const int nr = rr + dr, nc = c + dc;
        if (nr < 0 || nc < 0 || nr >= g.rows || nc >= g.cols) continue;
        const int j = nr * g.cols + nc;
        if (g.blocked[j] || comp_.get(j)) continue;
        if (dr && dc && (g.blocked[rr * g.cols + nc] || g.blocked[nr * g.cols + c])) continue;
        comp_.set(j, 1); stack_.push_back(j);
      }
    }
    done_ = true;
    return true;
  }
  /** Take over a finished component: the cells of `labels` equal to `value` (see ConnectSearch::takeSideB). */
  void adopt(const NavGrid& g, Labels& labels, std::uint8_t value) {
    grid_ = &g; std::swap(comp_, labels); value_ = value; stack_.clear(); done_ = true;  // swap: both keep a buffer
  }
  void reserve(std::size_t n) { comp_.reserve(n); }
  bool done() const { return done_; }
  bool in(int cell) const { return comp_.get(cell) == value_; }

 private:
  const NavGrid* grid_ = nullptr;
  Labels comp_;
  std::uint8_t value_ = 1;
  std::vector<int> stack_;
  bool done_ = true;
};

/**
 * Resumable connectivity test between two seed sets: two greedy best-first searches, one from each side,
 * each heading for the other side, expanded alternately. They usually meet after exploring a corridor, so
 * a connected answer costs a small fraction of a full flood; a disconnected answer ends as soon as the
 * smaller side is exhausted. Same answer as NavGrid::reachableMulti.
 */
class ConnectSearch {
 public:
  enum class Result { Running, Connected, Disconnected };

  void start(const NavGrid& g, const std::vector<int>& a, Vec2 aTarget, const std::vector<int>& b, Vec2 bTarget) {
    grid_ = &g;
    mark_.reset(static_cast<std::size_t>(g.cols * g.rows));
    for (auto* h : {&heapA_, &heapB_}) h->clear();
    targetA_ = g.cellIndexAt(aTarget.x, aTarget.y);
    targetB_ = g.cellIndexAt(bTarget.x, bTarget.y);
    result_ = Result::Running;
    for (int i : a) visit(i, 1, heapA_, targetA_);
    for (int i : b) {
      if (mark_.get(i) == 1) { result_ = Result::Connected; return; }
      visit(i, 2, heapB_, targetB_);
    }
    if (heapA_.empty() || heapB_.empty()) result_ = Result::Disconnected;
  }

  Result step(double budgetMs) {
    if (result_ != Result::Running) return result_;
    const double t0 = nowMs();
    for (long n = 1;; n++) {
      if ((n & 63) == 0 && nowMs() - t0 > budgetMs) return result_;
      if (expand(heapA_, 1, targetA_) || expand(heapB_, 2, targetB_)) return result_ = Result::Connected;
      if (heapA_.empty() || heapB_.empty()) return result_ = Result::Disconnected;
    }
  }

  /**
   * Which side reached this cell: 0 neither, 1 A, 2 B. After a Connected result a side-B cell is grid-connected
   * to B's seeds. A side-A cell is connected to one of A's seeds, and so to B only if A started from a single
   * seed: A's seeds can lie in several components (see DropTask's re-check).
   */
  std::uint8_t side(int cell) const { return static_cast<std::uint8_t>(mark_.get(cell)); }

  /** After a Disconnected result: finish exploring B's side (what the pixel escape needs). Resumable. */
  bool finishSideB(double budgetMs) {
    const double t0 = nowMs();
    for (long n = 1; !heapB_.empty(); n++) {
      if ((n & 63) == 0 && nowMs() - t0 > budgetMs) return false;
      expand(heapB_, 2, targetB_);
    }
    return true;
  }
  /** B's component as a flood, without copying: the search's labels move into it. */
  void takeSideB(ComponentFlood& out) { out.adopt(*grid_, mark_, 2); }
  void reserve(std::size_t n) { mark_.reserve(n); }

 private:
  using Item = std::pair<int, int>;  // (octile distance to the target in cell units x 10, cell)
  struct Cmp { bool operator()(const Item& x, const Item& y) const { return x.first > y.first; } };
  const NavGrid* grid_ = nullptr;
  Labels mark_;  // 0 unseen, 1 side A, 2 side B
  std::vector<Item> heapA_, heapB_;
  int targetA_ = 0, targetB_ = 0;
  Result result_ = Result::Disconnected;

  int key(int i, int target) const {
    const int cols = grid_->cols;
    const int dx = std::abs(i % cols - target % cols), dy = std::abs(i / cols - target / cols);
    return 10 * (dx + dy) - 6 * std::min(dx, dy);  // octile, integer
  }
  void visit(int i, std::uint8_t side, std::vector<Item>& heap, int target) {
    if (mark_.get(i)) return;
    mark_.set(i, side);
    heap.push_back({key(i, target), i});
    std::ranges::push_heap(heap, Cmp{});
  }
  /** Expand one cell of a side; true when it touches the other side. */
  bool expand(std::vector<Item>& heap, std::uint8_t side, int target) {
    if (heap.empty()) return false;
    std::ranges::pop_heap(heap, Cmp{});
    const int cur = heap.back().second;
    heap.pop_back();
    const NavGrid& g = *grid_;
    const int c = cur % g.cols, r = cur / g.cols;
    for (int dr = -1; dr <= 1; dr++) for (int dc = -1; dc <= 1; dc++) {
      if (!dr && !dc) continue;
      const int nr = r + dr, nc = c + dc;
      if (nr < 0 || nc < 0 || nr >= g.rows || nc >= g.cols) continue;
      const int j = nr * g.cols + nc;
      if (g.blocked[j]) continue;
      if (dr && dc && (g.blocked[r * g.cols + nc] || g.blocked[nr * g.cols + c])) continue;
      if (const auto m = mark_.get(j); m && m != side) return true;
      visit(j, side, heap, target);
    }
    return false;
  }
};

/**
 * Planning tier 3, resumable: pixel-level escape onto the fine grid (see js/pathfinder.js EscapeSearch).
 * Phase 1 floods the goal's grid component, phase 2 runs a pixel BFS around `from` until it reaches a
 * cell of that component. Unsliced this is the slowest step on an MCU; step() keeps it in budget.
 */
class EscapeSearch {
 public:
  void start(const World& w, const NavGrid& g, Vec2 from, real r, const std::vector<int>& goalSeedIds, int window = 160) {
    begin(w, g, from, r, window);
    flood_.start(g, goalSeedIds);
  }
  /** Same, reusing an already flooded goal component (drop validation has one); swapped in, not copied. */
  void start(const World& w, const NavGrid& g, Vec2 from, real r, ComponentFlood& goalComponent, int window = 160) {
    begin(w, g, from, r, window);
    std::swap(flood_, goalComponent);
  }
  void reserve(std::size_t gridCells, int window = 160) {
    flood_.reserve(gridCells);
    px_.reserve(static_cast<std::size_t>((2 * window + 1) * (2 * window + 1)));
  }

  /** Returns true once finished; result() is the escape, or nullopt if none exists in the window. */
  bool step(double budgetMs) {
    if (done_) return true;
    const double t0 = nowMs();
    if (!flood_.step(budgetMs)) return false;
    const NavGrid& grid = *grid_;
    long n = 0;
    while (qi_ < queue_.size()) {
      if ((++n & 3) == 0 && nowMs() - t0 > budgetMs) return false;
      const int k = queue_[qi_++], px = x0_ + k % size_, py = y0_ + k / size_;
      const int cell = grid.cellIndexAt(px, py);
      if (k != startK_ && flood_.in(cell) && !grid.blocked[cell]) {
        std::vector<Vec2> path;
        for (int m = k;; m = back(m)) {
          path.push_back({static_cast<real>(x0_ + m % size_), static_cast<real>(y0_ + m / size_)});
          if (m == startK_) break;
        }
        std::ranges::reverse(path);
        path[0] = from_;
        const real len = pathLength(path);
        result_ = Escape{std::move(path), cell, len};
        done_ = true;
        return true;
      }
      const int kx = k % size_, ky = k / size_;
      constexpr int d4[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
      for (int d = 0; d < 4; d++) {
        const int nx = kx + d4[d][0], ny = ky + d4[d][1];
        if (nx < 0 || ny < 0 || nx >= size_ || ny >= size_) continue;
        const int j = ny * size_ + nx;
        if ((px_.get(j) >> 2) || j == startK_ || !isFree(j)) continue;
        px_.set(j, static_cast<std::uint8_t>(px_.get(j) | (d + 1) << 2));
        queue_.push_back(j);
      }
    }
    done_ = true;
    return true;
  }

  bool done() const { return done_; }
  const std::optional<Escape>& result() const { return result_; }

 private:
  const World* world_ = nullptr;
  const NavGrid* grid_ = nullptr;
  Vec2 from_{};
  real r_ = 0;
  int window_ = 0, size_ = 0, x0_ = 0, y0_ = 0, startK_ = 0;
  std::size_t qi_ = 0;
  Labels px_;  // bits 0-1: 1 free, 2 blocked (0 untested); bits 2-4: d4 index+1 of the step that reached it
  std::vector<int> queue_;
  ComponentFlood flood_;
  std::optional<Escape> result_;
  bool done_ = true;

  void begin(const World& w, const NavGrid& g, Vec2 from, real r, int window) {
    world_ = &w; grid_ = &g; from_ = from; r_ = r; window_ = window;
    size_ = 2 * window + 1;
    x0_ = static_cast<int>(std::floor(from.x + 0.5f)) - window;  // JS Math.round
    y0_ = static_cast<int>(std::floor(from.y + 0.5f)) - window;
    px_.reset(static_cast<std::size_t>(size_ * size_));
    startK_ = window * size_ + window;
    px_.set(startK_, 1);
    queue_.assign(1, startK_);
    qi_ = 0;
    result_.reset();
    done_ = false;
  }
  int back(int k) const {  // the pixel this one was reached from
    constexpr int d4[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    const auto& d = d4[(px_.get(k) >> 2) - 1];
    return k - d[1] * size_ - d[0];
  }
  bool isFree(int k) {  // reduced clearance only near the start; see js/pathfinder.js
    auto st = px_.get(k);
    if (!(st & 3)) {
      const int kx = k % size_, ky = k / size_;
      const bool nearStart = (kx - window_) * (kx - window_) + (ky - window_) * (ky - window_) <= 16;
      st |= world_->discFree(x0_ + kx, y0_ + ky, nearStart ? r_ - 0.3f : r_ + world_->fineMargin) ? 1 : 2;
      px_.set(k, st);
    }
    return (st & 3) == 1;
  }
};

inline std::optional<Escape> escapeRoute(const World& world, const NavGrid& grid, Vec2 from, real r,
                                         const std::vector<int>& goalSeedIds, int window = 160) {
  EscapeSearch e;
  e.start(world, grid, from, r, goalSeedIds, window);
  e.step(kInf);
  return e.result();
}

inline bool NavGrid::connected(Vec2 from, Vec2 to, real agentR) const {
  std::vector<int> s, g;
  for (auto& c : seedCells(from.x, from.y, agentR)) s.push_back(c.i);
  for (auto& c : seedCells(to.x, to.y, agentR)) g.push_back(c.i);
  if (reachableMulti(s, g)) return true;
  return escapeRoute(*world, *this, from, agentR, g).has_value();  // same last resort as the planner
}

}  // namespace sim
