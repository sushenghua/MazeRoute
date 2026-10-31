// pathfinder.hpp — C++23 port of js/pathfinder.js: resumable multi-source A* + path smoothing.
#pragma once
#include "world.hpp"

namespace sim {

enum class SearchStatus { Idle, Running, Found, Failed };

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
    if (g_.size() != n) {
      g_.resize(n); f_.resize(n); parent_.resize(n); closed_.resize(n); isGoal_.resize(n);
    }
    // Float32 costs, exactly like the JS Float32Array, so ties break the same way.
    std::ranges::fill(g_, std::numeric_limits<float>::infinity());
    std::ranges::fill(f_, std::numeric_limits<float>::infinity());
    std::ranges::fill(parent_, -1);
    std::ranges::fill(closed_, 0);
    std::ranges::fill(isGoal_, 0);
    heap_.clear();
    expanded = 0; msSpent = 0; slices = 0; reached_ = -1;
    if (starts.empty() || goals.empty()) { status = SearchStatus::Failed; return; }
    goalIdx_ = goals[0].i;
    goalSpread_ = 0;
    for (auto& gl : goals) { isGoal_[gl.i] = 1; goalSpread_ = std::max(goalSpread_, octile(gl.i, goalIdx_)); }
    for (auto& st : starts) {
      const double g0 = st.d / gr.cellSize;
      if (g0 < g_[st.i]) { g_[st.i] = static_cast<float>(g0); f_[st.i] = static_cast<float>(g0 + h(st.i)); push(st.i); }
    }
    status = SearchStatus::Running;
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
      if (closed_[cur]) continue;
      closed_[cur] = 1;
      expanded++; n++;
      if (isGoal_[cur]) { reached_ = cur; status = SearchStatus::Found; break; }
      const int c = cur % cols, r = cur / cols;
      for (int dr = -1; dr <= 1; dr++) for (int dc = -1; dc <= 1; dc++) {
        if (!dr && !dc) continue;
        const int nr = r + dr, nc = c + dc;
        if (nr < 0 || nc < 0 || nr >= rows || nc >= cols) continue;
        const int ni = nr * cols + nc;
        if (blocked[ni] || closed_[ni]) continue;
        if (dr && dc && (blocked[r * cols + nc] || blocked[nr * cols + c])) continue;
        const double ng = static_cast<double>(g_[cur]) + ((dr && dc) ? std::numbers::sqrt2 : 1.0);
        if (ng < g_[ni]) {
          g_[ni] = static_cast<float>(ng);
          f_[ni] = static_cast<float>(ng + h(ni) * 1.001);
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

  const std::vector<std::uint8_t>& closed() const { return closed_; }

 private:
  std::vector<float> g_, f_;
  std::vector<int> parent_;
  std::vector<std::uint8_t> closed_, isGoal_;
  std::vector<int> heap_;
  int goalIdx_ = 0, reached_ = -1;
  double goalSpread_ = 0;

  double octile(int a, int b) const {
    const int cols = grid->cols;
    const double dx = std::abs(a % cols - b % cols), dy = std::abs(a / cols - b / cols);
    return (dx + dy) + (std::numbers::sqrt2 - 2) * std::min(dx, dy);
  }
  double h(int i) const { return std::max(0.0, octile(i, goalIdx_) - goalSpread_); }

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
    const double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    const double dt = (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y);
    if (std::abs(cross) > 1e-9 || dt <= 0) out.push_back(b);
  }
  out.push_back(p.back());
  return out;
}

/** Forward string-pulling, one sweep test per point; see js/pathfinder.js smoothPath. */
inline std::vector<Vec2> smoothPath(const std::vector<Vec2>& input, const World& w, double r) {
  const auto pts = dropCollinear(input);
  if (pts.size() <= 2) return pts;
  std::vector<Vec2> out{pts[0]};
  std::size_t i = 0, j = 1;
  while (j < pts.size() - 1) {
    if (w.segmentFree(pts[i].x, pts[i].y, pts[j + 1].x, pts[j + 1].y, r)) { j++; continue; }
    out.push_back(pts[j]);
    i = j; j = i + 1;
  }
  out.push_back(pts.back());
  return out;
}

inline double pathLength(const std::vector<Vec2>& p) {
  double l = 0;
  for (std::size_t i = 1; i < p.size(); i++) l += dist(p[i - 1], p[i]);
  return l;
}

struct Escape { std::vector<Vec2> path; int cell; double length; };

/** Planning tier 3: pixel-level escape onto the fine grid; see js/pathfinder.js escapeRoute. */
inline std::optional<Escape> escapeRoute(const World& world, const NavGrid& grid, Vec2 from, double r,
                                         const std::vector<int>& goalSeedIds, int window = 160) {
  const int n = grid.cols * grid.rows;
  std::vector<std::uint8_t> goalComp(static_cast<std::size_t>(n), 0);
  std::vector<int> stack;
  for (int g : goalSeedIds) if (!goalComp[g]) { goalComp[g] = 1; stack.push_back(g); }
  while (!stack.empty()) {
    const int i = stack.back(); stack.pop_back();
    const int c = i % grid.cols, rr = i / grid.cols;
    for (int dr = -1; dr <= 1; dr++) for (int dc = -1; dc <= 1; dc++) {
      if (!dr && !dc) continue;
      const int nr = rr + dr, nc = c + dc;
      if (nr < 0 || nc < 0 || nr >= grid.rows || nc >= grid.cols) continue;
      const int j = nr * grid.cols + nc;
      if (grid.blocked[j] || goalComp[j]) continue;
      if (dr && dc && (grid.blocked[rr * grid.cols + nc] || grid.blocked[nr * grid.cols + c])) continue;
      goalComp[j] = 1; stack.push_back(j);
    }
  }
  const int size = 2 * window + 1;
  const int x0 = static_cast<int>(std::floor(from.x + 0.5)) - window, y0 = static_cast<int>(std::floor(from.y + 0.5)) - window;  // JS Math.round
  std::vector<std::uint8_t> state(static_cast<std::size_t>(size * size), 0);
  std::vector<int> prev(static_cast<std::size_t>(size * size), -1);
  auto isFree = [&](int k) {  // reduced clearance only near the start; see js/pathfinder.js
    if (!state[k]) {
      const int kx = k % size, ky = k / size;
      const bool nearStart = (kx - window) * (kx - window) + (ky - window) * (ky - window) <= 16;
      state[k] = world.discFree(x0 + kx, y0 + ky, nearStart ? r - 0.3 : r + world.fineMargin) ? 1 : 2;
    }
    return state[k] == 1;
  };
  const int startK = window * size + window;
  state[startK] = 1;
  std::vector<int> queue{startK};
  for (std::size_t qi = 0; qi < queue.size(); qi++) {
    const int k = queue[qi], px = x0 + k % size, py = y0 + k / size;
    const int cell = grid.cellIndexAt(px, py);
    if (k != startK && goalComp[cell] && !grid.blocked[cell]) {
      std::vector<Vec2> path;
      for (int m = k; m != -1; m = prev[m]) path.push_back({static_cast<double>(x0 + m % size), static_cast<double>(y0 + m / size)});
      std::ranges::reverse(path);
      path[0] = from;
      const double len = pathLength(path);
      return Escape{std::move(path), cell, len};
    }
    const int kx = k % size, ky = k / size;
    constexpr int d4[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (auto [dx, dy] : d4) {
      const int nx = kx + dx, ny = ky + dy;
      if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;
      const int j = ny * size + nx;
      if (prev[j] != -1 || j == startK || !isFree(j)) continue;
      prev[j] = k; queue.push_back(j);
    }
  }
  return std::nullopt;
}

inline bool NavGrid::connected(Vec2 from, Vec2 to, double agentR) const {
  std::vector<int> s, g;
  for (auto& c : seedCells(from.x, from.y, agentR)) s.push_back(c.i);
  for (auto& c : seedCells(to.x, to.y, agentR)) g.push_back(c.i);
  if (reachableMulti(s, g)) return true;
  return escapeRoute(*world, *this, from, agentR, g).has_value();  // same last resort as the planner
}

}  // namespace sim
