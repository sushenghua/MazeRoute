// main.cpp — runs the steering simulation on the ESP32-S3 and streams it to the web page.
//
// Core 1: the simulation task. A fixed 60 Hz frame: one planner (or background grid build) slice with a
//         time budget, then one controller tick, then telemetry. This is the same frame structure as the
//         browser app, so the frame budget it reports is the chip's real cost.
// Core 0: the command reader. Lines arrive on UART0 (the board's "UART" USB port) and are queued.
//
// Protocol: one command per line in, one JSON object per line out (the bridge ignores anything else,
// such as ROM boot messages). Logs and panics go to the USB-Serial/JTAG port, not UART0.
#include <atomic>
#include <deque>
#include <cstdio>
#include <cstring>
#include <format>
#include <iterator>
#include <memory>
#include <string>

#include "driver/uart.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "harness.hpp"

using namespace sim;

namespace {

constexpr const char* TAG = "steer";
constexpr uart_port_t kPort = UART_NUM_0;
constexpr int kBaud = 921600;
constexpr real kRadius = 9;
constexpr std::int64_t kFrameUs = 16667;  // 60 Hz, the same fixed timestep as the browser app

struct Command { char text[120]; };
QueueHandle_t commands;
std::atomic<bool> abortRequested{false};

// ---------------- output ----------------

std::string out;
void send() {
  out.push_back('\n');
  uart_write_bytes(kPort, out.data(), out.size());
  out.clear();
}
template <class... A>
void put(std::format_string<A...> f, A&&... a) { std::format_to(std::back_inserter(out), f, std::forward<A>(a)...); }

void putPoints(const std::vector<Vec2>& pts) {
  out.push_back('[');
  for (std::size_t i = 0; i < pts.size(); i++) put("{}{:.1f},{:.1f}", i ? "," : "", pts[i].x, pts[i].y);
  out.push_back(']');
}

void putHeap() {
  put("\"heap\":{{\"intFree\":{},\"intMin\":{},\"psramFree\":{},\"psramMin\":{},\"psramTotal\":{}}}",
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
      heap_caps_get_free_size(MALLOC_CAP_SPIRAM), heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
      heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
}

void sendError(std::string_view what) { put("{{\"k\":\"error\",\"msg\":\"{}\"}}", what); send(); }

double msSince(std::int64_t t0) { return (esp_timer_get_time() - t0) / 1000.0; }

// ---------------- simulation state ----------------

struct App {
  std::unique_ptr<World> world;
  Agent agent{kRadius};
  AStar astar;
  std::unique_ptr<Controller> ctl;
  bool running = false, allowSealing = false;
  double planBudget = 6, buildBudget = 6, dropBudget = 5;  // ms per frame, of the 16.7 ms frame
  DropTask drop;  // obstacle validation in progress, one slice per frame
  struct PendingDrop { real x, y; bool sealing; };
  std::deque<PendingDrop> dropQueue;  // clicks that arrived while a drop was being validated
  long dropStartFrame = 0, frameNo = 0;
  std::optional<Transition> lastSent;
  // Frame statistics since Start.
  long frames = 0, overBudget = 0;
  double worstFrame = 0, sumFrame = 0, worstPlan = 0, worstTick = 0;
} app;

void sendWorld(double genMs) {
  const World& w = *app.world;
  put("{{\"k\":\"world\",\"seed\":{},\"w\":{},\"h\":{},\"r\":{},\"genMs\":{:.2f},\"start\":[{:.1f},{:.1f}],\"goal\":[{:.1f},{:.1f}],"
      "\"coarse\":[{},{}],\"fine\":[{},{}],\"polys\":[",
      w.seed, w.width, w.height, kRadius, genMs, w.start.x, w.start.y, w.goal.x, w.goal.y, w.nav->cols, w.nav->rows, w.fine->cols, w.fine->rows);
  for (std::size_t i = 0; i < w.polys.size(); i++) { if (i) out.push_back(','); putPoints(w.polys[i].pts); }
  put("],");
  putHeap();
  out.push_back('}');
  send();
}

void sendPath() {
  const Controller& c = *app.ctl;
  put("{{\"k\":\"path\",\"tier\":{},\"nodes\":{},\"slices\":{},\"planMs\":{:.2f},\"path\":", c.planLevel, c.stats.planNodes, c.stats.planSlices, c.stats.planMs);
  putPoints(c.path);
  put(",\"raw\":");
  putPoints(c.rawPath);
  out.push_back('}');
  send();
}

std::string jsonEscape(std::string_view s) {
  std::string r;
  for (char ch : s) { if (ch == '"' || ch == '\\') r.push_back('\\'); r.push_back(ch); }
  return r;
}

/** Emit transitions added since the last call (the controller keeps the newest 14, newest first). */
void sendNewTransitions() {
  const auto& log = app.ctl->log;
  std::size_t n = 0;
  auto same = [](const Transition& a, const Transition& b) { return a.t == b.t && a.from == b.from && a.to == b.to && a.reason == b.reason; };
  while (n < log.size() && !(app.lastSent && same(log[n], *app.lastSent))) n++;
  if (n == 0) return;
  bool pathChanged = false;
  for (std::size_t i = n; i-- > 0;) {
    const auto& t = log[i];
    put("{{\"k\":\"tr\",\"t\":{:.2f},\"from\":\"{}\",\"to\":\"{}\",\"reason\":\"{}\"}}", t.t, name(t.from), name(t.to), jsonEscape(t.reason));
    send();
    if (t.to == State::Following || t.to == State::Failed) pathChanged = true;
  }
  app.lastSent = log.front();
  if (pathChanged) sendPath();
}

void sendState(double dropMs, double planMs, double tickMs, double frameMs) {
  const Controller& c = *app.ctl;
  const Agent& a = app.agent;
  const Vec2 tg = c.target ? *c.target : a.pos;
  put("{{\"k\":\"s\",\"t\":{:.3f},\"x\":{:.2f},\"y\":{:.2f},\"h\":{:.3f},\"v\":{:.1f},\"st\":\"{}\",\"tx\":{:.1f},\"ty\":{:.1f},\"tv\":{},"
      "\"drop\":{:.3f},\"plan\":{:.3f},\"tick\":{:.3f},\"frame\":{:.3f},\"fineRows\":{},\"tier\":{},\"nodes\":{},\"replans\":{},\"recov\":{},\"coll\":{},\"remaining\":{:.1f}}}",
      c.simTime, a.pos.x, a.pos.y, a.heading, a.speed, name(c.state), tg.x, tg.y, c.targetVisible && c.state == State::Following ? 1 : 0,
      dropMs, planMs, tickMs, frameMs, app.world->fine->builtRows, c.planLevel, app.astar.expanded, c.stats.replans, c.stats.recoveries,
      c.stats.collisions, c.remaining);
  send();
}

void sendSummary() {
  const Controller& c = *app.ctl;
  put("{{\"k\":\"done\",\"state\":\"{}\",\"t\":{:.2f},\"frames\":{},\"overBudget\":{},\"worstFrame\":{:.3f},\"avgFrame\":{:.3f},\"worstPlan\":{:.3f},"
      "\"worstTick\":{:.3f},\"replans\":{},\"recov\":{},\"coll\":{},",
      name(c.state), c.simTime, app.frames, app.overBudget, app.worstFrame, app.frames ? app.sumFrame / app.frames : 0.0, app.worstPlan,
      app.worstTick, c.stats.replans, c.stats.recoveries, c.stats.collisions);
  putHeap();
  out.push_back('}');
  send();
}

// ---------------- commands ----------------

void cmdGenerate(unsigned seed, int density) {
  app.running = false;
  app.drop.cancel();  // it points into the world being replaced
  app.dropQueue.clear();
  app.ctl.reset();
  app.world = std::make_unique<World>(960, 640, 10);
  const auto t0 = esp_timer_get_time();
  if (!app.world->generate(seed, density, kRadius)) { app.world.reset(); sendError("generation failed"); return; }
  const double genMs = msSince(t0);
  app.ctl = std::make_unique<Controller>(*app.world, app.agent, app.astar);
  app.ctl->reserve();  // allocate search buffers now, while idle, not in the first frame that needs them
  app.drop.reserve(static_cast<std::size_t>(app.world->fine->cols * app.world->fine->rows));
  app.lastSent.reset();
  sendWorld(genMs);
}

/** Start validating a drop. The cheap checks answer at once; the rest runs a slice per frame (see DropTask). */
void cmdDrop(real x, real y, bool sealing) {
  if (!app.world) return;
  if (app.drop.busy()) { app.dropQueue.push_back({x, y, sealing}); return; }
  const auto t0 = esp_timer_get_time();
  if (auto reason = app.drop.start(*app.world, x, y, kRadius, app.agent.pos, sealing)) {
    put("{{\"k\":\"drop\",\"ok\":false,\"reason\":\"{}\",\"ms\":{:.2f},\"slices\":0,\"frames\":0}}", *reason, msSince(t0));
    send();
    return;
  }
  app.dropStartFrame = app.frameNo;
}

void sendDropOutcome(const DropOutcome& r) {
  const long frames = app.frameNo - app.dropStartFrame;
  if (!r.accepted) {
    put("{{\"k\":\"drop\",\"ok\":false,\"reason\":\"{}\",\"ms\":{:.2f},\"slices\":{},\"frames\":{}}}", r.reason, r.workMs, r.slices, frames);
    send();
    return;
  }
  put("{{\"k\":\"drop\",\"ok\":true,\"sealed\":{},\"ms\":{:.2f},\"slices\":{},\"frames\":{},\"poly\":", r.sealed, r.workMs, r.slices, frames);
  putPoints(app.world->polys[r.index].pts);
  out.push_back('}');
  send();
}

void cmdInfo() {
  esp_chip_info_t ci;
  esp_chip_info(&ci);
  std::uint32_t flash = 0;
  esp_flash_get_size(nullptr, &flash);
  put("{{\"k\":\"info\",\"chip\":\"ESP32-S3\",\"rev\":{},\"cores\":{},\"mhz\":{},\"flash\":{},\"real\":\"{}\",\"idf\":\"{}\",\"compiler\":\"{}\","
      "\"frameHz\":60,\"budgetMs\":{{\"plan\":{},\"build\":{},\"drop\":{}}},",
      ci.revision, ci.cores, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, flash, sizeof(real) == 4 ? "float" : "double", IDF_VER, jsonEscape(__VERSION__),
      app.planBudget, app.buildBudget, app.dropBudget);
  putHeap();
  out.push_back('}');
  send();
}

/**
 * Free the live demo (world, controller and the search buffers reserved for it, ~5 MB of PSRAM) before a
 * bench, profile or suite, which build their own worlds: with both resident, 8 MB is not enough.
 */
void releaseDemo() {
  app.running = false;
  app.drop = DropTask{};
  app.dropQueue.clear();
  app.ctl.reset();
  app.world.reset();
  app.astar = AStar{};
  app.lastSent.reset();
  put("{{\"k\":\"cleared\"}}");
  send();
}

/** Operation benchmark, the same operations as cpp/src/bench.cpp, on `n` worlds. */
void cmdBench(int n) {
  std::vector<double> gen, build, add, coarse, fine, seeds, smooth;
  for (int seed = 1; seed <= n && !abortRequested; seed++) {
    World w(960, 640, 10);
    auto t = esp_timer_get_time(); w.generate(seed, 36, kRadius); gen.push_back(msSince(t));
    t = esp_timer_get_time(); w.fine->buildStep(kInf); build.push_back(msSince(t));
    for (int k = 0; k < 5; k++) {  // total work of one validation, all slices together
      DropTask task;
      t = esp_timer_get_time();
      if (!task.start(w, 100 + k * 170, 320, kRadius, w.start, false)) task.step(kInf, w.start);
      add.push_back(msSince(t));
    }
    AStar a;
    t = esp_timer_get_time();
    a.start(*w.nav, w.nav->seedCells(w.start.x, w.start.y, kRadius), w.nav->seedCells(w.goal.x, w.goal.y, kRadius));
    while (a.step(1e9) == SearchStatus::Running) {}
    coarse.push_back(msSince(t));
    t = esp_timer_get_time();
    a.start(*w.fine, w.fine->seedCells(w.start.x, w.start.y, kRadius), w.fine->seedCells(w.goal.x, w.goal.y, kRadius));
    while (a.step(1e9) == SearchStatus::Running) {}
    fine.push_back(msSince(t));
    // The two unsliced steps around a fine search: seeding (start + goal) and smoothing its path.
    t = esp_timer_get_time();
    const auto s0 = w.fine->seedCells(w.start.x, w.start.y, kRadius), g0 = w.fine->seedCells(w.goal.x, w.goal.y, kRadius);
    seeds.push_back(msSince(t));
    auto raw = a.path();
    raw.insert(raw.begin(), w.start);
    raw.push_back(w.goal);
    t = esp_timer_get_time(); (void)smoothPath(raw, w, kRadius + 3); smooth.push_back(msSince(t));
    put("{{\"k\":\"benchProgress\",\"done\":{},\"of\":{}}}", seed, n);
    send();
  }
  auto row = [](std::string_view label, std::vector<double> v) {
    if (v.empty()) return;
    std::ranges::sort(v);
    put("{{\"label\":\"{}\",\"median\":{:.3f},\"p90\":{:.3f},\"max\":{:.3f},\"n\":{}}}", label, v[v.size() / 2],
        v[std::min(v.size() - 1, v.size() * 9 / 10)], v.back(), v.size());
  };
  put("{{\"k\":\"bench\",\"rows\":[");
  row("generate + coarse grid", gen); out.push_back(',');
  row("fine grid full build", build); out.push_back(',');
  row("drop validation (all slices)", add); out.push_back(',');
  row("coarse A* (unsliced)", coarse); out.push_back(',');
  row("fine A* (unsliced)", fine); out.push_back(',');
  row("fine seedCells (start + goal)", seeds); out.push_back(',');
  row("smooth fine path", smooth);
  put("],");
  putHeap();
  out.push_back('}');
  send();
}

/** Where drop validation spends its time: each phase of DropTask, run unsliced, on n worlds x 5 drops. */
void cmdProfile(int n) {
  std::vector<double> patch, goalSeeds, startSeeds, connect, astarStart, astarRestart, escStart, commit;
  for (int seed = 1; seed <= n && !abortRequested; seed++) {
    World w(960, 640, 10);
    w.generate(seed, 36, kRadius);
    w.fine->buildStep(kInf);
    NavGrid& g = *w.fine;
    for (int k = 0; k < 5; k++) {
      Polygon p = w.dropPolygon(100 + k * 170, 320);
      auto t = esp_timer_get_time();
      std::vector<int> nb;
      const real inf = g.inflation, cs = g.cellSize;
      const int c0 = clampi(static_cast<int>(std::floor((p.minX - inf) / cs)), 0, g.cols - 1), c1 = clampi(static_cast<int>(std::ceil((p.maxX + inf) / cs)), 0, g.cols - 1);
      const int r0 = clampi(static_cast<int>(std::floor((p.minY - inf) / cs)), 0, g.rows - 1), r1 = clampi(static_cast<int>(std::ceil((p.maxY + inf) / cs)), 0, g.rows - 1);
      for (int r = r0; r <= r1; r++) for (int c = c0; c <= c1; c++) {
        const int i = r * g.cols + c;
        if (!g.blocked[i] && p.circlePenetration((c + 0.5f) * cs, (r + 0.5f) * cs, inf)) nb.push_back(i);
      }
      patch.push_back(msSince(t));
      w.overlay = &p;
      for (int i : nb) g.blocked[i] = 1;
      t = esp_timer_get_time();
      std::vector<int> gi, si;
      for (auto& sd : g.seedCells(w.goal.x, w.goal.y, kRadius)) gi.push_back(sd.i);
      goalSeeds.push_back(msSince(t));
      t = esp_timer_get_time();
      for (auto& sd : g.seedCells(w.start.x, w.start.y, kRadius)) si.push_back(sd.i);
      startSeeds.push_back(msSince(t));
      t = esp_timer_get_time();
      ConnectSearch cs2;
      cs2.start(g, si, w.goal, gi, w.start);
      cs2.step(kInf);
      connect.push_back(msSince(t));
      t = esp_timer_get_time();
      AStar as;
      as.start(g, {Seed{si.empty() ? 0 : si[0], 0}}, {Seed{gi.empty() ? 0 : gi[0], 0}});
      astarStart.push_back(msSince(t));
      t = esp_timer_get_time();
      as.start(g, {Seed{si.empty() ? 0 : si[0], 0}}, {Seed{gi.empty() ? 0 : gi[0], 0}});
      astarRestart.push_back(msSince(t));
      t = esp_timer_get_time();
      EscapeSearch es;
      es.start(w, g, w.start, kRadius, gi);
      escStart.push_back(msSince(t));
      for (int i : nb) g.blocked[i] = 0;
      w.overlay = nullptr;
      t = esp_timer_get_time();
      w.commitObstacle(std::move(p), nb);
      commit.push_back(msSince(t));
    }
  }
  auto row = [](std::string_view label, std::vector<double> v) {
    std::ranges::sort(v);
    put("{{\"label\":\"{}\",\"median\":{:.3f},\"p90\":{:.3f},\"max\":{:.3f},\"n\":{}}}", label, v[v.size() / 2], v[v.size() * 9 / 10], v.back(), v.size());
  };
  put("{{\"k\":\"profile\",\"rows\":[");
  row("patch (rasterise the new obstacle)", patch); out.push_back(',');
  row("seed B", goalSeeds); out.push_back(',');
  row("seed agent", startSeeds); out.push_back(',');
  row("connect (meet in the middle)", connect); out.push_back(',');
  row("AStar::start, fine grid (first)", astarStart); out.push_back(',');
  row("AStar::start, fine grid (again)", astarRestart); out.push_back(',');
  row("EscapeSearch::start", escStart); out.push_back(',');
  row("commitObstacle", commit);
  put("]}}");
  send();
}

/** Run the zero-failure suite on the chip: seeds [s0, s0+n) of every scenario, as fast as possible. */
void cmdSuite(int n, int s0) {
  const auto t0 = esp_timer_get_time();
  int totalRuns = 0, totalFail = 0;
  for (const auto& sc : scenarios()) {
    int runs = 0, arrived = 0, fails = 0, honest = 0;
    for (int s = s0; s < s0 + n && !abortRequested; s++) {
      const auto r = runMission(s, sc.density, sc.drops(s), sc.allowSealing, sc.allowSealing);
      vTaskDelay(1);  // let lower-priority work on this core run between missions
      if (!r.gen) continue;
      put("{{\"k\":\"suiteMission\",\"scenario\":\"{}\",\"seed\":{},\"state\":\"{}\",\"t\":{:.2f},\"placed\":{},\"rejected\":{},"
          "\"replans\":{},\"recov\":{},\"coll\":{},\"sealed\":{}}}",
          sc.name, s, name(r.state), r.t, r.placed, r.rejected, r.replans, r.recov, r.coll, r.sealed);
      send();
      runs++;
      if (r.state == State::Arrived) arrived++;
      else if (sc.allowSealing && r.state == State::Failed && r.solvableAtGuarantee == false) honest++;
      else {
        fails++;
        put("{{\"k\":\"suiteFail\",\"scenario\":\"{}\",\"seed\":{},\"state\":\"{}\",\"reason\":\"{}\"}}", sc.name, s, name(r.state), jsonEscape(r.reason));
        send();
      }
    }
    totalRuns += runs; totalFail += fails;
    put("{{\"k\":\"suiteRow\",\"scenario\":\"{}\",\"runs\":{},\"arrived\":{},\"honestFailed\":{},\"fail\":{},\"elapsed\":{:.1f}}}", sc.name, runs, arrived,
        honest, fails, msSince(t0) / 1000);
    send();
  }
  put("{{\"k\":\"suite\",\"runs\":{},\"fail\":{},\"seconds\":{:.1f},\"aborted\":{}}}", totalRuns, totalFail, msSince(t0) / 1000, abortRequested.load());
  send();
}

void handle(const char* line) {
  unsigned seed; int a, b; float x, y;
  if (std::sscanf(line, "gen %u %d", &seed, &a) == 2) cmdGenerate(seed, a);
  else if (!std::strcmp(line, "start")) {
    if (!app.ctl) return sendError("generate first");
    app.ctl->startMission();
    app.running = true;
    app.frames = app.overBudget = 0; app.worstFrame = app.sumFrame = app.worstPlan = app.worstTick = 0;
  }
  else if (!std::strcmp(line, "reset")) { if (app.ctl) { app.ctl->reset(); app.running = false; app.lastSent.reset(); sendState(0, 0, 0, 0); } }
  else if (std::sscanf(line, "drop %f %f %d", &x, &y, &a) == 3) cmdDrop(x, y, a != 0);
  else if (float z; std::sscanf(line, "budget %f %f %f", &x, &y, &z) == 3) { app.planBudget = x; app.buildBudget = y; app.dropBudget = z; }
  else if (std::sscanf(line, "bench %d", &a) == 1) { releaseDemo(); cmdBench(a); }
  else if (std::sscanf(line, "suite %d %d %f", &a, &b, &x) == 3) {  // with the real-time slice budget, as live
    releaseDemo();
    setenv("SLICE_MS", std::format("{}", x).c_str(), 1);  // read by runMission (harness.hpp)
    cmdSuite(a, b);
    unsetenv("SLICE_MS");
  }
  else if (std::sscanf(line, "suite %d %d", &a, &b) == 2) { releaseDemo(); cmdSuite(a, b); }
  else if (!std::strcmp(line, "info")) cmdInfo();
  else if (std::sscanf(line, "profile %d", &a) == 1) { releaseDemo(); cmdProfile(a); }
  else if (!std::strcmp(line, "stop")) {}  // handled by the reader; nothing left to do here
  else sendError("unknown command");
}

// ---------------- tasks ----------------

void readerTask(void*) {
  Command cmd{};
  std::size_t len = 0;
  std::uint8_t buf[64];
  for (;;) {
    const int n = uart_read_bytes(kPort, buf, sizeof buf, pdMS_TO_TICKS(20));
    for (int i = 0; i < n; i++) {
      const char ch = static_cast<char>(buf[i]);
      if (ch == '\r') continue;
      if (ch != '\n') { if (len + 1 < sizeof cmd.text) cmd.text[len++] = ch; continue; }
      cmd.text[len] = 0;
      len = 0;
      if (!std::strcmp(cmd.text, "stop")) abortRequested = true;  // must reach a running bench/suite directly
      else xQueueSend(commands, &cmd, portMAX_DELAY);
    }
  }
}

void simTask(void*) {
  std::int64_t next = esp_timer_get_time();
  Command cmd;
  for (;;) {
    while (xQueueReceive(commands, &cmd, 0) == pdTRUE) { abortRequested = false; handle(cmd.text); next = esp_timer_get_time(); }

    if (app.world) {
      Controller& c = *app.ctl;
      app.frameNo++;
      const auto fd = esp_timer_get_time();
      std::optional<DropOutcome> dropDone;
      // A drop gets the planner's slice too when the planner and the grid build have nothing to do, which is
      // the usual case while following: validation then finishes in a few frames, before the agent arrives.
      const bool plannerIdle = !(app.running && c.isPlanning()) && app.world->fine->ready;
      if (app.drop.busy()) dropDone = app.drop.step(app.dropBudget + (plannerIdle ? app.planBudget : 0), app.agent.pos);
      if (dropDone && dropDone->accepted) c.onWorldChanged();
      const auto f0 = esp_timer_get_time();
      // One work budget per frame: what the drop slice used (it may have borrowed the planner's share) is not
      // available to the planner or the background grid build. A commit that triggers a replan therefore
      // plans next frame. While a drop is pending, its own first phase builds the grid, so the background
      // build stands down (both together overran: a grid row is ~2.8 ms on this chip).
      const double workLeft = app.dropBudget + app.planBudget - (f0 - fd) / 1000.0;
      if (app.running && c.isPlanning()) { if (const double b = std::min(app.planBudget, workLeft); b > 0.5) c.plan(b); }
      else if (!app.world->fine->ready && !app.drop.busy()) {  // background build, as in the browser
        if (const double b = std::min(app.buildBudget, workLeft); b > 0.5) app.world->fine->buildStep(b);
      }
      const auto f1 = esp_timer_get_time();
      if (app.running) c.tick(1.0f / 60);
      const auto f2 = esp_timer_get_time();
      const double dropMs = (f0 - fd) / 1000.0, planMs = (f1 - f0) / 1000.0, tickMs = (f2 - f1) / 1000.0, frameMs = (f2 - fd) / 1000.0;
      if (app.running) {
        app.frames++; app.sumFrame += frameMs;
        app.worstFrame = std::max(app.worstFrame, frameMs);
        app.worstPlan = std::max(app.worstPlan, planMs);
        app.worstTick = std::max(app.worstTick, tickMs);
        if (frameMs > kFrameUs / 1000.0) app.overBudget++;
      }
      // Telemetry is sent after the frame is measured, so it does not count against the budget.
      if (dropDone) {
        sendDropOutcome(*dropDone);
        while (!app.drop.busy() && !app.dropQueue.empty()) {
          const auto d = app.dropQueue.front();
          app.dropQueue.pop_front();
          cmdDrop(d.x, d.y, d.sealing);
        }
      }
      sendNewTransitions();
      if (app.running || frameMs > 0.05) sendState(dropMs, planMs, tickMs, frameMs);
      if (app.running && c.isTerminal()) { app.running = false; sendSummary(); }
    }

    next += kFrameUs;
    const std::int64_t wait = next - esp_timer_get_time();
    if (wait > 0) vTaskDelay(pdMS_TO_TICKS(std::max<std::int64_t>(1, wait / 1000)));
    else next = esp_timer_get_time();  // overran: do not try to catch up
  }
}

}  // namespace

extern "C" void app_main() {
  const uart_config_t cfg = {.baud_rate = kBaud, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
                             .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
  ESP_ERROR_CHECK(uart_driver_install(kPort, 4096, 32768, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(kPort, &cfg));
  commands = xQueueCreate(16, sizeof(Command));
  out.reserve(16384);
  ESP_LOGI(TAG, "agent steering sim ready, protocol on UART0 @ %d baud", kBaud);
  put("{{\"k\":\"hello\"}}");
  send();
  xTaskCreatePinnedToCore(readerTask, "reader", 4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(simTask, "sim", 32768, nullptr, 4, nullptr, 1);
}
