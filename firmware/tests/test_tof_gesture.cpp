// Host-side tests for firmware/tsp05/tof_gesture.cpp.
// Run: firmware/tests/run_tests.sh
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "config.h"
#include "tof_gesture.h"

using namespace tof_gesture;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; } else { g_fail++; \
  printf("FAIL %s:%d  ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

// Hand position over time -> what each zone sees. The sensor alternates zones every
// 40 ms (TOF_PERIOD_MS), so each zone is sampled every 80 ms, like the real firmware.
struct Scene {
  // hand present in a zone during [from, to) ms, at distance mm
  struct Seg { Zone zone; uint32_t from, to; uint16_t mm; };
  std::vector<Seg> segs;
  uint16_t background = 1200;

  uint16_t mmAt(Zone z, uint32_t t) const {
    for (const Seg &s : segs) if (s.zone == z && t >= s.from && t < s.to) return s.mm;
    return background;
  }
};

// Runs the detector over [0, endMs) and returns every swipe it reports.
static std::vector<Swipe> run(const Scene &sc, uint32_t endMs, uint32_t startOffset = 0) {
  Detector d; reset(d);
  std::vector<Swipe> out;
  Zone z = LEFT;
  for (uint32_t t = 40 + startOffset; t < endMs; t += 40) {
    Swipe s = update(d, z, sc.mmAt(z, t), true, t);
    if (s != NONE) out.push_back(s);
    z = (z == LEFT) ? RIGHT : LEFT;
  }
  return out;
}

static void test_swipes() {
  for (uint32_t off : {0u, 20u}) {
    Scene lr; lr.segs = {{LEFT, 200, 450, 150}, {RIGHT, 330, 600, 150}};
    auto a = run(lr, 1500, off);
    CHECK(a.size() == 1 && a[0] == LEFT_TO_RIGHT, "left->right swipe (offset %u): got %zu events", off, a.size());

    Scene rl; rl.segs = {{RIGHT, 200, 450, 180}, {LEFT, 330, 600, 180}};
    auto b = run(rl, 1500, off);
    CHECK(b.size() == 1 && b[0] == RIGHT_TO_LEFT, "right->left swipe (offset %u): got %zu events", off, b.size());
  }
}

static void test_fast_swipe_with_gap() {
  // Hand leaves the left zone 40 ms before the right zone sees it.
  Scene s; s.segs = {{LEFT, 200, 300, 150}, {RIGHT, 340, 460, 150}};
  auto a = run(s, 1500);
  CHECK(a.size() == 1 && a[0] == LEFT_TO_RIGHT, "fast swipe with a gap between zones: got %zu events", a.size());
}

static void test_not_swipes() {
  Scene push; push.segs = {{LEFT, 200, 700, 120}, {RIGHT, 200, 700, 120}};
  CHECK(run(push, 1500).empty(), "hand pushed straight in must not be a swipe");

  Scene held; held.segs = {{LEFT, 200, 2500, 150}, {RIGHT, 400, 2500, 150}};
  CHECK(run(held, 3500).empty(), "hand held still for > GESTURE_MAX_MS must not be a swipe");

  Scene far; far.segs = {{LEFT, 200, 450, GESTURE_NEAR_MM + 50}, {RIGHT, 330, 600, GESTURE_NEAR_MM + 50}};
  CHECK(run(far, 1500).empty(), "motion beyond GESTURE_NEAR_MM must be ignored");

  Scene oneZone; oneZone.segs = {{LEFT, 200, 500, 150}};
  CHECK(run(oneZone, 1500).empty(), "a hand seen by only one zone must not be a swipe");
}

static void test_two_swipes_and_invalid() {
  Scene two; two.segs = {{LEFT, 200, 450, 150}, {RIGHT, 330, 600, 150},
                         {RIGHT, 1000, 1250, 150}, {LEFT, 1130, 1400, 150}};
  auto a = run(two, 2200);
  CHECK(a.size() == 2 && a[0] == LEFT_TO_RIGHT && a[1] == RIGHT_TO_LEFT,
        "two swipes back to back: got %zu events", a.size());

  Detector d; reset(d);
  Swipe s = NONE;
  for (uint32_t t = 40; t < 800; t += 40) s = update(d, (t / 40) % 2 ? LEFT : RIGHT, 100, false, t);
  CHECK(s == NONE && !d.active, "invalid readings (bad range status) must count as no hand");
}

// Replays the bench recording (8x10 zones, swipes at 30-50 cm): 6 left-to-right,
// 5 right-to-left, then 5 more whose direction is clear from both entry and exit.
static void test_bench_recording() {
  std::ifstream f("../../calibration/data/tof_swipe_8x10.csv");
  CHECK(f.good(), "bench recording not found");
  std::string line; std::getline(f, line);
  Detector d; reset(d);
  std::vector<Swipe> got;
  while (std::getline(f, line)) {
    std::stringstream ss(line); std::string ms, zone, mm, st;
    std::getline(ss, ms, ','); std::getline(ss, zone, ','); std::getline(ss, mm, ','); std::getline(ss, st, ',');
    Swipe s = update(d, (Zone)std::stoi(zone), (uint16_t)std::stoi(mm), std::stoi(st) == 0, (uint32_t)std::stoul(ms));
    if (s != NONE) got.push_back(s);
  }
  const Swipe L = LEFT_TO_RIGHT, R = RIGHT_TO_LEFT;
  std::vector<Swipe> want = {L, L, L, L, L, L, R, R, R, R, R, L, L, L, R, L};
  CHECK(got == want, "bench recording: got %zu swipes, want 16 in the recorded order", got.size());
}

int main() {
  test_swipes();
  test_fast_swipe_with_gap();
  test_not_swipes();
  test_two_swipes_and_invalid();
  test_bench_recording();
  printf("tof_gesture: %d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
