#include "config.h"
#include "tof_gesture.h"

namespace tof_gesture {

void reset(Detector &d) {
  d.on[LEFT] = d.on[RIGHT] = false;
  d.firstMs[LEFT] = d.firstMs[RIGHT] = 0;
  d.lastMs[LEFT] = d.lastMs[RIGHT] = 0;
  d.startMs = 0;
  d.lastNearMs = 0;
  d.active = false;
  d.tooLong = false;
}

// A pass ends when both zones have been clear for GESTURE_CLEAR_MS. It is a swipe if
// both zones saw the hand, the zone that saw it first also lost it first, and the two
// leads add up to GESTURE_MIN_LEAD_MS. Zones are sampled one 40 ms step apart, so a
// single lead of one step can be sampling order alone; requiring entry and exit to
// agree rejects most straight pushes, whose two orders are unrelated.
static Swipe finish(Detector &d) {
  Swipe s = NONE;
  if (!d.tooLong && d.firstMs[LEFT] && d.firstMs[RIGHT]) {
    long entry = (long)d.firstMs[RIGHT] - (long)d.firstMs[LEFT];
    long exit  = (long)d.lastMs[RIGHT] - (long)d.lastMs[LEFT];
    if (entry > 0 && exit > 0 && entry + exit >= GESTURE_MIN_LEAD_MS) s = LEFT_TO_RIGHT;
    if (entry < 0 && exit < 0 && -(entry + exit) >= GESTURE_MIN_LEAD_MS) s = RIGHT_TO_LEFT;
  }
  reset(d);
  if (GESTURE_FLIP) s = (Swipe)(-s);
  return s;
}

Swipe update(Detector &d, Zone zone, uint16_t mm, bool valid, uint32_t ms) {
  bool near = valid && mm > 0 && mm < GESTURE_NEAR_MM;
  d.on[zone] = near;

  if (near) {
    if (!d.active) { d.active = true; d.startMs = ms; }
    if (!d.firstMs[zone]) d.firstMs[zone] = ms;
    d.lastNearMs = ms;
    d.lastMs[zone] = ms;
  }
  if (d.active && ms - d.startMs > GESTURE_MAX_MS) d.tooLong = true;

  if (d.active && !d.on[LEFT] && !d.on[RIGHT] && ms - d.lastNearMs >= GESTURE_CLEAR_MS) return finish(d);
  return NONE;
}

}
