#include "config.h"
#include "tof_gesture.h"

namespace tof_gesture {

void reset(Detector &d) {
  d.on[LEFT] = d.on[RIGHT] = false;
  d.firstMs[LEFT] = d.firstMs[RIGHT] = 0;
  d.startMs = 0;
  d.lastNearMs = 0;
  d.active = false;
  d.tooLong = false;
}

// A pass ends when both zones have been clear for GESTURE_CLEAR_MS. It is a swipe
// only if both zones saw the hand, far enough apart in time that the order is motion
// and not just which zone happened to be sampled first (a hand pushed straight in
// reaches both together).
static Swipe finish(Detector &d) {
  Swipe s = NONE;
  if (!d.tooLong && d.firstMs[LEFT] && d.firstMs[RIGHT]) {
    uint32_t l = d.firstMs[LEFT], r = d.firstMs[RIGHT];
    if (r >= l + GESTURE_MIN_GAP_MS) s = LEFT_TO_RIGHT;
    else if (l >= r + GESTURE_MIN_GAP_MS) s = RIGHT_TO_LEFT;
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
  }
  if (d.active && ms - d.startMs > GESTURE_MAX_MS) d.tooLong = true;

  if (d.active && !d.on[LEFT] && !d.on[RIGHT] && ms - d.lastNearMs >= GESTURE_CLEAR_MS) return finish(d);
  return NONE;
}

}
