#pragma once
#include <stdint.h>

// Hardware-free swipe detector for a two-zone TOF reading (left and right halves of
// the VL53L1X's view, measured alternately). Feed every zone reading; a swipe is
// reported once the hand has left both zones.
namespace tof_gesture {

enum Zone : uint8_t { LEFT = 0, RIGHT = 1 };
enum Swipe : int8_t { NONE = 0, LEFT_TO_RIGHT = 1, RIGHT_TO_LEFT = -1 };

struct Detector {
  bool     on[2];       // hand currently seen in each zone
  uint32_t firstMs[2];  // when each zone first saw the hand in this pass (0 = not yet)
  uint32_t lastMs[2];   // when each zone last saw the hand in this pass
  uint32_t startMs;     // when the pass began
  uint32_t lastNearMs;  // last reading that saw the hand
  bool     active;      // a pass is in progress
  bool     tooLong;     // pass exceeded GESTURE_MAX_MS: report nothing for it
};

void  reset(Detector &d);

// One zone reading. valid = the sensor's range status was OK. ms must be >= 1.
Swipe update(Detector &d, Zone zone, uint16_t mm, bool valid, uint32_t ms);

}
