// EPD_Painter2 — hysteresis experiment: are there greys between the pulses?
//
// The panel gives ~7 distinct levels from direct pulse counts (measured).
// Greyscale eink drivers classically mint EXTRA levels from hysteresis:
// arriving at position n via overshoot-and-return lands at a different
// optical grey than arriving directly, because the ink's response is
// asymmetric between darkening and lightening.
//
// This sketch draws 16 vertical bands, split horizontally:
//   TOP half:    band n driven DIRECTLY to position n        (reference)
//   BOTTOM half: band n driven to n+2, settled, then back to n (overshoot)
//
// Both halves end at the same ledger position n. Study the horizontal
// boundary in each band:
//   - bottom visibly darker than top  → hysteresis exists → intermediate
//     greys are mintable → trajectory table gains a second dimension
//   - halves identical                → no free levels from path; dwell
//     (dose) control becomes the main lever
//
// Uses an identity grey→position table so band index == raw pulse position.

// Choose your board (or leave both commented for auto-detect).
//#define EPD_PAINTER2_PRESET_M5PAPER_S3
//#define EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include "EPD_Painter2.h"

EPD_Painter2 epd(EPD_PAINTER2_PRESET);

// Optional: raise per-pulse dose (µs of extra row dwell) to test dose
// scaling in the same flash. 0 = baseline.
static const uint8_t PULSE_DWELL_US = 0;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) delay(10);

  if (!epd.begin()) {
    Serial.println("EPD_Painter2 init failed!");
    while (1) delay(1000);
  }

  const uint8_t identity[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  epd.setGreyPositions(identity);
  epd.setPulseDwell(PULSE_DWELL_US);

  const int W = epd.width(), H = epd.height();
  const int bandW = W / 16;
  const int half  = H / 2;

  // Phase A: top = direct to n; bottom = overshoot start (n+2, capped).
  Serial.println("Phase A: driving...");
  epd.beginUpdate();
  for (int g = 0; g < 16; g++) {
    int over = g + 2; if (over > 15) over = 15;
    epd.fillRect(g * bandW, 0,    bandW, half,     g);     // direct
    epd.fillRect(g * bandW, half, bandW, H - half, over);  // overshoot leg
  }
  epd.endUpdate();
  epd.waitSettled(10000);
  delay(500);   // let the ink fully relax before the return leg

  // Phase B: bottom returns to n (path: 0 → n+2 → n).
  Serial.println("Phase B: returning...");
  epd.beginUpdate();
  for (int g = 0; g < 16; g++) {
    epd.fillRect(g * bandW, half, bandW, H - half, g);
  }
  epd.endUpdate();
  epd.waitSettled(10000);

  Serial.println("Settled. Photograph the panel.");
  Serial.println("Compare top (direct) vs bottom (overshoot-return) of each band.");
}

void loop() { delay(1000); }
