// EPD_Painter2 — SIXTEEN GREYS. The moment of truth.
//
// Calibration measured from the pulse_width_staircase photo (July 8 2026):
// at a 6ms pulse window the ascent curve is ~14 monotonic steps, and the
// mid-range slope is ~0.05 reflectance per pulse — almost exactly one step
// of an even 16-level greyscale per pulse. This sketch maps grey 0-15 onto
// that measured curve and draws the staircase THROUGH the normal grey API.
//
// If calibration is right, the photo shows 16 visually even bands, white
// to black. Uneven neighbours = nudge that LUT entry by ±1 pulse.
//
// (All bands are drawn from hard white, so this validates the ASCENT curve
// only — descent/lightening at short pulses is the next measurement.)

// Choose your board (or leave both commented for auto-detect).
//#define EPD_PAINTER2_PRESET_M5PAPER_S3
//#define EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include "EPD_Painter2.h"

EPD_Painter2 epd(EPD_PAINTER2_PRESET);

// 7ms: measured at 6ms first, then lengthened a touch — the deep end wasn't
// reaching true black. Longer window = more dose per pulse across the board.
static const uint32_t PULSE_WINDOW_US = 7000;

// Grey 0-15 → pulse position at 6ms window, from the measured reflectances:
//   n:  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14
//   R: 1.00 0.99 0.95 0.89 0.82 0.76 0.68 0.60 0.56 0.50 0.46 0.42 0.37 0.31 0.27
// Targets are R = 1.0 - g*(0.79/15). The first pulse is nearly invisible
// (ink inertia), hence grey 1 starts at position 2. Black = saturation
// (~17-18) plus 2 overdrive pulses to re-anchor the rail.
static const uint8_t kGreys6ms[16] =
  { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 26 };

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) delay(10);

  if (!epd.begin()) {
    Serial.println("EPD_Painter2 init failed!");
    while (1) delay(1000);
  }

  epd.setPulseWindow(PULSE_WINDOW_US);
  epd.setGreyPositions(kGreys6ms);

  const int W = epd.width(), H = epd.height();
  const int bandW = W / 16;

  epd.beginUpdate();
  for (int g = 0; g < 16; g++) {
    epd.fillRect(g * bandW, 0, bandW, H - 40, (uint8_t)g);
    for (int n = 0; n < g; n++) {
      epd.fillRect(g * bandW + 6 + n * 3, H - 30, 2, 20, 15);
    }
  }
  epd.endUpdate();
  epd.waitSettled(15000);

  Serial.println("Settled — photograph. 16 even bands = calibrated.");
}

void loop() { delay(1000); }
