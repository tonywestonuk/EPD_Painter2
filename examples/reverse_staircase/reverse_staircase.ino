// EPD_Painter2 calibration — the DESCENT curve: greys minted from black.
//
// The hysteresis test showed the lighten-from-black response is far better
// resolved than darken-from-white. This measures it: the whole panel is
// driven to overdriven black, settled, then band n receives exactly n light
// pulses. A photo of the result is the descent curve — if it shows 12-16
// distinguishable stops, the 16-grey trajectory is "saturate black, then
// count calibrated light pulses back up".
//
// Band index notches at the foot of each band (n notches for band n),
// drawn LAST so they stay black.

// Choose your board (or leave both commented for auto-detect).
//#define EPD_PAINTER2_PRESET_M5PAPER_S3
//#define EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include "EPD_Painter2.h"

EPD_Painter2 epd(EPD_PAINTER2_PRESET);

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) delay(10);

  if (!epd.begin()) {
    Serial.println("EPD_Painter2 init failed!");
    while (1) delay(1000);
  }

  // Identity table: grey n == raw pulse position n.
  const uint8_t identity[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  epd.setGreyPositions(identity);

  const int W = epd.width(), H = epd.height();
  const int bandW = W / 16;

  // Phase A: everything to overdriven black (position 8 ≈ saturation + 2).
  Serial.println("Phase A: saturating black...");
  epd.fillScreen(8);
  epd.waitSettled(10000);
  delay(500);

  // Phase B: band n gets n light pulses (descend to position 8 - n,
  // clamped at 0; bands 9-15 also get white-rail overdrive dwell).
  Serial.println("Phase B: descending...");
  epd.beginUpdate();
  for (int g = 0; g < 16; g++) {
    int target = 8 - g; if (target < 0) target = 0;
    epd.fillRect(g * bandW, 0, bandW, H - 40, (uint8_t)target);
  }
  epd.endUpdate();
  epd.waitSettled(10000);

  // Band index notches, drawn last (black on whatever the band became).
  epd.beginUpdate();
  for (int g = 1; g < 16; g++) {
    for (int n = 0; n < g; n++) {
      epd.fillRect(g * bandW + 6 + n * 3, H - 30, 2, 20, 8);
    }
  }
  epd.endUpdate();
  epd.waitSettled(10000);

  Serial.println("Settled — photograph the descent curve.");
  Serial.println("Band n = n light pulses from saturated black.");
}

void loop() { delay(1000); }
