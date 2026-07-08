// EPD_Painter2 calibration — the 100Hz "rested pulse" ascent curve.
//
// At 100Hz with the pulse window equal to the tick period (10ms), the
// driver switches to rested pulses: a fine pulse is one whole drive tick,
// ended by the NEXT tick's scan writing 0V, and the pixel rests the
// following tick. 10ms field on, 10ms rest — no field-off pass, one scan
// per tick, the 10ms budget holds.
//
// Identity LUT: band n = n rested 10ms pulses from hard white. This photo
// is the 100Hz equivalent of the pulse_width_staircase measurement and
// writes the 100Hz grey LUT. Expectations vs the 7ms/50Hz curve: a 10ms
// pulse carries more dose (longer, and the rest gap is 10ms not 13ms), so
// the ramp should be somewhat steeper — the question is how many
// distinguishable stops survive. 12+ and 100Hz greyscale is on.
//
// Serial stats matter too: tick must stay under 10000us.

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

  epd.setTickRate(100);          // before begin()
  if (!epd.begin()) {
    Serial.println("EPD_Painter2 init failed!");
    while (1) delay(1000);
  }
  epd.setPulseWindow(10000);     // == tick period → rested pulses

  const uint8_t identity[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  epd.setGreyPositions(identity);

  const int W = epd.width(), H = epd.height();
  const int bandW = W / 16;

  for (int g = 0; g < 16; g++) {
    epd.fillRect(g * bandW, 0, bandW, H - 40, g);
    for (int n = 0; n < g; n++) {
      epd.fillRect(g * bandW + 6 + n * 3, H - 30, 2, 20, 15);
    }
  }
  epd.waitSettled(10000);

  Serial.println("Settled — photograph the 100Hz rested-pulse curve.");
}

void loop() {
  static uint32_t lastStats = 0;
  if (millis() - lastStats > 2000) {
    lastStats = millis();
    auto s = epd.getStats();
    Serial.printf("tick=%luus (max %luus) rows=%u\n",
                  (unsigned long)s.lastTickUs, (unsigned long)s.maxTickUs,
                  s.activeRows);
  }
  delay(100);
}
