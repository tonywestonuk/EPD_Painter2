// EPD_Painter2 calibration — raw pulse-response staircase.
//
// Uses an IDENTITY grey→position table, so band n receives exactly n pulses
// of darkening from the hard-white baseline. A photo of the result is the
// panel's raw response curve:
//   - the first band visibly darker than band 0  → pulses to leave white
//   - the first band indistinguishable from its neighbour → quantum resolution
//   - the first fully black band                 → saturation point
// Those three numbers write the real kDefaultPosLUT.
//
// Bands are drawn as full-height columns, labelled by tick marks at the
// bottom edge (1px notch per band index, so band 5 has 5 notches).

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

  // Identity table: grey n = n raw pulses from hard white.
  const uint8_t identity[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  epd.setGreyPositions(identity);

  const int W = epd.width(), H = epd.height();
  const int bandW = W / 16;

  for (int g = 0; g < 16; g++) {
    epd.fillRect(g * bandW, 0, bandW, H - 40, g);
    // tick marks: g notches at the bottom of band g
    for (int n = 0; n < g; n++) {
      epd.fillRect(g * bandW + 8 + n * 10, H - 30, 4, 20, 15);
    }
  }

  epd.waitSettled(10000);
  Serial.println("Raw staircase settled — photograph the panel.");
  Serial.println("Report: first band darker than band 0, first indistinct pair, first black band.");
}

void loop() { delay(1000); }
