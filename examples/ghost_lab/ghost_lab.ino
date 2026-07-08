// EPD_Painter2 — ghost lab: anti-ghost white refresh + DC trim engine.
//
// The sequence:
//   1. Heavy black content is drawn and left to sit (ghost seasoning).
//   2. The screen is cleared to white — look closely: faint ghosts of the
//      old content survive on most panels.
//   3. Every REFRESH_S seconds of idle, the maintenance engine sweeps the
//      settled whites with short lighten pulses. Watch the ghosts fade a
//      little with each sweep — and note the panel powers itself up for the
//      sweep and back down after.
//
// Underneath, every maintenance pulse is booked in a per-pixel charge
// account (ms of field, + = dark), and the trim engine fires short opposing
// pulses at rail-parked pixels to pull accounts back toward zero — charge
// without visible ink motion. Serial reports the totals:
//   maint=N    lifetime maintenance pulses
//   dcPeak=Nms worst per-pixel charge imbalance seen in the last sweep band

// Choose your board (or leave both commented for auto-detect).
//#define EPD_PAINTER2_PRESET_M5PAPER_S3
//#define EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include "EPD_Painter2.h"

EPD_Painter2 epd(EPD_PAINTER2_PRESET);

static const uint16_t REFRESH_S = 15;

static const uint8_t kGreys[16] =
  { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 26 };

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) delay(10);

  if (!epd.begin()) {
    Serial.println("EPD_Painter2 init failed!");
    while (1) delay(1000);
  }
  epd.setPulseWindow(7000);
  epd.setGreyPositions(kGreys);
  epd.setTravelBoost(7);
  epd.setWhiteRefresh(REFRESH_S);

  const int W = epd.width(), H = epd.height();

  // Phase 1: ghost seasoning — bold black shapes, left to sit.
  Serial.println("Seasoning ghosts...");
  epd.beginUpdate();
  for (int i = 0; i < 6; i++) {
    epd.fillRect(40 + i * 150, 60, 110, H - 120, 15);
  }
  epd.endUpdate();
  epd.waitSettled(10000);
  delay(4000);

  // Phase 2: clear to white. The ghosts remain — for now.
  Serial.println("Clearing. Watch the ghosts, then watch the sweeps.");
  epd.fillScreen(0);
  epd.waitSettled(10000);
  Serial.printf("White refresh every %us of idle. Observe.\n", REFRESH_S);
}

void loop() {
  static uint32_t lastStats = 0;
  if (millis() - lastStats > 2000) {
    lastStats = millis();
    auto s = epd.getStats();
    Serial.printf("maint=%lu dcPeak=%dms powered=%d\n",
                  (unsigned long)s.maintPulses, (int)s.dcPeakMs,
                  (int)s.powered);
  }
  delay(100);
}
