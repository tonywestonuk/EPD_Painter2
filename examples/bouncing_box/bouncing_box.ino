// EPD_Painter2 demo — a box bouncing at the full 50Hz simulation rate.
//
// What to watch for:
//   - The box moves EVERY 20ms — command latency is one tick, not one
//     waveform transaction. No blocking paint() exists in this library.
//   - The trail behind the box is pixels mid-flight back to white: motion
//     blur rendered by the physics of the ink itself.
//   - The grey staircase (drawn once at the top) shows all 16 levels.
//
// Serial prints tick-engine stats once per second:
//   tick=NNNNus  — how long the last driven frame took (the feasibility
//                  number for the whole architecture; budget is 20000us)
//   rows=NNN     — rows with in-flight pixels

// Choose your board (or leave both commented for auto-detect).
//#define EPD_PAINTER2_PRESET_M5PAPER_S3
//#define EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include "EPD_Painter2.h"

EPD_Painter2 epd(EPD_PAINTER2_PRESET);

// Box state
static const int BOX = 96;
static float bx = 100, by = 200;
static float vx = 3.0f, vy = 2.2f;   // pixels per tick (50Hz) → 150/110 px/s
static int   prevX = -1, prevY = -1;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) delay(10);

  epd.setTickRate(100);   // before begin()
  if (!epd.begin()) {
    Serial.println("EPD_Painter2 init failed!");
    while (1) delay(1000);
  }
  Serial.printf("EPD_Painter2 %dx%d, tick %dHz\n",
                epd.width(), epd.height(), epd.getConfig().tick_hz);

  // 16-grey calibration (measured — see sixteen_greys) + dynamic pulse
  // width, now at 100Hz: fine pixels fire on-beat every other tick — the
  // identical 7ms-on/13ms-off pulse train as 50Hz, so the same LUT holds —
  // while the box's black/white travel rides full 10ms ticks in between.
  // Commands land every 10ms: 100 updates/second.
  static const uint8_t kGreys[16] =
    { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 26 };
  epd.setPulseWindow(7000);
  epd.setGreyPositions(kGreys);
  epd.setTravelBoost(6);   // one gapped 10ms pulse ≈ 6 fine positions (measured)

  // 16-level grey staircase across the top, drawn once.
  const int bandW = epd.width() / 16;
  for (int g = 0; g < 16; g++) {
    epd.fillRect(g * bandW, 0, bandW, 60, g);
  }
}

void loop() {
  // --- one animation step per simulation tick ---
  const int W = epd.width(), H = epd.height();

  bx += vx; by += vy;
  if (bx < 0)           { bx = 0;           vx = -vx; }
  if (bx + BOX > W)     { bx = W - BOX;     vx = -vx; }
  if (by < 70)          { by = 70;          vy = -vy; }   // keep below staircase
  if (by + BOX > H)     { by = H - BOX;     vy = -vy; }

  int x = (int)bx, y = (int)by;
  if (x != prevX || y != prevY) {
    // Composite draw: erase-old + draw-new must be atomic vs the tick scan,
    // or the scan can catch the overlap region mid-update (target briefly
    // white) and tear a white band through the moving box.
    epd.beginUpdate();
    if (prevX >= 0) epd.fillRect(prevX, prevY, BOX, BOX, 0);   // old → white
    epd.fillRect(x, y, BOX, BOX, 15);                          // new → black
    epd.endUpdate();
    prevX = x; prevY = y;
  }

  static uint32_t lastStats = 0;
  if (millis() - lastStats > 1000) {
    lastStats = millis();
    auto s = epd.getStats();
    Serial.printf("frames=%lu tick=%luus (max %luus) rows=%u powered=%d\n",
                  (unsigned long)s.frames, (unsigned long)s.lastTickUs,
                  (unsigned long)s.maxTickUs, s.activeRows, (int)s.powered);
  }

  delay(10);   // pace the animation to the 100Hz simulation tick
}
