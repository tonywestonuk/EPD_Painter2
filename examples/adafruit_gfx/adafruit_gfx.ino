// EPD_Painter2 + Adafruit GFX — the familiar API on the continuous driver.
//
// The twist vs every other e-paper GFX wrapper: there is NO paint(), NO
// display(), NO update(). GFX calls write straight into the driver's target
// buffer and the 50Hz simulation moves the ink concurrently. print() and the
// text is already fading in.
//
// Colours are 8-bit luminance like GFXcanvas8: 0 = black … 255 = white.
//
// Requires: Adafruit GFX Library.

// Choose your board (or leave both commented for auto-detect).
//#define EPD_PAINTER2_PRESET_M5PAPER_S3
//#define EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include "EPD_Painter2_Adafruit.h"

EPD_Painter2Adafruit gfx(EPD_PAINTER2_PRESET);

// Measured 16-grey calibration (see the sixteen_greys example).
static const uint8_t kGreys[16] =
  { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 26 };

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) delay(10);

  if (!gfx.begin()) {
    Serial.println("EPD_Painter2 init failed!");
    while (1) delay(1000);
  }
  gfx.setPulseWindow(7000);
  gfx.setGreyPositions(kGreys);
  gfx.setTravelBoost(7);

  const int W = gfx.width(), H = gfx.height();

  // Title
  gfx.setTextColor(0x00);
  gfx.setTextSize(4);
  gfx.setCursor(24, 24);
  gfx.print("EPD_Painter2 meets Adafruit GFX");

  // Grey swatches
  const int sw = (W - 48) / 16;
  for (int i = 0; i < 16; i++) {
    gfx.fillRect(24 + i * sw, 90, sw - 4, 60, 255 - i * 17);
    gfx.drawRect(24 + i * sw, 90, sw - 4, 60, 0x00);
  }

  // Shape sampler
  gfx.fillCircle(140, 300, 80, 0x40);
  gfx.drawCircle(140, 300, 90, 0x00);
  gfx.fillTriangle(300, 380, 400, 220, 500, 380, 0x90);
  gfx.drawTriangle(300, 380, 400, 220, 500, 380, 0x00);
  gfx.fillRoundRect(560, 230, 200, 140, 24, 0xC8);
  gfx.drawRoundRect(560, 230, 200, 140, 24, 0x00);
  for (int i = 0; i < 8; i++) {
    gfx.drawLine(24, 420 + i * 12, W - 24, 420 + i * 12, (i * 32 > 255) ? 255 : i * 32);
  }

  // Footer
  gfx.setTextSize(2);
  gfx.setCursor(24, H - 40);
  gfx.print("no paint(). no waveform tables. the ink is just... going.");

  gfx.waitSettled(10000);
  Serial.println("Settled.");
}

void loop() {
  // Live element: a clock-ish counter re-drawn in place, no flicker, no
  // paint calls — deltas only.
  static uint32_t last = 0;
  static int n = 0;
  if (millis() - last > 1000) {
    last = millis();
    gfx.beginUpdate();                       // composite: erase + redraw
    gfx.fillRect(gfx.width() - 220, 20, 200, 40, 0xFF);
    gfx.setTextSize(3);
    gfx.setCursor(gfx.width() - 220, 24);
    gfx.setTextColor(0x00);
    gfx.print(n++);
    gfx.endUpdate();
  }
  delay(20);
}
