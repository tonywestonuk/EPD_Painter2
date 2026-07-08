// EPD_Painter2 calibration — pulse-WIDTH staircase (the fine-dose lever).
//
// A driven pixel's field stays on until its row is next rewritten: by
// default that's next tick's scan, so one pulse = ~20ms of field. With
// setPulseWindow(), every drive scan is chased by an all-neutral scan that
// writes 0V back onto the pixel caps, cutting each pulse to ~window µs.
//
// This sketch splits the screen horizontally:
//   TOP half:    band n = n FULL-TICK pulses  (~20ms each — Tuesday's curve)
//   BOTTOM half: band n = n SHORT pulses      (PULSE_WINDOW_US each)
//
// Reading the photo:
//   - bottom staircase much lighter & finer-stepped  → dose scales with
//     field time → the quantum is OURS to choose, not the panel's
//   - find k where bottom band k matches top band 1: the dose ratio.
//     If k ≈ 20000/PULSE_WINDOW_US, dose is linear in field time.
//   - if the bottom rail-to-first-step gap shrinks too, the white→grey
//     threshold is dose-based (good), not per-pulse (bad).
//
// Identity LUT throughout, so band index == raw pulse count.

// Choose your board (or leave both commented for auto-detect).
//#define EPD_PAINTER2_PRESET_M5PAPER_S3
//#define EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include "EPD_Painter2.h"

EPD_Painter2 epd(EPD_PAINTER2_PRESET);

// Short-pulse field time, µs (drive-scan start → neutral-scan start).
// Floor is one scan duration (~4-6ms); 6000 ≈ 1/3 of a 20ms tick.
static const uint32_t PULSE_WINDOW_US = 6000;

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

  const int W = epd.width(), H = epd.height();
  const int bandW = W / 16;
  const int half  = H / 2;

  // Phase A — reference: top half at full-tick pulses, plus the band-index
  // notches at the very bottom (drawn now so they get coarse pulses = black).
  Serial.println("Phase A: top half, full-tick pulses...");
  epd.beginUpdate();
  for (int g = 0; g < 16; g++) {
    epd.fillRect(g * bandW, 0, bandW, half - 2, g);
    for (int n = 0; n < g; n++) {
      epd.fillRect(g * bandW + 6 + n * 3, H - 30, 2, 20, 15);
    }
  }
  epd.endUpdate();
  epd.waitSettled(10000);

  // Phase B — same bands below the midline, at short pulses.
  Serial.printf("Phase B: bottom half, %luus pulses...\n",
                (unsigned long)PULSE_WINDOW_US);
  epd.setPulseWindow(PULSE_WINDOW_US);
  epd.beginUpdate();
  for (int g = 0; g < 16; g++) {
    epd.fillRect(g * bandW, half + 2, bandW, H - 40 - (half + 2), g);
  }
  epd.endUpdate();
  epd.waitSettled(10000);

  Serial.println("Settled — photograph the panel.");
  Serial.println("Top = 20ms pulses, bottom = short pulses, same pulse counts.");
}

void loop() { delay(1000); }
