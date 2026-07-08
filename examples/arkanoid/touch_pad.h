#pragma once
// Minimal GT911 touch polling for M5PaperS3 (addr 0x5D on the internal I2C
// bus the EPD driver already opened — pins 41/42). No INT pin needed: the
// status register is polled once per game frame.

#include <Wire.h>

static const uint8_t GT911_ADDR = 0x5D;

static bool gt911_read(TwoWire* w, uint16_t reg, uint8_t* buf, size_t n) {
  w->beginTransmission(GT911_ADDR);
  w->write((uint8_t)(reg >> 8));
  w->write((uint8_t)(reg & 0xFF));
  if (w->endTransmission(false) != 0) return false;
  if (w->requestFrom((int)GT911_ADDR, (int)n) != (int)n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = w->read();
  return true;
}

static void gt911_clear(TwoWire* w) {
  w->beginTransmission(GT911_ADDR);
  w->write((uint8_t)0x81);
  w->write((uint8_t)0x4E);
  w->write((uint8_t)0x00);
  w->endTransmission();
}

// Returns true while a finger is down; x/y in panel coordinates.
static bool touchRead(TwoWire* w, int& x, int& y) {
  if (!w) return false;
  static int lx = 0, ly = 0;
  static bool down = false;
  uint8_t s;
  if (!gt911_read(w, 0x814E, &s, 1)) return false;
  if (s & 0x80) {
    const int n = s & 0x0F;
    if (n >= 1) {
      uint8_t p[6];
      if (gt911_read(w, 0x8150, p, 6)) {
        lx = p[1] | (p[2] << 8);
        ly = p[3] | (p[4] << 8);
        down = true;
      }
    } else {
      down = false;               // fresh status, zero touches = release
    }
    gt911_clear(w);
  }
  x = lx; y = ly;
  return down;
}
