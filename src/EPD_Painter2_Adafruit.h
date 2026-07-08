#pragma once

#include <string.h>
#include <Adafruit_GFX.h>
#include "EPD_Painter2.h"

// =============================================================================
// EPD_Painter2Adafruit
//
// Adafruit GFX API on top of the continuous-simulation driver — with a twist
// that makes it simpler than the EPD_Painter version: there is NO paint().
//
// EPD_Painter2's target buffer is already one byte per pixel, so this wrapper
// draws straight into it. Every GFX call retargets pixels; the 50Hz tick task
// moves the ink toward them concurrently. Draw whenever you like — the screen
// is always chasing the buffer.
//
// Colours: 8-bit luminance, 0 = black … 255 = white (same convention as the
// EPD_Painter Adafruit wrapper / GFXcanvas8). Internally mapped to the 16
// grey levels. Standard GFX colour parameters just work:
//   gfx.fillScreen(0xFF);            // white
//   gfx.setTextColor(0x00);          // black text
//   gfx.fillCircle(x, y, r, 0x80);   // mid grey
//
// Composite updates (erase-old + draw-new of a moving object) should be
// wrapped in beginUpdate()/endUpdate() so the concurrent tick never catches
// the half-made state — same rule as the native API.
//
// Usage:
//   EPD_Painter2Adafruit gfx(EPD_PAINTER2_PRESET);
//   gfx.begin();
//   gfx.setRotation(1);              // GFX rotation fully supported
//   gfx.setCursor(20, 40);
//   gfx.print("Hello, ink!");        // ...and it is already on its way
// =============================================================================
class EPD_Painter2Adafruit : public Adafruit_GFX {
public:
    explicit EPD_Painter2Adafruit(const EPD_Painter2::Config &config)
        : Adafruit_GFX(config.width, config.height), _epd(config) {}

    // begin() — driver init (board auto-detect included). Canvas dimensions
    // are re-synced afterwards in case auto-detect chose the panel.
    bool begin() {
        if (!_epd.begin()) return false;
        WIDTH  = _epd.width();
        HEIGHT = _epd.height();
        _width  = WIDTH;
        _height = HEIGHT;
        setRotation(getRotation());   // recompute _width/_height for rotation
        return true;
    }
    bool end() { return _epd.end(); }

    // ---- Core GFX hooks -----------------------------------------------------
    void drawPixel(int16_t x, int16_t y, uint16_t color) override {
        int16_t t;
        switch (getRotation()) {
            case 1: t = x; x = WIDTH - 1 - y;  y = t;              break;
            case 2: x = WIDTH - 1 - x;  y = HEIGHT - 1 - y;        break;
            case 3: t = x; x = y;  y = HEIGHT - 1 - t;             break;
        }
        _epd.setPixel(x, y, toGrey(color));   // clips + smart span marking
    }

    // Fast path: the native fillRect (row memsets + precise dirty spans) at
    // rotation 0; other rotations fall back to the GFX pixel loop.
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                  uint16_t color) override {
        if (getRotation() != 0) {
            Adafruit_GFX::fillRect(x, y, w, h, color);
            return;
        }
        _epd.fillRect(x, y, w, h, toGrey(color));
    }

    void fillScreen(uint16_t color) override { _epd.fillScreen(toGrey(color)); }

    // ---- Composite atomicity (same rule as the native API) -------------------
    void beginUpdate() { _epd.beginUpdate(); }
    void endUpdate()   { _epd.endUpdate(); }

    // ---- Driver passthroughs --------------------------------------------------
    void setGreyPositions(const uint8_t pos[16]) { _epd.setGreyPositions(pos); }
    void setPulseWindow(uint32_t us)   { _epd.setPulseWindow(us); }
    void setTravelBoost(uint8_t gain)  { _epd.setTravelBoost(gain); }
    void setTickRate(uint16_t hz)      { _epd.setTickRate(hz); }        // before begin()
    void setComputeBudget(uint32_t us) { _epd.setComputeBudget(us); }
    void setWhiteRefresh(uint16_t s)   { _epd.setWhiteRefresh(s); }
    void waitSettled(uint32_t timeout_ms = 5000) { _epd.waitSettled(timeout_ms); }
    bool waitFrame(uint32_t timeout_ms = 100) { return _epd.waitFrame(timeout_ms); }
    void onFrame(EPD_Painter2::FrameCallback cb, void* arg = nullptr) { _epd.onFrame(cb, arg); }
    EPD_Painter2::Stats getStats()     { return _epd.getStats(); }

    // The underlying driver, for anything not wrapped.
    EPD_Painter2 &driver() { return _epd; }

    // Luminance (0=black..255=white) → driver grey (0=white..15=black).
    static uint8_t toGrey(uint16_t c) {
        const uint8_t v = (c > 255) ? 255 : (uint8_t)c;
        return 15 - (v >> 4);
    }

private:
    EPD_Painter2 _epd;
};
