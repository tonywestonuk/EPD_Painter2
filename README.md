# EPD_Painter2 — continuous-simulation e-paper driver (PROTOTYPE)

A different architecture from [EPD_Painter](../EPD_Painter): instead of
blocking waveform transactions, **every pixel carries its own trajectory**
and a fixed 50Hz tick advances all in-flight pixels one pulse at a time.

| | EPD_Painter | EPD_Painter2 |
|---|---|---|
| Update model | `paint()` = blocking pulse-train transaction | free-running 50Hz simulation tick |
| Command latency | one full waveform (~150–600ms) | one tick (~20ms) |
| Grey levels | 4 (2bpp) | 16 (4bpp) |
| Waveforms | calibrated per-board tables | computed per pixel per tick |
| Concurrent updates | queue behind each other | every pixel independently in flight |
| Motion | discrete repaints | pixels fade continuously — natural motion blur |
| Assembly | Xtensa SIMD | none (deliberately, v0) |

## How it works

- `target[]` (1 byte/px, PSRAM): where each pixel should be (0=white … 15=black).
- `state[]` (1 byte/px, PSRAM): the driver's estimate of where it is.
- A pixel is *in flight* iff `state != target`. Drawing just writes targets.
- The tick task (core 0) fires at 50Hz: for each row with in-flight pixels it
  emits one pulse per pixel (darken/lighten one level) via LCD_CAM DMA; clean
  rows are clocked with neutral data. Full white↔black = 15 ticks ≈ 300ms of
  gentle fade; a one-level tweak lands in a single tick.
- A per-row active bitmask gates all work — an idle screen costs ~nothing and
  the panel powers itself down after ~1s of no in-flight pixels.

The hardware layer (LCD_CAM i8080 @ 80MHz PCLK, GDMA row streaming, row-end
padding for the panel's source-driver shift chain, CKV/SPV/LE timing,
TPS65185/PCA9555 power sequencing) is ported unchanged from EPD_Painter.

## Status: v0 prototype

Supported boards: **M5PaperS3**, **LilyGo T5 S3 GPS** (auto-detected).
Shift-register boards (H752/H716) are not supported yet — their pin driver
needs EPD_Painter's assembly.

Known v0 simplifications, in rough priority order:

1. **Linear trajectory model** — one level per tick, symmetric. Real ink is
   nonlinear; greys will land approximately. Planned: a 16×16×phase trajectory
   LUT with empirical calibration (the "waveform computed on the fly" proper).
2. **DC balance is approximate** — each level step is one pulse so A→B→A nets
   zero, but interrupted journeys accumulate small bias. Planned: per-pixel
   signed charge residual folded into trajectory selection.
3. **Scalar tick kernel** — the per-row kernel is plain C++ and doubles as the
   architecture's feasibility benchmark (watch `tick=` in the example's serial
   output; budget is 20000µs). SIMD if and when the numbers demand it.
4. **Row-mask race** — a draw call racing the kernel's row-clear can lose a
   wakeup; it self-heals on the next draw to that row.
5. Landscape only; no rotation, no LVGL/Adafruit bindings yet.

## Quick start

```cpp
#include "EPD_Painter2.h"
EPD_Painter2 epd(EPD_PAINTER2_PRESET);   // auto-detects the board

void setup() {
  epd.begin();                  // hard-clears to white, starts the tick task
  epd.fillRect(100, 100, 200, 200, 15);  // just write targets — no paint()!
}
void loop() { /* draw whenever you like; the tick task moves the ink */ }
```

See `examples/bouncing_box` — a box animating at 50Hz with ink-physics motion
blur, plus per-second tick timing stats on serial.
