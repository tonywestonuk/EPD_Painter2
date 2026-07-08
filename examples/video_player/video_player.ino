// EPD_Painter2 — VIDEO on e-paper. The ultimate test.
//
// Streams an EPV1 file ("/video.epv") from the SD card into the target
// buffer at its encoded frame rate. No paint(), no waveform transactions:
// each video frame just re-aims every pixel's target, and the 100Hz
// simulation drives the ink toward it. Pixels that can't keep up render as
// motion blur — by physics, not post-processing.
//
// EPV1 format (see tools/encode_epv.py in the repo):
//   16-byte header: "EPV1", u16 width, u16 height, u8 fps, u8 flags,
//   u16 reserved, u32 frame count — then raw frames, 2 px/byte, high
//   nibble = left pixel, values 0=white..15=black (display-ready).
//
// This sketch pixel-doubles WxH onto the panel, centered (360x270 -> 720x540
// on the 960x540 M5PaperS3). Card: FAT32, file at root as /video.epv.

// M5PaperS3 only (TF slot pins below are board-specific).
#define EPD_PAINTER2_PRESET_M5PAPER_S3

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "EPD_Painter2.h"

// M5PaperS3 TF slot (per M5Stack docs)
static const int SD_SCK = 39, SD_MISO = 40, SD_MOSI = 38, SD_CS = 47;

EPD_Painter2 epd(EPD_PAINTER2_PRESET);

File     vf;
uint16_t vw, vh;
uint8_t  vfps;
bool     oneBit;         // flags bit0: 1 px/bit (pure black/white content)
uint32_t vframes, frameIdx = 0;
size_t   frameBytes;
uint8_t* frameBuf;
int      ox, oy;
int64_t  nextFrameUs;
// 1bpp fast path: source byte (8 px, MSB=left) -> 16 doubled target bytes.
static uint8_t bw16[256][16];

bool openVideo() {
  vf = SD.open("/video.epv");
  if (!vf) { Serial.println("no /video.epv on card"); return false; }
  uint8_t h[16];
  if (vf.read(h, 16) != 16 || memcmp(h, "EPV1", 4) != 0) {
    Serial.println("bad EPV1 header");
    return false;
  }
  vw = h[4] | (h[5] << 8);
  vh = h[6] | (h[7] << 8);
  vfps = h[8];
  oneBit = (h[9] & 1) != 0;
  vframes = (uint32_t)h[12] | ((uint32_t)h[13] << 8) |
            ((uint32_t)h[14] << 16) | ((uint32_t)h[15] << 24);
  frameBytes = oneBit ? (size_t)vw * vh / 8 : (size_t)vw * vh / 2;
  ox = (epd.width()  - vw * 2) / 2;
  oy = (epd.height() - vh * 2) / 2;
  Serial.printf("video %ux%u @%ufps, %lu frames (%.1fs)\n",
                vw, vh, vfps, (unsigned long)vframes,
                (float)vframes / vfps);
  return (ox >= 0 && oy >= 0);
}

// Expand one packed frame 2x into the target buffer. Only chunks (64px)
// that actually changed are copied and marked — with the encoder's temporal
// deadband most of the frame is byte-identical, and every chunk we skip is
// PSRAM the 100Hz tick kernel never has to look at.
void drawFrame(const uint8_t* src) {
  uint8_t* tgt = epd.targetBuffer();
  const int PW = epd.width();
  static uint8_t line[960];

  // No beginUpdate() on purpose: a video frame is a pure target overwrite,
  // not a composite (nothing is erased then redrawn), so kernel atomicity
  // doesn't matter — a chunk catching the new frame one tick early is
  // invisible. Locking here serialized us against the tick's compute phase
  // and starved the ink of pulses.
  for (int r = 0; r < vh; r++) {
    uint8_t* L = line;
    if (oneBit) {
      const uint8_t* s = src + (size_t)r * (vw / 8);
      for (int b = 0; b < vw / 8; b++) { memcpy(L, bw16[s[b]], 16); L += 16; }
    } else {
      const uint8_t* s = src + (size_t)r * (vw / 2);
      for (int b = 0; b < vw / 2; b++) {
        const uint8_t hi = s[b] >> 4, lo = s[b] & 0x0F;
        *L++ = hi; *L++ = hi; *L++ = lo; *L++ = lo;
      }
    }
    const int y = oy + 2 * r;
    uint8_t* d = tgt + (size_t)y * PW + ox;
    // Compare/copy in strides aligned to the panel's 64px dirty chunks.
    for (int x = ox; x < ox + vw * 2; ) {
      int xe = (x / 64 + 1) * 64;
      if (xe > ox + vw * 2) xe = ox + vw * 2;
      const int n = xe - x, off = x - ox;
      if (memcmp(d + off, line + off, n) != 0) {
        memcpy(d + off,      line + off, n);
        memcpy(d + off + PW, line + off, n);
        epd.touchSpan(y,     x, xe);
        epd.touchSpan(y + 1, x, xe);
      }
      x = xe;
    }
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) delay(10);

  // 50Hz on purpose: fine pulses run at 50/s either way (at 100Hz they only
  // fire on-beat), and video frames arrive every 100ms, so 100Hz buys no
  // latency — it just doubles the scan/sweep overhead that competes with SD
  // reads and frame expansion. 100Hz is for fast-command demos (games).
  if (!epd.begin()) {
    Serial.println("EPD_Painter2 init failed!");
    while (1) delay(1000);
  }

  // Measured 16-grey LUT, 7ms fine pulses, full-tick travel boost for the
  // big scene-to-scene swings.
  static const uint8_t kGreys[16] =
    { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 26 };
  epd.setPulseWindow(7000);
  epd.setGreyPositions(kGreys);
  epd.setTravelBoost(7);       // one full 20ms pulse ≈ 7 fine positions
  epd.setComputeBudget(14000); // full-screen storms pause rows, not the clock

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, 40000000) && !SD.begin(SD_CS, SPI, 25000000)) {
    Serial.println("SD init failed");
    while (1) delay(1000);
  }
  if (!openVideo()) while (1) delay(1000);

  if (oneBit) {
    // Rail-exact travel for pure black/white: black at position 21 = 3x the
    // travel gain, so every crossing is coarse pulses only (continuous
    // ~60ms field) with no fine-pulse landing tail. Still past saturation,
    // so the rail re-anchors each arrival.
    static const uint8_t kBW[16] =
      { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 21 };
    epd.setGreyPositions(kBW);
    for (int v = 0; v < 256; v++)
      for (int bit = 0; bit < 8; bit++) {
        const uint8_t g = (v & (0x80 >> bit)) ? 15 : 0;
        bw16[v][bit * 2] = g;
        bw16[v][bit * 2 + 1] = g;
      }
  }

  frameBuf = (uint8_t*)heap_caps_malloc(frameBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!frameBuf) frameBuf = (uint8_t*)ps_malloc(frameBytes);
  if (!frameBuf) { Serial.println("no RAM for frame"); while (1) delay(1000); }

  nextFrameUs = esp_timer_get_time();
}

void loop() {
  // Pipeline: read the NEXT frame first — the SD transfer happens during
  // what would otherwise be idle pacing time — then wait for the frame's
  // slot, then draw. Read and draw costs overlap the frame clock instead
  // of stacking on top of it.
  static uint32_t readUs = 0, drawUs = 0, nStat = 0;

  int64_t t = esp_timer_get_time();
  if (vf.read(frameBuf, frameBytes) != (int)frameBytes) {
    vf.seek(16);                       // loop the video
    frameIdx = 0;
    nextFrameUs = esp_timer_get_time();
    return;
  }
  readUs += (uint32_t)(esp_timer_get_time() - t);

  // Wait for this frame's slot; if we've fallen behind, resync rather than
  // letting the deficit accumulate (video runs slightly slow, never bursts).
  nextFrameUs += 1000000LL / vfps;
  const int64_t now = esp_timer_get_time();
  if (nextFrameUs > now) {
    delay((uint32_t)((nextFrameUs - now) / 1000));
  } else if (now - nextFrameUs > 500000) {
    nextFrameUs = now;
  }

  t = esp_timer_get_time();
  drawFrame(frameBuf);                 // blocks on the draw mutex if the
  drawUs += (uint32_t)(esp_timer_get_time() - t);   // tick's compute is busy
  frameIdx++;
  nStat++;

  static uint32_t lastStats = 0;
  if (millis() - lastStats > 2000) {
    lastStats = millis();
    auto s = epd.getStats();
    Serial.printf("frame %lu/%lu read=%lums draw=%lums tick=%luus rows=%u paused=%u\n",
                  (unsigned long)frameIdx, (unsigned long)vframes,
                  (unsigned long)(readUs / nStat / 1000),
                  (unsigned long)(drawUs / nStat / 1000),
                  (unsigned long)s.lastTickUs, s.activeRows, s.pausedRows);
    readUs = drawUs = 0; nStat = 0;
  }
}
