#ifndef EPD_Painter2_H
#define EPD_Painter2_H

// =============================================================================
// EPD_Painter2 — continuous-simulation e-paper driver (PROTOTYPE)
//
// Architecture (vs EPD_Painter):
//   EPD_Painter:  paint() is a blocking waveform transaction — diff the frame,
//                 look up a calibrated pulse table, drive 7-13 passes, return.
//   EPD_Painter2: every pixel carries its own trajectory. A 50Hz tick advances
//                 all in-flight pixels one pulse toward their target and
//                 returns. Command latency is one tick (~20ms); transitions
//                 complete over several ticks and read as a natural fade.
//
// Pixel model (v0):
//   - 16 grey levels: 0 = white … 15 = black.
//   - target[]  1 byte/px (PSRAM): where the pixel should be.
//   - state[]   1 byte/px (PSRAM): the driver's estimate of where it is.
//   - A pixel is in flight iff state != target. Each tick applies one pulse
//     (darken 0b01 / lighten 0b10) and moves state one level. Full white↔black
//     therefore takes 15 ticks ≈ 300ms of gentle fade; a one-level tweak
//     completes in a single tick.
//   - DC balance: a v0 approximation — every level step is one pulse, so any
//     A→B→A round trip nets zero charge. Interrupted/asymmetric journeys can
//     accumulate small bias; a per-pixel residual field is the planned fix.
//
// Scheduling:
//   - FreeRTOS tick task (core 0) at tick_hz (default 50).
//   - A per-row active bitmask gates all work: idle screens cost ~nothing,
//     small animations only pay for their own rows.
//   - Panel power management is automatic: on when work exists, off after
//     ~1s idle.
//
// Supported boards (v0): M5PaperS3, LilyGo T5 S3 GPS (direct-GPIO boards).
// Shift-register boards (H752/H716) need the assembly SR driver — later.
// No assembly anywhere in this library (deliberately, for now).
// =============================================================================

#include <stddef.h>
#include <stdint.h>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <esp_private/gdma.h>
#include <hal/dma_types.h>

#ifdef ARDUINO
  #include <Wire.h>
#endif

#if !defined(EPD_PAINTER2_PRESET_M5PAPER_S3) && \
    !defined(EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS)
  #ifndef EPD_PAINTER2_PRESET_AUTO
    #define EPD_PAINTER2_PRESET_AUTO
  #endif
#endif

class EPD2_PinDriver;
class EPD2_PowerDriver;

class EPD_Painter2 {
public:

  struct I2CBusConfig {
#ifdef ARDUINO
      TwoWire* wire = nullptr;
#else
      void* i2c_bus = nullptr;
#endif
      int sda = -1;
      int scl = -1;
      uint32_t freq = 100000;
  };

  struct PowerCtlConfig {
      int pca_addr = -1;
      int tps_addr = -1;
  };

  struct Config {
      uint16_t width;
      uint16_t height;
      int16_t pin_pwr = -1;
      int16_t pin_sph = -1;
      int16_t pin_oe  = -1;
      int16_t pin_cl  = -1;
      int16_t pin_spv = -1;
      int16_t pin_ckv = -1;
      int16_t pin_le  = -1;
      int8_t  data_pins[8];
      I2CBusConfig i2c{};
      PowerCtlConfig power{};

      // Dummy zero bytes clocked out after each row's pixel data — the
      // panel's source-driver shift chain has more stages than visible
      // columns and needs the flush. Same fix as EPD_Painter; see its
      // How_It_Works.md §7.
      uint8_t row_pad_bytes = 4;

      // Simulation tick rate. Each tick is one pulse for every in-flight
      // pixel, so this sets both command latency (1/tick_hz) and fade speed
      // (15 ticks for a full white↔black swing).
      uint16_t tick_hz = 50;
  };

  struct ProbeSettings {
      Config *preset;
      int i2c_sda;
      int i2c_scl;
      int i2c_addr;
      bool found = false;
  };

  struct Stats {
      uint32_t frames;        // tick frames actually driven to the panel
      uint32_t lastTickUs;    // duration of the last driven frame
      uint32_t maxTickUs;     // worst frame since reset
      uint16_t activeRows;    // rows with in-flight pixels, last frame
      bool     powered;       // panel rails currently on
  };

  explicit EPD_Painter2(const Config &config);
  bool begin();
  bool end();

  // ---- Drawing (writes targets; the tick task moves the ink) --------------
  // Grey levels: 0 = white … 15 = black. All calls clip to the panel.
  //
  // Composite updates (e.g. erase-old + draw-new of a moving object) MUST be
  // wrapped in beginUpdate()/endUpdate(): the tick scan runs concurrently and
  // will otherwise catch the intermediate state — a moving box gets a white
  // band torn through it where the scan crossed between your two draw calls.
  // The kernel waits at a row boundary while an update is open (≤ one row's
  // work of latency for the drawing task).
  void beginUpdate();
  void endUpdate();
  void setPixel(int x, int y, uint8_t grey);
  void fillRect(int x, int y, int w, int h, uint8_t grey);
  void drawLine(int x1, int y1, int x2, int y2, uint8_t grey);
  void fillScreen(uint8_t grey);

  int width()  const { return _config.width;  }
  int height() const { return _config.height; }

  // Direct access to the target buffer (1 byte/px, row-major, values 0-15).
  // Call touchRows(y0,y1) after writing directly so the tick task notices.
  uint8_t* targetBuffer() { return _target; }
  void touchRows(int y0, int y1);

  // ---- Grey calibration -----------------------------------------------------
  // Maps user grey (0=white … 15=black) to a pulse position on the ink's
  // actual response curve. The panel saturates in far fewer pulses than 16,
  // and the response is nonlinear — this table is where that knowledge lives
  // (the first piece of "waveform computed on the fly").
  // Ends may overshoot the physical rails: extra pulses on arrival at a rail
  // re-anchor the ink and erase ghost trails; they stay inside the position
  // ledger, so round trips still cancel.
  // Tune with the staircase test pattern; entries must be ascending, 0..31.
  void setGreyPositions(const uint8_t pos[16]);

  // Block until every pixel has arrived at its target (optional).
  void waitSettled(uint32_t timeout_ms = 5000);

  Stats getStats();
  const Config& getConfig() { return _config; }

  Config _config;

private:
  // ---- LCD_CAM / DMA (ported from EPD_Painter, row padding included) ----
  gdma_channel_handle_t dma_chan = nullptr;
  dma_descriptor_t      dma_desc1 = {};
  dma_descriptor_t      dma_desc2 = {};

  uint8_t* dma_buffer  = nullptr;   // points at one of the two below
  uint8_t* dma_buffer1 = nullptr;
  uint8_t* dma_buffer2 = nullptr;

  int packed_row_bytes = 0;         // width / 4 (2 bits per pixel drive data)
  int dma_row_bytes    = 0;         // packed_row_bytes + row_pad_bytes

  // ---- Simulation state ----
  uint8_t* _target = nullptr;       // 1 byte/px, PSRAM: user grey 0-15
  uint8_t* _state  = nullptr;       // 1 byte/px, PSRAM: pulse POSITION (0.._posLUT[15])
  uint8_t  _posLUT[16];             // grey → pulse position (calibration)

  static constexpr int MAX_ROWS = 1024;
  static constexpr int CHUNK_PX = 64;          // dirty-tracking granularity
  std::atomic<uint32_t> _rowMask[MAX_ROWS / 32];
  // Per-row bitmask of dirty 64px column chunks (bit c = pixels c*64..c*64+63).
  // The tick kernel only reads/writes PSRAM for dirty chunks — this is what
  // keeps the tick inside budget: full-row scans are PSRAM-bandwidth-bound.
  std::atomic<uint16_t> _chunkMask[MAX_ROWS];

  // ---- Tick task ----
  SemaphoreHandle_t _draw_mtx = nullptr;   // guards target[] coherency vs the scan
  TaskHandle_t _tick_task_h = nullptr;
  volatile bool _tick_running = false;
  uint16_t _idle_ticks = 0;
  bool _powered = false;

  // stats
  std::atomic<uint32_t> _st_frames{0};
  std::atomic<uint32_t> _st_lastUs{0};
  std::atomic<uint32_t> _st_maxUs{0};
  std::atomic<uint16_t> _st_activeRows{0};

  // ---- Hardware drivers ----
  EPD2_PowerDriver* _powerDriver = nullptr;
  EPD2_PinDriver* _pin_spv = nullptr;
  EPD2_PinDriver* _pin_ckv = nullptr;
  EPD2_PinDriver* _pin_le  = nullptr;
  EPD2_PinDriver* _pin_sph = nullptr;

  // ---- Internals ----
  bool autoDetectBoard();
  bool initHardware();
  void powerOn();
  void powerOff();
  void sendRow(bool firstLine, bool lastLine = false);
  void hardClear();                     // establish all-white baseline
  bool tickFrame();                     // one simulation frame; true if any pixel still active
  bool rowKernel(int row, uint8_t* out);// advance one row; true if still active
  // Mark pixel columns [x0, x1) of row y as dirty (chunk-granular).
  void markSpan(int y, int x0, int x1) {
      uint16_t bits = 0;
      for (int c = x0 / CHUNK_PX; c <= (x1 - 1) / CHUNK_PX; c++) bits |= (1u << c);
      _chunkMask[y].fetch_or(bits, std::memory_order_relaxed);
      _rowMask[y >> 5].fetch_or(1u << (y & 31), std::memory_order_relaxed);
  }
  bool anyActive() const;

  static void _tick_task_entry(void *arg);
  void _tick_task_body();
};

#include "EPD_Painter2_presets.h"

#endif
