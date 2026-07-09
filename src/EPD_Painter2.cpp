// =============================================================================
// EPD_Painter2 — continuous-simulation e-paper driver (PROTOTYPE)
//
// Hardware layer (LCD_CAM, DMA, sendRow, row padding, power) ported from
// EPD_Painter. The paint transaction + waveform tables are replaced by a
// per-pixel trajectory simulation advanced by a fixed-rate tick task.
// =============================================================================
#ifdef ARDUINO
#include "esp32-hal.h"
#endif
#include <string.h>
#include "epd2_build_opt.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"
#include <driver/periph_ctrl.h>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>
#include <hal/gpio_hal.h>
#include <soc/lcd_cam_struct.h>
#include <soc/gdma_struct.h>
#include "EPD_Painter2.h"
#include "epd2_powerctl.h"
#include "epd2_pin_driver.h"

#ifdef ARDUINO
  #include "Wire.h"
#endif

// LCD_CAM signal indices for the 8 parallel data lines
static const uint8_t kDataSignals[8] = {
  LCD_DATA_OUT0_IDX, LCD_DATA_OUT1_IDX, LCD_DATA_OUT2_IDX, LCD_DATA_OUT3_IDX,
  LCD_DATA_OUT4_IDX, LCD_DATA_OUT5_IDX, LCD_DATA_OUT6_IDX, LCD_DATA_OUT7_IDX,
};

static inline void epd2_gpio_func_sel(int pin) {
  esp_rom_gpio_pad_select_gpio((gpio_num_t)pin);
}

// Drive values, 2 bits per pixel (4 px/byte, first pixel in the high bits):
//   0b00 neutral, 0b01 darken (black voltage), 0b10 lighten (white voltage)
static constexpr uint8_t DRIVE_DARK  = 0b01;
static constexpr uint8_t DRIVE_LIGHT = 0b10;

// Default grey→pulse-position calibration, measured on M5PaperS3 with the
// staircase_calibration example (identity table + notch-labelled bands):
//   - the hard-clear rail IS position 0 — the first pulse is already a
//     visible grey, so there is no overdrive headroom above white;
//   - positions 1-5 are each distinguishable;
//   - the ink saturates black at ~6 pulses.
// That is ~7 honest levels at one-sweep dose, so the 16 grey indices share
// positions (adjacent duplicates are a pulse-quantum limit, not a bug —
// finer greys need variable-dose sweeps or hysteresis trajectories).
// Grey 15 sits 2 pulses past saturation: crisp blacks, and the 8-pulse
// return journey over-whitens by ~2 pulses against the white rail, which
// erases ghost trails on every return from black.
static const uint8_t kDefaultPosLUT[16] =
  { 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 8 };

EPD_Painter2::EPD_Painter2(const Config &config) {
  _config = config;
  memcpy(_posLUT, kDefaultPosLUT, sizeof(_posLUT));
  for (auto &w : _rowMask)   w.store(0, std::memory_order_relaxed);
  for (auto &w : _chunkMask) w.store(0, std::memory_order_relaxed);
}

void EPD_Painter2::setGreyPositions(const uint8_t pos[16]) {
  memcpy(_posLUT, pos, sizeof(_posLUT));
}

// =============================================================================
// sendRow() — ported verbatim from EPD_Painter (including AFIFO prefill
// ordering and row-end padding).
// =============================================================================
void EPD_Painter2::sendRow(bool firstLine, bool lastLine) {
  while (LCD_CAM.lcd_user.lcd_start) {}

  // Dose control: extend the currently-selected row's field time before
  // advancing the gate. Uniform across the frame (neutral rows unaffected —
  // no drive voltage, just time).
  if (_dwell_us) EPD2_DELAY_US(_dwell_us);

  dma_descriptor_t *desc;
  if (dma_buffer == dma_buffer1) {
    desc = &dma_desc1;
    dma_buffer = dma_buffer2;
  } else {
    desc = &dma_desc2;
    dma_buffer = dma_buffer1;
  }

  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  gdma_start(dma_chan, (intptr_t)desc);

  if (firstLine) {
    _pin_spv->set(false);
    _pin_ckv->set(false);
    EPD2_DELAY_US(1);
    _pin_ckv->set(true);
    _pin_spv->set(true);
  } else {
    _pin_le->set(true);
    _pin_le->set(false);
    _pin_ckv->set(false);
    EPD2_DELAY_US(1);
    _pin_ckv->set(true);
  }

  LCD_CAM.lcd_user.lcd_start = 1;
  if (lastLine) {
    while (LCD_CAM.lcd_user.lcd_start) {}
    _pin_ckv->set(false);
    _pin_le->set(true);
    _pin_le->set(false);
    _pin_ckv->set(true);
  }
}

// =============================================================================
// autoDetectBoard() — probes the supported boards' I2C buses.
// =============================================================================
bool EPD_Painter2::autoDetectBoard() {
#if defined(ARDUINO) && defined(EPD_PAINTER2_PRESET_AUTO)
  if (_config.data_pins[0] >= 0) return true;

  int i = 1;
  for (auto& probe : EPD2_Probe) {
    printf("[EPD2] Probing board %d on SDA=%d, SCL=%d, addr=0x%x\n",
           i, probe.i2c_sda, probe.i2c_scl, probe.i2c_addr);
    TwoWire _w(1);
    _w.begin(probe.i2c_sda, probe.i2c_scl, 100000);
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    _w.beginTransmission(probe.i2c_addr);
    bool found = (_w.endTransmission() == 0);
    esp_log_level_set("i2c.master", ESP_LOG_WARN);
    _w.end();
    EPD2_PIN_OUTPUT(probe.i2c_sda);
    EPD2_PIN_OUTPUT(probe.i2c_scl);
    if (found) {
      printf("[EPD2] Board %d found\n", i);
      _config = *probe.preset;
      return true;
    }
    ++i;
  }
  printf("[EPD2] No known board found; begin() will fail\n");
  return false;
#else
  return _config.data_pins[0] >= 0;
#endif
}

// =============================================================================
// initHardware() — LCD_CAM + GDMA + buffers, ported from EPD_Painter::begin()
// =============================================================================
bool EPD_Painter2::initHardware() {
  packed_row_bytes = _config.width / 4;
  dma_row_bytes    = packed_row_bytes + _config.row_pad_bytes;

  // ---- Enable and reset LCD_CAM peripheral ----
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);
  LCD_CAM.lcd_user.lcd_reset = 1;
  EPD2_DELAY_US(100);

  // ---- Pixel clock: 160MHz source / 2 = 80MHz PCLK ----
  LCD_CAM.lcd_clock.clk_en = 1;
  LCD_CAM.lcd_clock.lcd_clk_sel = 2;
  LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;
  LCD_CAM.lcd_clock.lcd_clkm_div_num = 1;
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
  LCD_CAM.lcd_clock.lcd_clkcnt_n = 1;

  // ---- i8080 8-bit parallel mode ----
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0;
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;
  LCD_CAM.lcd_data_dout_mode.val = 0;
  LCD_CAM.lcd_user.lcd_always_out_en = 0;
  LCD_CAM.lcd_user.lcd_8bits_order = 0;
  LCD_CAM.lcd_user.lcd_bit_order = 0;
  LCD_CAM.lcd_user.lcd_2byte_en = 0;
  LCD_CAM.lcd_user.lcd_dummy = 0;
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 0;
  LCD_CAM.lcd_user.lcd_cmd = 0;
  LCD_CAM.lcd_user.lcd_dout_cyclelen = dma_row_bytes - 1;
  LCD_CAM.lcd_user.lcd_dout = 1;
  LCD_CAM.lcd_user.lcd_update = 1;

  // ---- Route data bus + pixel clock ----
  for (int i = 0; i < 8; i++) {
    int8_t pin = _config.data_pins[i];
    esp_rom_gpio_connect_out_signal(pin, kDataSignals[i], false, false);
    epd2_gpio_func_sel(pin);
    gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)3);
  }
  esp_rom_gpio_connect_out_signal(_config.pin_cl, LCD_PCLK_IDX, false, false);
  epd2_gpio_func_sel(_config.pin_cl);
  gpio_set_drive_capability((gpio_num_t)_config.pin_cl, (gpio_drive_cap_t)3);

  // ---- GDMA channel ----
  gdma_channel_alloc_config_t dma_chan_config = {
    .sibling_chan = NULL,
    .direction = GDMA_CHANNEL_DIRECTION_TX,
    .flags = { .reserve_sibling = 0 },
  };
  gdma_new_channel(&dma_chan_config, &dma_chan);
  gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));

  gdma_strategy_config_t strategy_config = {
    .owner_check = false,
    .auto_update_desc = false,
  };
  gdma_apply_strategy(dma_chan, &strategy_config);

  // ---- DMA row buffers (zeroed once; pad region stays neutral forever) ----
  dma_buffer1 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, dma_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  dma_buffer2 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, dma_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  if (!dma_buffer1 || !dma_buffer2) return false;
  memset(dma_buffer1, 0x00, dma_row_bytes);
  memset(dma_buffer2, 0x00, dma_row_bytes);
  dma_buffer = dma_buffer1;

  dma_desc1.dw0.suc_eof = 1;
  dma_desc1.dw0.size = dma_row_bytes;
  dma_desc1.dw0.length = dma_row_bytes;
  dma_desc1.buffer = dma_buffer1;
  dma_desc1.next = nullptr;

  dma_desc2.dw0.suc_eof = 1;
  dma_desc2.dw0.size = dma_row_bytes;
  dma_desc2.dw0.length = dma_row_bytes;
  dma_desc2.buffer = dma_buffer2;
  dma_desc2.next = nullptr;

  return true;
}

// =============================================================================
// begin()
// =============================================================================
bool EPD_Painter2::begin() {
#ifdef EPD_PAINTER2_PRESET_AUTO
  if (!autoDetectBoard()) return false;
#endif
  if (_tick_override) _config.tick_hz = _tick_override;

#ifdef ARDUINO
  if (_config.i2c.scl != -1 && _config.i2c.wire == nullptr) {
    TwoWire *w = new TwoWire(0);
    w->begin(_config.i2c.sda, _config.i2c.scl, _config.i2c.freq);
    _config.i2c.wire = w;
    EPD2_DELAY_MS(50);
  }
#endif

  // ---- Control pins (direct GPIO only in v0) ----
  if (_config.pin_pwr >= 0) EPD2_PIN_OUTPUT(_config.pin_pwr);
  if (_config.pin_spv >= 0) EPD2_PIN_OUTPUT(_config.pin_spv);
  if (_config.pin_oe  >= 0) EPD2_PIN_OUTPUT(_config.pin_oe);
  EPD2_PIN_OUTPUT(_config.pin_ckv);
  EPD2_PIN_OUTPUT(_config.pin_sph);
  EPD2_PIN_OUTPUT(_config.pin_le);
  EPD2_PIN_OUTPUT(_config.pin_cl);

  if (!initHardware()) return false;

  // ---- Simulation buffers ----
  const size_t npx = (size_t)_config.width * _config.height;
  _target = static_cast<uint8_t *>(heap_caps_aligned_alloc(16, npx, MALLOC_CAP_SPIRAM));
  _state  = static_cast<uint8_t *>(heap_caps_aligned_alloc(16, npx, MALLOC_CAP_SPIRAM));
  if (!_target || !_state) return false;
  memset(_target, 0, npx);   // all white
  memset(_state,  0, npx);

  // Per-pixel charge account for DC trim / anti-ghost (optional: driver
  // works without it if PSRAM is tight — maintenance just stays off).
  _dc = static_cast<int16_t *>(
    heap_caps_aligned_alloc(16, npx * sizeof(int16_t), MALLOC_CAP_SPIRAM));
  if (_dc) memset(_dc, 0, npx * sizeof(int16_t));

  // Deghost grid + the sustain rows the scrub pulses ride on.
  _gridW = (_config.width  + GHOST_CELL - 1) / GHOST_CELL;
  _gridH = (_config.height + GHOST_CELL - 1) / GHOST_CELL;
  if (_gridW > GRID_MAX) _gridW = GRID_MAX;
  if (_gridH > GRID_MAX) _gridH = GRID_MAX;
  _sustain = static_cast<uint8_t *>(heap_caps_aligned_alloc(
    16, (size_t)MAINT_BAND * (_config.width / 4), MALLOC_CAP_INTERNAL));

  // Frame staging — internal RAM preferred (fast phase-2 memcpys), PSRAM ok.
  const size_t staging_size = (size_t)packed_row_bytes * _config.height;
  _staging = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, staging_size, MALLOC_CAP_INTERNAL));
  if (!_staging) {
    _staging = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(16, staging_size, MALLOC_CAP_SPIRAM));
  }
  if (!_staging) return false;

  // ---- Power driver ----
  if (_config.power.tps_addr != -1) {
    auto* pc = new EPD2_PowerCtl();
    if (!pc->begin(_config)) {
      printf("[EPD2] FATAL: powerctl init failed\n");
      return false;
    }
    _powerDriver = pc;
  } else {
    _powerDriver = new EPD2_GpioPowerDriver(_config.pin_oe, _config.pin_pwr);
  }

  _pin_spv = new EPD2_GpioPin(uint8_t(_config.pin_spv));
  _pin_ckv = new EPD2_GpioPin(uint8_t(_config.pin_ckv));
  _pin_le  = new EPD2_GpioPin(uint8_t(_config.pin_le));
  _pin_sph = new EPD2_GpioPin(uint8_t(_config.pin_sph));

  _draw_mtx = xSemaphoreCreateMutex();
  if (!_vbl_sem) _vbl_sem = xSemaphoreCreateBinary();

  // ---- Establish a known all-white baseline ----
  powerOn();
  hardClear();
  powerOff();

  // ---- Start the simulation tick ----
  _tick_running = true;
  xTaskCreatePinnedToCore(_tick_task_entry, "epd2_tick", 8000, this, 10,
                          &_tick_task_h, 0);
  return true;
}

bool EPD_Painter2::end() {
  _tick_running = false;
  if (_tick_task_h) {
    EPD2_DELAY_MS(50);           // let the task exit its loop
    vTaskDelete(_tick_task_h);
    _tick_task_h = nullptr;
  }
  if (_powered) powerOff();
  if (dma_chan) {
    gdma_disconnect(dma_chan);
    gdma_del_channel(dma_chan);
    dma_chan = nullptr;
  }
  periph_module_disable(PERIPH_LCD_CAM_MODULE);
  return true;
}

// =============================================================================
// Power sequencing — ported from EPD_Painter
// =============================================================================
void EPD_Painter2::powerOn() {
  _pin_le->set(false);
  _pin_spv->set(false);
  _pin_sph->set(false);

  _powerDriver->powerOn();

  _pin_spv->set(false);
  _pin_ckv->set(false);
  EPD2_DELAY_US(1);
  _pin_ckv->set(true);
  _pin_spv->set(true);
  _powered = true;
}

void EPD_Painter2::powerOff() {
  _powerDriver->powerOff();
  _powered = false;
}

// =============================================================================
// hardClear() — full-panel erase, ported from EPD_Painter::clear() (HARD).
// Establishes the all-white baseline the simulation starts from.
// =============================================================================
void EPD_Painter2::hardClear() {
  const int prb = packed_row_bytes;
  const int H   = _config.height;
  const int num_phases = 4;
  const int totpass[4] = { 6, 2, 4, 8 };

  dma_buffer = dma_buffer1;

  for (int phase = 0; phase < num_phases; phase++) {
    uint8_t pattern = (phase % 2 == 0) ? 0b01010101 : 0b10101010;
    for (int passes = 0; passes < totpass[phase]; passes++) {
      for (int row = 0; row < H; ++row) {
        memset(dma_buffer, pattern, prb);
        sendRow(row == 0);
      }
      EPD2_DELAY_MS(5);
    }
  }

  // Neutral close-out
  memset(dma_buffer1, 0x00, prb);
  memset(dma_buffer2, 0x00, prb);
  for (int row = 0; row < H; ++row) {
    sendRow(row == 0, row == H - 1);
  }
}

// =============================================================================
// Drawing — writes targets and marks rows; the tick task moves the ink.
// =============================================================================
void EPD_Painter2::beginUpdate() { xSemaphoreTake(_draw_mtx, portMAX_DELAY); }
void EPD_Painter2::endUpdate()   { xSemaphoreGive(_draw_mtx); }

void EPD_Painter2::setPixel(int x, int y, uint8_t grey) {
  if (x < 0 || y < 0 || x >= _config.width || y >= _config.height) return;
  grey &= 0x0F;
  const size_t i = (size_t)y * _config.width + x;
  _target[i] = grey;
  if (_state[i] != _posLUT[grey]) markSpan(y, x, x + 1);
}

void EPD_Painter2::fillRect(int x, int y, int w, int h, uint8_t grey) {
  grey &= 0x0F;
  int x0 = x < 0 ? 0 : x;
  int y0 = y < 0 ? 0 : y;
  int x1 = x + w; if (x1 > _config.width)  x1 = _config.width;
  int y1 = y + h; if (y1 > _config.height) y1 = _config.height;
  if (x0 >= x1 || y0 >= y1) return;

  for (int yy = y0; yy < y1; yy++) {
    uint8_t* t = _target + (size_t)yy * _config.width + x0;
    memset(t, grey, x1 - x0);
    markSpan(yy, x0, x1);   // cheap; the kernel clears chunks that settle
  }
}

void EPD_Painter2::drawLine(int x1, int y1, int x2, int y2, uint8_t grey) {
  // Bresenham; setPixel clips, so out-of-bounds segments are safe.
  int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
  int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    setPixel(x1, y1, grey);
    if (x1 == x2 && y1 == y2) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x1 += sx; }
    if (e2 <= dx) { err += dx; y1 += sy; }
  }
}

void EPD_Painter2::fillScreen(uint8_t grey) {
  fillRect(0, 0, _config.width, _config.height, grey);
}

void EPD_Painter2::touchRows(int y0, int y1) {
  if (y0 < 0) y0 = 0;
  if (y1 >= _config.height) y1 = _config.height - 1;
  for (int y = y0; y <= y1; y++) markSpan(y, 0, _config.width);
}

void EPD_Painter2::touchSpan(int y, int x0, int x1) {
  if (y < 0 || y >= _config.height) return;
  if (x0 < 0) x0 = 0;
  if (x1 > _config.width) x1 = _config.width;
  if (x0 >= x1) return;
  markSpan(y, x0, x1);
}

bool EPD_Painter2::waitFrame(uint32_t timeout_ms) {
  if (!_vbl_sem) return false;
  return xSemaphoreTake(_vbl_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void EPD_Painter2::waitSettled(uint32_t timeout_ms) {
  uint32_t start = (uint32_t)(esp_timer_get_time() / 1000);
  while (anyActive()) {
    if ((uint32_t)(esp_timer_get_time() / 1000) - start > timeout_ms) return;
    EPD2_DELAY_MS(5);
  }
}

bool EPD_Painter2::anyActive() const {
  const int words = (_config.height + 31) / 32;
  for (int i = 0; i < words; i++) {
    if (_rowMask[i].load(std::memory_order_relaxed)) return true;
  }
  return false;
}

EPD_Painter2::Stats EPD_Painter2::getStats() {
  Stats s;
  s.frames     = _st_frames.load(std::memory_order_relaxed);
  s.lastTickUs = _st_lastUs.load(std::memory_order_relaxed);
  s.maxTickUs  = _st_maxUs.load(std::memory_order_relaxed);
  s.activeRows = _st_activeRows.load(std::memory_order_relaxed);
  s.pausedRows = _st_pausedRows.load(std::memory_order_relaxed);
  s.maintPulses = _st_maintPulses.load(std::memory_order_relaxed);
  s.dcPeakMs   = _st_dcPeak.load(std::memory_order_relaxed);
  s.ghostCells = 0;
  for (int gy = 0; gy < _gridH; gy++)
    for (int gx = 0; gx < _gridW; gx++)
      if (_ghostGrid[gy][gx]) s.ghostCells++;
  s.powered    = _powered;
  return s;
}

// =============================================================================
// rowKernel() — advance every in-flight pixel in one row by one pulse.
//
// Chunk-granular: only 64px chunks marked dirty in _chunkMask[row] touch
// PSRAM. The kernel is PSRAM-bandwidth-bound, so skipping clean chunks is
// what keeps the tick inside its 20ms budget (full-row scans measured
// ~120µs/row; a bouncing box dirties 2-3 of the 15 chunks per row).
//
// Scalar C++ on purpose (v0): this IS the feasibility benchmark for the
// architecture. SIMD comes later if the numbers demand it.
// Returns true if any pixel in the row is still in flight afterwards.
// out must already contain neutral (zero) drive data for clean chunks.
// =============================================================================
#pragma GCC push_options
#pragma GCC optimize("O3")
bool EPD_Painter2::rowKernel(int row, uint8_t* out, uint8_t step) {
  const uint8_t* rowTgt = _target + (size_t)row * _config.width;
  uint8_t*       rowCur = _state  + (size_t)row * _config.width;
  int16_t*       rowDc  = _dc ? _dc + (size_t)row * _config.width : nullptr;
  const uint8_t gain    = _travel_gain;
  const bool boosted    = (gain > 1) && (_pulse_window_us > 0);

  uint16_t chunks = _chunkMask[row].load(std::memory_order_relaxed);
  uint16_t settled = 0;

  for (int c = 0; chunks >> c; c++) {
    if (!((chunks >> c) & 1)) continue;

    const uint8_t* tgt = rowTgt + c * CHUNK_PX;
    uint8_t*       cur = rowCur + c * CHUNK_PX;
    uint8_t*       o8  = out + c * (CHUNK_PX / 4);
    bool chunkActive = false;

    for (int b = 0; b < CHUNK_PX / 4; b++) {
      uint8_t o = 0;
      for (int i = 0; i < 4; i++) {
        const uint8_t cv = cur[i];
        const uint8_t tv = _posLUT[tgt[i] & 0x0F];   // grey → pulse position
        const uint8_t delta = (cv > tv) ? (uint8_t)(cv - tv) : (uint8_t)(tv - cv);
        uint8_t d = 0, nv = cv;
        // How far does this pixel move this tick?
        //  - windowed mode: the global step (1, or gain on coarse frames);
        //    pixels that can't take the whole step sit out (0b00 = hold)
        //    and land on a later fine frame.
        //  - rested mode: per-pixel — travel pixels (≥ gain to go) keep the
        //    field on every tick; landing pixels fire on-beat and take the
        //    off-beat tick as the rest the ink's inertia needs.
        uint8_t s = 0;
        if (delta) {
          if (_rested) {
            const bool travel = boosted && delta >= gain;
            s = travel ? gain : (_fineHold ? 0 : 1);
          } else if (delta >= step && (step > 1 || !_fineHold)) {
            // step>1 = coarse frame: travel pixels fire, others sat out by
            // the delta guard. step==1: fine frame unless this is a rest
            // beat (_fineHold), where everyone holds and the plain scan
            // just terminates the previous frame's pulses.
            s = step;
          }
        }
        if (s) {
          if (cv < tv) { d = DRIVE_DARK;  nv = cv + s; }
          else         { d = DRIVE_LIGHT; nv = cv - s; }
          cur[i] = nv;
          _anyDrive = true;
          if (s > 1 && rowDc) {   // coarse: book the charge/optics gap
            int16_t& a = rowDc[(size_t)(cur - rowCur) + i];
            int32_t v = a + ((d == DRIVE_DARK) ? _coarseCorrMs : -_coarseCorrMs);
            if (v > 32000) v = 32000; else if (v < -32000) v = -32000;
            a = (int16_t)v;
          }
        }
        if (nv != tv) {
          chunkActive = true;
          if (!_rested) {   // classify remaining work → frame scheduling
            const uint8_t rem = (nv > tv) ? (uint8_t)(nv - tv) : (uint8_t)(tv - nv);
            if (boosted && rem >= gain) _needCoarse = true; else _needFine = true;
          }
        }
        o = (o << 2) | d;
      }
      o8[b] = o;
      cur += 4;
      tgt += 4;
    }
    if (!chunkActive) settled |= (1u << c);
  }

  if (settled) {
    // Benign race: a draw between our scan and this clear can lose a wakeup;
    // it self-heals on the next draw to the same chunk.
    _chunkMask[row].fetch_and((uint16_t)~settled, std::memory_order_relaxed);
  }
  return (chunks & ~settled) != 0;
}
#pragma GCC pop_options

// =============================================================================
// tickFrame() — one simulation frame in two strictly separated phases.
//
// Phase 1 (compute, interruptible): advance every active row's pixels and
// write their drive data into _staging. Holds the draw mutex once, so
// beginUpdate()/endUpdate() composites are atomic vs the simulation, and
// PSRAM traffic/contention happens HERE, not during the scan.
//
// Phase 2 (scan-out, gapless): stream all rows back-to-back over DMA with
// nothing between sendRow()s but a memcpy/memset. This must never pause:
// between rows the gate driver holds one row selected with OE live, so a
// stalled scan over-doses that row — the torn-line bug. State bookkeeping
// happened in phase 1, so any stall risk here no longer diverges the ledger,
// and the loop contains no locks or PSRAM writes to stall on.
//
// Returns true if work remains.
// =============================================================================
bool EPD_Painter2::tickFrame() {
  const int H = _config.height;
  bool anyLeft = false;
  int activeRows = 0;

  // Frame scheduling. The beat (_offBeat) alternates every frame. A fine
  // pulse's dose is only calibrated WITH its rest (~13ms of field-off after
  // it — ink inertia resets); when ticks come too fast to provide that rest
  // between consecutive frames, fine pixels fire on-beat only. At 50Hz every
  // frame has the rest built in; at 100Hz fine pixels fire every other
  // frame — the identical 7ms-on/13ms-off pulse train, so one calibration
  // serves both tick rates. Coarse (full-tick) travel takes the off beats.
  const uint32_t tickUs = 1000000UL / _config.tick_hz;
  const bool hadCoarse = _needCoarse, hadFine = _needFine;
  _needCoarse = _needFine = false;
  _rested  = _pulse_window_us >= tickUs;   // pulse = whole tick; rest = next
  _offBeat = !_offBeat;
  const bool boostOn  = (_travel_gain > 1) && (_pulse_window_us > 0);
  const bool needRest = !_rested && _pulse_window_us &&
                        (tickUs < _pulse_window_us + 10000);
  bool coarse, fineGo;
  if (_rested) {          // per-pixel in the kernel: travel rides every tick
    coarse = false;       fineGo = !_offBeat;
  } else if (needRest) {  // fast windowed ticks: fine on-beat, coarse off-beat
    fineGo = !_offBeat && (hadFine || !hadCoarse);
    coarse = boostOn && hadCoarse && !fineGo;
  } else {                // slow windowed ticks (50Hz): alternate on demand
    coarse = boostOn && hadCoarse && (!hadFine || !_lastCoarse);
    fineGo = !coarse;
  }
  _lastCoarse = coarse;
  _fineHold   = !fineGo;
  _anyDrive   = false;
  const uint8_t step = coarse ? _travel_gain : 1;

  // Charge booking for coarse fires: actual field time (one tick) minus the
  // fine-nominal the position ledger assumes (gain × window). Fine pulses
  // book nothing — their charge matches the ledger exactly.
  const int16_t tickMs = (int16_t)(tickUs / 1000);
  const int16_t winMs  = _pulse_window_us ? (int16_t)(_pulse_window_us / 1000)
                                          : tickMs;
  _coarseCorrMs = coarse ? (int16_t)(tickMs - (int16_t)step * winMs) : 0;

  // ---- Phase 1: simulate + stage ----
  memset(_frameActive, 0, sizeof(_frameActive));
  const uint32_t budget = _compute_budget_us;
  xSemaphoreTake(_draw_mtx, portMAX_DELAY);
  int totalActive = 0;
  for (int w = 0; w < (H + 31) / 32; w++)
    totalActive += __builtin_popcount(_rowMask[w].load(std::memory_order_relaxed));
  // Budget clock starts AFTER the mutex: it caps compute, not queueing —
  // otherwise a draw holding the lock eats the budget and the tick stages
  // nothing (every pixel starves mid-journey and the panel goes grey).
  const int64_t p1Start = esp_timer_get_time();
  for (int k = 0; k < H; k++) {
    const int row = (_rowCursor + k) % H;   // rotating start: paused rows go first
    const uint32_t bit = 1u << (row & 31);
    if (!(_rowMask[row >> 5].load(std::memory_order_relaxed) & bit)) continue;

    if (budget && (uint32_t)(esp_timer_get_time() - p1Start) > budget) {
      // Budget exhausted: the remaining active rows pause this tick — they
      // stage nothing, the scan writes them neutral (field off, ink and
      // ledger hold), and the cursor makes them first in line next tick.
      _rowCursor = (uint16_t)row;
      anyLeft = true;
      break;
    }

    activeRows++;
    uint8_t* stage = _staging + (size_t)row * packed_row_bytes;
    memset(stage, 0x00, packed_row_bytes);   // neutral base; kernel fills dirty chunks
    const bool still = rowKernel(row, stage, step);
    _frameActive[row >> 5] |= bit;
    if (!still) {
      _rowMask[row >> 5].fetch_and(~bit, std::memory_order_relaxed);
    } else {
      anyLeft = true;
    }
  }

  // Live deghosting: animation-heavy apps never go idle, so merge a band
  // of maintenance pulses into this frame when pacing and budget allow.
  // The frame type picks the pulse width: !coarse frames bank the chased
  // short pre-charges, coarse frames carry the full-tick scrubs.
  // Throttled to every 3rd frame — deghosting is a background chore and
  // must never cost the animation its tick budget.
  if (_dc && _white_refresh_s && _ghostAny && activeRows &&
      (++_maintDiv % 3) == 0 &&
      esp_timer_get_time() >= _next_refresh_us &&
      (!budget || (uint32_t)(esp_timer_get_time() - p1Start) < budget)) {
    mergeMaintenance(coarse, winMs, tickMs);
  }
  xSemaphoreGive(_draw_mtx);

  // ---- Phase 2: gapless scan-out ----
  const int64_t scanStart = esp_timer_get_time();
  for (int row = 0; row < H; row++) {
    if (_frameActive[row >> 5] & (1u << (row & 31))) {
      memcpy(dma_buffer, _staging + (size_t)row * packed_row_bytes, packed_row_bytes);
    } else {
      memset(dma_buffer, 0x00, packed_row_bytes);
    }
    sendRow(row == 0, row == H - 1);
  }

  // ---- Phase 3 (optional): field-off pass ----
  // The drive scan above only STARTS each pixel's dose — the pixel cap holds
  // the drive voltage after its row is deselected. A second all-neutral scan
  // writes 0V back onto every cap, ending the field ~_pulse_window_us after
  // the drive scan began (per-row gap = one pass duration + wait, uniform
  // across rows since both passes run at the same pace). Without it the
  // field stays on until next tick's scan: pulse width == tick period.
  // Coarse frames skip it on purpose — full-tick pulses ARE the boost; their
  // field is ended by the next tick's scan, or by the idle flush frame.
  if (_pulse_window_us && activeRows && !coarse && !_rested && _anyDrive) {
    // Hybrid wait: hand the coarse milliseconds back to the scheduler, then
    // spin the last ~2ms for microsecond dose precision. A late wakeup only
    // lengthens the field uniformly (no row is selected while we wait), but
    // dose wobble is grey wobble — hence the guard band before the spin.
    const int64_t remain =
        (int64_t)_pulse_window_us - (esp_timer_get_time() - scanStart);
    if (remain > 2000) vTaskDelay(pdMS_TO_TICKS((uint32_t)(remain - 2000) / 1000));
    while ((esp_timer_get_time() - scanStart) < (int64_t)_pulse_window_us) {}
    neutralFrame();
    _flushPending = false;
  } else if (activeRows) {
    // Coarse/rested frames leave full-tick pulses in flight — if the screen
    // settles before another scan runs, the idle flush frame terminates
    // them. A frame that drove nothing (rest beat) already terminated all
    // prior pulses with its all-neutral scan.
    _flushPending = _anyDrive;
  }

  _st_activeRows.store((uint16_t)activeRows, std::memory_order_relaxed);
  _st_pausedRows.store(
      (uint16_t)(totalActive > activeRows ? totalActive - activeRows : 0),
      std::memory_order_relaxed);
  return anyLeft;
}

// Scan one full frame of neutral (0V) onto every pixel cap, ending any field
// left by a previous drive scan. Gapless, like every scan.
void EPD_Painter2::neutralFrame() {
  const int H = _config.height;
  for (int row = 0; row < H; row++) {
    memset(dma_buffer, 0x00, packed_row_bytes);
    sendRow(row == 0, row == H - 1);
  }
}

// Pass complete: cells that received a scrub are one treatment closer to
// clean; re-arm the pass pacing timer.
void EPD_Painter2::finishGhostPass() {
  bool any = false;
  for (int gy = 0; gy < _gridH; gy++) {
    for (int gx = 0; gx < _gridW; gx++) {
      if (_ghostGrid[gy][gx]) {
        if (_cellScrubbed[gy] & (1u << gx)) _ghostGrid[gy][gx]--;
        if (_ghostGrid[gy][gx]) any = true;
      }
    }
    _cellScrubbed[gy] = 0;
  }
  _ghostAny = any;
  _next_refresh_us =
    esp_timer_get_time() + (int64_t)_white_refresh_s * 1000000LL;
}

// =============================================================================
// mergeMaintenance() — live deghosting underneath animation.
//
// Called from tickFrame's phase 1 (mutex held): folds a band of deghost
// pulses into the CURRENT frame's staging, so cells the animation dirties
// get cleaned without waiting for idle. Pulse widths come free from the
// frame type: on !coarse frames the field-off pass chases everything, so
// pre-charge DARK shorts (~window) are banked; on coarse frames pulses run
// the full tick, so that's when accounts are spent as long LIGHT scrubs.
// Mixed workloads alternate frame types, so both halves of the cycle run.
// Maintenance pixels are settled (their staged lanes are 0b00), so OR-ing
// into an already-staged row can't disturb live drive data.
// =============================================================================
void EPD_Painter2::mergeMaintenance(bool coarse, int16_t shortMs, int16_t longMs) {
  const int H = _config.height, W = _config.width;
  _maintSalt = _maintSalt * 1103515245u + 12345u;
  const int64_t t0 = esp_timer_get_time();

  int done = 0;
  for (int n = 0; n < MAINT_BAND; n++, done++) {
    // Hard time cap: this is a background chore — never stretch the tick.
    if ((esp_timer_get_time() - t0) > 1500) break;
    const int row = (_maintRow + n) % H;
    const uint8_t* gRow = _ghostGrid[row / GHOST_CELL];

    bool anyCell = false;
    for (int gx = 0; gx < _gridW; gx++) if (gRow[gx]) { anyCell = true; break; }
    if (!anyCell) continue;

    const uint8_t* st = _state  + (size_t)row * W;
    const uint8_t* tg = _target + (size_t)row * W;
    int16_t*       dc = _dc     + (size_t)row * W;
    uint8_t*    stage = _staging + (size_t)row * packed_row_bytes;
    const uint32_t bit = 1u << (row & 31);
    bool rowStaged = (_frameActive[row >> 5] & bit) != 0;

    for (int xi = 0; xi < W; xi++) {
      if (!gRow[xi / GHOST_CELL]) {              // hop to the next cell
        xi += GHOST_CELL - 1 - (xi % GHOST_CELL);
        continue;
      }
      if (st[xi] != 0 || (tg[xi] & 0x0F) != 0) continue;   // settled whites only
      const uint32_t h =
        ((uint32_t)xi * 2654435761u) ^ ((uint32_t)row * 40503u) ^ _maintSalt;
      if (!(h & 1)) continue;                    // scattered batches
      int16_t a = dc[xi];
      uint8_t d = 0;
      if (coarse) {
        if (a >= longMs - 2) {                   // account full: the scrub
          d = DRIVE_LIGHT;
          a -= longMs;
          _cellScrubbed[row / GHOST_CELL] |= (1u << (xi / GHOST_CELL));
        }
      } else if (a < longMs - 2) {               // bank a chased short pulse
        d = DRIVE_DARK;
        a += shortMs;
      }
      if (!d) continue;
      if (!rowStaged) {
        memset(stage, 0x00, packed_row_bytes);
        _frameActive[row >> 5] |= bit;
        rowStaged = true;
      }
      stage[xi >> 2] |= d << ((3 - (xi & 3)) * 2);
      dc[xi] = a;
      _anyDrive = true;
      _st_maintPulses.fetch_add(1, std::memory_order_relaxed);
    }
  }
  if (!done) return;
  const int prev = _maintRow;
  _maintRow = (_maintRow + done) % H;
  if (_maintRow <= prev) finishGhostPass();      // wrapped: one pass complete
}

// =============================================================================
// maintenanceTick() — idle-time deghosting (pre-charged scrub) + DC trim.
//
// Runs only when nothing is in flight and the rails are up. Scans a band of
// rows per tick and fires one frame: a drive scan chased by a second scan
// that writes 0V to short pulses (~one scan, ~5ms of field) but SUSTAINS
// scrub pulses, which then run until the next tick's scan (or the idle
// flush) terminates them at ~one full tick.
//
// Deghost cycle (zero net charge by construction): a white pixel in a dirty
// cell banks short sub-threshold DARK pulses — one per pass, spread over
// seconds, at the rail where first-pulse stiction is highest (measured:
// 7ms from white moved reflectance ~0.01) — and once its account holds one
// tick's worth, spends it all as a single long LIGHT pulse: the scrub that
// actually pushes ghost particles home. The cycle ends hard against the
// white rail, so even sub-threshold creep is self-healed. A random lottery
// per pass scatters treatment instead of marching it in blocks.
//
// Trim: rail-parked pixels outside dirty cells with |account| > TRIM_AT get
// short opposing pulses — mops up coarse-pulse booking gaps.
// Returns true if anything was fired (caller holds off the idle power-down).
// =============================================================================
bool EPD_Painter2::maintenanceTick() {
  if (!_dc) return false;
  const int H = _config.height, W = _config.width;
  static constexpr int16_t TRIM_AT  = 15;  // ms of imbalance before trimming
  static constexpr int16_t PULSE_MS = 5;   // short pulse: ~one scan of field
  const int16_t disMs = (int16_t)(1000 / _config.tick_hz);  // scrub length

  if (_white_refresh_s && _refreshRow < 0 && _ghostAny &&
      esp_timer_get_time() >= _next_refresh_us)
    _refreshRow = 0;

  const bool pass = (_refreshRow >= 0);
  const int r0 = pass ? _refreshRow : _maintRow;
  const int nrows = pass ? ((H - r0) < MAINT_BAND ? (H - r0) : MAINT_BAND)
                         : MAINT_BAND;

  bool bandFired[MAINT_BAND] = {false};
  bool anySustain = false;
  uint32_t pulses = 0;
  int16_t peak = 0;
  const uint8_t blackPos = _posLUT[15];
  _maintSalt = _maintSalt * 1103515245u + 12345u;

  xSemaphoreTake(_draw_mtx, portMAX_DELAY);
  for (int n = 0; n < nrows; n++) {
    const int row = (r0 + n) % H;
    const uint8_t* st = _state  + (size_t)row * W;
    const uint8_t* tg = _target + (size_t)row * W;
    int16_t*       dc = _dc     + (size_t)row * W;
    uint8_t*    stage = _staging + (size_t)row * packed_row_bytes;
    uint8_t*      sus = _sustain + (size_t)n   * packed_row_bytes;
    const uint8_t* gRow = _ghostGrid[row / GHOST_CELL];
    bool rowFired = false;
    for (int x = 0; x < W; x += 4) {
      uint8_t o = 0, os = 0;
      for (int i = 0; i < 4; i++) {
        const int xi = x + i;
        const uint8_t sv = st[xi];
        uint8_t d = 0;
        bool sustain = false;
        const bool ghostW = (sv == 0) && (tg[xi] & 0x0F) == 0 && gRow[xi / GHOST_CELL];
        if (ghostW) {
          if (pass) {
            const uint32_t h =
              ((uint32_t)xi * 2654435761u) ^ ((uint32_t)row * 40503u) ^ _maintSalt;
            if (h & 1) {                       // scattered batch, ~half per pass
              int16_t a = dc[xi];
              if (a >= disMs - 2) {            // account full: the scrub
                d = DRIVE_LIGHT; sustain = true; anySustain = true;
                a -= disMs;
                _cellScrubbed[row / GHOST_CELL] |= (1u << (xi / GHOST_CELL));
              } else {                         // bank another crumb of charge
                d = DRIVE_DARK; a += PULSE_MS;
              }
              dc[xi] = a; pulses++; rowFired = true;
              const int16_t m = (a < 0) ? -a : a;
              if (m > peak) peak = m;
            }
          }
          // Between passes / off-lottery: leave in-cycle accounts alone.
        } else if (sv == 0 || sv >= blackPos) {  // rail-parked, clean cell
          int16_t a = dc[xi];
          if (a >= TRIM_AT)       { d = DRIVE_LIGHT; a -= PULSE_MS; }
          else if (a <= -TRIM_AT) { d = DRIVE_DARK;  a += PULSE_MS; }
          if (d) { dc[xi] = a; pulses++; rowFired = true; }
          const int16_t m = (a < 0) ? -a : a;
          if (m > peak) peak = m;
        }
        o  = (o  << 2) | d;
        os = (os << 2) | (sustain ? DRIVE_LIGHT : 0);
      }
      stage[x >> 2] = o;
      sus[x >> 2]   = os;
    }
    bandFired[n] = rowFired;
  }
  xSemaphoreGive(_draw_mtx);

  if (pass) {
    _refreshRow += nrows;
    if (_refreshRow >= H) {
      _refreshRow = -1;
      finishGhostPass();
    }
  } else {
    _maintRow = (_maintRow + nrows) % H;
  }

  _st_dcPeak.store(peak, std::memory_order_relaxed);
  if (!pulses) return false;
  _st_maintPulses.fetch_add(pulses, std::memory_order_relaxed);

  // Drive scan, then the chase scan: shorts cut at ~one scan of field,
  // scrub pulses re-written (sustained) — the NEXT scan ends them at ~one
  // tick. If maintenance goes quiet before then, the idle flush does it.
  for (int row = 0; row < H; row++) {
    int n = row - r0; if (n < 0) n += H;
    if (n < nrows && bandFired[n]) {
      memcpy(dma_buffer, _staging + (size_t)row * packed_row_bytes, packed_row_bytes);
    } else {
      memset(dma_buffer, 0x00, packed_row_bytes);
    }
    sendRow(row == 0, row == H - 1);
  }
  for (int row = 0; row < H; row++) {
    int n = row - r0; if (n < 0) n += H;
    if (n < nrows && bandFired[n]) {
      memcpy(dma_buffer, _sustain + (size_t)n * packed_row_bytes, packed_row_bytes);
    } else {
      memset(dma_buffer, 0x00, packed_row_bytes);
    }
    sendRow(row == 0, row == H - 1);
  }
  if (anySustain) _flushPending = true;
  return true;
}

// =============================================================================
// Tick task — fixed-rate simulation heartbeat.
// =============================================================================
void EPD_Painter2::_tick_task_entry(void *arg) {
  static_cast<EPD_Painter2 *>(arg)->_tick_task_body();
  vTaskDelete(nullptr);
}

void EPD_Painter2::_tick_task_body() {
  const TickType_t period = pdMS_TO_TICKS(1000 / _config.tick_hz);
  TickType_t lastWake = xTaskGetTickCount();
  const uint16_t idleOffTicks = _config.tick_hz;   // power off after ~1s idle

  while (_tick_running) {
    vTaskDelayUntil(&lastWake, period ? period : 1);

    if (anyActive()) {
      if (!_powered) powerOn();
      _idle_ticks = 0;

      const int64_t t0 = esp_timer_get_time();
      tickFrame();
      const uint32_t us = (uint32_t)(esp_timer_get_time() - t0);

      _st_lastUs.store(us, std::memory_order_relaxed);
      if (us > _st_maxUs.load(std::memory_order_relaxed))
        _st_maxUs.store(us, std::memory_order_relaxed);
      _st_frames.fetch_add(1, std::memory_order_relaxed);

      // If the frame overran the tick period, give lower-priority tasks air.
      if (us > (uint32_t)(1000000 / _config.tick_hz)) vTaskDelay(1);
    } else {   // nothing in flight
      // Wake the rails if deghost work is pending on a sleeping screen —
      // maintenance is the one thing allowed to power up unasked.
      if (!_powered && _white_refresh_s && _dc && _ghostAny &&
          esp_timer_get_time() >= _next_refresh_us) {
        powerOn();
      }
      if (_powered) {
        if (_flushPending) {
          // Terminate the final full-tick pulses of the transition: with no
          // further scans coming, the caps would otherwise hold the last
          // drive voltage until power-off — overdosing the last pulse.
          neutralFrame();
          _flushPending = false;
        }
        if (maintenanceTick()) {
          _idle_ticks = 0;               // stay powered while trimming
        } else if (++_idle_ticks >= idleOffTicks) {
          powerOff();
        }
      }
    }

    // Vertical blank: the frame (or idle heartbeat) is done; targets have
    // been sampled and won't be looked at again until next period. Fire the
    // user hooks — anything drawn from here on is picked up whole.
    if (_frame_cb) _frame_cb(_frame_cb_arg);
    if (_vbl_sem) xSemaphoreGive(_vbl_sem);
  }
}
