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
bool EPD_Painter2::rowKernel(int row, uint8_t* out) {
  const uint8_t* rowTgt = _target + (size_t)row * _config.width;
  uint8_t*       rowCur = _state  + (size_t)row * _config.width;

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
        uint8_t d = 0;
        if (cv < tv)      { d = DRIVE_DARK;  cur[i] = cv + 1; chunkActive = true; }
        else if (cv > tv) { d = DRIVE_LIGHT; cur[i] = cv - 1; chunkActive = true; }
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

// =============================================================================
// tickFrame() — one simulation frame: drive every row once (active rows get
// their pulse, inactive rows get neutral). Returns true if work remains.
// =============================================================================
bool EPD_Painter2::tickFrame() {
  const int H = _config.height;
  bool anyLeft = false;
  int activeRows = 0;

  for (int row = 0; row < H; row++) {
    const uint32_t bit = 1u << (row & 31);
    const bool rowActive =
      (_rowMask[row >> 5].load(std::memory_order_relaxed) & bit) != 0;

    if (rowActive) {
      activeRows++;
      memset(dma_buffer, 0x00, packed_row_bytes);   // neutral base; kernel fills dirty chunks
      // Hold the draw mutex per row: an open beginUpdate()/endUpdate() block
      // pauses the scan at this row boundary, so composite draws are atomic
      // with respect to the kernel and no intermediate target state is read.
      xSemaphoreTake(_draw_mtx, portMAX_DELAY);
      const bool still = rowKernel(row, dma_buffer);
      xSemaphoreGive(_draw_mtx);
      if (!still) {
        // Benign race: a draw between our scan and this clear can lose a
        // wakeup; it self-heals on the next draw to this row.
        _rowMask[row >> 5].fetch_and(~bit, std::memory_order_relaxed);
      } else {
        anyLeft = true;
      }
    } else {
      memset(dma_buffer, 0x00, packed_row_bytes);
    }
    sendRow(row == 0, row == H - 1);
  }

  _st_activeRows.store((uint16_t)activeRows, std::memory_order_relaxed);
  return anyLeft;
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
    } else if (_powered) {
      if (++_idle_ticks >= idleOffTicks) powerOff();
    }
  }
}
