// =============================================================================
// EPD2_PowerCtl — TPS65185 + PCA9535/9555 power sequencing over I2C.
// Ported from EPD_Painter's epd_painter_powerctl.cpp (I2C boards only).
// =============================================================================
#include "epd2_powerctl.h"
#include <stdio.h>

EPD2_PowerCtl::EPD2_PowerCtl() {
  _pca_out[0] = 0x00;
  _pca_out[1] = 0x00;
  _pca_cfg[0] = 0xFF;
  _pca_cfg[1] = 0xFF;
}

bool EPD2_PowerCtl::begin(EPD_Painter2::Config cfg) {
  config = cfg;

  // ---- Configure PCA9535: pins 8-13 outputs, 14-15 inputs ----
  for (int pin = 8; pin <= 13; ++pin) {
    if (!pcaPinMode(pin, PCA_OUTPUT)) {
      printf("[PWRCTL2] Failed setting PCA pin %d OUTPUT\n", pin);
      return false;
    }
  }
  if (!pcaPinMode(14, PCA_INPUT)) return false;
  if (!pcaPinMode(15, PCA_INPUT)) return false;

  printf("[PWRCTL2] PCA init OK. CFG1=0x%02X OUT1=0x%02X\n",
         _pca_cfg[1], _pca_out[1]);
  return true;
}

bool EPD2_PowerCtl::powerOn() {
  printf("[PWRCTL2] Power-on sequence...\n");

  if (!pcaWrite(PIN_OE, true))     return false;
  if (!pcaWrite(PIN_MODE, true))   return false;
  if (!pcaWrite(PIN_WAKEUP, true)) return false;
  if (!pcaWrite(PIN_PWRUP, true))  return false;
  if (!pcaWrite(PIN_VCOM, true))   return false;

  EPD2_DELAY_MS(3);

  int timeout = 0;
  bool good = false;
  while (timeout < 400) {
    if (!pcaRead(PIN_PWRGOOD, good)) return false;
    if (good) break;
    EPD2_DELAY_MS(1);
    ++timeout;
  }
  if (!good) {
    printf("[PWRCTL2] PWR_GOOD TIMEOUT\n");
    return false;
  }

  if (!tpsWrite(TPS_UPSEQ0, 0xE1)) return false;
  if (!tpsWrite(TPS_UPSEQ1, 0xAA)) return false;
  if (!tpsWrite(TPS_ENABLE, 0x3F)) return false;

  int to2 = 0;
  uint8_t pg = 0;
  while (to2 < 400) {
    if (!tpsRead(TPS_PG, pg)) return false;
    if ((pg & 0xFA) == 0xFA) break;
    EPD2_DELAY_MS(1);
    ++to2;
  }
  printf("[PWRCTL2] up in %d ms, PG=0x%02X\n", timeout + to2, pg);
  return ((pg & 0xFA) == 0xFA);
}

void EPD2_PowerCtl::powerOff() {
  printf("[PWRCTL2] Power-off\n");

  // Disable the TPS65185 rails first — without this the rails stay hot and
  // charge bleeds through the panel, pulling pixels darker over time.
  tpsWrite(TPS_ENABLE, 0x00);

  pcaWrite(PIN_OE, false);
  pcaWrite(PIN_MODE, false);
  pcaWrite(PIN_PWRUP, false);
  pcaWrite(PIN_VCOM, false);
  EPD2_DELAY_MS(1);
  pcaWrite(PIN_WAKEUP, false);
}

// ---- PCA low-level ----

bool EPD2_PowerCtl::pcaWriteReg(uint8_t reg, uint8_t val) {
#ifdef ARDUINO
  config.i2c.wire->beginTransmission(config.power.pca_addr);
  config.i2c.wire->write(reg);
  config.i2c.wire->write(val);
  return (config.i2c.wire->endTransmission() == 0);
#else
  return false;
#endif
}

bool EPD2_PowerCtl::pcaReadReg(uint8_t reg, uint8_t& val) {
#ifdef ARDUINO
  config.i2c.wire->beginTransmission(config.power.pca_addr);
  config.i2c.wire->write(reg);
  if (config.i2c.wire->endTransmission(false) != 0) return false;
  int n = config.i2c.wire->requestFrom(config.power.pca_addr, 1);
  if (n != 1 || !config.i2c.wire->available()) return false;
  val = config.i2c.wire->read();
  return true;
#else
  return false;
#endif
}

bool EPD2_PowerCtl::pcaPinMode(uint8_t pin, uint8_t mode) {
  uint8_t port = pin / 8;
  uint8_t bit  = pin % 8;
  if (port > 1) return false;

  if (mode == PCA_INPUT) _pca_cfg[port] |=  (1 << bit);
  else                   _pca_cfg[port] &= ~(1 << bit);
  return pcaWriteReg(6 + port, _pca_cfg[port]);
}

bool EPD2_PowerCtl::pcaWrite(uint8_t pin, bool val) {
  uint8_t port = pin / 8;
  uint8_t bit  = pin % 8;
  if (port > 1) return false;

  if (val) _pca_out[port] |=  (1 << bit);
  else     _pca_out[port] &= ~(1 << bit);
  return pcaWriteReg(2 + port, _pca_out[port]);
}

bool EPD2_PowerCtl::pcaRead(uint8_t pin, bool& val) {
  uint8_t port    = pin / 8;
  uint8_t bit     = pin % 8;
  uint8_t reg_val = 0;
  if (port > 1) return false;
  if (!pcaReadReg(port, reg_val)) return false;
  val = ((reg_val >> bit) & 0x01) != 0;
  return true;
}

// ---- TPS low-level ----

bool EPD2_PowerCtl::tpsWrite(uint8_t reg, uint8_t val) {
#ifdef ARDUINO
  config.i2c.wire->beginTransmission(config.power.tps_addr);
  config.i2c.wire->write(reg);
  config.i2c.wire->write(val);
  return (config.i2c.wire->endTransmission() == 0);
#else
  return false;
#endif
}

bool EPD2_PowerCtl::tpsRead(uint8_t reg, uint8_t& val) {
#ifdef ARDUINO
  config.i2c.wire->beginTransmission(config.power.tps_addr);
  config.i2c.wire->write(reg);
  if (config.i2c.wire->endTransmission(false) != 0) return false;
  int n = config.i2c.wire->requestFrom(config.power.tps_addr, 1);
  if (n != 1 || !config.i2c.wire->available()) return false;
  val = config.i2c.wire->read();
  return true;
#else
  return false;
#endif
}
