#pragma once

#include "epd2_build_opt.h"
#include "EPD_Painter2.h"
#include "epd2_pin_driver.h"

// =============================================================================
// EPD2_PowerCtl — TPS65185 PMIC + PCA9535/9555 I/O expander over I2C.
// Used by the LilyGo T5 S3 GPS. Ported from EPD_Painter's epd_painter_powerctl.
// =============================================================================
class EPD2_PowerCtl : public EPD2_PowerDriver {
public:
  EPD2_PowerCtl();

  bool begin(EPD_Painter2::Config config);

  bool powerOn() override;
  void powerOff() override;

private:
  EPD_Painter2::Config config;

  // ---- PCA9535 cached state ----
  uint8_t _pca_out[2];
  uint8_t _pca_cfg[2];

  // ---- PCA9535 logical pin mapping ----
  static constexpr uint8_t PIN_OE      = 8;   // port1 bit0
  static constexpr uint8_t PIN_MODE    = 9;   // port1 bit1
  static constexpr uint8_t PIN_PWRUP   = 11;  // port1 bit3
  static constexpr uint8_t PIN_VCOM    = 12;  // port1 bit4
  static constexpr uint8_t PIN_WAKEUP  = 13;  // port1 bit5
  static constexpr uint8_t PIN_PWRGOOD = 14;  // port1 bit6 input

  static constexpr uint8_t PCA_OUTPUT = 0;
  static constexpr uint8_t PCA_INPUT  = 1;

  // ---- TPS65185 registers ----
  static constexpr uint8_t TPS_ENABLE = 0x01;
  static constexpr uint8_t TPS_VCOM1  = 0x03;
  static constexpr uint8_t TPS_VCOM2  = 0x04;
  static constexpr uint8_t TPS_UPSEQ0 = 0x09;
  static constexpr uint8_t TPS_UPSEQ1 = 0x0A;
  static constexpr uint8_t TPS_PG     = 0x0F;

  bool pcaWriteReg(uint8_t reg, uint8_t val);
  bool pcaReadReg(uint8_t reg, uint8_t& val);
  bool pcaPinMode(uint8_t pin, uint8_t mode);
  bool pcaWrite(uint8_t pin, bool val);
  bool pcaRead(uint8_t pin, bool& val);

  bool tpsWrite(uint8_t reg, uint8_t val);
  bool tpsRead(uint8_t reg, uint8_t& val);
};
