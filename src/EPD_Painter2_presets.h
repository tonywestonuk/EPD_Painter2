#pragma once

// =============================================================================
// EPD_Painter2 board presets.
//
// Optional explicit selection via #define before including the library:
//   #define EPD_PAINTER2_PRESET_M5PAPER_S3
//   #define EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS
// If none is defined, EPD_PAINTER2_PRESET_AUTO is enabled and begin() probes
// the supported boards at runtime over I2C.
//
// v0 supports direct-GPIO boards only (no shift-register boards — those need
// the assembly SR driver from EPD_Painter).
// =============================================================================

#if defined(EPD_PAINTER2_PRESET_M5PAPER_S3) || defined(EPD_PAINTER2_PRESET_AUTO)
    inline EPD_Painter2::Config EPD2_M5PAPER_S3_PRESET = {
        .width    = 960,
        .height   = 540,
        .pin_pwr  = 46,
        .pin_sph  = 13,
        .pin_oe   = 45,
        .pin_cl   = 16,
        .pin_spv  = 17,
        .pin_ckv  = 18,
        .pin_le   = 15,
        .data_pins = { 6, 14, 7, 12, 9, 11, 8, 10 },
        .i2c = {
            .sda = 41,
            .scl = 42,
            .freq = 100000
        },
    };
#endif

#if defined(EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS) || defined(EPD_PAINTER2_PRESET_AUTO)
    inline EPD_Painter2::Config EPD2_LILYGO_T5_S3_GPS_PRESET = {
        .width    = 960,
        .height   = 540,
        .pin_pwr  = -1,  // managed via TPS65185 over I2C (powerctl)
        .pin_sph  = 41,
        .pin_oe   = -1,  // managed via PCA9555 over I2C (powerctl)
        .pin_cl   = 4,
        .pin_spv  = 45,
        .pin_ckv  = 48,
        .pin_le   = 42,
        .data_pins = { 5, 6, 7, 15, 16, 17, 18, 8 },
        .i2c = {
            .sda = 39,
            .scl = 40,
            .freq = 100000
        },
        .power = {
            .pca_addr = 0x20,
            .tps_addr = 0x68,
        },
    };
#endif

#if defined(EPD_PAINTER2_PRESET_M5PAPER_S3)
    inline EPD_Painter2::Config& EPD_PAINTER2_PRESET = EPD2_M5PAPER_S3_PRESET;
#elif defined(EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS)
    inline EPD_Painter2::Config& EPD_PAINTER2_PRESET = EPD2_LILYGO_T5_S3_GPS_PRESET;
#elif defined(EPD_PAINTER2_PRESET_AUTO)
    inline EPD_Painter2::Config EPD_PAINTER2_PRESET = {
        .width    = 960,
        .height   = 540,
        .data_pins = { -1, -1, -1, -1, -1, -1, -1, -1 },
    };

    inline EPD_Painter2::ProbeSettings EPD2_Probe[] = {
        // Boards with unique I2C addresses go first.
        { &EPD2_LILYGO_T5_S3_GPS_PRESET, 39, 40, 0x20, false }, // PCA9555 — unique addr
        { &EPD2_M5PAPER_S3_PRESET,       41, 42, 0x51, false }, // BM8563 RTC on bus 41/42
    };
#else
    #error "No EPD_PAINTER2_PRESET defined; define a board preset or EPD_PAINTER2_PRESET_AUTO"
#endif
