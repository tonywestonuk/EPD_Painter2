#pragma once
#include <stdint.h>
#include <soc/gpio_reg.h>

// =============================================================================
// EPD2_PinDriver — abstract interface for a single EPD control pin.
//
// Trimmed from EPD_Painter's epd_pin_driver.h: the shift-register drivers
// (74HCT4094D boards) depend on Xtensa assembly and are out of scope for the
// EPD_Painter2 prototype — only direct-GPIO boards are supported.
// =============================================================================
class EPD2_PinDriver {
public:
    virtual void set(bool high) = 0;
    virtual ~EPD2_PinDriver() = default;
};

// =============================================================================
// EPD2_GpioPin — direct GPIO register write, IRAM-safe.
// =============================================================================
class EPD2_GpioPin : public EPD2_PinDriver {
public:
    EPD2_GpioPin() : _pin(0) {}
    explicit EPD2_GpioPin(uint8_t pin) : _pin(pin) {}

    void set(bool high) override {
        if (high) {
            if (_pin < 32) REG_WRITE(GPIO_OUT_W1TS_REG,  1UL << _pin);
            else           REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (_pin - 32));
        } else {
            if (_pin < 32) REG_WRITE(GPIO_OUT_W1TC_REG,  1UL << _pin);
            else           REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (_pin - 32));
        }
    }

private:
    uint8_t _pin;
};

// =============================================================================
// EPD2_PowerDriver — abstract interface for board-level power sequencing.
// =============================================================================
class EPD2_PowerDriver {
public:
    virtual bool powerOn() = 0;
    virtual void powerOff() = 0;
    virtual ~EPD2_PowerDriver() = default;
};

// =============================================================================
// EPD2_GpioPowerDriver — direct GPIO power control (M5PaperS3).
// =============================================================================
#include "epd2_build_opt.h"

class EPD2_GpioPowerDriver : public EPD2_PowerDriver {
public:
    EPD2_GpioPowerDriver(int8_t pin_oe, int8_t pin_pwr)
        : _pin_oe(pin_oe), _pin_pwr(pin_pwr) {}

    bool powerOn() override {
        EPD2_PIN_HIGH(_pin_oe);
        EPD2_DELAY_US(100);
        EPD2_PIN_HIGH(_pin_pwr);
        EPD2_DELAY_US(100);
        return true;
    }

    void powerOff() override {
        EPD2_PIN_LOW(_pin_oe);
        EPD2_DELAY_US(100);
        EPD2_PIN_LOW(_pin_pwr);
        EPD2_DELAY_US(100);
    }

private:
    int8_t _pin_oe;
    int8_t _pin_pwr;
};
