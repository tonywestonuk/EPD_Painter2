#pragma once

// ---- HAL compatibility: Arduino-ESP32 vs pure ESP-IDF ----
//
// Include this header instead of esp32-hal.h / Arduino.h.
// All EPD2_* macros resolve to the correct platform API.

#ifdef ARDUINO
  #include "esp32-hal.h"   // delay, pinMode, digitalWrite, yield, HIGH/LOW
  #define EPD2_DELAY_MS(ms)    delay(ms)
  #define EPD2_DELAY_US(us)    delayMicroseconds(us)
  #define EPD2_PIN_OUTPUT(p)   pinMode((p), OUTPUT)
  #define EPD2_PIN_HIGH(p)     digitalWrite((p), HIGH)
  #define EPD2_PIN_LOW(p)      digitalWrite((p), LOW)
  #define EPD2_YIELD()         yield()
#else
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "driver/gpio.h"
  #include "esp_rom_sys.h"
  #define EPD2_DELAY_MS(ms)    vTaskDelay(pdMS_TO_TICKS(ms))
  #define EPD2_DELAY_US(us)    esp_rom_delay_us(us)
  #define EPD2_PIN_OUTPUT(p)   gpio_set_direction((gpio_num_t)(p), GPIO_MODE_OUTPUT)
  #define EPD2_PIN_HIGH(p)     gpio_set_level((gpio_num_t)(p), 1)
  #define EPD2_PIN_LOW(p)      gpio_set_level((gpio_num_t)(p), 0)
  #define EPD2_YIELD()         taskYIELD()
#endif
