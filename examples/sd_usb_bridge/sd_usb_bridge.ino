// M5PaperS3 as a USB SD-card reader. Exposes the TF card raw over USB-C as
// a mass-storage device — the Mac/PC mounts it like a thumb drive. Handy
// for loading /video.epv when no card reader is to hand.
//
// BUILD SETTINGS (Arduino IDE, Tools menu) — REQUIRED:
//   USB Mode:    "USB-OTG (TinyUSB)"   <-- not the default CDC/JTAG!
//   USB CDC On Boot: Enabled           (keeps Serial for the log)
//
// Notes:
//   - No EPD driver in this sketch; the panel keeps its last image.
//   - Raw sector bridge over SPI: expect roughly 0.3-1 MB/s. A 116MB video
//     takes a few minutes. Eject cleanly before unplugging.
//   - To go back to normal sketches afterwards, just flash them with USB
//     Mode back on "Hardware CDC and JTAG". If the port plays hard to get,
//     hold the reset combo for download mode.

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "USB.h"
#include "USBMSC.h"

// M5PaperS3 TF slot
static const int SD_SCK = 39, SD_MISO = 40, SD_MOSI = 38, SD_CS = 47;

USBMSC msc;
static uint32_t sectors = 0, secSize = 512;
static volatile uint32_t rdCount = 0, wrCount = 0;

static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  if (offset != 0 || (bufsize % secSize) != 0) return -1;
  const uint32_t n = bufsize / secSize;
  for (uint32_t i = 0; i < n; i++) {
    if (!SD.writeRAW(buffer + i * secSize, lba + i)) return -1;
  }
  wrCount += n;
  return (int32_t)bufsize;
}

static int32_t onRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  if (offset != 0 || (bufsize % secSize) != 0) return -1;
  const uint32_t n = bufsize / secSize;
  for (uint32_t i = 0; i < n; i++) {
    if (!SD.readRAW((uint8_t*)buffer + i * secSize, lba + i)) return -1;
  }
  rdCount += n;
  return (int32_t)bufsize;
}

static bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
  return true;
}

void setup() {
  Serial.begin(115200);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, 40000000) && !SD.begin(SD_CS, SPI, 25000000)) {
    Serial.println("SD init failed — is a card inserted?");
    while (1) delay(1000);
  }
  sectors = SD.numSectors();
  secSize = SD.sectorSize();

  msc.vendorID("M5");
  msc.productID("PaperS3-SD");
  msc.productRevision("1.0");
  msc.onRead(onRead);
  msc.onWrite(onWrite);
  msc.onStartStop(onStartStop);
  msc.mediaPresent(true);
  msc.begin(sectors, secSize);
  USB.begin();
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last > 2000) {
    last = millis();
    Serial.printf("card %.1f GB, read %lu sectors, wrote %lu sectors\n",
                  (double)sectors * secSize / 1e9,
                  (unsigned long)rdCount, (unsigned long)wrCount);
  }
  delay(50);
}
