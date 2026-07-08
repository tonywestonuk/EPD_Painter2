#pragma once
// Minimal USB HOST boot-protocol mouse for ESP32-S3 (raw IDF usb_host API).
//
// The S3's OTG controller runs in HOST mode: plug a mouse in via a USB-C
// OTG adapter. Note the port then belongs to the mouse — no CDC serial, and
// reflashing means unplugging the mouse (use the boot-button combo if the
// port plays hard to get).
//
// Exposes:
//   usbMouseBegin()          — install host stack + tasks; returns false on error
//   usbMouseConnected()      — a mouse is enumerated and streaming
//   usbMouseTakeDX()/TakeDY()— accumulated movement since last call
//   usbMouseButtons()        — current button bitmap (bit0 = left)

#include <atomic>
#include "usb/usb_host.h"

static std::atomic<int32_t>  g_mouse_dx{0}, g_mouse_dy{0};
static std::atomic<uint8_t>  g_mouse_btn{0};
static std::atomic<bool>     g_mouse_up{false};

static usb_host_client_handle_t g_usb_client = nullptr;
static usb_device_handle_t      g_usb_dev = nullptr;
static usb_transfer_t*          g_usb_xfer = nullptr;
static uint8_t                  g_usb_iface = 0;

static void _mouse_xfer_cb(usb_transfer_t* t) {
  if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes >= 3) {
    g_mouse_btn.store(t->data_buffer[0], std::memory_order_relaxed);
    g_mouse_dx.fetch_add((int8_t)t->data_buffer[1], std::memory_order_relaxed);
    g_mouse_dy.fetch_add((int8_t)t->data_buffer[2], std::memory_order_relaxed);
  }
  if (g_usb_dev) usb_host_transfer_submit(t);   // stream forever
}

static void _mouse_ctrl_cb(usb_transfer_t* t) { usb_host_transfer_free(t); }

// SET_PROTOCOL(boot): make report format the fixed [buttons, dx, dy, ...].
static void _mouse_set_boot_protocol() {
  usb_transfer_t* c = nullptr;
  if (usb_host_transfer_alloc(sizeof(usb_setup_packet_t), 0, &c) != ESP_OK) return;
  usb_setup_packet_t* s = (usb_setup_packet_t*)c->data_buffer;
  s->bmRequestType = 0x21;   // host->device, class, interface
  s->bRequest      = 0x0B;   // SET_PROTOCOL
  s->wValue        = 0;      // 0 = boot protocol
  s->wIndex        = g_usb_iface;
  s->wLength       = 0;
  c->num_bytes       = sizeof(usb_setup_packet_t);
  c->device_handle   = g_usb_dev;
  c->bEndpointAddress = 0;
  c->callback        = _mouse_ctrl_cb;
  usb_host_transfer_submit_control(g_usb_client, c);
}

static void _mouse_open(uint8_t addr) {
  if (usb_host_device_open(g_usb_client, addr, &g_usb_dev) != ESP_OK) return;
  const usb_config_desc_t* cfg = nullptr;
  if (usb_host_get_active_config_descriptor(g_usb_dev, &cfg) != ESP_OK) return;

  // Raw descriptor walk: find a HID (class 3) interface, remember it, and
  // take its interrupt IN endpoint. Boot mice advertise subclass 1 / proto 2
  // but plenty of mice only behave once SET_PROTOCOL(boot) is issued.
  const uint8_t* p = (const uint8_t*)cfg;
  const uint8_t* end = p + cfg->wTotalLength;
  int hidIface = -1;
  uint8_t ep = 0; uint16_t mps = 8;
  while (p + 2 <= end && p[0] >= 2) {
    if (p[1] == 0x04 && p[0] >= 9) {              // interface descriptor
      if (hidIface < 0 && p[5] == 0x03) hidIface = p[2];   // class HID
      else if (hidIface >= 0 && p[2] != hidIface) break;   // past our iface
    } else if (p[1] == 0x05 && p[0] >= 7 && hidIface >= 0) {  // endpoint
      if ((p[2] & 0x80) && (p[3] & 0x03) == 0x03) {            // IN, interrupt
        ep = p[2];
        mps = p[4] | (p[5] << 8);
        break;
      }
    }
    p += p[0];
  }
  if (hidIface < 0 || !ep) return;
  g_usb_iface = (uint8_t)hidIface;

  if (usb_host_interface_claim(g_usb_client, g_usb_dev, g_usb_iface, 0) != ESP_OK)
    return;
  _mouse_set_boot_protocol();

  if (mps > 64) mps = 64;
  if (usb_host_transfer_alloc(mps, 0, &g_usb_xfer) != ESP_OK) return;
  g_usb_xfer->device_handle    = g_usb_dev;
  g_usb_xfer->bEndpointAddress = ep;
  g_usb_xfer->num_bytes        = mps;
  g_usb_xfer->callback         = _mouse_xfer_cb;
  if (usb_host_transfer_submit(g_usb_xfer) == ESP_OK)
    g_mouse_up.store(true, std::memory_order_relaxed);
}

static void _mouse_client_cb(const usb_host_client_event_msg_t* msg, void*) {
  if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
    _mouse_open(msg->new_dev.address);
  } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
    g_mouse_up.store(false, std::memory_order_relaxed);
    if (g_usb_dev) {
      usb_host_interface_release(g_usb_client, g_usb_dev, g_usb_iface);
      usb_host_device_close(g_usb_client, g_usb_dev);
      g_usb_dev = nullptr;
    }
  }
}

static void _usb_lib_task(void*) {
  for (;;) {
    uint32_t flags;
    usb_host_lib_handle_events(portMAX_DELAY, &flags);
    if (flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) usb_host_device_free_all();
  }
}

static void _usb_client_task(void*) {
  for (;;) usb_host_client_handle_events(g_usb_client, portMAX_DELAY);
}

static bool usbMouseBegin() {
  usb_host_config_t cfg = {};
  cfg.skip_phy_setup = false;
  cfg.intr_flags = ESP_INTR_FLAG_LEVEL1;
  if (usb_host_install(&cfg) != ESP_OK) return false;

  usb_host_client_config_t ccfg = {};
  ccfg.is_synchronous = false;
  ccfg.max_num_event_msg = 5;
  ccfg.async.client_event_callback = _mouse_client_cb;
  ccfg.async.callback_arg = nullptr;
  if (usb_host_client_register(&ccfg, &g_usb_client) != ESP_OK) return false;

  xTaskCreatePinnedToCore(_usb_lib_task,    "usbh",  4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(_usb_client_task, "usbhc", 4096, nullptr, 5, nullptr, 1);
  return true;
}

static bool    usbMouseConnected() { return g_mouse_up.load(std::memory_order_relaxed); }
static int32_t usbMouseTakeDX()    { return g_mouse_dx.exchange(0, std::memory_order_relaxed); }
static int32_t usbMouseTakeDY()    { return g_mouse_dy.exchange(0, std::memory_order_relaxed); }
static uint8_t usbMouseButtons()   { return g_mouse_btn.load(std::memory_order_relaxed); }
