#pragma once

#include <Arduino.h>
#include <esp_now.h>

// Thin ESP-NOW wrapper for a small (2-5 unit), single-room star network:
// every dryer-monitor Tough broadcasts its readings, and the gateway Tough
// listens. Broadcast means nodes never need to know the gateway's MAC
// address - no pairing step. All units should join the same WiFi network
// (see NetworkManager) so they land on the same channel automatically;
// ESP-NOW runs alongside that STA connection.
class EspNowLink {
 public:
  using ReceiveCallback = void (*)(const uint8_t* senderMac, const uint8_t* data, size_t len);

  bool begin();

  // Only one callback is supported - fine for this project's single
  // gateway/consumer per device.
  void onReceive(ReceiveCallback callback) { _onReceive = callback; }

  bool broadcast(const uint8_t* data, size_t len);

 private:
  static void handleReceive(const uint8_t* senderMac, const uint8_t* data, int len);
  static ReceiveCallback _onReceive;
};

extern EspNowLink EspNow;
