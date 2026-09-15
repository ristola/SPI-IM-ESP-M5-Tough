#include "EspNowLink.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <cstring>

EspNowLink EspNow;
EspNowLink::ReceiveCallback EspNowLink::_onReceive = nullptr;

static const uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Channel to pin the radio to when there's no WiFi AP to inherit a channel
// from. Lets units reach each other over ESP-NOW with no router present at
// all (e.g. a laptop's ESP-NOW dongle talking to a Tough out in the field).
// All units relying on this fallback must agree on the same value.
static constexpr uint8_t kFallbackChannel = 1;

bool EspNowLink::begin() {
  WiFi.mode(WIFI_STA);  // ESP-NOW rides on the WiFi driver even if not joining an AP

  if (WiFi.status() != WL_CONNECTED) {
    esp_wifi_set_channel(kFallbackChannel, WIFI_SECOND_CHAN_NONE);
  }

  if (esp_now_init() != ESP_OK) return false;

  if (!esp_now_is_peer_exist(kBroadcastMac)) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kBroadcastMac, 6);
    peer.channel = 0;  // 0 = use the radio's current channel
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) return false;
  }

  esp_now_register_recv_cb(handleReceive);
  return true;
}

bool EspNowLink::broadcast(const uint8_t* data, size_t len) {
  return esp_now_send(kBroadcastMac, data, len) == ESP_OK;
}

void EspNowLink::handleReceive(const uint8_t* senderMac, const uint8_t* data, int len) {
  if (_onReceive) _onReceive(senderMac, data, static_cast<size_t>(len));
}
