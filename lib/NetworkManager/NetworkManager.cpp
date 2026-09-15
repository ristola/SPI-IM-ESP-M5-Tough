#include "NetworkManager.h"

NetworkManager Network;

#if defined(BOARD_ATOMS3_POE)

// AtomS3 + Atomic PoE Base (W5500) - see pins.h's ETH_SCK/ETH_MISO/
// ETH_MOSI/ETH_CS. ESP-NOW/RTS-NOW still needs the WiFi radio powered on
// (WiFi.mode(WIFI_STA), done by EspNowLink::begin()) to pick a channel and
// transmit/receive, but this board never associates with any WiFi AP -
// every IP-level thing (Modbus TCP, the web dashboard) rides the wired
// Ethernet link instead. Uses M5Stack's M5_Ethernet (a polling-based fork
// of the classic Arduino Ethernet.h API) rather than the ESP32 core's
// native ETH.h - confirmed on real hardware (see pins.h's comment) that
// this board's W5500 has no RST/INT wired at all, and ETH.h's beginSPI()
// rejects boards without those.
#include <M5_Ethernet.h>
#include <SPI.h>

#include "pins.h"

namespace {
uint8_t s_mac[6];  // filled from the chip's own WiFi MAC in begin() - unique per board, no per-unit config needed
constexpr uint32_t kMaintainIntervalMs = 1000;
uint32_t s_lastMaintainMs = 0;
}  // namespace

bool NetworkManager::begin(const char* ssid, const char* password, uint32_t timeoutMs) {
  (void)ssid;
  (void)password;

  // Deliberately checked in stages, with a Serial print after each one,
  // rather than going straight to Ethernet.begin(mac) (which does DHCP) -
  // that call blocks internally with its own long-ish timeout, and doing
  // it before confirming the W5500 chip even responds over SPI at all (or
  // that a cable is actually linked up) made an early real-hardware test
  // of this board look like a total hang, with zero Serial output the
  // whole time to tell a wiring problem apart from "no cable plugged in
  // yet, just waiting on DHCP".
  Serial.println("Ethernet: starting SPI + W5500 init...");
  WiFi.macAddress(s_mac);
  SPI.begin(Pins::ETH_SCK, Pins::ETH_MISO, Pins::ETH_MOSI, -1);
  Ethernet.init(Pins::ETH_CS);

  EthernetHardwareStatus hw = Ethernet.hardwareStatus();
  Serial.printf("Ethernet: hardwareStatus=%d (%s)\n", static_cast<int>(hw),
                hw == EthernetW5500 ? "W5500 detected" : "NOT detected - check wiring/seating");
  if (hw != EthernetW5500) {
    _lastAttemptMs = millis();
    return false;
  }

  uint32_t start = millis();
  EthernetLinkStatus link;
  do {
    link = Ethernet.linkStatus();
    delay(100);
  } while (link != LinkON && millis() - start < timeoutMs);
  Serial.printf("Ethernet: linkStatus=%d (%s) after %lums\n", static_cast<int>(link),
                link == LinkON ? "LinkON" : "not up - check cable", static_cast<unsigned long>(millis() - start));
  if (link != LinkON) {
    _lastAttemptMs = millis();
    return false;
  }

  Serial.println("Ethernet: link up, starting DHCP...");
  bool dhcpOk = Ethernet.begin(s_mac) != 0;
  Serial.printf("Ethernet: DHCP %s\n", dhcpOk ? "OK" : "FAILED");

  _lastAttemptMs = millis();
  return isConnected();
}

bool NetworkManager::isConnected() const {
  return Ethernet.linkStatus() == LinkON && Ethernet.localIP() != IPAddress(0, 0, 0, 0);
}

IPAddress NetworkManager::localIP() const { return Ethernet.localIP(); }

void NetworkManager::loop() {
  if (millis() - s_lastMaintainMs < kMaintainIntervalMs) return;
  s_lastMaintainMs = millis();
  Ethernet.maintain();  // renews the DHCP lease as needed; no-op otherwise
}

#else

bool NetworkManager::begin(const char* ssid, const char* password, uint32_t timeoutMs) {
  _ssid = ssid;
  _password = password;

  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid.c_str(), _password.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
  }

  _lastAttemptMs = millis();
  return isConnected();
}

bool NetworkManager::isConnected() const { return WiFi.status() == WL_CONNECTED; }
IPAddress NetworkManager::localIP() const { return WiFi.localIP(); }

void NetworkManager::loop() {
  if (isConnected()) return;
  if (millis() - _lastAttemptMs < kRetryIntervalMs) return;

  _lastAttemptMs = millis();
  WiFi.disconnect();
  WiFi.begin(_ssid.c_str(), _password.c_str());
}

#endif
