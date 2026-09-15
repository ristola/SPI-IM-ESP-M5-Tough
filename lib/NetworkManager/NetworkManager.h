#pragma once

#include <WiFi.h>

// Manages this node's IP connectivity - WiFi station on every board except
// BOARD_ATOMS3_POE (see NetworkManager.cpp), which uses wired Ethernet
// (M5_Ethernet/W5500) instead. Same public interface either way, so every
// caller (main_atom_node.cpp, DryerWebServer, etc.) works unmodified
// regardless of which board/transport is actually active.
class NetworkManager {
 public:
  // Blocks up to timeoutMs waiting for the initial connection. ssid/
  // password are ignored on the Ethernet build (kept as parameters so
  // every board's setup() can call this the same way).
  bool begin(const char* ssid, const char* password, uint32_t timeoutMs = 15000);

  bool isConnected() const;
  IPAddress localIP() const;

  // Call periodically (e.g. every loop()) to retry a dropped connection
  // (WiFi build) or renew the DHCP lease (Ethernet build) without blocking.
  void loop();

 private:
  String _ssid;
  String _password;
  uint32_t _lastAttemptMs = 0;
  static constexpr uint32_t kRetryIntervalMs = 10000;
};

extern NetworkManager Network;
