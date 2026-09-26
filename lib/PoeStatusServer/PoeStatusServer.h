#pragma once

// Read-only HTTP status page for the AtomS3 + Atomic PoE Base board
// (BOARD_ATOMS3_POE). Primary IP connectivity is Ethernet (M5_Ethernet/
// W5500) - see NetworkManager.cpp - with a WiFi fallback attempted
// separately in main_atom_node.cpp's setup() (parallel to, not instead
// of, Ethernet). Serves over BOTH independently when both are up:
// Ethernet's WebServer.h (what DryerWebServer.cpp uses) is backed by
// WiFiServer/WiFiClient, ESP32's native WiFi-based lwIP stack - a
// completely different, incompatible stack from the W5500's own hardware
// TCP/IP implementation, so this uses M5_Ethernet's EthernetServer/
// EthernetClient AND WiFiServer/WiFiClient directly, side by side, with
// hand-rolled request parsing (see PoeStatusServer.cpp) - the Ethernet
// side follows the same pattern M5Stack's own M5-Ethernet WebServer.ino
// example uses.
//
// Deliberately NOT a second RTS-NOW gateway: ESP-NOW only allows one
// esp_now_register_recv_cb per chip (already owned by rtsnow_node.cpp,
// via EspNowLink::begin()), and this board must keep its existing "node"
// role - reporting into and remotely flashable by the real gateway. This
// is read-only diagnostics only: this node's own health (IP, uptime,
// firmware version, free heap) plus a live view of nearby mesh devices,
// built entirely from data rtsnow_node.cpp already collects for its own
// neighbor-discovery/relay feature (see rtsnowNodePeers()) - no
// provisioning, no routing, no writes.
class PoeStatusServer
{
public:
    void begin();

    // Non-blocking - call every loop(). Mirrors DryerWebServer's
    // begin()/handleClient() shape only, not its implementation - see
    // PoeStatusServer.cpp for why WebServer.h's handleClient() can't be
    // reused here.
    void handleClient();
};

extern PoeStatusServer PoeStatus;
