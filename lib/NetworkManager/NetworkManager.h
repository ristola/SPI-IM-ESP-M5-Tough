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

  // Boot-time only, no side effects (no connect/disconnect call) - just
  // sets the flag that begin()/loop() below both honor, so it must be
  // called *before* begin() (see main_atom_node.cpp's setup()). Inert on
  // the Ethernet (BOARD_ATOMS3_POE) build - that transport has no
  // equivalent toggle, see setEnabled()'s own comment for why.
  void initEnabled(bool enabled);

  // Live runtime toggle - unlike initEnabled() above, this immediately
  // acts (disconnects right now, or reconnects right now with whichever
  // credentials begin() last stored) and is what main_atom_node.cpp's
  // onRemoteSetting() calls for the remote "wifiEnabled" kill switch.
  // Needed as its own method rather than a bare WiFi.disconnect() from
  // outside this class, because loop()'s own reconnect watchdog doesn't
  // know about an intentional disable otherwise - confirmed live: the
  // node's WiFi kept coming back on its own within kRetryIntervalMs of
  // being switched off remotely, since loop() just saw a dropped link and
  // "helpfully" reconnected it. Inert on the Ethernet build (no toggle
  // concept - that board's WiFi is a fallback transport, not something a
  // user turns off independently).
  void setEnabled(bool enabled);

  // Call periodically (e.g. every loop()) to retry a dropped connection
  // (WiFi build) or renew the DHCP lease (Ethernet build) without blocking.
  void loop();

  // Plain-string snapshot of begin()'s own staged checks (see
  // NetworkManager.cpp's own comment on why Ethernet is checked in
  // stages) - lets a caller with no serial/USB access at all (e.g.
  // PoeStatusServer, reachable only over a WiFi fallback while Ethernet
  // itself is what's being diagnosed) still see exactly where the
  // connection attempt got to: chip detected? link up? DHCP result?
  // Kept as a plain char buffer rather than exposing M5_Ethernet's own
  // EthernetHardwareStatus/EthernetLinkStatus enum types here, since this
  // header is compiled for every board and those types only exist when
  // M5_Ethernet.h is included (POE builds only). Empty ("") on the WiFi
  // build - nothing there needs this same staged breakdown.
  const char *lastDiagnostic() const { return _lastDiagnostic; }

  // True once begin()'s very first stage (Ethernet: SPI + chip probe)
  // has confirmed a W5500 actually responds - i.e. the PoE Base itself is
  // physically present and talking to this chip, regardless of whether
  // the link ever comes up or DHCP ever succeeds afterward. False on the
  // WiFi build (no such probe exists there) and false until begin() has
  // actually run once on the Ethernet build.
  bool hardwareDetected() const { return _hardwareDetected; }

  // Raw byte read back from the W5500's own VERSIONR register (fixed
  // hardware ID, always 0x04 on a genuine, responding chip) via a
  // hand-rolled SPI transaction - completely bypassing M5_Ethernet's own
  // hardwareDetected()/hardwareStatus() check, as an independent
  // cross-check of the exact same fact using different code. 0xFF or 0x00
  // (not 0x04) means nothing is answering on the SPI bus at the chip-select
  // line this board uses - see NetworkManager.cpp's rawReadW5500Version().
  // Always 0x00 on the WiFi build (no W5500 to probe).
  uint8_t lastRawVersionRead() const { return _lastRawVersionRead; }

  // True if a raw write to the W5500's Mode Register (0x08), read straight
  // back, actually returned 0x08 - isolates whether SPI WRITES take effect
  // at all, independent of lastRawVersionRead() (a pure read, no writes).
  // M5_Ethernet's own chip-detection does exactly this kind of
  // write-then-verify as its first step (softReset()) - if writes don't
  // take effect, that alone explains hardwareDetected() being false even
  // when a raw read proves the chip is there and answering.
  bool lastRawWriteWorked() const { return _lastRawWriteWorked; }

  // Result of replicating M5_Ethernet's own softReset() by hand: MR's
  // value after writing the reset bit (0x80) and polling for up to 20ms -
  // 0x00 means the chip completed its reset in time (matching what the
  // library itself requires); anything else (commonly still 0x80) means
  // it didn't, which alone would make the library's own detection fail
  // regardless of whether plain reads/writes work.
  uint8_t lastSoftResetFinalMr() const { return _lastSoftResetFinalMr; }
  uint32_t lastSoftResetElapsedMs() const { return _lastSoftResetElapsedMs; }

  // Result of replicateIsW5500Exactly() (NetworkManager.cpp) - a
  // byte-for-byte replica of M5_Ethernet's own isW5500(), run inside one
  // continuous SPI transaction with zero gaps between steps, unlike the
  // other raw*() tests above (each opens/closes its own transaction).
  // 0 = every step passed (chip should have been detected); 1-4 = which
  // specific step first failed (1=softReset, 2/3/4=the three MR write-
  // verify steps); 5 = every step passed but VERSIONR still wasn't 0x04.
  uint8_t lastIsW5500ReplicaResult() const { return _lastIsW5500ReplicaResult; }

 private:
  String _ssid;
  String _password;
  bool _enabled = true;
  uint32_t _lastAttemptMs = 0;
  static constexpr uint32_t kRetryIntervalMs = 10000;
  char _lastDiagnostic[192] = "";
  bool _hardwareDetected = false;
  uint8_t _lastRawVersionRead = 0x00;
  bool _lastRawWriteWorked = false;
  uint8_t _lastSoftResetFinalMr = 0xFF;
  uint32_t _lastSoftResetElapsedMs = 0;
  uint8_t _lastIsW5500ReplicaResult = 0xFF;
};

extern NetworkManager Network;
