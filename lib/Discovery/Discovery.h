#pragma once

#include <Arduino.h>

// One CMD1/CMD2 pair found to return actual data during a command scan
// (see DiscoveryScanner::startCommandScan()) - "actual data" meaning a
// valid, CRC-checked reply with at least one byte between <DLE><STX> and
// <DLE><ETX>. Replies with zero data bytes are dismissed immediately and
// never stored - per the exact rule requested: a real but "nothing here"
// response looks identical to an invalid command at this protocol layer,
// so it's not worth keeping.
struct DiscoveredCommand {
  uint8_t cmd1;
  uint8_t cmd2;
  uint8_t dataLen;
  static constexpr uint8_t kMaxData = 32;  // display/study only, not a real protocol limit
  uint8_t data[kMaxData];
};

// Brute-force SPI-CCP discovery, run incrementally (one attempt per
// task() call, called every loop()) rather than as one long blocking
// call - a full CMD1/CMD2 scan is tens of thousands of attempts, and
// blocking loop() for that long would stall ArduinoOTA/Modbus TCP/ESP-NOW
// for the whole scan and risk a watchdog reset. Both scan phases use a
// short per-attempt timeout (see kScanTimeoutMs) - real replies observed
// so far arrive quickly, and a non-existent DEVID/command otherwise costs
// the full timeout with nothing to show for it, so keeping this short is
// what makes brute-forcing the full address/command space tractable at
// all.
class DiscoveryScanner {
 public:
  enum class Mode : uint8_t { kNone, kDevIdScan, kCommandScan };

  // 50ms was tried first and missed a device confirmed to be present and
  // responsive under normal 1000ms-timeout polling (the crystallizer at
  // DEVID 0x5C) - its actual reply latency (device processing time, not
  // just wire transmission time) apparently exceeds that. 150ms gives
  // real replies much more margin while still keeping a full DEVID scan
  // under a minute and a full CMD1xCMD2 scan around ~82 minutes.
  static constexpr uint32_t kScanTimeoutMs = 150;

  // DEVID range per the user's own spec: 0x20-0xFF (224 values) - below
  // 0x20 overlaps the protocol's own control characters (SOH/STX/ETX/EOT/
  // ENQ/DLE/NAK, all <= 0x15), so no real device should use those anyway.
  static constexpr uint16_t kDevIdFirst = 0x20;
  static constexpr uint16_t kDevIdLast = 0xFF;
  static constexpr uint8_t kMaxDevIds = kDevIdLast - kDevIdFirst + 1;

  static constexpr uint8_t kMaxCommands = 128;

  // Starts (or restarts, discarding old results) scanning every DEVID in
  // range with a protocol-mandated ECHO poll (cmd1=0x20, cmd2=0x20) at the
  // given station address.
  void startDevIdScan(uint8_t addr);

  // Starts (or restarts) scanning every CMD1 (0x00-0xFF) x CMD2 (even
  // values only, 0x00-0xFE - odd would be SELECT/write, not safe to
  // brute-force blind) against one specific devId/addr.
  void startCommandScan(uint8_t devId, uint8_t addr);

  void stop();

  // Advances the running scan by exactly one attempt - call every
  // loop(). No-op if nothing is running.
  void task();

  Mode mode() const { return mode_; }
  bool isRunning() const { return mode_ != Mode::kNone; }
  uint32_t progressCurrent() const { return progressCurrent_; }
  uint32_t progressTotal() const { return progressTotal_; }

  uint8_t devIdCount() const { return devIdCount_; }
  uint8_t devIdAt(uint8_t i) const { return devIds_[i]; }

  uint8_t commandCount() const { return commandCount_; }
  const DiscoveredCommand& commandAt(uint8_t i) const { return commands_[i]; }
  uint8_t commandScanDevId() const { return commandScanDevId_; }

 private:
  Mode mode_ = Mode::kNone;
  uint8_t addr_ = 0x20;
  uint8_t commandScanDevId_ = 0x22;
  uint32_t progressCurrent_ = 0;
  uint32_t progressTotal_ = 0;

  uint8_t devIds_[kMaxDevIds] = {0};
  uint8_t devIdCount_ = 0;

  DiscoveredCommand commands_[kMaxCommands];
  uint8_t commandCount_ = 0;

  uint16_t devIdCursor_ = 0;  // 0..kMaxDevIds-1, maps to devId kDevIdFirst+cursor
  uint16_t cmd1Cursor_ = 0;   // 0..255
  uint16_t cmd2Cursor_ = 0;   // 0,2,4,...,254

  void stepDevIdScan();
  void stepCommandScan();
};

extern DiscoveryScanner Discovery;
