#pragma once

#include <Arduino.h>

// One CMD1/CMD2 pair found to return actual data during a command scan
// (see DiscoveryScanner::startCommandScan()) - "actual data" meaning a
// valid, CRC-checked reply with at least one byte between <DLE><STX> and
// <DLE><ETX>. Replies with zero data bytes are dismissed immediately and
// never stored - per the exact rule requested: a real but "nothing here"
// response looks identical to an invalid command at this protocol layer,
// so it's not worth keeping.
struct DiscoveredCommand
{
  uint8_t cmd1;
  uint8_t cmd2;
  uint8_t dataLen;
  static constexpr uint8_t kMaxData = 32; // display/study only, not a real protocol limit
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
class DiscoveryScanner
{
public:
  enum class Mode : uint8_t
  {
    kNone,
    kDevIdScan,
    kCommandScan
  };

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

  // Extra attempts for a CMD1/CMD2 pair whose first attempt failed
  // ambiguously (anything but a clean EOT "no such command" answer)
  // before giving up on it - see stepCommandScan()'s comment for why one
  // attempt isn't enough to trust a "not found" result.
  static constexpr uint8_t kMaxRetries = 2;

  // Mounts the LittleFS partition (formatting it on first-ever use) and
  // reloads whatever devIds_/commands_ a previous run of this scanner
  // (possibly cut short by a reset - see stepDevIdScan()/stepCommandScan()'s
  // persist* calls) already found for the currently configured equipment
  // model, so a brownout or crash mid-scan doesn't erase results that were
  // already in hand. Call once from setup(), after Settings.begin() (needs
  // Settings.model() to pick which persisted command capture to reload).
  // No-op (beyond mounting) if nothing was persisted yet.
  void begin();

  // Starts (or restarts, discarding old results - including any persisted
  // from a previous interrupted run) scanning every DEVID in range with a
  // protocol-mandated ECHO poll (cmd1=0x20, cmd2=0x20) at the given station
  // address.
  void startDevIdScan(uint8_t addr);

  // Starts (or restarts, discarding old results - including any persisted
  // from a previous interrupted run) scanning every CMD1 (0x00-0xFF) x
  // CMD2 (even values only, 0x00-0xFE - odd would be SELECT/write, not
  // safe to brute-force blind) against one specific devId/addr.
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
  const DiscoveredCommand &commandAt(uint8_t i) const { return commands_[i]; }
  uint8_t commandScanDevId() const { return commandScanDevId_; }

  // Everything below reads straight from LittleFS rather than this
  // object's live devIds_/commands_/etc, and doesn't touch that live
  // state either - so DryerWebServer's export endpoint (dumping every
  // model's capture, not just whichever one happens to be loaded for the
  // currently configured equipment) can't disturb an in-progress or
  // just-completed scan.

  // Every equipment model with a persisted command capture on flash,
  // written into outModels[0..return value). Names come from
  // DeviceSettings::kDryerModels/kCrystallizerModels/kEthernetModels (see
  // DeviceSettings.h), so a caller-supplied maxCount matching
  // kDryerModelCount+kCrystallizerModelCount+kEthernetModelCount always
  // has room for all of them.
  uint8_t listPersistedModels(String *outModels, uint8_t maxCount) const;

  // Reads one model's persisted command capture, up to maxOut entries.
  // Returns false (outCount untouched) if that model has never had a
  // command scan persisted.
  bool readPersistedCommands(const String &model, uint8_t &devId, DiscoveredCommand *out, uint8_t maxOut,
                              uint8_t &outCount) const;

  // Elapsed time of the scan currently running, or of the last one that
  // finished (naturally or via stop()) - kept around specifically so the
  // dashboard still has something to show (which DEVID, how long it took)
  // once a long scan ends, instead of everything reverting to "no scan
  // running" with no trace of what just happened.
  uint32_t liveElapsedMs() const { return isRunning() ? millis() - scanStartMs_ : 0; }
  bool devIdScanHasResult() const { return devIdScanHasResult_; }
  uint32_t devIdScanDurationMs() const { return devIdScanDurationMs_; }
  bool commandScanHasResult() const { return commandScanHasResult_; }
  uint32_t commandScanDurationMs() const { return commandScanDurationMs_; }

private:
  Mode mode_ = Mode::kNone;
  uint8_t addr_ = 0x20;
  uint8_t commandScanDevId_ = 0x22;
  uint32_t progressCurrent_ = 0;
  uint32_t progressTotal_ = 0;

  uint32_t scanStartMs_ = 0;
  bool devIdScanHasResult_ = false;
  uint32_t devIdScanDurationMs_ = 0;
  bool commandScanHasResult_ = false;
  uint32_t commandScanDurationMs_ = 0;

  uint8_t devIds_[kMaxDevIds] = {0};
  uint8_t devIdCount_ = 0;

  DiscoveredCommand commands_[kMaxCommands];
  uint8_t commandCount_ = 0;

  uint16_t devIdCursor_ = 0;   // 0..kMaxDevIds-1, maps to devId kDevIdFirst+cursor
  uint16_t cmd1Cursor_ = 0;    // 0..255
  uint16_t cmd2Cursor_ = 0;    // 0,2,4,...,254
  uint8_t cmd2RetryCount_ = 0; // extra attempts made on the current pair so far

  // Which model commands_/commandCount_/commandScanDevId_ currently
  // reflect - set by begin()/startCommandScan(), checked by
  // syncCommandsToCurrentModel(). Without this, switching the equipment
  // model live from the dashboard (no reboot) left commands_ showing
  // whatever the *previous* model had, mislabeled under the new one -
  // reported as "I shouldn't have anything to export yet, and the
  // commands are NOT the same" after switching to a never-scanned model.
  String loadedCommandsModel_;

  void stepDevIdScan();
  void stepCommandScan();
  void finishScan(); // records elapsed time for mode_ before it's reset to kNone

  // Called every task() while idle (mode_ == kNone) - if the configured
  // equipment model has changed since commands_ was last loaded (a live
  // dashboard change, not a reboot - begin() already covers that case),
  // reloads commands_ from that model's persisted capture, or clears it
  // if that model has never had one. Deliberately not hooked into every
  // Settings.setModel()/setModelTypeCode() call site instead (there are
  // several, across main.cpp/main_atom_node.cpp/DryerWebServer.cpp) -
  // polling from task() self-corrects the same way main_atom_node.cpp's
  // updateLed() does, with one place to maintain instead of several.
  void syncCommandsToCurrentModel();
};

extern DiscoveryScanner Discovery;
