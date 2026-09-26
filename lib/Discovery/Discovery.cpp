#include "Discovery.h"

#include <cstring>

#include <LittleFS.h>

#include "DeviceSettings.h"
#include "SpiCcp.h"

DiscoveryScanner Discovery;

namespace
{
  // Persisted on LittleFS (the "spiffs"-labeled 1.5MB data partition every
  // board in this project already has, per platformio.ini's board default
  // partition table - previously unused), not NVS/Preferences: NVS here is
  // a single 20KB partition shared with WiFi credentials and every other
  // device setting, too tight to also hold a growing library of per-model
  // command captures. A real filesystem also lets each equipment model
  // keep its own file rather than one slot that the next scan overwrites -
  // see commandsPath()'s comment.
  //
  // Deliberately raw binary, not JSON/text: this data is never read by
  // anything other than this same struct layout (the dashboard gets its
  // own JSON built fresh in DryerWebServer), so there's nothing to gain
  // from a text format and a parser dependency to gain it with.
  constexpr const char *kDiscoveryDir = "/discovery";
  constexpr const char *kDevIdsPath = "/discovery/devids.bin";

  bool ensureDir()
  {
    if (LittleFS.exists(kDiscoveryDir))
      return true;
    return LittleFS.mkdir(kDiscoveryDir);
  }

  void persistDevIds(const uint8_t *devIds, uint8_t count)
  {
    if (!ensureDir())
      return;
    File f = LittleFS.open(kDevIdsPath, "w");
    if (!f)
      return;
    f.write(devIds, count);
    f.close();
  }

  void clearPersistedDevIds() { LittleFS.remove(kDevIdsPath); }

  // One file per equipment model (e.g. "/discovery/cmds_FN-XTLR.bin",
  // "/discovery/cmds_FD.bin") - model() is always one of
  // DeviceSettings::kDryerModels/kCrystallizerModels/kEthernetModels (see
  // DeviceSettings.h), all plain filename-safe strings, so no sanitizing
  // needed. Keeping a capture per model - rather than one shared slot the
  // next command scan overwrites - means scanning a dryer today doesn't
  // erase what was already learned about a crystallizer scanned last
  // month, even though both go through this same node over time.
  String commandsPath(const String &model) { return String(kDiscoveryDir) + "/cmds_" + model + ".bin"; }

  // File layout: [devId byte][DiscoveredCommand...] - devId is the one
  // piece of metadata that isn't already implied by the struct array
  // itself or the model-keyed filename.
  void persistCommands(const DiscoveredCommand *commands, uint8_t count, uint8_t devId, const String &model)
  {
    if (!ensureDir())
      return;
    File f = LittleFS.open(commandsPath(model), "w");
    if (!f)
      return;
    f.write(devId);
    f.write(reinterpret_cast<const uint8_t *>(commands), count * sizeof(DiscoveredCommand));
    f.close();
  }

  void clearPersistedCommands(const String &model) { LittleFS.remove(commandsPath(model)); }
} // namespace

void DiscoveryScanner::begin()
{
  // formatOnFail=true: the very first boot after flashing this feature,
  // the "spiffs" partition has never been formatted as LittleFS (or holds
  // leftover data from something else entirely) - format it rather than
  // fail to mount and silently lose persistence forever.
  if (!LittleFS.begin(/*formatOnFail=*/true))
  {
    Serial.println("Discovery: LittleFS mount failed - scan results won't survive a reset");
    return;
  }

  File devIdsFile = LittleFS.open(kDevIdsPath, "r");
  if (devIdsFile)
  {
    size_t n = devIdsFile.read(devIds_, kMaxDevIds);
    devIdsFile.close();
    devIdCount_ = static_cast<uint8_t>(n);
  }

  File cmdFile = LittleFS.open(commandsPath(Settings.model()), "r");
  if (cmdFile)
  {
    int devId = cmdFile.read();
    if (devId >= 0)
    {
      commandScanDevId_ = static_cast<uint8_t>(devId);
      size_t n = cmdFile.read(reinterpret_cast<uint8_t *>(commands_), kMaxCommands * sizeof(DiscoveredCommand));
      commandCount_ = static_cast<uint8_t>(n / sizeof(DiscoveredCommand));
    }
    cmdFile.close();
  }
  loadedCommandsModel_ = Settings.model();

  if (devIdCount_ > 0 || commandCount_ > 0)
  {
    Serial.printf("Discovery: recovered %u persisted devId(s), %u persisted command(s) (model \"%s\") from a "
                  "previous run\n",
                  devIdCount_, commandCount_, Settings.model().c_str());
  }
}

uint8_t DiscoveryScanner::listPersistedModels(String *outModels, uint8_t maxCount) const
{
  uint8_t count = 0;
  File dir = LittleFS.open(kDiscoveryDir);
  if (!dir || !dir.isDirectory())
    return 0;

  constexpr const char *kPrefix = "cmds_";
  constexpr size_t kPrefixLen = 5; // strlen("cmds_")
  constexpr const char *kSuffix = ".bin";
  constexpr size_t kSuffixLen = 4; // strlen(".bin")

  for (File f = dir.openNextFile(); f && count < maxCount; f = dir.openNextFile())
  {
    String name = f.name(); // just the entry name, not the full "/discovery/..." path
    if (name.startsWith(kPrefix) && name.endsWith(kSuffix) && name.length() > kPrefixLen + kSuffixLen)
    {
      outModels[count++] = name.substring(kPrefixLen, name.length() - kSuffixLen);
    }
    f.close();
  }
  dir.close();
  return count;
}

bool DiscoveryScanner::readPersistedCommands(const String &model, uint8_t &devId, DiscoveredCommand *out,
                                             uint8_t maxOut, uint8_t &outCount) const
{
  File f = LittleFS.open(commandsPath(model), "r");
  if (!f)
    return false;
  int devIdByte = f.read();
  if (devIdByte < 0)
  {
    f.close();
    return false;
  }
  devId = static_cast<uint8_t>(devIdByte);
  size_t n = f.read(reinterpret_cast<uint8_t *>(out), maxOut * sizeof(DiscoveredCommand));
  f.close();
  outCount = static_cast<uint8_t>(n / sizeof(DiscoveredCommand));
  return true;
}

void DiscoveryScanner::startDevIdScan(uint8_t addr)
{
  mode_ = Mode::kDevIdScan;
  addr_ = addr;
  devIdCursor_ = 0;
  devIdCount_ = 0;
  progressCurrent_ = 0;
  progressTotal_ = kMaxDevIds;
  scanStartMs_ = millis();
  clearPersistedDevIds();
}

void DiscoveryScanner::startCommandScan(uint8_t devId, uint8_t addr)
{
  mode_ = Mode::kCommandScan;
  addr_ = addr;
  commandScanDevId_ = devId;
  cmd1Cursor_ = 0;
  cmd2Cursor_ = 0;
  cmd2RetryCount_ = 0;
  commandCount_ = 0;
  progressCurrent_ = 0;
  progressTotal_ = 256UL * 128UL; // 256 cmd1 values x 128 even cmd2 values
  scanStartMs_ = millis();
  clearPersistedCommands(Settings.model());
  loadedCommandsModel_ = Settings.model();
}

void DiscoveryScanner::stop()
{
  finishScan();
  mode_ = Mode::kNone;
}

// Called right before mode_ is reset to kNone, whether that's because a
// scan ran to completion or because the user hit Stop Scan partway
// through - either way the elapsed time and which mode just ended are
// worth keeping, since this is the only record of "what just happened"
// once the DEVID/progress counters below start meaning something new.
void DiscoveryScanner::finishScan()
{
  uint32_t elapsed = millis() - scanStartMs_;
  if (mode_ == Mode::kDevIdScan)
  {
    devIdScanDurationMs_ = elapsed;
    devIdScanHasResult_ = true;
  }
  else if (mode_ == Mode::kCommandScan)
  {
    commandScanDurationMs_ = elapsed;
    commandScanHasResult_ = true;
  }
}

void DiscoveryScanner::task()
{
  switch (mode_)
  {
  case Mode::kNone:
    syncCommandsToCurrentModel();
    return;
  case Mode::kDevIdScan:
    stepDevIdScan();
    return;
  case Mode::kCommandScan:
    stepCommandScan();
    return;
  }
}

void DiscoveryScanner::syncCommandsToCurrentModel()
{
  String current = Settings.model();
  if (current == loadedCommandsModel_)
    return;
  loadedCommandsModel_ = current;

  uint8_t devId = 0;
  uint8_t count = 0;
  if (readPersistedCommands(current, devId, commands_, kMaxCommands, count))
  {
    commandScanDevId_ = devId;
    commandCount_ = count;
  }
  else
  {
    commandCount_ = 0;
  }
}

void DiscoveryScanner::stepDevIdScan()
{
  uint8_t devId = static_cast<uint8_t>(kDevIdFirst + devIdCursor_);

  uint8_t data[4];
  size_t len;
  uint8_t err;
  // ECHO is protocol-mandated (see DataSheets/SPI Protocol.pdf), but not
  // at a single universal cmd1 in practice: dryers answer it at cmd1=0x20,
  // while the crystallizer needs cmd1=0xC2 (see Crystallizer::echoCmd1())
  // - confirmed the hard way, this scan originally hardcoded 0x20 only
  // and missed a crystallizer already known to be present and responsive.
  // A blind DEVID scan can't know a device's family ahead of time, so try
  // both rather than assume.
  bool found = SpiIm.poll(devId, addr_, 0x20, 0x20, data, sizeof(data), len, err, kScanTimeoutMs) ||
               SpiIm.poll(devId, addr_, 0xC2, 0x20, data, sizeof(data), len, err, kScanTimeoutMs);
  if (found)
  {
    if (devIdCount_ < kMaxDevIds)
    {
      devIds_[devIdCount_++] = devId;
      persistDevIds(devIds_, devIdCount_);
    }
  }

  devIdCursor_++;
  progressCurrent_ = devIdCursor_;
  if (devIdCursor_ >= kMaxDevIds)
  {
    finishScan();
    mode_ = Mode::kNone;
  }
}

void DiscoveryScanner::stepCommandScan()
{
  uint8_t cmd1 = static_cast<uint8_t>(cmd1Cursor_);
  uint8_t cmd2 = static_cast<uint8_t>(cmd2Cursor_);

  uint8_t data[DiscoveredCommand::kMaxData];
  size_t len;
  uint8_t err;
  // pollRaw(), not poll(): the whole point of this scan is that we don't
  // know the reply length for an arbitrary command ahead of time.
  bool found = SpiIm.pollRaw(commandScanDevId_, addr_, cmd1, cmd2, data, sizeof(data), len, err, kScanTimeoutMs);

  // A bare EOT and flat silence (kNoReply, zero bytes ever received) are
  // both a clean "this device isn't answering that command" signal - on
  // real hardware, the overwhelming majority of the 32768-pair sweep is
  // nonexistent commands that fall into one of these two buckets, so
  // retrying either would multiply total scan time by kMaxRetries+1
  // (measured: turned an ~82-minute sweep into 4+ hours, which is
  // indistinguishable from a hang to whoever's watching the progress bar).
  // The genuine "lost the race" signature - a device that demonstrably
  // started responding but didn't finish cleanly within kScanTimeoutMs -
  // is everything else: kShortReply/kFramingMismatch/kMissingTrailer (got
  // some real bytes, ran out of time) or kCrcMismatch (got a full frame,
  // but corrupted). That's the actual near-miss category this retry is
  // for - exactly what a bigger reply like 0xED/0x90's blanket poll would
  // produce if it ran past the deadline mid-transmission - and it's a
  // small enough slice of the address space that retrying it doesn't
  // meaningfully change total scan time.
  SpiCcp::PollOutcome outcome = SpiIm.lastOutcome();
  bool worthRetrying = outcome != SpiCcp::PollOutcome::kEot && outcome != SpiCcp::PollOutcome::kNoReply;
  if (!found && worthRetrying && cmd2RetryCount_ < kMaxRetries)
  {
    cmd2RetryCount_++;
    return; // retry this exact pair on the next task() call
  }
  cmd2RetryCount_ = 0;

  // Dismiss "valid but empty" replies per the requested rule - a real
  // device can perfectly validly answer a nonsense command with zero
  // data bytes, and that's indistinguishable from "nothing interesting
  // here," so only replies with actual data get kept.
  if (found && len > 0 && commandCount_ < kMaxCommands)
  {
    DiscoveredCommand &d = commands_[commandCount_++];
    d.cmd1 = cmd1;
    d.cmd2 = cmd2;
    d.dataLen = static_cast<uint8_t>(len < DiscoveredCommand::kMaxData ? len : DiscoveredCommand::kMaxData);
    memcpy(d.data, data, d.dataLen);
    persistCommands(commands_, commandCount_, commandScanDevId_, Settings.model());
  }

  cmd2Cursor_ += 2;
  if (cmd2Cursor_ > 0xFE)
  {
    cmd2Cursor_ = 0;
    cmd1Cursor_++;
  }
  progressCurrent_++;
  if (cmd1Cursor_ > 0xFF)
  {
    finishScan();
    mode_ = Mode::kNone;
  }
}
