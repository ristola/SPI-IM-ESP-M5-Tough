#include "Discovery.h"

#include <cstring>

#include "SpiCcp.h"

DiscoveryScanner Discovery;

void DiscoveryScanner::startDevIdScan(uint8_t addr) {
  mode_ = Mode::kDevIdScan;
  addr_ = addr;
  devIdCursor_ = 0;
  devIdCount_ = 0;
  progressCurrent_ = 0;
  progressTotal_ = kMaxDevIds;
}

void DiscoveryScanner::startCommandScan(uint8_t devId, uint8_t addr) {
  mode_ = Mode::kCommandScan;
  addr_ = addr;
  commandScanDevId_ = devId;
  cmd1Cursor_ = 0;
  cmd2Cursor_ = 0;
  commandCount_ = 0;
  progressCurrent_ = 0;
  progressTotal_ = 256UL * 128UL;  // 256 cmd1 values x 128 even cmd2 values
}

void DiscoveryScanner::stop() { mode_ = Mode::kNone; }

void DiscoveryScanner::task() {
  switch (mode_) {
    case Mode::kNone:
      return;
    case Mode::kDevIdScan:
      stepDevIdScan();
      return;
    case Mode::kCommandScan:
      stepCommandScan();
      return;
  }
}

void DiscoveryScanner::stepDevIdScan() {
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
  if (found) {
    if (devIdCount_ < kMaxDevIds) {
      devIds_[devIdCount_++] = devId;
    }
  }

  devIdCursor_++;
  progressCurrent_ = devIdCursor_;
  if (devIdCursor_ >= kMaxDevIds) {
    mode_ = Mode::kNone;
  }
}

void DiscoveryScanner::stepCommandScan() {
  uint8_t cmd1 = static_cast<uint8_t>(cmd1Cursor_);
  uint8_t cmd2 = static_cast<uint8_t>(cmd2Cursor_);

  uint8_t data[DiscoveredCommand::kMaxData];
  size_t len;
  uint8_t err;
  // pollRaw(), not poll(): the whole point of this scan is that we don't
  // know the reply length for an arbitrary command ahead of time.
  if (SpiIm.pollRaw(commandScanDevId_, addr_, cmd1, cmd2, data, sizeof(data), len, err, kScanTimeoutMs)) {
    // Dismiss "valid but empty" replies per the requested rule - a real
    // device can perfectly validly answer a nonsense command with zero
    // data bytes, and that's indistinguishable from "nothing interesting
    // here," so only replies with actual data get kept.
    if (len > 0 && commandCount_ < kMaxCommands) {
      DiscoveredCommand& d = commands_[commandCount_++];
      d.cmd1 = cmd1;
      d.cmd2 = cmd2;
      d.dataLen = static_cast<uint8_t>(len < DiscoveredCommand::kMaxData ? len : DiscoveredCommand::kMaxData);
      memcpy(d.data, data, d.dataLen);
    }
  }

  cmd2Cursor_ += 2;
  if (cmd2Cursor_ > 0xFE) {
    cmd2Cursor_ = 0;
    cmd1Cursor_++;
  }
  progressCurrent_++;
  if (cmd1Cursor_ > 0xFF) {
    mode_ = Mode::kNone;
  }
}
