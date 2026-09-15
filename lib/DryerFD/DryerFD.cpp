#include "DryerFD.h"

#include "SpiCcp.h"

DryerFD Dryer;

namespace {
uint16_t bigEndianToU16(const uint8_t* b) { return (static_cast<uint16_t>(b[0]) << 8) | b[1]; }
}  // namespace

// FD's blanket poll (CMD1=0xC2, CMD2=0x2E) returns one leading discard
// byte followed by 28 sequential big-endian u16 words - NOT the mixed
// u16/u32 layout "DataSheets/SPI-CCP Notes - Dryer and Crystallizer
// Polls.md" section 3.3 describes (that table was wrong for this board;
// superseded here). Confirmed against a real captured reply by validating
// the frame's CRC against 28 destuffed 16-bit words (see SpiCcp::poll()'s
// DLE-destuffing - this exact reply is what proved that was missing:
// its raw bytes contained two literal 0x10 data bytes back to back).
// Total logical data length is 57 bytes (1 + 28*2), not 58 - there is no
// trailing discard byte, unlike the Reference gateway's assumed 'c28hc'
// struct format.
//
// Word 17 is confirmed by value to be Process Dewpoint: the real trace
// read 0xFF96 (-106 signed), matching the notes file's independently-
// documented "Dew Point (signed, e.g. FF 96)" description exactly, and
// its position (17th word) lines up with that file's field ordering for
// every word before it too (all 16 preceding words plausibly-ranged
// temperatures, matching known register names in sequence). Word 18 is
// assigned to Return Dewpoint by position (immediately follows the
// confirmed anchor, matching the notes file's ordering) but wasn't
// independently value-confirmed - it read 0 in the only real trace seen
// so far.
//
// Words 19-28 are deliberately NOT mapped to any register. The notes
// file's guess that this range holds Airflow (CFM/FPM) and 3 Pressure
// Sensors as u32 fields is now disproven (all 28 words are plain u16,
// confirmed by the successful CRC match) and there's no way to safely
// re-derive what these 10 words actually are: 9 of them read 0 in the
// only trace available (no distinguishing signal) and the remaining 2
// (words 27-28) are nonzero but don't correspond to any named field in
// the notes file at all - a real, unresolved gap, not a guess.
void DryerFD::pollBlanket() {
  uint8_t data[57];
  size_t len;
  uint8_t err;
  if (!SpiIm.poll(devId_, addr_, 0xC2, 0x2E, data, sizeof(data), len, err)) return;

  setStatusRegister(40012, bigEndianToU16(data + 1));   // word1: Process 1 Temp
  setStatusRegister(40015, bigEndianToU16(data + 3));   // word2: Return 1 Temp
  setStatusRegister(40022, bigEndianToU16(data + 5));   // word3: Process 2 Temp
  setStatusRegister(40023, bigEndianToU16(data + 7));   // word4: Return 2 Temp
  setStatusRegister(40018, bigEndianToU16(data + 9));   // word5: Regen Temp
  setStatusRegister(40019, bigEndianToU16(data + 11));  // word6: Regen Outlet Temp
  setStatusRegister(40020, bigEndianToU16(data + 13));  // word7: Dryer Inlet Temp
  setStatusRegister(40025, bigEndianToU16(data + 15));  // word8: Throat Temp ("Hopper Throat" in the trace)
  setStatusRegister(40026, bigEndianToU16(data + 17));  // word9: Left Bed Temp
  setStatusRegister(40027, bigEndianToU16(data + 19));  // word10: Right Bed Temp
  setStatusRegister(40028, bigEndianToU16(data + 21));  // word11: Hopper 1 Temp
  setStatusRegister(40029, bigEndianToU16(data + 23));  // word12: Hopper 2 Temp
  setStatusRegister(40030, bigEndianToU16(data + 25));  // word13: Hopper 3 Temp
  setStatusRegister(40031, bigEndianToU16(data + 27));  // word14: Hopper 4 Temp
  setStatusRegister(40032, bigEndianToU16(data + 29));  // word15: Hopper 5 Temp
  setStatusRegister(40033, bigEndianToU16(data + 31));  // word16: Hopper 6 Temp
  setStatusRegister(40034, bigEndianToU16(data + 33));  // word17: Process Dewpoint (signed) - value-confirmed
  setStatusRegister(40035, bigEndianToU16(data + 35));  // word18: Return Dewpoint (signed) - position only
  // words 19-28 (data+37 .. data+55): unmapped, see header comment above.
}

void DryerFD::pollNext() {
  switch (queryIndex_) {
    case 0:
      pollProcessSetpoint(kCmd1);
      break;
    case 1:
      pollProcessDelta(kCmd1);
      break;
    case 2:
      pollProcessStatus(kCmd1);
      break;
    case 3:
      pollMachineStatus(kCmd1);
      break;
    case 4:
      pollDewTrigger(kCmd1);
      break;
    case 5:
      pollBlanket();
      break;
  }
  queryIndex_ = (queryIndex_ + 1) % kQueryCount;
}

SpiCcpQueryInfo DryerFD::queryInfo(size_t index) const {
  static constexpr SpiCcpQueryInfo kQueries[kQueryCount] = {
      {"Process Setpoint", kCmd1, 0x30, "40010"},
      {"Process Delta", kCmd1, 0x32, "40011"},
      {"Process Status", kCmd1, 0x40, "40013"},
      {"Machine Status", kCmd1, 0x48, "40014"},
      {"Dew Trigger", kCmd1, 0x80, "40017"},
      {"Blanket (bulk)", 0xC2, 0x2E, "40012,40015,40018-40020,40022,40023,40025-40035"},
  };
  return kQueries[index];
}

// Extended-block names per the "#PXB-SPI-MOD-485" Modbus map's FD column
// for 40018-40033; 40034/40035 aren't in that sheet's cropped view but
// were already independently confirmed against a real captured reply
// (see pollBlanket()'s comment).
const char* DryerFD::registerName(uint16_t reg) const {
  switch (reg) {
    case 40018:
      return "Regen Temp";
    case 40019:
      return "Regen Out Temp";
    case 40020:
      return "Dryer Inlet Temp";
    case 40021:
      return "Dryer Outlet Temp";
    case 40022:
      return "Process 2 Temp";
    case 40023:
      return "Return 2 Temp";
    case 40024:
      return "Inlet Temp";
    case 40025:
      return "Throat Temp";
    case 40026:
      return "Left Bed Temp";
    case 40027:
      return "Right Bed Temp";
    case 40028:
      return "Hopper 1 Temp";
    case 40029:
      return "Hopper 2 Temp";
    case 40030:
      return "Hopper 3 Temp";
    case 40031:
      return "Hopper 4 Temp";
    case 40032:
      return "Hopper 5 Temp";
    case 40033:
      return "Hopper 6 Temp";
    case 40034:
      return "Process Dewpoint";
    case 40035:
      return "Return Dewpoint";
    default:
      return EquipmentModel::registerName(reg);
  }
}
