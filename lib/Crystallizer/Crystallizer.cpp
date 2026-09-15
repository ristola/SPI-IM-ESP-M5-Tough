#include "Crystallizer.h"

#include "SpiCcp.h"

Crystallizer XtlrCrystallizer;

namespace {
uint16_t bigEndianToU16(const uint8_t* b) { return (static_cast<uint16_t>(b[0]) << 8) | b[1]; }
}  // namespace

// See header comment. Byte 0 (poll response version) is discarded. The
// real reply is 27 bytes total (1 discard + 13 words) - maxLen must match
// that exactly (SpiIm::poll() needs the true reply length to find the
// trailer/validate the CRC), even though only the first 7 words are
// mapped to registers below; the trailing 6 are read but ignored.
//
// The first two words ("Process Heater"/"Hopper Return" per the wire
// trace's own field names) turned out to be this device's actual Process
// Temp/Return Temp readings - the same standard registers (40012/40015)
// every dryer model uses, just delivered via the blanket poll instead of
// a discrete command (confirmed: the discrete cmd2=0x70/0x72 path this
// device explicitly rejects, see the header comment). So they land on
// 40012/40015, not their own dedicated registers.
void Crystallizer::pollBlanket() {
  uint8_t data[27];
  size_t len;
  uint8_t err;
  if (!SpiIm.poll(devId_, addr_, kCmd1, 0x3A, data, sizeof(data), len, err)) return;

  setStatusRegister(40012, bigEndianToU16(data + 1));   // Process Temperature ("Process Heater" in the wire trace)
  setStatusRegister(40015, bigEndianToU16(data + 3));   // Hopper Return Temperature ("Hopper Return" in the wire trace)
  setStatusRegister(40024, bigEndianToU16(data + 5));   // Hopper High
  setStatusRegister(40025, bigEndianToU16(data + 7));   // Hopper Mid-High
  setStatusRegister(40026, bigEndianToU16(data + 9));   // Hopper Mid-Low
  setStatusRegister(40027, bigEndianToU16(data + 11));  // Hopper Low
  setStatusRegister(40028, bigEndianToU16(data + 13));  // Hopper Throat
}

void Crystallizer::pollNext() {
  switch (queryIndex_) {
    case 0:
      pollProcessSetpoint(kCmd1);
      break;
    case 1:
      pollProcessDelta(kCmd1);
      break;
    case 2:
      pollBlanket();
      break;
  }
  queryIndex_ = (queryIndex_ + 1) % kQueryCount;
}

SpiCcpQueryInfo Crystallizer::queryInfo(size_t index) const {
  static constexpr SpiCcpQueryInfo kQueries[kQueryCount] = {
      {"Process Set Point", kCmd1, 0x30, "40010"},
      {"Process High Delta", kCmd1, 0x32, "40011"},
      {"Blanket Poll", kCmd1, 0x3A, "40012,40015,40024-40028"},
  };
  return kQueries[index];
}

// 40016/40017 (Dew Point/Dew Point Alarm Trigger) explicitly excluded -
// the "#PXB-SPI-MOD-485" Modbus map confirms crystallizers don't have
// them at all, unlike every dryer model. 40012/40015 override the base
// class's generic "Process Temp"/"Return Temp" with this device's own
// wire-trace field names (see pollBlanket()). 40020/40021 (Aux 1/2 Temp)
// are named per that same sheet even though not yet populated (cmd2
// unknown). 40024-40028 are this project's own invention for the
// blanket poll's remaining hopper fields.
const char* Crystallizer::registerName(uint16_t reg) const {
  switch (reg) {
    case 40012:
      return "Process Temperature";
    case 40015:
      return "Hopper Return Temperature";
    case 40016:
    case 40017:
      return nullptr;
    case 40020:
      return "Aux 1 Temp";
    case 40021:
      return "Aux 2 Temp";
    case 40024:
      return "Hopper High";
    case 40025:
      return "Hopper Mid-High";
    case 40026:
      return "Hopper Mid-Low";
    case 40027:
      return "Hopper Low";
    case 40028:
      return "Hopper Throat";
    default:
      return EquipmentModel::registerName(reg);
  }
}
