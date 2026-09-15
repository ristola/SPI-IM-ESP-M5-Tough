#include "DryerADV.h"

#include <cstring>

#include "SpiCcp.h"

DryerADV AdvDryer;

namespace {
float bigEndianToFloat(const uint8_t* b) {
  uint32_t bits = (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
                  (static_cast<uint32_t>(b[2]) << 8) | b[3];
  float f;
  memcpy(&f, &bits, sizeof(f));
  return f;
}
}  // namespace

// See header comment - untested against real hardware, layout from
// "DataSheets/SPI-CCP Notes - Dryer and Crystallizer Polls.md" section
// 4.2 (cmd1=0xD0, cmd2=0x84).
void DryerADV::pollBlanket() {
  uint8_t data[32];  // 8 x f32
  size_t len;
  uint8_t err;
  if (!SpiIm.poll(devId_, addr_, kBlanketCmd1, kBlanketCmd2, data, sizeof(data), len, err)) return;

  setFloatRegister(40012, bigEndianToFloat(data));       // Process Heater #1
  setFloatRegister(40015, bigEndianToFloat(data + 4));   // Return #1
  setFloatRegister(40022, bigEndianToFloat(data + 8));   // Process Heater #2
  setFloatRegister(40023, bigEndianToFloat(data + 12));  // Return #2
  setFloatRegister(40019, bigEndianToFloat(data + 16));  // Left Bed Outlet
  setFloatRegister(40021, bigEndianToFloat(data + 20));  // Right Bed Outlet
  setFloatRegister(40018, bigEndianToFloat(data + 24));  // Left Bed Heater
  setFloatRegister(40020, bigEndianToFloat(data + 28));  // Right Bed Heater
}

void DryerADV::pollNext() {
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
      pollDewPoint(kCmd1);
      break;
    case 6:
      pollBlanket();
      break;
  }
  queryIndex_ = (queryIndex_ + 1) % kQueryCount;
}

SpiCcpQueryInfo DryerADV::queryInfo(size_t index) const {
  static constexpr SpiCcpQueryInfo kQueries[kQueryCount] = {
      {"Process Setpoint", kCmd1, 0x30, "40010"},
      {"Process Delta", kCmd1, 0x32, "40011"},
      {"Process Status", kCmd1, 0x40, "40013"},
      {"Machine Status", kCmd1, 0x48, "40014"},
      {"Dew Trigger", kCmd1, 0x80, "40017"},
      {"Dew Point", kCmd1, 0x7C, "40016"},
      {"Blanket (bulk)", kBlanketCmd1, kBlanketCmd2, "40012,40015,40018-40023"},
  };
  return kQueries[index];
}

// Names per DataSheets/Modbus Registers V2.pdf's ADV column (this model
// isn't in the newer "#PXB-SPI-MOD-485" sheet, which stops at FC/FD/FN).
const char* DryerADV::registerName(uint16_t reg) const {
  switch (reg) {
    case 40018:
      return "Left Bed Heater";
    case 40019:
      return "Left Bed Outlet";
    case 40020:
      return "Right Bed Heater";
    case 40021:
      return "Right Bed Outlet";
    case 40022:
      return "Process 2 Temp";
    case 40023:
      return "Return 2 Temp";
    default:
      return EquipmentModel::registerName(reg);
  }
}
