#include "DryerFN.h"

#include <cstring>

#include "SpiCcp.h"

DryerFN FnDryer;

namespace {
float bigEndianToFloat(const uint8_t* b) {
  uint32_t bits = (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
                  (static_cast<uint32_t>(b[2]) << 8) | b[3];
  float f;
  memcpy(&f, &bits, sizeof(f));
  return f;
}
}  // namespace

// Same layout as DryerFC::pollBlanket() - see this file's header comment
// for why.
void DryerFN::pollBlanket() {
  uint8_t data[28];  // 7 x f32
  size_t len;
  uint8_t err;
  if (!SpiIm.poll(devId_, addr_, kBlanketCmd1, kBlanketCmd2, data, sizeof(data), len, err)) return;

  setFloatRegister(40012, bigEndianToFloat(data));       // Process Temp
  setFloatRegister(40018, bigEndianToFloat(data + 4));   // Regen Temp
  setFloatRegister(40015, bigEndianToFloat(data + 8));   // Return Temp
  setFloatRegister(40019, bigEndianToFloat(data + 12));  // Regen Outlet Temp
  setFloatRegister(40020, bigEndianToFloat(data + 16));  // Aux 1 Temp
  setFloatRegister(40021, bigEndianToFloat(data + 20));  // Aux 2 Temp
  setFloatRegister(40016, bigEndianToFloat(data + 24));  // Dew Point
}

void DryerFN::pollNext() {
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

SpiCcpQueryInfo DryerFN::queryInfo(size_t index) const {
  static constexpr SpiCcpQueryInfo kQueries[kQueryCount] = {
      {"Process Setpoint", kCmd1, 0x30, "40010"},
      {"Process Delta", kCmd1, 0x32, "40011"},
      {"Process Status", kCmd1, 0x40, "40013"},
      {"Machine Status", kCmd1, 0x48, "40014"},
      {"Dew Trigger", kCmd1, 0x80, "40017"},
      {"Blanket (bulk)", kBlanketCmd1, kBlanketCmd2, "40012,40015,40016,40018-40021"},
  };
  return kQueries[index];
}

// Extended-block names per the "#PXB-SPI-MOD-485" Modbus map's FN column
// - identical to FC's.
const char* DryerFN::registerName(uint16_t reg) const {
  switch (reg) {
    case 40018:
      return "Regen Temp";
    case 40019:
      return "Regen Out Temp";
    case 40020:
      return "Aux 1 Temp";
    case 40021:
      return "Aux 2 Temp";
    default:
      return EquipmentModel::registerName(reg);
  }
}
