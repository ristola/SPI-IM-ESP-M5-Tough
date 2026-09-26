#include "EquipmentModel.h"

#include <cstring>

#include "SpiCcp.h"

namespace {
float bigEndianToFloat(const uint8_t* b) {
  uint32_t bits = (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
                  (static_cast<uint32_t>(b[2]) << 8) | b[3];
  float f;
  memcpy(&f, &bits, sizeof(f));
  return f;
}

uint16_t bigEndianToU16(const uint8_t* b) { return (static_cast<uint16_t>(b[0]) << 8) | b[1]; }

void floatToBigEndian(float value, uint8_t* out) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  out[0] = (bits >> 24) & 0xFF;
  out[1] = (bits >> 16) & 0xFF;
  out[2] = (bits >> 8) & 0xFF;
  out[3] = bits & 0xFF;
}
}  // namespace

// See header comment - names per the "#PXB-SPI-MOD-485" Modbus map's
// STANDARD COMMON SPI DATA section, worded identically across every model
// there.
const char* EquipmentModel::registerName(uint16_t reg) const {
  switch (reg) {
    case 40010:
      return "Process Set Point";
    case 40011:
      return "Process Limit Delta";
    case 40012:
      return "Process Temp";
    case 40013:
      return "Process Status";
    case 40014:
      return "Machine Status";
    case 40015:
      return "Return Temp";
    case 40016:
      return "Dew Point";
    case 40017:
      return "Dew Point Alarm Trigger";
    default:
      return nullptr;
  }
}

uint16_t EquipmentModel::getRegister(uint16_t modbusReg) const {
  if (!hasRegister(modbusReg)) return 0;
  return regs_[modbusReg - kFirstRegister];
}

bool EquipmentModel::hasRegister(uint16_t modbusReg) const {
  if (modbusReg < kFirstRegister || modbusReg > kLastRegister) return false;
  return present_[modbusReg - kFirstRegister];
}

bool EquipmentModel::isSignedRegister(uint16_t modbusReg) const {
  if (modbusReg < kFirstRegister || modbusReg > kLastRegister) return false;
  return signed_[modbusReg - kFirstRegister];
}

uint32_t EquipmentModel::registerAgeMs(uint16_t modbusReg) const {
  if (!hasRegister(modbusReg)) return UINT32_MAX;
  return millis() - lastUpdatedMs_[modbusReg - kFirstRegister];
}

// See header comment. Deliberately does not update regs_/present_ itself
// after a successful SELECT - the next round-robin poll of this same
// register (every model polls these at least every couple of pollNext()
// cycles) picks up the confirmed new value from the tributary itself,
// rather than optimistically trusting what we just sent.
bool EquipmentModel::writeRegister(uint16_t reg, float value) {
  // Process Setpoint (40010), Process Limit Delta (40011), and Dew Point
  // Trigger (40017) are 4-byte float writes. Machine Status (40014) is a
  // 2-byte u16 instead - per the user's own confirmed usage on an FN
  // dryer, 0=off, 1=on, 3=clear alarms (not itself a state, a momentary
  // command). Found the same way for all four: whichever query has an
  // exact (not range/list) match for this register in its registers spec
  // supplies the right cmd1/cmd2 - this is why it works unmodified for
  // every model that has that query, and correctly does nothing for one
  // that doesn't (e.g. Crystallizer, which polls neither Machine Status
  // nor Dew Point Trigger at all - cmd2=0x48/0x80 mean something else or
  // nothing on that device). 40017 specifically: every dryer model
  // (FC/FD/FN/ADV/CD) has an identical "Dew Trigger" query (cmd2=0x80,
  // registers="40017"), confirmed by reading each model's own query
  // table - this was simply never added to the allowlist below even
  // though the read side has polled it since the very first pass.
  if (reg != 40010 && reg != 40011 && reg != 40014 && reg != 40017) return false;
  char regStr[6];
  snprintf(regStr, sizeof(regStr), "%u", reg);
  for (size_t i = 0; i < queryCount(); i++) {
    SpiCcpQueryInfo q = queryInfo(i);
    if (strcmp(q.registers, regStr) == 0) {
      if (reg == 40014) {
        uint16_t v = static_cast<uint16_t>(value);
        uint8_t data[2] = {static_cast<uint8_t>((v >> 8) & 0xFF), static_cast<uint8_t>(v & 0xFF)};
        return SpiIm.select(devId_, addr_, q.cmd1, q.cmd2, data, sizeof(data));
      }
      uint8_t data[4];
      floatToBigEndian(value, data);
      return SpiIm.select(devId_, addr_, q.cmd1, q.cmd2, data, sizeof(data));
    }
  }
  return false;
}

void EquipmentModel::setFloatRegister(uint16_t modbusReg, float value) {
  if (modbusReg < kFirstRegister || modbusReg > kLastRegister) return;
  // Registers hold the value truncated to a plain integer in its
  // engineering unit (e.g. degrees F), matching this project's existing
  // convention (see Reference/'s int(v) truncation, and DryerFD's
  // original implementation this was lifted from). Truncating straight to
  // uint16_t is undefined behavior for a negative value (e.g. a -40 dew
  // point) since it's out of that type's range - go through int16_t
  // first, which is well-defined for any value in the actual sensor
  // range, then let the int16_t->uint16_t conversion give the correct
  // two's-complement bit pattern for the Modbus register.
  regs_[modbusReg - kFirstRegister] = static_cast<uint16_t>(static_cast<int16_t>(value));
  present_[modbusReg - kFirstRegister] = true;
  signed_[modbusReg - kFirstRegister] = true;
  lastUpdatedMs_[modbusReg - kFirstRegister] = millis();
}

void EquipmentModel::setStatusRegister(uint16_t modbusReg, uint16_t value) {
  if (modbusReg < kFirstRegister || modbusReg > kLastRegister) return;
  regs_[modbusReg - kFirstRegister] = value;
  present_[modbusReg - kFirstRegister] = true;
  signed_[modbusReg - kFirstRegister] = false;
  lastUpdatedMs_[modbusReg - kFirstRegister] = millis();
}

void EquipmentModel::pollProcessSetpoint(uint8_t cmd1) {
  uint8_t data[4];
  size_t len;
  uint8_t err;
  if (SpiIm.poll(devId_, addr_, cmd1, 0x30, data, sizeof(data), len, err)) {
    setFloatRegister(40010, bigEndianToFloat(data));
  }
}

void EquipmentModel::pollProcessDelta(uint8_t cmd1) {
  uint8_t data[4];
  size_t len;
  uint8_t err;
  if (SpiIm.poll(devId_, addr_, cmd1, 0x32, data, sizeof(data), len, err)) {
    setFloatRegister(40011, bigEndianToFloat(data));
  }
}

void EquipmentModel::pollProcessStatus(uint8_t cmd1) {
  uint8_t data[2];
  size_t len;
  uint8_t err;
  if (SpiIm.poll(devId_, addr_, cmd1, 0x40, data, sizeof(data), len, err)) {
    setStatusRegister(40013, bigEndianToU16(data));
  }
}

void EquipmentModel::pollMachineStatus(uint8_t cmd1) {
  uint8_t data[2];
  size_t len;
  uint8_t err;
  if (SpiIm.poll(devId_, addr_, cmd1, 0x48, data, sizeof(data), len, err)) {
    setStatusRegister(40014, bigEndianToU16(data));
  }
}

void EquipmentModel::pollDewTrigger(uint8_t cmd1) {
  uint8_t data[4];
  size_t len;
  uint8_t err;
  if (SpiIm.poll(devId_, addr_, cmd1, 0x80, data, sizeof(data), len, err)) {
    setFloatRegister(40017, bigEndianToFloat(data));
  }
}

void EquipmentModel::pollProcessTemp(uint8_t cmd1) {
  uint8_t data[4];
  size_t len;
  uint8_t err;
  if (SpiIm.poll(devId_, addr_, cmd1, 0x70, data, sizeof(data), len, err)) {
    setFloatRegister(40012, bigEndianToFloat(data));
  }
}

void EquipmentModel::pollReturnTemp(uint8_t cmd1) {
  uint8_t data[4];
  size_t len;
  uint8_t err;
  if (SpiIm.poll(devId_, addr_, cmd1, 0x72, data, sizeof(data), len, err)) {
    setFloatRegister(40015, bigEndianToFloat(data));
  }
}

void EquipmentModel::pollDewPoint(uint8_t cmd1) {
  uint8_t data[4];
  size_t len;
  uint8_t err;
  if (SpiIm.poll(devId_, addr_, cmd1, 0x7C, data, sizeof(data), len, err)) {
    setFloatRegister(40016, bigEndianToFloat(data));
  }
}
