#pragma once

#include <cstdint>

// ESP-NOW payload broadcast by each dryer-monitor Tough. Mirrors a Modbus
// holding-register block 1:1 so the gateway can drop it straight into its
// aggregate register map once the Modbus TCP server exists. kMaxRegisters
// covers FD's full register range (40001-40041, see ModbusRegisterMap.h and
// DataSheets/Modbus Registers V2.pdf) plus 2 extra slots past the sheet's
// own range for temporary SpiCcp poll diagnostics (see
// main_atom_node.cpp's kPollOutcomeRegister/kPollAttemptCountRegister) -
// 43 regs = 88 bytes, well under ESP-NOW's 250-byte payload limit.
struct DryerPacket {
  static constexpr uint8_t kMaxRegisters = 43;

  uint8_t nodeId;
  uint8_t registerCount;
  uint16_t registers[kMaxRegisters];
};
