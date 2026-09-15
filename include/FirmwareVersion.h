#pragma once

#include <cstdint>

// Single source of truth for this project's own firmware version - feeds
// both RTS-NOW's device identity (see rtsnowConfig in main.cpp and
// main_atom_node.cpp) and Modbus register 40001 ("Software Version", see
// ModbusRegisterMap.h) so the two can't drift apart.
namespace FirmwareVersion {
constexpr uint8_t kMajor = 1;
constexpr uint8_t kMinor = 0;
constexpr uint8_t kPatch = 0;
}  // namespace FirmwareVersion
