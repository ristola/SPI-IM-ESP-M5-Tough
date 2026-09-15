#pragma once

#include "EquipmentModel.h"

// Placeholder EquipmentModel for Model Type 7 (ETHERNET) - see
// DeviceSettings.h's kEthernetModels comment. Deliberately polls
// nothing over SPI-CCP at all: this model exists for an AtomS3 + Atomic
// PoE Base (W5500) node that has no RS-485 equipment attached in the
// first place (that bottom pogo-pin interface is occupied by the PoE
// base instead of an RS485 tail/base module) - it only exists to be a
// reachable ESP-NOW/RTS-NOW node with wired Ethernet for its IP traffic.
// Every Modbus register this node exposes simply stays at 0/not-present,
// same as any register a "real" model doesn't populate.
class EthernetNode : public EquipmentModel {
 public:
  const char* modelName() const override { return "ETHERNET"; }

  void pollNext() override {}

  size_t queryCount() const override { return 0; }
  SpiCcpQueryInfo queryInfo(size_t) const override { return SpiCcpQueryInfo{}; }

  // No SPI-CCP polling at all - not even the standard common block - so
  // there's nothing to report past the XBEE SETUP config registers
  // (40001-40009: station ID, baud rate, model type, etc.).
  uint16_t lastUsedRegister() const override { return 40009; }
};

// Named to avoid colliding with the M5_Ethernet library's own global
// `Ethernet` object (this class only ever appears alongside that library
// in files that also #include <M5_Ethernet.h> - see NetworkManager.cpp).
extern EthernetNode EthernetEquipmentModel;
