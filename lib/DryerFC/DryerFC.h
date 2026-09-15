#pragma once

#include "EquipmentModel.h"

// Polls a UNADYN FC-model dryer over SPI-CCP (see SpiCcp). Register
// numbers match DataSheets/Modbus Registers V2.pdf.
//
// CMD1 for FC's common discrete polls (setpoint/delta/status/machine
// status/dew trigger) is 0xC2, per Ristola Technical Services' prior
// working XBee gateway (Reference/model.py TypeFC) - a real, previously-
// deployed system, trusted over "DataSheets/SPI-CCP Notes - Dryer and
// Crystallizer Polls.md" section 2's conflicting claim that all of FC/
// FD-X/FN/Advantage share cmd1=0x20 for these. That conflict is
// unresolved and flagged here rather than silently picking one - **not
// yet tested against real FC hardware**, re-verify cmd1 first if an FC
// unit gives no reply at all.
//
// Blanket poll (cmd1=0xED, cmd2=0x90) returns 7 big-endian floats in a
// fixed order per the notes file's section 3.2: Process Temp, Regen
// Temp, Return Temp, Regen Outlet Temp, Aux 1 Temp, Aux 2 Temp, Dew
// Point - mapped to 40012, 40018, 40015, 40019, 40020, 40021, 40016.
class DryerFC : public EquipmentModel {
 public:
  const char* modelName() const override { return "FC"; }

  void pollNext() override;

  size_t queryCount() const override { return kQueryCount; }
  SpiCcpQueryInfo queryInfo(size_t index) const override;
  const char* registerName(uint16_t reg) const override;

  // Blanket poll's highest mapped register (Aux 2 Temp, 40021) - see the
  // class comment above for the full 7-field layout.
  uint16_t lastUsedRegister() const override { return 40021; }

 private:
  static constexpr uint8_t kCmd1 = 0xC2;
  static constexpr uint8_t kBlanketCmd1 = 0xED;
  static constexpr uint8_t kBlanketCmd2 = 0x90;
  static constexpr size_t kQueryCount = 6;
  uint8_t queryIndex_ = 0;

  void pollBlanket();
};

extern DryerFC FcDryer;
