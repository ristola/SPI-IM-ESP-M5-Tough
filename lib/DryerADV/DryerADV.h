#pragma once

#include "EquipmentModel.h"

// Polls a UNADYN Advantage-model dryer over SPI-CCP (see SpiCcp).
// Register numbers match DataSheets/Modbus Registers V2.pdf.
//
// CMD1=0x20 for the common discrete polls, matching Reference/config.py
// and "DataSheets/SPI-CCP Notes - Dryer and Crystallizer Polls.md"
// section 2.
//
// Blanket poll (cmd1=0xD0, cmd2=0x84) returns 8 big-endian floats in a
// fixed order per the notes file's section 4.2: Process Heater #1,
// Return #1, Process Heater #2, Return #2, Left Bed Outlet, Right Bed
// Outlet, Left Bed Heater, Right Bed Heater - mapped to 40012, 40015,
// 40022, 40023, 40019, 40021, 40018, 40020, cross-checked against the
// PDF sheet's ADV column register names. It doesn't cover Dew Point
// (40016), so that's polled as its own discrete query (cmd2=0x7C) to
// fill the gap.
//
// **Not yet tested against real Advantage hardware.**
class DryerADV : public EquipmentModel {
 public:
  const char* modelName() const override { return "ADV"; }

  void pollNext() override;

  size_t queryCount() const override { return kQueryCount; }
  SpiCcpQueryInfo queryInfo(size_t index) const override;
  const char* registerName(uint16_t reg) const override;

  // Highest register between the blanket poll's own mapping (up to
  // 40023, Right Bed Outlet) and the separately-polled Dew Point
  // (40016) - see the class comment above.
  uint16_t lastUsedRegister() const override { return 40023; }

 private:
  static constexpr uint8_t kCmd1 = 0x20;
  static constexpr uint8_t kBlanketCmd1 = 0xD0;
  static constexpr uint8_t kBlanketCmd2 = 0x84;
  static constexpr size_t kQueryCount = 7;
  uint8_t queryIndex_ = 0;

  void pollBlanket();
};

extern DryerADV AdvDryer;
