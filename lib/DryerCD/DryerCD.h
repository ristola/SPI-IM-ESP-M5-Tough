#pragma once

#include "EquipmentModel.h"

// Polls a Conair CD-model dryer over SPI-CCP (see SpiCcp). Register
// numbers match DataSheets/Modbus Registers V2.pdf.
//
// CMD1=0x20 for all of CD's polls, matching Reference/config.py. CD has
// no blanket poll at all and no vendor-specific registers on the sheet
// (its column is empty past the standard 40010-40017 block) - just the 5
// common discrete polls plus Process Temp (cmd2=0x70 -> 40012) and
// Return Temp (cmd2=0x72 -> 40015), also discrete.
//
// **Not yet tested against real CD hardware.**
class DryerCD : public EquipmentModel {
 public:
  const char* modelName() const override { return "CD"; }

  void pollNext() override;

  size_t queryCount() const override { return kQueryCount; }
  SpiCcpQueryInfo queryInfo(size_t index) const override;

 private:
  static constexpr uint8_t kCmd1 = 0x20;
  static constexpr size_t kQueryCount = 7;
  uint8_t queryIndex_ = 0;
};

extern DryerCD CdDryer;
