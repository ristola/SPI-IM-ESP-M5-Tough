#pragma once

#include "EquipmentModel.h"

// Polls a UNADYN FN-model dryer over SPI-CCP (see SpiCcp). Register
// numbers match DataSheets/Modbus Registers V2.pdf.
//
// CMD1=0x20 for FN's common discrete polls, per both Ristola Technical
// Services' prior XBee gateway (Reference/config.py) and "DataSheets/
// SPI-CCP Notes - Dryer and Crystallizer Polls.md" section 2 - no
// conflict between sources here, unlike DryerFC's cmd1.
//
// Blanket poll (cmd1=0xED, cmd2=0x90) uses the exact same 7-float layout
// as DryerFC's blanket poll (same command, same field order per the
// notes file's section 3.2 "FC/FN/Advantage dryer") - Reference/model.py
// has a latent bug where its TypeFN(TypeFD) never overrides dispatch(),
// so it silently drops this data instead of parsing it; deliberately not
// replicated here (see the project memory this ported from for detail) -
// this reuses FC's known-good field mapping instead.
//
// **Not yet tested against real FN hardware.**
class DryerFN : public EquipmentModel {
 public:
  const char* modelName() const override { return "FN"; }

  void pollNext() override;

  size_t queryCount() const override { return kQueryCount; }
  SpiCcpQueryInfo queryInfo(size_t index) const override;
  const char* registerName(uint16_t reg) const override;

  // Reuses FC's blanket poll layout - see the class comment above -
  // whose highest mapped register is Aux 2 Temp (40021).
  uint16_t lastUsedRegister() const override { return 40021; }

 private:
  static constexpr uint8_t kCmd1 = 0x20;
  static constexpr uint8_t kBlanketCmd1 = 0xED;
  static constexpr uint8_t kBlanketCmd2 = 0x90;
  static constexpr size_t kQueryCount = 6;
  uint8_t queryIndex_ = 0;

  void pollBlanket();
};

extern DryerFN FnDryer;
