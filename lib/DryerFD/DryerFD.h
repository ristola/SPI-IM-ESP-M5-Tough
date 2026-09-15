#pragma once

#include "EquipmentModel.h"

// Polls a UNADYN FD-model dryer over SPI-CCP (see SpiCcp), matching
// Ristola Technical Services' prior XBee gateway (Reference/model.py
// TypeFD). Register numbers match DataSheets/Modbus Registers V2.pdf
// (1-based Modbus holding register numbers, e.g. 40010). Common
// discrete polls (setpoint/delta/status/machine status/dew trigger) use
// cmd1=0x20, per Reference/config.py and "DataSheets/SPI-CCP Notes -
// Dryer and Crystallizer Polls.md" section 2.
//
// FD's "blanket" command (CMD1=0xC2, CMD2=0x2E) returns more registers in
// one round trip: 18 of them (40012, 40015, 40018-40020, 40022, 40023,
// 40025-40035) are implemented, confirmed against a real captured reply -
// see pollBlanket()'s comment for the full derivation, including how that
// trace disproved this file's earlier assumption (based on the SPI-CCP
// notes file's section 3.3) that the poll's tail held Airflow/Pressure as
// u32 fields. It's actually 28 plain u16 words throughout; the last 10
// (40036-40041 and 4 more with no assigned register at all) remain a
// genuine unresolved gap, not guessed at. Two more sheet registers -
// 40021 (Dryer Outlet Temp) and 40024 (Inlet Temp) - don't appear
// anywhere in the blanket poll at all, on top of that gap.
class DryerFD : public EquipmentModel {
 public:
  const char* modelName() const override { return "FD"; }

  // Polls one query per call (round-robin over the 6 implemented
  // queries - 5 discrete + the blanket poll) and updates the internal
  // register table on success. Call periodically from loop() - each call
  // blocks for up to ~1s if the dryer doesn't respond.
  void pollNext() override;

  size_t queryCount() const override { return kQueryCount; }
  SpiCcpQueryInfo queryInfo(size_t index) const override;
  const char* registerName(uint16_t reg) const override;

  // Blanket poll's highest mapped register (40035) - see the class
  // comment above for the full 18-register derivation.
  uint16_t lastUsedRegister() const override { return 40035; }

 private:
  static constexpr uint8_t kCmd1 = 0x20;
  static constexpr size_t kQueryCount = 6;
  uint8_t queryIndex_ = 0;

  void pollBlanket();  // CMD1=0xC2, CMD2=0x2E -> 18 registers, see .cpp
};

extern DryerFD Dryer;
