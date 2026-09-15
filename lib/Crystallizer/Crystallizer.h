#pragma once

#include "EquipmentModel.h"

// Polls an FC/FN-Crystallizer over SPI-CCP (see SpiCcp).
//
// Register numbers 40010-40021 now come from an authoritative (if old)
// "#PXB-SPI-MOD-485" Modbus map the user provided, which has a real
// Crystallizer FC/FN column: crystallizers share most of the dryers'
// STANDARD COMMON SPI DATA block (40010 Process Set Point, 40011 Process
// Limit Delta, 40012 Process Temp, 40013 Process Status, 40014 Machine
// Status, 40015 Return Temp) but do NOT have 40016 (Dew Point) or 40017
// (Dew Point Trigger). In the EXTENDED block, crystallizers have no Regen
// Temp/Regen Out Temp (40018/40019 blank in that sheet) but do have Aux 1
// Temp (40020) and Aux 2 Temp (40021) - cmd2 for those two isn't known
// yet (not the blanket poll's own fields, see below), so they're
// reserved/unimplemented for now rather than guessed at.
//
// CMD1=0xC2 for everything, per "DataSheets/SPI-CCP Notes - Dryer and
// Crystallizer Polls.md" section 5 and confirmed against real hardware
// for Process Set Point/Delta (cmd2=0x30/0x32). DevID differs by which
// physical unit this is (FC-Crystallizer=0x5C, FN-Crystallizer=0x22 per
// the notes file's section 1) - passed in via begin(), not hardcoded
// here; the two share this identical command set.
//
// Tried a discrete Process Temp/Return Temp poll (cmd2=0x70/0x72, same
// cmd2 dryers use for the same registers) and got a clean, explicit "not
// supported" from the real hardware rather than silence or a bad guess:
// the reply's ERR byte was 0x28 instead of the usual 0x20, with zero data
// bytes between STX and ETX. That's a genuinely useful signal worth
// recognizing elsewhere too - this device tells you when a command
// doesn't exist instead of just going quiet. It turned out those two
// registers are delivered a different way instead - see the blanket
// poll's comment below.
//
// Deliberately does NOT use EquipmentModel::pollProcessStatus()/
// pollMachineStatus() (cmd2=0x40/0x48) even though the Modbus map above
// says crystallizers should have Process Status (40013) and Machine
// Status (40014) - tried cmd2=0x40/0x48 against real hardware already and
// confirmed those cmd2 values mean something else entirely for this
// device: the reply is 7-10 bytes of still-unidentified data (matching
// the notes file's own "(unlabeled) ???" rows for 0x40/0x48), not a
// simple 2-byte status word like the dryer models return at the same
// cmd2. Whatever cmd2 actually produces Process Status/Machine Status
// here (if anything) is still unknown - 40013/40014 stay unpopulated
// rather than guessed at.
//
// Blanket poll (cmd2=0x3A) returns a poll-response-version byte (discard)
// followed by 13 words. The first 7 field names and byte ranges are
// confirmed (non-overlapping, sequential: Process Heater, Hopper Return,
// Hopper High, Hopper Mid-High, Hopper Mid-Low, Hopper Low, Hopper
// Throat) - this superseded an earlier transcription of the same table
// that had Mid-High/Mid-Low's byte ranges overlapping by one byte, which
// was the actual error, not a real ordering ambiguity. The first two of
// those ("Process Heater"/"Hopper Return") turned out to actually be this
// device's Process Temp/Return Temp - the standard 40012/40015 registers,
// delivered here instead of via the discrete polls that don't work on
// this device (see above) - so pollBlanket() routes them there instead of
// their own dedicated registers. The remaining 5 (Hopper High through
// Throat) and 6 more all-zero words in that trace don't have standard
// register slots, so they keep this project's own invented numbers
// (40024-40028) / stay unmapped.
class Crystallizer : public EquipmentModel {
 public:
  const char* modelName() const override { return "XTLR"; }

  uint8_t echoCmd1() const override { return kCmd1; }

  void pollNext() override;

  size_t queryCount() const override { return kQueryCount; }
  SpiCcpQueryInfo queryInfo(size_t index) const override;
  const char* registerName(uint16_t reg) const override;

  // Blanket poll's highest mapped register - this project's own invented
  // Hopper Throat number (40028), see the class comment above. Neither
  // FC-XTLR nor FN-XTLR ever populates anything past this.
  uint16_t lastUsedRegister() const override { return 40028; }

 private:
  static constexpr uint8_t kCmd1 = 0xC2;
  static constexpr size_t kQueryCount = 3;
  uint8_t queryIndex_ = 0;

  void pollBlanket();
};

extern Crystallizer XtlrCrystallizer;
