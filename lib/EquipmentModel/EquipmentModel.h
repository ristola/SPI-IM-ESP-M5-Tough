#pragma once

#include <Arduino.h>

// Describes one SPI-CCP query a model issues - lets the web dashboard show
// "what we're polling" generically for whichever model is active, instead
// of a hardcoded per-model table living in DryerWebServer.
struct SpiCcpQueryInfo {
  const char* name;
  uint8_t cmd1;
  uint8_t cmd2;
  const char* registers;  // human-readable register list, e.g. "40010" or "40012,40015"
};

// Common interface + shared register storage for every dryer/crystallizer
// model this project supports (DryerFC/FD/FN/ADV/CD, CrystallizerFC). Only
// one concrete model is active per node at a time (see ModelFactory) - the
// register table, lookup, and devId/addr storage are identical across
// every model, so they live here once; each subclass only implements its
// own SPI-CCP command table and reply parsing.
class EquipmentModel {
 public:
  virtual ~EquipmentModel() = default;

  virtual const char* modelName() const = 0;

  virtual void begin(uint8_t devId, uint8_t addr) {
    devId_ = devId;
    addr_ = addr;
  }

  // Polls one query per call, round-robin - see each subclass for its
  // exact query count/order. Call periodically from loop() - each call
  // blocks for up to ~1s if the device doesn't respond.
  virtual void pollNext() = 0;

  // Introspection for the web dashboard - the full set of queries this
  // model issues, in round-robin order.
  virtual size_t queryCount() const = 0;
  virtual SpiCcpQueryInfo queryInfo(size_t index) const = 0;

  // cmd1 for the protocol-mandated ECHO/REVISION commands (see
  // DataSheets/SPI Protocol.pdf) - 0x20 for every dryer model per that
  // spec, overridden by Crystallizer (0xC2, confirmed by a real captured
  // trace - see "DataSheets/SPI-CCP Notes - Dryer and Crystallizer
  // Polls.md" section 5). Lets the web dashboard's generic Echo/Version
  // test buttons work correctly no matter which model is active.
  virtual uint8_t echoCmd1() const { return 0x20; }

  // Human-readable field name for one of this model's registers, for the
  // web dashboard's Live Register Data table - nullptr if not known.
  // Default implementation covers the STANDARD COMMON SPI DATA block
  // (40010-40017), which the "#PXB-SPI-MOD-485" Modbus map confirms is
  // worded identically across every dryer model; subclasses override to
  // add their own vendor-specific (40018+) names, falling back to this
  // for anything outside that range. Crystallizer overrides 40016/40017
  // back to nullptr since crystallizers don't have Dew Point/Dew Point
  // Alarm Trigger at all, per that same source.
  virtual const char* registerName(uint16_t reg) const;

  uint16_t getRegister(uint16_t modbusReg) const;
  bool hasRegister(uint16_t modbusReg) const;

  // Attempts to write a new value to a register via SPI-CCP SELECT.
  // Currently wired up for Process Setpoint (40010), Process Limit Delta
  // (40011), and Machine Status (40014) - finds this model's own cmd1 for
  // the target register by reusing queryInfo() (matching its exact
  // single-register spec, e.g. "40010") rather than duplicating each
  // subclass's cmd1 constant here, so it works unmodified for every model
  // that has that query and correctly does nothing for one that doesn't.
  // Returns false (and sends nothing) for any other register - more will
  // be added once tested against real hardware, same "don't guess"
  // discipline as the read side.
  virtual bool writeRegister(uint16_t reg, float value);

  uint8_t devId() const { return devId_; }

  // Highest Modbus register (400xx) this model ever actually populates -
  // see each subclass's own override for the derivation (matches its
  // setFloatRegister()/setStatusRegister() calls in pollNext()/
  // pollBlanket()). Used by fillRegisterBlock() (main.cpp/
  // main_atom_node.cpp) so a "Poll Registers" reply only reports as many
  // registers as this model actually uses, instead of always padding
  // out to the FD dryer's full 40001-40041 range with zeros for every
  // other model - e.g. the FC/FN Crystallizer's blanket poll never gets
  // past 40028. Default covers the STANDARD COMMON SPI DATA block
  // (40010-40017) that nearly every dryer model populates (see
  // registerName()'s comment) - DryerCD has no vendor-specific
  // registers at all, so it doesn't need to override this.
  virtual uint16_t lastUsedRegister() const { return 40017; }

 protected:
  static constexpr uint16_t kFirstRegister = 40001;
  static constexpr uint16_t kLastRegister = 40041;

  uint8_t devId_ = 0x22;
  uint8_t addr_ = 0x20;

  void setFloatRegister(uint16_t modbusReg, float value);
  void setStatusRegister(uint16_t modbusReg, uint16_t value);

  // Shared discrete polls, common to every dryer model per "DataSheets/
  // SPI-CCP Notes - Dryer and Crystallizer Polls.md" section 2 - only the
  // cmd1 ("zone") byte differs per model, passed in by the caller.
  void pollProcessSetpoint(uint8_t cmd1);  // cmd2=0x30 -> 40010, f32
  void pollProcessDelta(uint8_t cmd1);     // cmd2=0x32 -> 40011, f32
  void pollProcessStatus(uint8_t cmd1);    // cmd2=0x40 -> 40013, u16
  void pollMachineStatus(uint8_t cmd1);    // cmd2=0x48 -> 40014, u16
  void pollDewTrigger(uint8_t cmd1);       // cmd2=0x80 -> 40017, f32
  void pollProcessTemp(uint8_t cmd1);      // cmd2=0x70 -> 40012, f32
  void pollReturnTemp(uint8_t cmd1);       // cmd2=0x72 -> 40015, f32
  void pollDewPoint(uint8_t cmd1);         // cmd2=0x7C -> 40016, f32

 private:
  static constexpr uint8_t kRegisterCount = kLastRegister - kFirstRegister + 1;
  uint16_t regs_[kRegisterCount] = {0};
  bool present_[kRegisterCount] = {false};
};
