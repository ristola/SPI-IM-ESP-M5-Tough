#pragma once

#include <ModbusIP_ESP8266.h>

#include "DryerPacket.h"

// Exposes this node's dryer readings as Modbus holding registers over TCP
// (function codes 03/06/16 - the only ones this project needs). Register
// layout mirrors DryerPacket 1:1 so SPI-IM can eventually write straight
// into it, and the same values get broadcast over ESP-NOW.
//
// begin() also populates and manages the "XBEE SETUP" block (40001-40004 -
// see ModbusRegisterMap.h and DataSheets/Modbus Registers V2.pdf): software
// version, SPI station ID, SPI baud rate, and model type. The station ID
// and baud rate registers are live-writable from a remote Modbus client -
// this is the only remote configuration surface these headless boards
// (AtomS3 Lite/Atom Lite, no touchscreen) have, mirroring what the Tough's
// Gateway Settings page already does for spiAddress via touch.
class DryerRegisters {
 public:
  void begin();
  void task();  // call every loop()

  // Services one pending SPI-CCP write per call, if a remote Modbus client
  // has written a data register since the last call - see
  // DryerRegisters.cpp's onSetSpiRegister() for why the actual write
  // happens here (called from loop(), the same blocking budget
  // ActiveModel->pollNext() already uses) instead of inside the Modbus
  // onSetHreg callback: a slow/unresponsive SPI-CCP write blocking there
  // would delay the Modbus write's own TCP response, which a remote client
  // sees as a false-negative timeout even when the write actually
  // succeeded (or would have). Call every loop(), same as task() above.
  void processPendingWrite();

  void set(uint8_t index, uint16_t value);
  uint16_t get(uint8_t index);

  // Real per-register age, but only for the two XBEE SETUP registers
  // (40001-40009) that are genuinely live - 40007 Board Temp and 40009
  // SPI CRC Error, both re-set every updateDryerReadings() tick in
  // main.cpp/main_atom_node.cpp. Mirrors EquipmentModel::registerAgeMs()'s
  // own pattern/UINT32_MAX-means-never-set convention, but deliberately
  // scoped to just these two rather than every index in this table - the
  // rest of the XBEE SETUP block is either compile-time-constant
  // (Software Version) or only changes on an explicit config write, so a
  // blanket "time since last set()" for those would show a misleadingly
  // large/growing number for a value that's simply unchanged, not stale
  // (see DryerWebServer.cpp's own comment where this is consumed).
  // Returns UINT32_MAX for any other register, or for one of these two
  // that hasn't been set yet (e.g. Board Temp on a board with no
  // temperature_sensor peripheral - see main_atom_node.cpp's own
  // BOARD_ATOMS3LITE guard).
  uint32_t registerAgeMs(uint16_t modbusReg) const;

  // Called by onSetSpiRegister() - not meant for other callers, but needs
  // to be public since that's a free function, not a member.
  void queuePendingWrite(uint16_t modbusReg, float value);

 private:
  ModbusIP _mb;

  bool _hasPendingWrite = false;
  uint16_t _pendingWriteReg = 0;
  float _pendingWriteValue = 0;

  uint32_t _boardTempUpdatedMs = UINT32_MAX;
  uint32_t _crcErrorUpdatedMs = UINT32_MAX;
};

extern DryerRegisters Registers;
