#include "DryerRegisters.h"

#include "DeviceSettings.h"
#include "FirmwareVersion.h"
#include "ModbusRegisterMap.h"
#include "ModelFactory.h"
#include "SpiCcp.h"

DryerRegisters Registers;

namespace {

constexpr uint32_t kValidSpiBauds[] = {1200, 2400, 4800, 9600, 19200};

bool isValidSpiBaud(uint32_t baud) {
  for (uint32_t b : kValidSpiBauds) {
    if (b == baud) return true;
  }
  return false;
}

// Fires when a remote Modbus client writes 40002 (SPI Station ID) - applies
// immediately, same as the Tough's Gateway Settings stepper does via touch.
// Keeps the active model's own DevID (dryers vs. crystallizers use
// different ones - see ModelFactory) rather than hardcoding one.
uint16_t onSetStationId(TRegister* reg, uint16_t val) {
  Settings.setSpiAddress(static_cast<uint8_t>(val));
  ActiveModel->begin(ActiveModel->devId(), static_cast<uint8_t>(val));
  return val;
}

// Fires when a remote Modbus client writes 40003 (SPI Baud Rate). Rejects
// (keeps the previous value) anything not in the sheet's documented set of
// 5 rates, rather than silently accepting an unsupported baud.
uint16_t onSetBaudRate(TRegister* reg, uint16_t val) {
  if (!isValidSpiBaud(val)) return reg->value;
  Settings.setSpiBaudRate(val);
  SpiIm.begin(val);
  return val;
}

// Fires when a remote Modbus client writes 40004 (Model Type) - switches
// equipment type + model and re-selects the active EquipmentModel
// immediately, same as the web dashboard's Equipment/Model buttons.
// Rejects (keeps the previous value) any code outside the documented
// 0-7 range (see DeviceSettings::modelTypeCode()'s comment) rather than
// silently accepting garbage.
uint16_t onSetModelType(TRegister* reg, uint16_t val) {
  if (val > 7) return reg->value;
  Settings.setModelTypeCode(static_cast<uint8_t>(val));
  SelectEquipmentModel();
  return val;
}

// Fires when a remote Modbus client writes one of the dryer/crystallizer
// data registers this project knows how to push out over SPI-CCP (see
// EquipmentModel::writeRegister()) - Process Setpoint (40010), Process
// Limit Delta (40011), Machine Status (40014). The modbus-esp8266 library
// calls onSetHreg once per register for both FC06 (single) and FC16
// (multiple) writes, so wiring it here covers both automatically.
//
// Deliberately does NOT call ActiveModel->writeRegister() here - that
// blocks on the real SPI-CCP SELECT handshake (selection confirm, then
// data block, then the tributary's ACK/NAK), and this callback runs
// synchronously inside the Modbus library's own request handling. Worst
// case is now longer still: SpiCcp::select() retries up to
// kMaxSelectRetries times on a garbled/missing reply (see its comment),
// so a fully exhausted retry budget can take several seconds, not the ~2s
// a single attempt cost when this comment was first written. Confirmed against a
// real Modbus client during development: it reported the *write's own TCP
// response* as a timeout purely because of that blocking, even on a write
// that had in fact gone out - a false negative from this callback taking
// too long to return, not from any actual comms failure. Queuing the write
// instead
// and doing the real SPI-CCP call from loop() (see
// DryerRegisters::processPendingWrite(), same blocking budget
// ActiveModel->pollNext() already uses every cycle) means the Modbus
// write's response always goes out immediately - matches the accepted
// value optimistically, same as every other onSetHreg callback in this
// file already does for its own register. The next round-robin poll of
// this same register (every model polls these at least every couple of
// pollNext() cycles) overwrites this with the tributary's own reported
// value regardless of whether the queued write actually succeeded, so
// this never drifts from truth for long even when it was wrong.
uint16_t onSetSpiRegister(TRegister* reg, uint16_t val) {
  uint16_t modbusReg = ModbusReg::kFirstRegister + reg->address.address;
  Registers.queuePendingWrite(modbusReg, static_cast<float>(val));
  return val;
}

}  // namespace

void DryerRegisters::begin() {
  _mb.server();
  for (uint8_t i = 0; i < DryerPacket::kMaxRegisters; i++) {
    _mb.addHreg(i);
  }

  // XBEE SETUP block (40001-40004) - see ModbusRegisterMap.h for why
  // 40005-40009 are left at 0 (unimplemented, inherited from the prior
  // XBee-based gateway this project replaces).
  set(ModbusReg::kSoftwareVersion - ModbusReg::kFirstRegister,
      static_cast<uint16_t>(FirmwareVersion::kMajor) * 100 + FirmwareVersion::kMinor);
  set(ModbusReg::kSpiStationId - ModbusReg::kFirstRegister, Settings.spiAddress());
  set(ModbusReg::kSpiBaudRate - ModbusReg::kFirstRegister, Settings.spiBaudRate());
  set(ModbusReg::kModelType - ModbusReg::kFirstRegister, Settings.modelTypeCode());

  _mb.onSetHreg(ModbusReg::kSpiStationId - ModbusReg::kFirstRegister, onSetStationId);
  _mb.onSetHreg(ModbusReg::kSpiBaudRate - ModbusReg::kFirstRegister, onSetBaudRate);
  _mb.onSetHreg(ModbusReg::kModelType - ModbusReg::kFirstRegister, onSetModelType);

  // Data registers this project can push out over SPI-CCP - see
  // EquipmentModel::writeRegister(). A remote Modbus client writing one of
  // these now reaches the physical equipment immediately instead of just
  // changing the in-memory copy until the next poll silently overwrote it.
  _mb.onSetHreg(ModbusReg::kProcessSetpoint - ModbusReg::kFirstRegister, onSetSpiRegister);
  _mb.onSetHreg(ModbusReg::kProcessDelta - ModbusReg::kFirstRegister, onSetSpiRegister);
  _mb.onSetHreg(ModbusReg::kMachineStatus - ModbusReg::kFirstRegister, onSetSpiRegister);
}

void DryerRegisters::task() { _mb.task(); }

void DryerRegisters::queuePendingWrite(uint16_t modbusReg, float value) {
  _pendingWriteReg = modbusReg;
  _pendingWriteValue = value;
  _hasPendingWrite = true;
}

void DryerRegisters::processPendingWrite() {
  if (!_hasPendingWrite) return;
  _hasPendingWrite = false;
  ActiveModel->writeRegister(_pendingWriteReg, _pendingWriteValue);
}

void DryerRegisters::set(uint8_t index, uint16_t value) {
  if (index >= DryerPacket::kMaxRegisters) return;
  // cbEnable(false) around this call is load-bearing, not cosmetic: modbus-
  // esp8266's Modbus::Reg(address, value) (what Hreg() calls under the
  // hood) invokes the exact same onSetHreg callback for this library-
  // internal write as it does for a genuine remote Modbus client write,
  // gated only by a single global cbEnabled flag with no way to tell the
  // two apart otherwise. Without this guard, every poll cycle's own
  // Machine Status (40014) mirror update - just echoing back what was
  // literally just read over SPI-CCP - re-triggered onSetSpiRegister() ->
  // queuePendingWrite() -> a real outbound SPI SELECT to the physical
  // dryer, on every single poll, not only when an operator actually
  // requested a change. Confirmed on real hardware: plugging in this node
  // while the dryer was already running from its own panel shut it down
  // (some early poll cycle's read - before the very first real SPI reply
  // settled - got echoed straight back out as a "Machine Status = 0/
  // stopped" command), while starting it from the desktop app instead (an
  // intentional external write) left it running, since the polled value
  // already matched whatever got echoed back afterward. Safe to toggle
  // here with no lock: this project's Modbus TCP handling
  // (DryerRegisters::task()) and this set() are both only ever called from
  // the single Arduino loop(), never concurrently.
  _mb.cbEnable(false);
  _mb.Hreg(index, value);
  _mb.cbEnable(true);
}

uint16_t DryerRegisters::get(uint8_t index) {
  if (index >= DryerPacket::kMaxRegisters) return 0;
  return _mb.Hreg(index);
}
