#pragma once

#include <cstdint>

// Modbus holding-register layout - see DataSheets/Modbus Registers V2.pdf
// ("MODBUS REGISTERS Python Version 2.0" sheet, UNADYN FD column) and
// Reference/main.py|model.py (the prior XBee/MicroPython gateway this
// project ports from, which populated the exact same register numbers) for
// what each register actually carries. DryerRegisters is what exposes these
// over Modbus TCP; that class's set()/get() index into its own array by
// (register - kFirstRegister).
namespace ModbusReg {

constexpr uint16_t kFirstRegister = 40001;
constexpr uint16_t kLastRegister = 40041;  // FD's highest-numbered register on the sheet

// --- XBEE SETUP (40001-40009) ---
// Device/link identity & config - not dryer data. Named "XBEE" because the
// sheet predates this project (it documented Ristola's prior XBee/
// MicroPython gateway, see Reference/) - kept as-is since it's the
// established register numbering other tools may already expect.
constexpr uint16_t kSoftwareVersion = 40001;   // this project's firmware version - see FirmwareVersion.h; encoded (major*100)+minor, e.g. major=0/minor=1 -> 1 (matches Reference/main.py's "version=200" meaning v2.00)
constexpr uint16_t kSpiStationId = 40002;      // mirrors DeviceSettings::spiAddress() - writable over Modbus, live-applies to DryerFD::begin()
constexpr uint16_t kSpiBaudRate = 40003;       // raw baud value, one of 1200/2400/4800/9600/19200 per the sheet - writable over Modbus, live-applies to SpiIm.begin()
constexpr uint16_t kModelType = 40004;         // 0=FC,1=FD,2=FN,3=ADV,4=CD,5=FC-XTLR,6=FN-XTLR,7=ETHERNET (last 3 are this project's own extension) - mirrors DeviceSettings::modelTypeCode(); writable, live-applies via SelectEquipmentModel() (see DeviceSettings.h)
constexpr uint16_t kRssi = 40005;              // reserved - not implemented (no radio-link RSSI concept on this hardware, unlike the XBee gateway this replaces)
constexpr uint16_t kWriteEnable = 40006;       // reserved - not implemented
constexpr uint16_t kBoardTemp = 40007;         // implemented on node_atoms3lite only (ESP32-S3's internal temperature_sensor, via Arduino's temperatureRead()), converted to degrees F to match this project's other temperature registers - the classic ESP32 in node_atomlite/the Tough has no such peripheral at all, stays reserved (0) there
constexpr uint16_t kXbeeRadioVoltage = 40008;  // reserved - not implemented; no equivalent "read my own supply voltage" API exists on ESP32/S3 (unlike ESP8266's ESP.getVcc()) without external divider hardware, which isn't wired up
constexpr uint16_t kSpiCrcError = 40009;       // implemented - mirrors SpiCcp::crcErrorCount(), a running count of CRC mismatches since boot (see SpiCcp.h)

// --- STANDARD SPI-IM DATA (40010-40017) - common across all dryer models ---
constexpr uint16_t kProcessSetpoint = 40010;  // implemented (DryerFD, discrete poll) - writable: a remote Modbus write queues the new value (see DryerRegisters::onSetSpiRegister/processPendingWrite) and acks the Modbus write immediately, pushing it out over SPI-CCP (EquipmentModel::writeRegister()) from the next loop() iteration rather than blocking the Modbus response on that round-trip; the next round-robin poll of this register corrects the displayed value either way, whether or not the queued write actually succeeded
constexpr uint16_t kProcessDelta = 40011;     // implemented (DryerFD, discrete poll) - writable, same SPI-CCP push-through as kProcessSetpoint above
constexpr uint16_t kProcessTemp = 40012;      // implemented (DryerFD::pollBlanket)
constexpr uint16_t kProcessStatus = 40013;    // implemented (DryerFD, discrete poll)
constexpr uint16_t kMachineStatus = 40014;    // implemented (DryerFD, discrete poll) - writable, same SPI-CCP push-through as kProcessSetpoint above (0=off/1=on/3=clear alarms on the FN dryer per confirmed field usage)
constexpr uint16_t kReturnTemp = 40015;       // implemented (DryerFD::pollBlanket)
constexpr uint16_t kDewpoint = 40016;         // reserved - not present in the blanket poll's 25 fields (see DryerFD.h); a discrete cmd1=0x20/cmd2=0x7C poll exists per the SPI-CCP notes but isn't wired up yet
constexpr uint16_t kDewpointTrigger = 40017;  // implemented (DryerFD, discrete poll)

// --- VENDOR/MODEL SPECIFIC DATA (40018-40041), FD column ---
// Implemented via DryerFD::pollBlanket() (CMD1=0xC2, CMD2=0x2E) unless
// noted otherwise - see that function for the exact wire-trace mapping.
constexpr uint16_t kRegenTemp = 40018;         // implemented (DryerFD::pollBlanket)
constexpr uint16_t kRegenOutletTemp = 40019;   // implemented (DryerFD::pollBlanket)
constexpr uint16_t kDryerInletTemp = 40020;    // implemented (DryerFD::pollBlanket)
constexpr uint16_t kDryerOutletTemp = 40021;   // reserved - no corresponding field found in the blanket poll's captured wire trace (see DryerFD.h); not guessed at
constexpr uint16_t kProcess2Temp = 40022;      // implemented (DryerFD::pollBlanket)
constexpr uint16_t kReturn2Temp = 40023;       // implemented (DryerFD::pollBlanket)
constexpr uint16_t kInletTemp = 40024;         // reserved - no corresponding field found in the blanket poll's captured wire trace (see DryerFD.h); not guessed at
constexpr uint16_t kThroatTemp = 40025;        // implemented (DryerFD::pollBlanket)
constexpr uint16_t kLeftBedTemp = 40026;       // implemented (DryerFD::pollBlanket)
constexpr uint16_t kRightBedTemp = 40027;      // implemented (DryerFD::pollBlanket)
constexpr uint16_t kHopper1Temp = 40028;       // implemented (DryerFD::pollBlanket)
constexpr uint16_t kHopper2Temp = 40029;       // implemented (DryerFD::pollBlanket)
constexpr uint16_t kHopper3Temp = 40030;       // implemented (DryerFD::pollBlanket)
constexpr uint16_t kHopper4Temp = 40031;       // implemented (DryerFD::pollBlanket)
constexpr uint16_t kHopper5Temp = 40032;       // implemented (DryerFD::pollBlanket)
constexpr uint16_t kHopper6Temp = 40033;       // implemented (DryerFD::pollBlanket)
constexpr uint16_t kProcessDewpoint = 40034;   // implemented (DryerFD::pollBlanket) - value-confirmed against a real captured trace (0xFF96)
constexpr uint16_t kReturnDewpoint = 40035;    // implemented (DryerFD::pollBlanket) - position-inferred only, not independently value-confirmed
constexpr uint16_t kProcessAirflowCfm = 40036; // reserved - the blanket poll's word19 (see DryerFD::pollBlanket); real hardware trace disproved the earlier u32 assumption here, and reads 0 in the only sample seen, so it isn't safely distinguishable from the other reserved words below
constexpr uint16_t kProcessAirflowFpm = 40037; // reserved - blanket poll's word20, same caveat as kProcessAirflowCfm above
constexpr uint16_t kPressureSensor1 = 40038;   // reserved - blanket poll's word21, same caveat as kProcessAirflowCfm above
constexpr uint16_t kPressureSensor2 = 40039;   // reserved - blanket poll's word22, same caveat as kProcessAirflowCfm above
constexpr uint16_t kPressureSensor3 = 40040;   // reserved - blanket poll's word23, same caveat as kProcessAirflowCfm above
constexpr uint16_t kAnalogLevelSensor = 40041; // reserved - blanket poll's word24, same caveat as kProcessAirflowCfm above (words 25-28 past this are also unmapped/unnamed, see DryerFD::pollBlanket)

}  // namespace ModbusReg
