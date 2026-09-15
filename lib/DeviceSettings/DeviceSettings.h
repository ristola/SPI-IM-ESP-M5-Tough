#pragma once

#include <Arduino.h>
#include <Preferences.h>

// Persisted (NVS/flash) settings, configurable from the on-screen Menu
// pages instead of requiring a reflash. WiFi credentials default to the
// compiled-in secrets.h values on first boot, but once changed via the
// WiFi Settings captive portal, the stored value takes over permanently.
//
// NOTE: "equipment type" and "model" are stored here so they survive
// reboots, but only the FD dryer polling path is actually implemented
// (see DryerFD) - changing these doesn't yet switch polling behavior.
class DeviceSettings {
 public:
  enum class EquipmentType : uint8_t { Dryer = 0, Crystallizer = 1 };

  // Valid model() strings per equipment type - single source of truth for
  // both the Tough's Gateway Settings page (main.cpp) and the web
  // dashboard (DryerWebServer), so the two can't drift out of sync. Each
  // string matches a concrete EquipmentModel subclass - see
  // lib/EquipmentModel/ModelFactory.
  static constexpr const char* kDryerModels[] = {"FC", "FD", "FN", "ADV", "CD"};
  static constexpr int kDryerModelCount = 5;
  static constexpr const char* kCrystallizerModels[] = {"FC-XTLR", "FN-XTLR"};
  static constexpr int kCrystallizerModelCount = 2;
  // ETHERNET is a model choice, not its own equipment type (an AtomS3 +
  // Atomic PoE Base/W5500, see pins.h's BOARD_ATOMS3_POE, that polls no
  // SPI-CCP equipment over RS-485 at all) - reachable regardless of
  // equipmentType(), same as picking FC vs. FD doesn't require switching
  // equipment type away from Dryer. modelTypeCode()/setModelTypeCode()
  // check for it ahead of the equipmentType()-based dispatch, and
  // ModelFactory::SelectEquipmentModel() does the same.
  static constexpr const char* kEthernetModels[] = {"ETHERNET"};
  static constexpr int kEthernetModelCount = 1;

  void begin();

  bool screenSaverEnabled();
  void setScreenSaverEnabled(bool enabled);

  uint16_t screenSaverTimeoutSec();
  void setScreenSaverTimeoutSec(uint16_t seconds);

  EquipmentType equipmentType();
  void setEquipmentType(EquipmentType type);

  String model();
  void setModel(const String& model);

  String deviceName();
  void setDeviceName(const String& name);

  uint8_t spiAddress();
  void setSpiAddress(uint8_t address);

  // One of 1200/2400/4800/9600/19200 - matches DataSheets/Modbus Registers
  // V2.pdf's "SPI Baud Rate" register (40003). Not validated here; callers
  // writing this (see DryerRegisters' Modbus write callback) are
  // responsible for only passing one of those five values.
  uint32_t spiBaudRate();
  void setSpiBaudRate(uint32_t baud);

  // 0=FC, 1=FD, 2=FN, 3=ADV, 4=CD match the sheet's "Model 0(FC)..Model
  // 4(CD)" columns and Reference/config.py's `model_type`; 5=FC-XTLR,
  // 6=FN-XTLR, and 7=ETHERNET are this project's own extension (the
  // original sheet predates crystallizer/Ethernet-node support and only
  // had 5 codes). 7 is checked first (model()=="ETHERNET" regardless of
  // equipmentType() - see kEthernetModels' comment); otherwise derived
  // from equipmentType()/model() so they can't disagree, falling back to
  // FD (1) or FC-XTLR (5) for an unrecognized model() string within the
  // current equipment type.
  uint8_t modelTypeCode();

  // Reverse of modelTypeCode() - sets model() from a single 0-7 code (see
  // modelTypeCode()'s comment for what each means). Silently does
  // nothing for a code outside 0-7. Codes 0-6 also set equipmentType() to
  // match (Dryer or Crystallizer); code 7 (ETHERNET) deliberately leaves
  // equipmentType() untouched - it's a model choice orthogonal to
  // equipment type, not its own category (see kEthernetModels' comment).
  // Does NOT itself re-select/begin() the active EquipmentModel - callers
  // (e.g. the web dashboard's write-register handler) need to call
  // ModelFactory's SelectEquipmentModel() afterward, same as any other
  // equipment/model change.
  void setModelTypeCode(uint8_t code);

  String wifiSsid();
  String wifiPassword();
  void setWifiCredentials(const String& ssid, const String& password);

 private:
  Preferences prefs_;
};

extern DeviceSettings Settings;
