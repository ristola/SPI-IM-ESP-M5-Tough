#include "DeviceSettings.h"

#include "secrets.h"

DeviceSettings Settings;

// Out-of-class definitions for the static constexpr array members - needed
// for linkage wherever they're ODR-used (e.g. passed by pointer), since not
// every toolchain treats static constexpr members as implicitly inline yet.
constexpr const char* DeviceSettings::kDryerModels[];
constexpr const char* DeviceSettings::kCrystallizerModels[];
constexpr const char* DeviceSettings::kEthernetModels[];

namespace {
constexpr char kNamespace[] = "m5tough";
}

void DeviceSettings::begin() { prefs_.begin(kNamespace, /*readOnly=*/false); }

bool DeviceSettings::screenSaverEnabled() { return prefs_.getBool("ss_en", true); }
void DeviceSettings::setScreenSaverEnabled(bool enabled) { prefs_.putBool("ss_en", enabled); }

uint16_t DeviceSettings::screenSaverTimeoutSec() { return prefs_.getUShort("ss_sec", 120); }
void DeviceSettings::setScreenSaverTimeoutSec(uint16_t seconds) { prefs_.putUShort("ss_sec", seconds); }

DeviceSettings::EquipmentType DeviceSettings::equipmentType() {
  return static_cast<EquipmentType>(prefs_.getUChar("equip_type", static_cast<uint8_t>(EquipmentType::Dryer)));
}
void DeviceSettings::setEquipmentType(EquipmentType type) {
  prefs_.putUChar("equip_type", static_cast<uint8_t>(type));
}

String DeviceSettings::model() { return prefs_.getString("model", "FD"); }
void DeviceSettings::setModel(const String& model) { prefs_.putString("model", model); }

String DeviceSettings::deviceName() { return prefs_.getString("name", "Dryer-1"); }
void DeviceSettings::setDeviceName(const String& name) { prefs_.putString("name", name); }

uint8_t DeviceSettings::spiAddress() { return prefs_.getUChar("spi_addr", 0x20); }
void DeviceSettings::setSpiAddress(uint8_t address) { prefs_.putUChar("spi_addr", address); }

uint32_t DeviceSettings::spiBaudRate() { return prefs_.getUInt("spi_baud", 9600); }
void DeviceSettings::setSpiBaudRate(uint32_t baud) { prefs_.putUInt("spi_baud", baud); }

uint8_t DeviceSettings::modelTypeCode() {
  String current = model();
  if (current == kEthernetModels[0]) {
    return 7;
  }
  if (equipmentType() == EquipmentType::Crystallizer) {
    return current == "FN-XTLR" ? 6 : 5;  // default FC-XTLR
  }
  for (uint8_t i = 0; i < kDryerModelCount; i++) {
    if (current == kDryerModels[i]) return i;
  }
  return 1;  // FD
}

void DeviceSettings::setModelTypeCode(uint8_t code) {
  switch (code) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
      setEquipmentType(EquipmentType::Dryer);
      setModel(kDryerModels[code]);
      break;
    case 5:
      setEquipmentType(EquipmentType::Crystallizer);
      setModel(kCrystallizerModels[0]);
      break;
    case 6:
      setEquipmentType(EquipmentType::Crystallizer);
      setModel(kCrystallizerModels[1]);
      break;
    case 7:
      // Deliberately does NOT call setEquipmentType() - ETHERNET is a
      // model choice orthogonal to equipment type (see kEthernetModels'
      // comment in the header), so whichever of Dryer/Crystallizer was
      // already set stays as-is.
      setModel(kEthernetModels[0]);
      break;
    default:
      break;  // invalid code - ignored
  }
}

String DeviceSettings::wifiSsid() { return prefs_.getString("wifi_ssid", WIFI_SSID); }
String DeviceSettings::wifiPassword() { return prefs_.getString("wifi_pass", WIFI_PASSWORD); }
void DeviceSettings::setWifiCredentials(const String& ssid, const String& password) {
  prefs_.putString("wifi_ssid", ssid);
  prefs_.putString("wifi_pass", password);
}

bool DeviceSettings::wifiEnabled() { return prefs_.getBool("wifi_en", true); }
void DeviceSettings::setWifiEnabled(bool enabled) { prefs_.putBool("wifi_en", enabled); }
