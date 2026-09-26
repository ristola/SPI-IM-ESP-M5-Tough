// "node" role for M5Stack's small Atom-family dev boards (AtomS3 Lite or
// classic Atom Lite) - see platformio.ini's env:node_atoms3lite/
// env:node_atomlite for why these exist (a cheap way to develop the node
// role - SPI-CCP polling of FN-MAIN, Modbus TCP, ESP-NOW, RTS-NOW - before
// committing to a Tough, or eventually an M5Stack Atom Tough, per physical
// unit). One shared file rather than two near-duplicates: the two boards'
// only differences are a handful of pins (see pins.h's BOARD_ATOMS3LITE/
// BOARD_ATOMLITE sections) and one GPIO quirk (see setup()'s button
// pinMode). Which board this build targets is picked at compile time by
// the active environment's BOARD_ATOMS3LITE/BOARD_ATOMLITE flag.
//
// Deliberately NOT src/main.cpp with #ifdefs: that file's UI (touch
// gestures, Settings/Menu pages, header-bar icons) is all M5.Display/
// M5.Touch code with no ROLE_NODE/ROLE_GATEWAY guards, so it can't run on a
// board with neither. What's dropped vs. that file's ROLE_NODE path:
//   - Display/touch UI entirely - status is a single WS2812 RGB LED instead
//     (see updateLed() below).
//   - SD card (TFCard) and RTC+NTP (TimeManager) - both wrap Tough-only
//     hardware (SD slot, BM8563 RTC) neither of these boards has.
//   - WiFiManager captive-portal WiFi reconfiguration - main.cpp's version
//     is triggered from a touchscreen Settings page these boards don't
//     have. WiFi credentials here come from DeviceSettings (secrets.h
//     default, overridable only by whatever sets NVS directly - e.g. a
//     future button-triggered portal, not built yet).
// Everything else (DeviceSettings, SpiCcp/RS485Bus, DryerFD,
// DryerRegisters, EspNowLink, ArduinoOTA, and the new RTS-NOW node) is the
// same shared library code src/main.cpp uses.

#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <WiFi.h>

#include <cstring>

#include "BootDiag.h"
#include "DeviceSettings.h"
#include "Discovery.h"
#include "DryerPacket.h"
#include "DryerRegisters.h"
#include "DryerWebServer.h"
#include "EspNowLink.h"
#include "FirmwareVersion.h"
#include "ModbusRegisterMap.h"
#include "ModelFactory.h"
#include "NetworkManager.h"
#include "SpiCcp.h"
#include "pins.h"
#include "rtsnow_node.h"
#include "rtsnow_protocol.h"
#include "secrets.h"

namespace
{

  // Identifies this unit on the mesh - derived from the chip's own MAC so no
  // per-unit configuration is needed for a handful of nodes. Matches
  // main.cpp's ROLE_NODE nodeId().
  uint8_t nodeId()
  {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    return mac[5];
  }

  bool modbusStarted = false;

  // Broadcasts the full 40010-40041 data range regardless of which model
  // is active - each model (see lib/EquipmentModel and its subclasses)
  // populates a different subset of this shared register space via
  // ActiveModel->hasRegister()/getRegister(), so there's no per-model
  // list to maintain here anymore; unpopulated registers just read 0.
  // Deliberately starts at 40010, not 40001: 40001-40009 are the XBEE
  // SETUP block (software version, SPI station ID/baud, model type) -
  // device config DryerRegisters::begin() owns, not data any
  // EquipmentModel populates. Starting this loop at 40001 briefly
  // clobbered that whole block back to 0 every 2s, since
  // ActiveModel->getRegister() on an unpopulated register (which
  // 40001-40009 always are, from the model's point of view) returns 0 -
  // caught when a Modbus-written SPI Station ID kept silently reverting.
  constexpr uint16_t kFirstBroadcastReg = 40010;
  constexpr uint16_t kLastBroadcastReg = 40041;
  constexpr uint8_t kBroadcastRegCount = kLastBroadcastReg - kFirstBroadcastReg + 1;

  // TEMPORARY diagnostic registers - modbus 40042/40043, i.e. 2 slots past
  // the official sheet's own range (see ModbusRegisterMap.h, which now
  // fully occupies indices 0-40/registers 40001-40041) - remove once SPI-CCP
  // polling of the physical FD board is confirmed working. This board has no
  // serial access once wired to the dryer, so these exist to make
  // SpiCcp::lastOutcome() (see SpiCcp.h) and proof the poll loop is actually
  // cycling both visible over Modbus TCP instead.
  constexpr uint8_t kPollOutcomeRegister = 41;
  constexpr uint8_t kPollAttemptCountRegister = 42;

  void updateDryerReadings()
  {
    ActiveModel->pollNext();

    if (modbusStarted)
    {
      static uint16_t attemptCount = 0;
      Registers.set(kPollAttemptCountRegister, ++attemptCount);
      Registers.set(kPollOutcomeRegister, static_cast<uint16_t>(SpiIm.lastOutcome()));

      // 40009 SPI CRC Error - clamped to fit the 16-bit register rather
      // than silently wrapping if it ever gets that high.
      uint32_t crcErrors = SpiIm.crcErrorCount();
      Registers.set(ModbusReg::kSpiCrcError - ModbusReg::kFirstRegister,
                    static_cast<uint16_t>(crcErrors > 0xFFFF ? 0xFFFF : crcErrors));

#if defined(BOARD_ATOMS3LITE)
      // 40007 Board Temp, in degrees F (matches this project's existing
      // convention of engineering-unit-F for every other temperature
      // register, e.g. DryerFD's process/hopper temps) - only the
      // ESP32-S3 (this board) has an internal temperature_sensor
      // peripheral; the classic ESP32 in node_atomlite/the Tough doesn't,
      // so this is deliberately not built there.
      float boardTempF = temperatureRead() * 9.0f / 5.0f + 32.0f;
      Registers.set(ModbusReg::kBoardTemp - ModbusReg::kFirstRegister, static_cast<uint16_t>(boardTempF));
#endif
    }

    DryerPacket packet{};
    packet.nodeId = nodeId();
    packet.registerCount = kBroadcastRegCount;

    for (uint8_t i = 0; i < packet.registerCount; i++)
    {
      uint16_t modbusReg = kFirstBroadcastReg + i;
      uint16_t value = ActiveModel->getRegister(modbusReg);
      packet.registers[i] = value;
      if (modbusStarted)
      {
        Registers.set(modbusReg - 40001, value);
      }
    }

    EspNow.broadcast(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  }

  // Answers RTSNow-Gateway/its desktop app's on-demand "poll registers"
  // request (see rtsnow_node.h's registerBlockProvider) - see main.cpp's
  // identical fillRegisterBlock() for the full rationale (same register
  // range as Registers.get() already serves over Modbus TCP, just pulled
  // on demand instead of periodically).
  void fillRegisterBlock(RTSNOW_RegisterBlock &outBlock)
  {
    outBlock.startRegister = ModbusReg::kFirstRegister;

    // Only report as many registers as the active model actually
    // populates (see EquipmentModel::lastUsedRegister()) instead of
    // always padding out to the FD dryer's full 40001-40041 range with
    // zeros - e.g. the FC/FN Crystallizer never gets past 40028. Floored
    // at kSpiCrcError (40009, end of the XBEE SETUP config block) so
    // station ID/baud/model type are always included regardless of model.
    uint16_t lastReg = ActiveModel != nullptr ? ActiveModel->lastUsedRegister() : ModbusReg::kSpiCrcError;
    if (lastReg < ModbusReg::kSpiCrcError)
      lastReg = ModbusReg::kSpiCrcError;
    uint16_t count = lastReg - ModbusReg::kFirstRegister + 1;
    if (count > DryerPacket::kMaxRegisters)
      count = DryerPacket::kMaxRegisters;

    outBlock.registerCount = count;
    for (uint16_t i = 0; i < count; i++)
    {
      outBlock.values[i] = Registers.get(i);
    }
  }

  // Wired up as RTSNowNodeConfig::onGenericSetting - lets RTS-ESPNOW-
  // Gateway's desktop app push the same 4 fields the web dashboard's own
  // Equipment/Model/Station ID/Baud buttons already write (see
  // DryerWebServer.cpp's handleSetEquipment/handleSetModel/
  // handleWriteStationId/handleSetBaud for the same validation rules,
  // duplicated here rather than shared - matches this project's existing
  // pattern of each entry point (web, Modbus, and now RTS-NOW) keeping
  // its own small copy rather than a central dispatcher). Returns false
  // (rejected) for anything unrecognized or out of range, rather than
  // silently accepting garbage.
  //
  // Every branch also mirrors the change into the live Modbus register
  // table (Registers.set(...)), exactly like every web dashboard handler
  // already does (see that file's own "Keeps that register mirrored so
  // both surfaces agree" comment) - missing this the first time around
  // meant a setting still applied correctly (Settings.setXxx() persisted
  // fine) but read back as the *old* value on the very next poll, since
  // fillRegisterBlock()/the Modbus TCP server both read from this table,
  // not from Settings directly.
  bool onRemoteSetting(const RTSNOW_SettingPayload &setting)
  {
    if (strcmp(setting.key, "equipmentType") == 0 && setting.valueType == RTSNOW_SETTING_STRING)
    {
      if (strcmp(setting.stringValue, "Dryer") == 0)
      {
        Settings.setEquipmentType(DeviceSettings::EquipmentType::Dryer);
        Settings.setModel(DeviceSettings::kDryerModels[0]);
      }
      else if (strcmp(setting.stringValue, "Crystallizer") == 0)
      {
        Settings.setEquipmentType(DeviceSettings::EquipmentType::Crystallizer);
        Settings.setModel(DeviceSettings::kCrystallizerModels[0]);
      }
      else
      {
        // No "Ethernet" branch here - it's a model choice, not its own
        // equipment-type button (see DeviceSettings.h's kEthernetModels
        // comment), so it's only ever reachable via the "model" key below.
        return false;
      }
      SelectEquipmentModel();
      if (modbusStarted)
        Registers.set(ModbusReg::kModelType - ModbusReg::kFirstRegister, Settings.modelTypeCode());
      return true;
    }

    if (strcmp(setting.key, "model") == 0 && setting.valueType == RTSNOW_SETTING_STRING)
    {
      // ETHERNET is reachable regardless of the current equipment type,
      // same as picking FC vs. FD doesn't require switching equipment
      // type away from Dryer - deliberately does NOT call
      // setEquipmentType(), leaving whichever of Dryer/Crystallizer was
      // already set untouched. Checked before the current-equipment-
      // scoped lookup below so it works no matter which equipment type
      // is currently active.
      if (strcmp(setting.stringValue, DeviceSettings::kEthernetModels[0]) == 0)
      {
        Settings.setModel(DeviceSettings::kEthernetModels[0]);
        SelectEquipmentModel();
        if (modbusStarted)
          Registers.set(ModbusReg::kModelType - ModbusReg::kFirstRegister, Settings.modelTypeCode());
        return true;
      }

      const char *const *options;
      int count;
      switch (Settings.equipmentType())
      {
      case DeviceSettings::EquipmentType::Dryer:
        options = DeviceSettings::kDryerModels;
        count = DeviceSettings::kDryerModelCount;
        break;
      case DeviceSettings::EquipmentType::Crystallizer:
      default:
        options = DeviceSettings::kCrystallizerModels;
        count = DeviceSettings::kCrystallizerModelCount;
        break;
      }
      for (int i = 0; i < count; i++)
      {
        if (strcmp(setting.stringValue, options[i]) == 0)
        {
          Settings.setModel(options[i]);
          SelectEquipmentModel();
          if (modbusStarted)
            Registers.set(ModbusReg::kModelType - ModbusReg::kFirstRegister, Settings.modelTypeCode());
          return true;
        }
      }
      return false;
    }

    if (strcmp(setting.key, "spiAddress") == 0 && setting.valueType == RTSNOW_SETTING_INT32)
    {
      if (setting.intValue < 0 || setting.intValue > 255)
        return false;
      uint8_t address = static_cast<uint8_t>(setting.intValue);
      Settings.setSpiAddress(address);
      ActiveModel->begin(ActiveModel->devId(), address);
      if (modbusStarted)
        Registers.set(ModbusReg::kSpiStationId - ModbusReg::kFirstRegister, address);
      return true;
    }

    if (strcmp(setting.key, "spiBaudRate") == 0 && setting.valueType == RTSNOW_SETTING_INT32)
    {
#if !defined(BOARD_ATOMS3_POE)
      constexpr uint32_t kValidBauds[] = {1200, 2400, 4800, 9600, 19200};
      for (uint32_t b : kValidBauds)
      {
        if (static_cast<uint32_t>(setting.intValue) == b)
        {
          Settings.setSpiBaudRate(b);
          SpiIm.begin(b);
          if (modbusStarted)
            Registers.set(ModbusReg::kSpiBaudRate - ModbusReg::kFirstRegister, static_cast<uint16_t>(b));
          return true;
        }
      }
#endif
      return false;
    }

    return false;
  }

  // --- Status LED --------------------------------------------------------
  // One onboard WS2812, standing in for the Tough's header-bar icons. Single
  // pixel on both boards, matches RTSNow-Gateway's node-firmware (same
  // AtomS3 Lite board) - see that project's main.cpp for the precedent this
  // follows, including OTA taking priority over every other state.
  constexpr uint16_t kLedCount = 1;
  Adafruit_NeoPixel led(kLedCount, Pins::RGB_LED, NEO_GRB + NEO_KHZ800);

  bool otaInProgress = false; // WiFi OTA only - gates loop()'s early-return below (ArduinoOTA blocks synchronously for the whole transfer)
  bool otaLedActive = false;  // true during ANY OTA method (WiFi or ESP-NOW) - the only thing updateLed() itself checks

  // Recomputed every loop() from current state rather than event-driven, so
  // it self-corrects if e.g. WiFi drops without needing an explicit hook at
  // every call site that could affect it.
  void updateLed()
  {
    constexpr uint32_t kBlinkIntervalMs = 250;
    bool blinkOn = (millis() / kBlinkIntervalMs) % 2 == 0;

    uint32_t color;
    if (otaLedActive)
    {
      color = blinkOn ? led.Color(255, 255, 255) : 0; // fast white blink - highest priority
    }
    else if (!Network.isConnected())
    {
      color = led.Color(120, 0, 0); // solid red - no WiFi
    }
    else
    {
      color = led.Color(0, 90, 0); // solid green - WiFi up, node running normally
    }

    led.setPixelColor(0, color);
    led.show();
  }

  // Flash-backed (NVS via Preferences), not RTC_DATA_ATTR - tried RTC slow
  // memory first since it's the usual lighter-weight way to pass a flag
  // across ESP.restart() on ESP32, but real-hardware testing on this
  // AtomS3 Lite (ESP32-S3) proved it does NOT survive here: the flag read
  // back false immediately after boot despite being confirmed set (via
  // Serial) right before the restart call. Preferences is the same
  // mechanism DeviceSettings/rtsnow_node.cpp's friendlyName already rely on
  // for surviving a reboot, so it's a known-good fallback rather than a
  // guess.
  constexpr const char *kBootFlagsNamespace = "boot_flags";
  constexpr const char *kRebootedRemotelyKey = "remoteReboot";

  // Wired up as RTSNowNodeConfig::onBeforeReboot, called right before
  // rtsnow_node.cpp's ESP.restart(). Deliberately doesn't blink here and
  // return - a blink *before* the reset risks getting cut short mid-pattern
  // right as power/LED state actually drops, and would need real hardware
  // in hand to even confirm it looked right. Just flags the upcoming boot
  // instead; showRebootBlinkIfPending() below does the actual blinking
  // once the board is fully back up and stable, guaranteed to run to
  // completion.
  void markRebootedRemotely()
  {
    Preferences prefs;
    prefs.begin(kBootFlagsNamespace, /*readOnly=*/false);
    prefs.putBool(kRebootedRemotelyKey, true);
    prefs.end();
  }

  // Called once from setup(), after the LED is usable. 3 fast red blinks -
  // the visible "this boot was a remote reboot command, not a power-cycle
  // or crash" signal the user asked for.
  void showRebootBlinkIfPending()
  {
    Preferences prefs;
    prefs.begin(kBootFlagsNamespace, /*readOnly=*/false);
    bool pending = prefs.getBool(kRebootedRemotelyKey, false);
    if (pending)
      prefs.putBool(kRebootedRemotelyKey, false);
    prefs.end();

    if (!pending)
      return;
    for (uint8_t i = 0; i < 3; i++)
    {
      led.setPixelColor(0, led.Color(255, 0, 0));
      led.show();
      delay(150);
      led.setPixelColor(0, 0);
      led.show();
      delay(150);
    }
  }

  // --- OTA -----------------------------------------------------------------
  // Standard ArduinoOTA, same pattern as main.cpp's setupOta() - display
  // progress replaced with the LED's fast-white-blink state above. Hostname
  // is derived from BOARD_NAME (set per-environment in platformio.ini) so
  // both boards' units are distinguishable on the network without another
  // board-specific branch here.
  //
  // updateLed() is called directly from onStart/onProgress below, not left to
  // the outer loop()'s `if (otaInProgress)` check - confirmed by reading the
  // framework's ArduinoOTA.cpp that handle()'s internal _runUpdate() blocks
  // synchronously for the whole transfer (its own while loop reads and writes
  // each chunk, only calling back out to onProgress) and doesn't return to
  // our loop() until the update finishes or fails. Relying on loop() to poll
  // otaInProgress during that window never actually ran - real hardware
  // testing confirmed the LED stayed unchanged through an entire OTA push.
  void setupOta()
  {
    char hostname[32];
    snprintf(hostname, sizeof(hostname), "%s-node-%02x", BOARD_NAME, nodeId());
    ArduinoOTA.setHostname(hostname);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]()
                       {
    otaInProgress = true;
    otaLedActive = true;
    updateLed();
    Serial.println("OTA: starting"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
    updateLed();
    uint8_t percent = total > 0 ? static_cast<uint8_t>((progress * 100UL) / total) : 0;
    Serial.printf("OTA: %u%%\n", percent); });
    ArduinoOTA.onEnd([]()
                     {
    // Update library reboots the chip on its own shortly after this fires.
    Serial.println("OTA: complete - rebooting"); });
    ArduinoOTA.onError([](ota_error_t error)
                       {
    otaInProgress = false;
    otaLedActive = false;
    Serial.printf("OTA: error (code %d) - resuming normal operation\n", static_cast<int>(error)); });

    ArduinoOTA.begin();
    Serial.printf("OTA ready - hostname \"%s\"\n", hostname);
  }

  // Wired up as RTSNowNodeConfig::onOtaActiveChanged - fires on an
  // ESP-NOW-over-mesh OTA transfer's start/end, giving it the same LED
  // signal as WiFi OTA above. Deliberately does NOT touch otaInProgress:
  // that flag also gates loop()'s early-return (see loop() below), which
  // exists because ArduinoOTA blocks synchronously for its whole transfer -
  // the ESP-NOW OTA path is the opposite, driven chunk-by-chunk through
  // rtsnowNodeLoop() on every normal loop() tick, so short-circuiting loop()
  // here would stop it from ever receiving another chunk.
  void onEspNowOtaActiveChanged(bool active)
  {
    otaLedActive = active;
    updateLed();
  }

  // Onboard button held 3s = restart. No provisioning/credentials to "forget"
  // here (unlike RTSNow-Gateway's node-firmware), so this is just a
  // physical-access restart, matching main.cpp's Tough restart affordance
  // (there, a deliberate button tap on the Settings page) with the closest
  // equivalent these boards' single button can offer.
  constexpr uint32_t kHoldToRestartMs = 3000;
  uint32_t buttonDownSinceMs = 0;

  void checkButton()
  {
    bool pressed = digitalRead(Pins::BUTTON) == LOW;
    if (!pressed)
    {
      buttonDownSinceMs = 0;
      return;
    }
    if (buttonDownSinceMs == 0)
    {
      buttonDownSinceMs = millis();
      return;
    }
    if (millis() - buttonDownSinceMs >= kHoldToRestartMs)
    {
      Serial.println("Button held 3s - restarting");
      delay(200);
      ESP.restart();
    }
  }

} // namespace

void setup()
{
  Serial.begin(115200);
  BootDiag::begin(); // first thing after Serial - see BootDiag.h

#if defined(BOARD_ATOMLITE)
  // Classic ESP32's GPIO34-39 are input-only with no internal pull
  // resistor, so INPUT_PULLUP would silently do nothing here - the Atom
  // Lite board itself provides an external pull-up on this button pin
  // (confirmed by M5Stack's own example sketches using plain INPUT).
  pinMode(Pins::BUTTON, INPUT);
#else
  pinMode(Pins::BUTTON, INPUT_PULLUP);
#endif

  led.begin();
  led.setBrightness(50);
  showRebootBlinkIfPending(); // first thing shown, before the normal boot sequence below
  led.setPixelColor(0, led.Color(0, 0, 120)); // solid blue while WiFi connects
  led.show();

  Settings.begin();
  Discovery.begin(); // recover any devIds/commands a prior interrupted scan already found

  bool wifiOk = Network.begin(Settings.wifiSsid().c_str(), Settings.wifiPassword().c_str());
  if (wifiOk)
  {
#if defined(BOARD_ATOMS3_POE)
    Serial.printf("Ethernet OK: IP=%s\n", Network.localIP().toString().c_str());
#else
    Serial.printf("WiFi OK: SSID=%s IP=%s RSSI=%d\n", WiFi.SSID().c_str(), Network.localIP().toString().c_str(),
                  WiFi.RSSI());
#endif
  }
  else
  {
#if defined(BOARD_ATOMS3_POE)
    Serial.println("Ethernet FAIL");
#else
    Serial.println("WiFi FAIL");
#endif
  }

  if (wifiOk)
  {
    Registers.begin();
    modbusStarted = true;
    DryerWeb.begin(); // dashboard at http://<ip>/ - shows the poll queries and live RS485 data
  }

#if defined(BOARD_ATOMS3_POE)
  // This board has no RS-485 transceiver wired to anything at all (see
  // pins.h's RS485_RX/TX comment) - never SpiIm.begin() an RS485 UART
  // that doesn't exist, and force Model Type to ETHERNET (7) regardless
  // of whatever equipment type happened to be persisted in NVS from
  // before (e.g. this exact chip previously ran different firmware) -
  // this hardware genuinely cannot poll a dryer/crystallizer, so there's
  // no reasonable "leave it as whatever it was" default.
  if (Settings.model() != DeviceSettings::kEthernetModels[0])
  {
    Settings.setModelTypeCode(7);
  }
#else
  // RS485_RX/RS485_TX target this board's RS-485 transceiver - see pins.h's
  // per-board section for the pin derivation. Baud is configurable from the
  // web dashboard (see DryerWebServer) or Modbus register 40003 and
  // persists in Settings, so it must be read here rather than hardcoded -
  // otherwise a baud change would silently revert on every reboot.
  SpiIm.begin(Settings.spiBaudRate());
#endif
  // Picks the active model (Dryer/Crystallizer/Ethernet, then
  // FC/FD/FN/ADV/CD or FC-XTLR/FN-XTLR or ETHERNET) from Settings and
  // begin()s it with the right DevID - see lib/EquipmentModel/
  // ModelFactory. Equipment type and model are live-changeable from the
  // web dashboard too (see DryerWebServer), which calls this same
  // function again on change.
  SelectEquipmentModel();

  bool espNowOk = EspNow.begin();
  Serial.printf("ESP-NOW: %s\n", espNowOk ? "OK" : "FAIL");

  // Lets RTSNow-Gateway discover this device - see rtsnow_node.h
  // (shared RTSNow library, pulled in via platformio.ini's lib_extra_dirs).
  // Called unconditionally (even if the network failed): rtsnow_node.cpp
  // doesn't require it, it just reports ipv4Address=0 until connected,
  // same as a mesh-only node.
  char friendlyName[24];
  snprintf(friendlyName, sizeof(friendlyName), "SPI-IM Node (%s)", BOARD_NAME);
  RTSNowNodeConfig rtsnowConfig{};
  rtsnowConfig.projectName = "RTSNow";
#if defined(BOARD_ATOMS3_POE)
  rtsnowConfig.deviceTypeName = "Ethernet-Node";
#else
  rtsnowConfig.deviceTypeName = "RTSNow-UNADYN";
#endif
  rtsnowConfig.boardName = BOARD_NAME;
  rtsnowConfig.defaultFriendlyName = friendlyName;
  rtsnowConfig.firmwareVersionMajor = FirmwareVersion::kMajor;
  rtsnowConfig.firmwareVersionMinor = FirmwareVersion::kMinor;
  rtsnowConfig.registerBlockProvider = fillRegisterBlock;
  rtsnowConfig.onBeforeReboot = markRebootedRemotely;
  rtsnowConfig.onGenericSetting = onRemoteSetting;
  rtsnowConfig.onOtaActiveChanged = onEspNowOtaActiveChanged;
  // Network abstracts WiFi vs. Ethernet (see NetworkManager.cpp) - using
  // it here instead of WiFi.localIP() directly is what lets this same
  // line correctly report an Ethernet board's IP too, even though
  // WiFi.status() is never WL_CONNECTED there (see rtsnow_node.h's
  // ipv4AddressProvider comment for why this hook exists).
  rtsnowConfig.ipv4AddressProvider = []() -> uint32_t {
    return Network.isConnected() ? static_cast<uint32_t>(Network.localIP()) : 0;
  };
  rtsnowNodeBegin(rtsnowConfig);

  if (wifiOk)
  {
    setupOta();
  }

  Serial.printf("SPI address: 0x%02X   Modbus: %s\n", Settings.spiAddress(), modbusStarted ? "running" : "not running");
}

void loop()
{
  ArduinoOTA.handle();
  if (otaInProgress)
  {
    updateLed();
    delay(1);
    return;
  }

  Network.loop();
  Registers.task();
  Registers.processPendingWrite();
  DryerWeb.handleClient();
  rtsnowNodeLoop(); // no-op until rtsnowNodeBegin() has run
  checkButton();
  updateLed();

  // Runs as fast as loop() cycles (not gated by the 2s timer below) - a
  // brute-force scan is tens of thousands of attempts, so it needs every
  // iteration it can get. No-op when no scan is running.
  Discovery.task();

  // Normal round-robin polling pauses for the duration of a scan -
  // keeps the RS485 traffic and dashboard state unambiguous (only ever
  // one thing polling at a time), rather than relying on it being
  // technically harmless to interleave.
  static uint32_t lastUpdateMs = 0;
  if (!Discovery.isRunning() && millis() - lastUpdateMs > 2000)
  {
    lastUpdateMs = millis();
    updateDryerReadings();
  }

  delay(10);
}
