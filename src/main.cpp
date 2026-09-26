#include <ArduinoOTA.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <esp_sleep.h>

#include <cstring>

#include "BootDiag.h"
#include "BootLogo.h"
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
#include "TFCard.h"
#include "TimeManager.h"
#include "pins.h"
#include "rtsnow_node.h"
#include "rtsnow_protocol.h"
#include "secrets.h"

#if defined(ROLE_GATEWAY)

void onDryerPacket(const uint8_t *senderMac, const uint8_t *data, size_t len)
{
  if (len != sizeof(DryerPacket))
    return;
  DryerPacket packet;
  memcpy(&packet, data, sizeof(packet));

  Serial.printf("Dryer %u: %u regs [", packet.nodeId, packet.registerCount);
  for (uint8_t i = 0; i < packet.registerCount; i++)
  {
    Serial.printf("%u ", packet.registers[i]);
  }
  Serial.println("]");

  M5.Display.printf("Dryer %u: %u regs\r\n", packet.nodeId, packet.registerCount);
}

#elif defined(ROLE_NODE)

// Identifies this unit on the mesh - derived from the chip's own MAC so no
// per-unit configuration is needed for a handful of nodes.
static uint8_t nodeId()
{
  uint8_t mac[6];
  WiFi.macAddress(mac);
  return mac[5];
}

static bool modbusStarted = false;

// Broadcasts the full 40010-40041 data range regardless of which model is
// active - each model (see lib/EquipmentModel and its subclasses)
// populates a different subset of this shared register space via
// ActiveModel->hasRegister()/getRegister(), so there's no per-model list
// to maintain here anymore; unpopulated registers just read 0.
// Deliberately starts at 40010, not 40001: 40001-40009 are the XBEE
// SETUP block (software version, SPI station ID/baud, model type) -
// device config DryerRegisters::begin() owns, not data any EquipmentModel
// populates. Starting this loop at 40001 briefly clobbered that whole
// block back to 0 every 2s, since ActiveModel->getRegister() on an
// unpopulated register (which 40001-40009 always are, from the model's
// point of view) returns 0 - caught when a Modbus-written SPI Station ID
// kept silently reverting.
static constexpr uint16_t kFirstBroadcastReg = 40010;
static constexpr uint16_t kLastBroadcastReg = 40041;
static constexpr uint8_t kBroadcastRegCount = kLastBroadcastReg - kFirstBroadcastReg + 1;

// Single source of truth for this node's dryer data, feeding both the
// direct-IP path (Modbus TCP holding registers) and the no-network path
// (ESP-NOW broadcast). Polls one SPI-CCP query per call (round-robin, see
// EquipmentModel::pollNext) rather than all of a model's queries at once,
// so a slow/unresponsive device can't stall WiFi/ESP-NOW/Modbus TCP for
// more than ~1s at a time.
static void updateDryerReadings()
{
  ActiveModel->pollNext();

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

  if (modbusStarted)
  {
    // 40009 SPI CRC Error - clamped to fit the 16-bit register rather than
    // silently wrapping if it ever gets that high. No Board Temp (40007)
    // here - the Tough uses the Core2/classic-ESP32 profile, which has no
    // internal temperature_sensor peripheral (only ESP32-S3 does).
    uint32_t crcErrors = SpiIm.crcErrorCount();
    Registers.set(ModbusReg::kSpiCrcError - ModbusReg::kFirstRegister,
                  static_cast<uint16_t>(crcErrors > 0xFFFF ? 0xFFFF : crcErrors));
  }

  EspNow.broadcast(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
}

// Answers RTSNow-Gateway/its desktop app's on-demand "poll registers"
// request (see rtsnow_node.h's registerBlockProvider) - the same live
// Modbus holding-register table Registers.get() already serves over
// Modbus TCP and EspNow.broadcast() above, just pulled on demand instead
// of periodically. Reports the full table (40001-40041 plus the 2 extra
// diagnostic slots past it - see DryerPacket::kMaxRegisters), not just the
// 40010+ range updateDryerReadings() broadcasts, since a human debugging
// over the desktop app benefits from also seeing the XBEE-SETUP block
// (station ID, baud, model type, CRC error count).
static void fillRegisterBlock(RTSNOW_RegisterBlock &outBlock)
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

// Wired up as RTSNowNodeConfig::onGenericSetting - see
// main_atom_node.cpp's identical onRemoteSetting() for the full
// rationale (lets RTSNow-Gateway's desktop app push the same 4
// fields the web dashboard's own Equipment/Model/Station ID/Baud buttons
// already write). No BOARD_ATOMS3_POE branch here - that board variant
// only exists in main_atom_node.cpp, the Tough always has RS-485.
// Every branch also mirrors the change into the live Modbus register
// table (Registers.set(...)), exactly like every web dashboard handler
// already does - missing this the first time around meant a setting
// still applied correctly (Settings.setXxx() persisted fine) but read
// back as the *old* value on the very next poll, since fillRegisterBlock()/
// the Modbus TCP server both read from this table, not from Settings
// directly. Guarded by modbusStarted, same as every other Registers.set()
// call in this file - it's never been begin()'d if WiFi never came up.
static bool onRemoteSetting(const RTSNOW_SettingPayload &setting)
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
    // same as picking FC vs. FD doesn't require switching equipment type
    // away from Dryer - deliberately does NOT call setEquipmentType(),
    // leaving whichever of Dryer/Crystallizer was already set untouched.
    // Checked before the current-equipment-scoped lookup below so it
    // works no matter which equipment type is currently active.
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
    return false;
  }

  return false;
}

// Wired up as RTSNowNodeConfig::onBeforeReboot - without this, a remote
// RTSNOW_REBOOT (from RTSNow-Gateway/its desktop app) would restart
// this unit with no warning at all, same gap the touch-triggered RESTART
// button on the Settings page already has (see its handler further down -
// straight to ESP.restart(), no message shown either). Fixing it here
// rather than there too since that one at least follows a deliberate touch
// the operator just made; this path can fire with no one even looking at
// the screen.
static void showRebootingScreen()
{
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextDatum(textdatum_t::middle_center);
  M5.Display.setTextColor(TFT_RED);
  M5.Display.drawString("REBOOTING", M5.Display.width() / 2, M5.Display.height() / 2);
  M5.Display.setTextDatum(textdatum_t::top_left);
  M5.Display.setTextColor(TFT_WHITE);
  delay(800); // long enough to actually be seen before the reset cuts the display off
}

#endif

// Icon header bar - the home screen is icons only, no text (see setup()).
// Status detail that used to be printed as text now lives on the Settings
// page instead (see enterSettings()).
static constexpr int32_t kHeaderHeight = 40;
static constexpr int32_t kIconY = 20;
static constexpr int32_t kMenuIconX = 20;
static constexpr int32_t kWifiIconX = 70;
static constexpr int32_t kSdIconX = 110;
static constexpr int32_t kMeshIconX = 150;

// The bar itself is a blue strip (distinct from the black body below);
// "off" icons use a dim grey so they still read against that background
// instead of disappearing the way TFT_DARKGREY would.
static constexpr uint16_t kHeaderBarColor = TFT_BLUE;
static constexpr uint16_t kHeaderBorderColor = TFT_LIGHTGREY;
static constexpr uint16_t kIconOnColor = TFT_WHITE;
static constexpr uint16_t kIconOffColor = lgfx::color565(60, 60, 60);

static bool sdOk = false;
static bool espNowOk = false;
static bool headerWifiIconState = false;
static String ntpStatusText = "";

// Hamburger menu icon, top-left corner. Not wired up to anything yet - the
// gear/Settings page covers the one page this UI has so far. Tap target
// mirrors the gear icon's hit box on the opposite corner.
static constexpr int32_t kMenuHitLeft = 0;
static constexpr int32_t kMenuHitTop = 0;
static constexpr int32_t kMenuHitRight = 52;
static constexpr int32_t kMenuHitBottom = 44;

static bool isInsideMenuIcon(int32_t x, int32_t y)
{
  return x >= kMenuHitLeft && x < kMenuHitRight && y >= kMenuHitTop && y < kMenuHitBottom;
}

static void drawMenuIcon(int32_t cx, int32_t cy)
{
  constexpr int32_t kBarWidth = 18;
  constexpr int32_t kBarHeight = 2;
  constexpr int32_t kBarGap = 6;
  for (int i = -1; i <= 1; i++)
  {
    M5.Display.fillRect(cx - kBarWidth / 2, cy + i * kBarGap - kBarHeight / 2, kBarWidth, kBarHeight,
                        kIconOnColor);
  }
}

static void drawWifiIcon(int32_t cx, int32_t cy, bool connected)
{
  uint16_t color = connected ? kIconOnColor : kIconOffColor;
  M5.Display.fillCircle(cx, cy + 7, 2, color);
  M5.Display.drawArc(cx, cy + 7, 7, 5, 200, 340, color);
  M5.Display.drawArc(cx, cy + 7, 12, 10, 200, 340, color);
}

static void drawSdIcon(int32_t cx, int32_t cy, bool ok)
{
  uint16_t color = ok ? kIconOnColor : kIconOffColor;
  int32_t x = cx - 7;
  int32_t y = cy - 9;
  M5.Display.fillRoundRect(x, y, 14, 18, 2, color);
  M5.Display.fillTriangle(x, y, x + 6, y, x, y + 6, kHeaderBarColor);
  M5.Display.fillRect(x + 3, y + 12, 2, 4, kHeaderBarColor);
  M5.Display.fillRect(x + 8, y + 12, 2, 4, kHeaderBarColor);
}

// Mesh/ESP-NOW icon: three linked nodes, distinct from the WiFi signal icon.
static void drawMeshIcon(int32_t cx, int32_t cy, bool ok)
{
  uint16_t color = ok ? kIconOnColor : kIconOffColor;
  int32_t r = 8;
  int32_t x0 = cx, y0 = cy - r;
  int32_t x1 = cx - r, y1 = cy + r / 2;
  int32_t x2 = cx + r, y2 = cy + r / 2;
  M5.Display.drawLine(x0, y0, x1, y1, color);
  M5.Display.drawLine(x1, y1, x2, y2, color);
  M5.Display.drawLine(x2, y2, x0, y0, color);
  M5.Display.fillCircle(x0, y0, 2, color);
  M5.Display.fillCircle(x1, y1, 2, color);
  M5.Display.fillCircle(x2, y2, 2, color);
}

// Settings gear icon, top-right corner of the header bar. Drawn as thin
// outline rings (hub + a ring-segment per tooth, both the same ~2px stroke
// weight) rather than a solid filled disc, to match the outline style of
// the WiFi/mesh icons instead of reading as a bold, unrelated blob. Built
// from fillArc ring-segments rather than hand-rotated triangles - simpler,
// and evenly-spaced segments look the same regardless of which way angle 0
// points, so there's no angle-convention guessing involved. The tap target
// is bigger than the icon itself for an easy-to-hit corner button.
static constexpr int32_t kGearCenterX = 300;
static constexpr int32_t kGearCenterY = 20;
static constexpr int32_t kGearBodyRadius = 7;
static constexpr int32_t kGearToothLen = 4;
static constexpr int32_t kGearStrokeWidth = 2;
static constexpr int32_t kGearToothCount = 8;
static constexpr float kGearToothHalfWidthDeg = 15.0f;

static constexpr int32_t kGearHitLeft = 268;
static constexpr int32_t kGearHitTop = 0;
static constexpr int32_t kGearHitRight = 320;
static constexpr int32_t kGearHitBottom = 44;

static bool isInsideGearIcon(int32_t x, int32_t y)
{
  return x >= kGearHitLeft && x < kGearHitRight && y >= kGearHitTop && y < kGearHitBottom;
}

static void drawGearIcon()
{
  M5.Display.drawArc(kGearCenterX, kGearCenterY, kGearBodyRadius, kGearBodyRadius - kGearStrokeWidth, 0, 360,
                     kIconOnColor);
  for (int i = 0; i < kGearToothCount; i++)
  {
    float centerDeg = i * (360.0f / kGearToothCount);
    int32_t outerR = kGearBodyRadius + kGearToothLen;
    M5.Display.fillArc(kGearCenterX, kGearCenterY, outerR, outerR - kGearStrokeWidth,
                       centerDeg - kGearToothHalfWidthDeg, centerDeg + kGearToothHalfWidthDeg, kIconOnColor);
  }
  M5.Display.fillCircle(kGearCenterX, kGearCenterY, 2, kIconOnColor); // axle dot, matches the WiFi icon's dot
}

// Redraws the whole header bar plus a blank body - the home screen's entire
// content. sdOk/espNowOk are snapshot once at boot (see setup()); WiFi is
// checked live here and again each loop() since it can drop and reconnect.
static void drawHeaderBar()
{
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.fillRect(0, 0, M5.Display.width(), kHeaderHeight, kHeaderBarColor);
  M5.Display.drawFastHLine(0, kHeaderHeight, M5.Display.width(), kHeaderBorderColor);

  drawMenuIcon(kMenuIconX, kIconY);
  headerWifiIconState = Network.isConnected();
  drawWifiIcon(kWifiIconX, kIconY, headerWifiIconState);
  drawSdIcon(kSdIconX, kIconY, sdOk);
  drawMeshIcon(kMeshIconX, kIconY, espNowOk);
  drawGearIcon();
}

// OTA (over-the-air) firmware updates via ArduinoOTA - lets a unit be
// reflashed over WiFi (e.g. `pio run -t upload --upload-protocol espota
// --upload-port <ip-or-hostname>.local`) without physically reaching its
// USB port. While an update is in progress, loop() skips everything else
// (touch gestures, dryer polling, WiFi reconnect) so nothing can interrupt
// or slow down the transfer - see the `otaActive` check in loop().
static bool otaActive = false;

static constexpr int32_t kOtaBarX = 40;
static constexpr int32_t kOtaBarY = 130;
static constexpr int32_t kOtaBarW = 240;
static constexpr int32_t kOtaBarH = 30;

static void drawOtaProgress(uint8_t percent)
{
  M5.Display.drawRoundRect(kOtaBarX, kOtaBarY, kOtaBarW, kOtaBarH, 6, TFT_WHITE);
  M5.Display.fillRoundRect(kOtaBarX + 2, kOtaBarY + 2, kOtaBarW - 4, kOtaBarH - 4, 4, TFT_BLACK);
  int32_t fillWidth = (kOtaBarW - 4) * percent / 100;
  if (fillWidth > 0)
  {
    M5.Display.fillRoundRect(kOtaBarX + 2, kOtaBarY + 2, fillWidth, kOtaBarH - 4, 4, TFT_BLUE);
  }

  char label[8];
  snprintf(label, sizeof(label), "%u%%", percent);
  M5.Display.fillRect(0, kOtaBarY + kOtaBarH + 10, M5.Display.width(), 30, TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextDatum(textdatum_t::top_center);
  M5.Display.drawString(label, M5.Display.width() / 2, kOtaBarY + kOtaBarH + 10);
  M5.Display.setTextDatum(textdatum_t::top_left);
}

static void otaOnStart()
{
  otaActive = true;
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(3);
  M5.Display.setTextDatum(textdatum_t::top_center);
  M5.Display.drawString("FIRMWARE UPDATING", M5.Display.width() / 2, 40);
  M5.Display.setTextDatum(textdatum_t::top_left);
  drawOtaProgress(0);
}

static void otaOnProgress(unsigned int progress, unsigned int total)
{
  uint8_t percent = total > 0 ? static_cast<uint8_t>((progress * 100UL) / total) : 0;
  drawOtaProgress(percent);
}

static void otaOnEnd()
{
  // The underlying Update library reboots the chip on its own shortly
  // after this fires - no need to call ESP.restart() here.
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(3);
  M5.Display.setTextDatum(textdatum_t::middle_center);
  M5.Display.drawString("Rebooting...", M5.Display.width() / 2, M5.Display.height() / 2);
  M5.Display.setTextDatum(textdatum_t::top_left);
}

static void otaOnError(ota_error_t error)
{
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("Firmware update failed");
  M5.Display.printf("Error code: %d\n", static_cast<int>(error));
  Serial.printf("OTA error [%d]\n", static_cast<int>(error));
  delay(2000);
  otaActive = false;
  drawHeaderBar();
}

static void setupOta()
{
#if defined(ROLE_GATEWAY)
  ArduinoOTA.setHostname("m5tough-gateway");
#elif defined(ROLE_NODE)
  ArduinoOTA.setHostname("m5tough-node");
#endif
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart(otaOnStart);
  ArduinoOTA.onProgress(otaOnProgress);
  ArduinoOTA.onEnd(otaOnEnd);
  ArduinoOTA.onError(otaOnError);
  ArduinoOTA.begin();
}

static void showBootSplash()
{
  M5.Display.fillScreen(TFT_BLACK);
  int32_t size = 200;
  M5.Display.drawPng(kBootLogoPng, kBootLogoPngLen, (M5.Display.width() - size) / 2,
                     (M5.Display.height() - size) / 2);
  delay(2000);
  M5.Display.fillScreen(TFT_BLACK);
}

void setup()
{
  Serial.begin(115200);
  BootDiag::begin(); // first thing after Serial - see BootDiag.h

  auto cfg = M5.config();
  M5.begin(cfg);

  showBootSplash();
  M5.Display.setTextSize(2);

  Settings.begin(); // must come before anything below reads a setting
  Discovery.begin(); // recover any devIds/commands a prior interrupted scan already found

  // The home screen is icons only (see drawHeaderBar()); all of this
  // diagnostic detail is still captured here, it just moved to the
  // Settings page (see enterSettings()) instead of being printed here.
  sdOk = SDCard.begin();

  Clock.begin(); // seed system clock from RTC, if present

  // Credentials default to secrets.h on first boot, but once changed via
  // the WiFi Settings captive portal (see runWifiCaptivePortal()), the
  // stored value in Settings takes over permanently.
  bool wifiOk = Network.begin(Settings.wifiSsid().c_str(), Settings.wifiPassword().c_str());
  if (wifiOk)
  {
    Serial.printf("WiFi OK: SSID=%s IP=%s RSSI=%d\n", WiFi.SSID().c_str(),
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }
  else
  {
    Serial.println("WiFi FAIL");
  }

  // The RTC keeps ticking across reboots as long as it has backup power, so
  // a fresh NTP sync is only needed the first time (or after the RTC's
  // backup battery has been out long enough to lose track of time). A
  // trustworthy RTC is left alone here - tapping the screen restarts the
  // unit (see checkTouchGestures()), which re-runs this check anyway.
  if (wifiOk)
  {
    if (!Clock.isRtcTrustworthy())
    {
      bool syncOk = Clock.syncFromNTP();
      ntpStatusText = syncOk ? "synced (RTC was reset)" : "sync FAILED (RTC was reset)";
      Serial.printf("NTP sync (RTC was reset): %s\n", syncOk ? "OK" : "FAIL");
    }
    else
    {
      ntpStatusText = "skipped (RTC OK)";
      Serial.println("NTP sync skipped: RTC already trustworthy");
    }
  }
  else
  {
    ntpStatusText = "skipped (no WiFi)";
  }

#if defined(ROLE_NODE)
  if (wifiOk)
  {
    Registers.begin();
    modbusStarted = true;
    DryerWeb.begin(); // dashboard at http://<ip>/ - shows the poll queries and live RS485 data
  }

  // The RS-485 port is the dryer's SPI-CCP link (see DataSheets/SPI
  // Protocol.pdf) - default baud/DEVID match Ristola Technical Services'
  // prior XBee gateway (Reference/config.py); baud/ADD are configurable
  // from the Gateway Settings menu page (see handleGatewaySettingsTouch())
  // and the web dashboard (see DryerWebServer) and persist in Settings, so
  // this must read the stored value rather than hardcoding it - otherwise
  // a baud change would silently revert on every reboot.
  SpiIm.begin(Settings.spiBaudRate());
  // Picks the active model (Dryer/Crystallizer, then FC/FD/FN/ADV/CD or
  // FC-XTLR/FN-XTLR) from Settings and begin()s it with the right DevID -
  // see lib/EquipmentModel/ModelFactory. Equipment type and model are
  // live-changeable from the Gateway Settings page (see
  // handleGatewaySettingsTouch()) and the web dashboard, both of which
  // call this same function again on change.
  SelectEquipmentModel();
#endif

  espNowOk = EspNow.begin();

#if defined(ROLE_GATEWAY)
  EspNow.onReceive(onDryerPacket);
#elif defined(ROLE_NODE)
  // Lets RTSNow-Gateway discover this device - see rtsnow_node.h
  // (shared RTSNow library, pulled in via platformio.ini's lib_extra_dirs).
  // Called unconditionally (even if WiFi failed): rtsnow_node.cpp doesn't
  // require WL_CONNECTED, it just reports ipv4Address=0 until it is, same
  // as a mesh-only node. Scoped to ROLE_NODE only - this project's
  // ROLE_GATEWAY is the Modbus TCP aggregation unit, a different concept
  // from RTSNow-Gateway's own AtomS3U hardware gateway, and isn't an
  // RTS-NOW node itself (yet - a reasonable follow-up).
  RTSNowNodeConfig rtsnowConfig{};
  rtsnowConfig.projectName = "RTSNow";
  rtsnowConfig.deviceTypeName = "RTSNow-UNADYN";
  rtsnowConfig.boardName = BOARD_NAME;
  rtsnowConfig.defaultFriendlyName = "SPI-IM Node";
  rtsnowConfig.firmwareVersionMajor = FirmwareVersion::kMajor;
  rtsnowConfig.firmwareVersionMinor = FirmwareVersion::kMinor;
  rtsnowConfig.registerBlockProvider = fillRegisterBlock;
  rtsnowConfig.onBeforeReboot = showRebootingScreen;
  rtsnowConfig.onGenericSetting = onRemoteSetting;
  // Network abstracts WiFi vs. Ethernet (see NetworkManager.cpp) - always
  // the Wi-Fi path on this board, but wired the same way as
  // main_atom_node.cpp for consistency (see rtsnow_node.h's
  // ipv4AddressProvider comment).
  rtsnowConfig.ipv4AddressProvider = []() -> uint32_t {
    return Network.isConnected() ? static_cast<uint32_t>(Network.localIP()) : 0;
  };
  rtsnowNodeBegin(rtsnowConfig);
#endif

  if (wifiOk)
  {
    setupOta();
  }

  drawHeaderBar();
}

// Touch UX, matching M5Stack's stock demo firmware: a quick tap restarts
// the unit; holding for 3 seconds brings up a slide-to-confirm power-off
// screen. The hold is done with the *same* touch that's still down when the
// 3s threshold fires, so that touch is ignored for dragging - the slider
// only arms itself once that touch is released, and a fresh touch is what
// actually drags the handle (sliding all the way to the right cuts power;
// releasing early cancels back to normal). A tap on the gear icon in the
// top-right corner opens a placeholder Settings page instead of restarting.
enum class ScreenMode
{
  NORMAL,
  POWER_OFF_WAIT_RELEASE,
  POWER_OFF_ARMED,
  SETTINGS,
  MENU,
  MENU_DEVICE,
  MENU_TIME,
  MENU_WIFI,
  MENU_GATEWAY
};
static ScreenMode screenMode = ScreenMode::NORMAL;
static bool touchActive = false;
static uint32_t touchStartMs = 0;
static int32_t touchStartX = 0;
static int32_t touchStartY = 0;
static int32_t handleX = 0;
static int32_t sliderX0 = 0;
static int32_t sliderX1 = 0;

// Screen saver only arms from the NORMAL home screen (not while browsing
// menus), and only blanks the display (M5.Display.sleep()) - the ESP32
// itself stays fully awake and running, unlike the power-off deep sleep
// path, so a tap wakes it instantly via the normal loop().
static bool screenSaverActive = false;
static uint32_t lastActivityMs = 0;

static constexpr uint32_t kHoldToConfirmMs = 3000;
static constexpr int32_t kHandleRadius = 26;
static constexpr int32_t kTrackMarginX = 20;
static constexpr int32_t kTrackY = 110;
static constexpr int32_t kTrackHeight = kHandleRadius * 2 + 8;

static void drawSliderTrack()
{
  M5.Display.drawRoundRect(kTrackMarginX, kTrackY, M5.Display.width() - 2 * kTrackMarginX, kTrackHeight,
                           kTrackHeight / 2, TFT_DARKGREY);
}

static void eraseSliderInterior()
{
  M5.Display.fillRoundRect(kTrackMarginX + 2, kTrackY + 2, M5.Display.width() - 2 * kTrackMarginX - 4,
                           kTrackHeight - 4, (kTrackHeight - 4) / 2, TFT_BLACK);
}

// Classic power-button glyph: a broken ring with a tick poking through the
// gap, drawn in white on the handle's red disc.
static void drawHandle(int32_t cx, int32_t cy)
{
  constexpr int32_t kOuterR = kHandleRadius - 8;
  constexpr int32_t kInnerR = kOuterR - 4;
  M5.Display.fillCircle(cx, cy, kHandleRadius, TFT_RED);
  M5.Display.drawArc(cx, cy, kOuterR, kInnerR, 0, 360, TFT_WHITE);
  M5.Display.fillRect(cx - 3, cy - kOuterR - 2, 6, kOuterR - kInnerR + 4, TFT_RED); // notch
  M5.Display.fillRect(cx - 1, cy - kOuterR - 4, 2, kOuterR, TFT_WHITE);             // tick
}

static void enterPowerOffConfirm()
{
  screenMode = ScreenMode::POWER_OFF_WAIT_RELEASE;
  sliderX0 = kTrackMarginX + kHandleRadius;
  sliderX1 = M5.Display.width() - kTrackMarginX - kHandleRadius;
  handleX = sliderX0;

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(30, 50);
  M5.Display.println("Slide to power off");
  drawSliderTrack();
  drawHandle(handleX, kTrackY + kTrackHeight / 2);
}

static void updateSliderDrag()
{
  int32_t x = M5.Touch.getDetail().x;
  if (x < sliderX0)
    x = sliderX0;
  if (x > sliderX1)
    x = sliderX1;
  if (x == handleX)
    return;

  handleX = x;
  eraseSliderInterior();
  drawSliderTrack();
  drawHandle(handleX, kTrackY + kTrackHeight / 2);
}

static void exitPowerOffConfirmToNormal()
{
  screenMode = ScreenMode::NORMAL;
  drawHeaderBar();
}

// Restart lives here now instead of on a plain home-screen tap - an
// accidental brush of the touchscreen shouldn't reboot a unit that might be
// mid-way through monitoring a running dryer. This is a deliberate button
// tap on a page you had to open on purpose.
static constexpr int32_t kRestartButtonX = 40;
static constexpr int32_t kRestartButtonY = 160;
static constexpr int32_t kRestartButtonW = 240;
static constexpr int32_t kRestartButtonH = 40;

static bool isInsideRestartButton(int32_t x, int32_t y)
{
  return x >= kRestartButtonX && x < kRestartButtonX + kRestartButtonW && y >= kRestartButtonY &&
         y < kRestartButtonY + kRestartButtonH;
}

// The Settings page is where all the diagnostic detail that used to be
// printed on the home screen now lives - the home screen itself stays
// icons-only (see drawHeaderBar()).
static void enterSettings()
{
  screenMode = ScreenMode::SETTINGS;
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("Settings");
  M5.Display.println();

  M5.Display.print("SD: ");
  M5.Display.print(sdOk ? "OK" : "FAIL");
  M5.Display.print("   ESP-NOW: ");
  M5.Display.println(espNowOk ? "OK" : "FAIL");

  if (Network.isConnected())
  {
    M5.Display.print("WiFi: ");
    M5.Display.println(WiFi.SSID());
    M5.Display.print("IP: ");
    M5.Display.println(WiFi.localIP());
  }
  else
  {
    M5.Display.println("WiFi: not connected");
  }

  M5.Display.print("NTP: ");
  M5.Display.println(ntpStatusText);

#if defined(ROLE_GATEWAY)
  M5.Display.println("Role: GATEWAY");
#elif defined(ROLE_NODE)
  M5.Display.print("Role: NODE   Modbus: ");
  M5.Display.println(modbusStarted ? "running" : "not running");
#endif

  M5.Display.println(Clock.nowString());

  M5.Display.fillRoundRect(kRestartButtonX, kRestartButtonY, kRestartButtonW, kRestartButtonH, 6, TFT_RED);
  M5.Display.setTextDatum(textdatum_t::middle_center);
  M5.Display.drawString("RESTART", kRestartButtonX + kRestartButtonW / 2, kRestartButtonY + kRestartButtonH / 2);
  M5.Display.setTextDatum(textdatum_t::top_left);

  M5.Display.setCursor(10, kRestartButtonY + kRestartButtonH + 10);
  M5.Display.println("(tap elsewhere to go back)");
}

static void exitSettingsToNormal()
{
  screenMode = ScreenMode::NORMAL;
  drawHeaderBar();
}

static bool isInsideBox(int32_t px, int32_t py, int32_t x, int32_t y, int32_t w, int32_t h)
{
  return px >= x && px < x + w && py >= y && py < y + h;
}

// The hamburger Menu is a separate navigation tree from the gear's
// Settings/diagnostics page: Menu -> one of 4 sub-pages -> back to Menu ->
// back to Home. Each sub-page's "tap elsewhere" returns one level up
// (to the Menu list), not all the way home, matching normal drill-down nav.

static constexpr int32_t kMenuRowH = 40;
static constexpr int32_t kMenuRowY0 = 50;
static constexpr int32_t kMenuRowCount = 4;
static const char *const kMenuLabels[kMenuRowCount] = {"Device Settings", "Time Setup", "WiFi Settings",
                                                       "Gateway Settings"};

static void drawMenuList()
{
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("Menu");

  for (int i = 0; i < kMenuRowCount; i++)
  {
    int32_t y = kMenuRowY0 + i * kMenuRowH;
    M5.Display.drawFastHLine(10, y, M5.Display.width() - 20, TFT_DARKGREY);
    M5.Display.setCursor(20, y + 12);
    M5.Display.println(kMenuLabels[i]);
  }
  M5.Display.drawFastHLine(10, kMenuRowY0 + kMenuRowCount * kMenuRowH, M5.Display.width() - 20, TFT_DARKGREY);

  M5.Display.setCursor(10, kMenuRowY0 + kMenuRowCount * kMenuRowH + 15);
  M5.Display.println("(tap elsewhere to go back)");
}

static void enterMenu()
{
  screenMode = ScreenMode::MENU;
  drawMenuList();
}

static int menuRowAt(int32_t y)
{
  if (y < kMenuRowY0)
    return -1;
  int row = (y - kMenuRowY0) / kMenuRowH;
  if (row < 0 || row >= kMenuRowCount)
    return -1;
  return row;
}

// --- Device Settings: screen saver timeout + enable -------------------

static constexpr int32_t kSsStepperSize = 36;
static constexpr int32_t kSsMinusX = 180;
static constexpr int32_t kSsPlusX = 230;
static constexpr int32_t kSsRowY = 60;
static constexpr int32_t kSsCheckboxX = 150;
static constexpr int32_t kSsCheckboxY = 105;
static constexpr int32_t kSsCheckboxSize = 30;
static constexpr int32_t kSsCheckRowY = 110;

static void drawDeviceSettings()
{
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("Device Settings");

  M5.Display.setCursor(10, kSsRowY + 8);
  M5.Display.printf("Screen Saver: %3u s", Settings.screenSaverTimeoutSec());
  M5.Display.drawRoundRect(kSsMinusX, kSsRowY, kSsStepperSize, kSsStepperSize, 4, TFT_WHITE);
  M5.Display.drawRoundRect(kSsPlusX, kSsRowY, kSsStepperSize, kSsStepperSize, 4, TFT_WHITE);
  M5.Display.setTextDatum(textdatum_t::middle_center);
  M5.Display.drawString("-", kSsMinusX + kSsStepperSize / 2, kSsRowY + kSsStepperSize / 2);
  M5.Display.drawString("+", kSsPlusX + kSsStepperSize / 2, kSsRowY + kSsStepperSize / 2);
  M5.Display.setTextDatum(textdatum_t::top_left);

  M5.Display.setCursor(10, kSsCheckRowY + 6);
  M5.Display.print("Enable:");
  M5.Display.drawRect(kSsCheckboxX, kSsCheckboxY, kSsCheckboxSize, kSsCheckboxSize, TFT_WHITE);
  if (Settings.screenSaverEnabled())
  {
    M5.Display.fillRect(kSsCheckboxX + 5, kSsCheckboxY + 5, kSsCheckboxSize - 10, kSsCheckboxSize - 10, TFT_WHITE);
  }

  M5.Display.setCursor(10, 200);
  M5.Display.println("(tap elsewhere to go back)");
}

static void enterDeviceSettings()
{
  screenMode = ScreenMode::MENU_DEVICE;
  drawDeviceSettings();
}

static void handleDeviceSettingsTouch(int32_t x, int32_t y)
{
  if (isInsideBox(x, y, kSsMinusX, kSsRowY, kSsStepperSize, kSsStepperSize))
  {
    uint16_t t = Settings.screenSaverTimeoutSec();
    if (t > 30)
      Settings.setScreenSaverTimeoutSec(t - 30);
    drawDeviceSettings();
  }
  else if (isInsideBox(x, y, kSsPlusX, kSsRowY, kSsStepperSize, kSsStepperSize))
  {
    uint16_t t = Settings.screenSaverTimeoutSec();
    if (t < 600)
      Settings.setScreenSaverTimeoutSec(t + 30);
    drawDeviceSettings();
  }
  else if (isInsideBox(x, y, kSsCheckboxX, kSsCheckboxY, kSsCheckboxSize, kSsCheckboxSize))
  {
    Settings.setScreenSaverEnabled(!Settings.screenSaverEnabled());
    drawDeviceSettings();
  }
  else
  {
    screenMode = ScreenMode::MENU;
    drawMenuList();
  }
}

// --- Time Setup: RTC time + manual NTP sync ----------------------------

static constexpr int32_t kNtpButtonX = 40;
static constexpr int32_t kNtpButtonY = 110;
static constexpr int32_t kNtpButtonW = 240;
static constexpr int32_t kNtpButtonH = 40;
static constexpr int32_t kNtpResultY = kNtpButtonY + kNtpButtonH + 20;

static void drawTimeSetup()
{
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("Time Setup");
  M5.Display.println();

  M5.Display.println("RTC Time:");
  M5.Display.println(Clock.nowString());

  M5.Display.fillRoundRect(kNtpButtonX, kNtpButtonY, kNtpButtonW, kNtpButtonH, 6, TFT_BLUE);
  M5.Display.setTextDatum(textdatum_t::middle_center);
  M5.Display.drawString("NTP UPDATE", kNtpButtonX + kNtpButtonW / 2, kNtpButtonY + kNtpButtonH / 2);
  M5.Display.setTextDatum(textdatum_t::top_left);

  M5.Display.setCursor(10, kNtpResultY + 30);
  M5.Display.println("(tap elsewhere to go back)");
}

static void enterTimeSetup()
{
  screenMode = ScreenMode::MENU_TIME;
  drawTimeSetup();
}

static void handleTimeSetupTouch(int32_t x, int32_t y)
{
  if (isInsideBox(x, y, kNtpButtonX, kNtpButtonY, kNtpButtonW, kNtpButtonH))
  {
    M5.Display.fillRect(0, kNtpResultY, M5.Display.width(), 20, TFT_BLACK);
    M5.Display.setCursor(10, kNtpResultY);
    if (!Network.isConnected())
    {
      M5.Display.println("No WiFi connection");
      return;
    }
    M5.Display.println("Syncing...");
    bool ok = Clock.syncFromNTP();
    drawTimeSetup(); // refresh with the newly-synced RTC time
    M5.Display.setCursor(10, kNtpResultY);
    M5.Display.println(ok ? "Synced OK" : "Sync FAILED");
  }
  else
  {
    screenMode = ScreenMode::MENU;
    drawMenuList();
  }
}

// --- WiFi Settings: captive-portal reconfiguration ---------------------

static WiFiManager wifiManager;

static constexpr int32_t kWifiSetupButtonX = 40;
static constexpr int32_t kWifiSetupButtonY = 170;
static constexpr int32_t kWifiSetupButtonW = 240;
static constexpr int32_t kWifiSetupButtonH = 40;

static void drawWifiSettings()
{
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("WiFi Settings");
  M5.Display.println();
  M5.Display.println("To reconfigure:");
  M5.Display.println("1. Tap START SETUP");
  M5.Display.println("2. On phone, join WiFi");
  M5.Display.println("   \"M5Tough-Setup\"");
  M5.Display.println("3. Setup page opens -");
  M5.Display.println("   pick network, enter");
  M5.Display.println("   its password");

  M5.Display.fillRoundRect(kWifiSetupButtonX, kWifiSetupButtonY, kWifiSetupButtonW, kWifiSetupButtonH, 6, TFT_BLUE);
  M5.Display.setTextDatum(textdatum_t::middle_center);
  M5.Display.drawString("START SETUP", kWifiSetupButtonX + kWifiSetupButtonW / 2,
                        kWifiSetupButtonY + kWifiSetupButtonH / 2);
  M5.Display.setTextDatum(textdatum_t::top_left);
}

static void enterWifiSettings()
{
  screenMode = ScreenMode::MENU_WIFI;
  drawWifiSettings();
}

// Blocks (up to 180s) while the captive portal is active - deliberately
// triggered and rare enough that pausing everything else (dryer polling,
// ESP-NOW, Modbus TCP) for the duration is an acceptable tradeoff, same as
// the OTA update screen.
static void runWifiCaptivePortal()
{
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("Setup portal active");
  M5.Display.println();
  M5.Display.println("Join WiFi network:");
  M5.Display.println("  M5Tough-Setup");
  M5.Display.println();
  M5.Display.println("Waiting for setup...");

  wifiManager.setConfigPortalTimeout(180);
  bool connected = wifiManager.startConfigPortal("M5Tough-Setup");

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(10, 10);
  if (connected)
  {
    Settings.setWifiCredentials(WiFi.SSID(), WiFi.psk());
    M5.Display.println("WiFi configured!");
    M5.Display.println();
    M5.Display.print("Connected to: ");
    M5.Display.println(WiFi.SSID());
    Serial.printf("WiFi captive portal: connected to %s\n", WiFi.SSID().c_str());
  }
  else
  {
    M5.Display.println("Setup timed out or");
    M5.Display.println("was cancelled.");
    Serial.println("WiFi captive portal: timed out/cancelled");
  }
  delay(3000);
  drawWifiSettings();
}

static void handleWifiSettingsTouch(int32_t x, int32_t y)
{
  if (isInsideBox(x, y, kWifiSetupButtonX, kWifiSetupButtonY, kWifiSetupButtonW, kWifiSetupButtonH))
  {
    runWifiCaptivePortal();
  }
  else
  {
    screenMode = ScreenMode::MENU;
    drawMenuList();
  }
}

// --- Gateway Settings: which dryer/crystallizer this unit talks to -----
// Equipment Type and Model select the active EquipmentModel subclass (see
// lib/EquipmentModel/ModelFactory) - changing either re-selects and
// re-begin()s it immediately, same live-apply pattern Address already
// used. Name has no on-screen keyboard yet, so it isn't editable from here.


static constexpr int32_t kGatewayRowH = 40;
static constexpr int32_t kEquipRowY = 60;
static constexpr int32_t kModelRowY = 100;
static constexpr int32_t kNameRowY = 140;
static constexpr int32_t kAddrRowY = 180;
static constexpr int32_t kAddrStepperSize = 36;
static constexpr int32_t kAddrMinusX = 180;
static constexpr int32_t kAddrPlusX = 230;

static const char *equipmentTypeLabel(DeviceSettings::EquipmentType t)
{
  return t == DeviceSettings::EquipmentType::Dryer ? "Dryer" : "Crystallizer";
}

static void drawGatewaySettings()
{
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("Gateway Settings");

  M5.Display.setCursor(10, kEquipRowY);
  M5.Display.printf("Equipment: %s", equipmentTypeLabel(Settings.equipmentType()));

  M5.Display.setCursor(10, kModelRowY);
  M5.Display.printf("Model: %s", Settings.model().c_str());

  M5.Display.setCursor(10, kNameRowY);
  M5.Display.printf("Name: %s", Settings.deviceName().c_str());

  M5.Display.setCursor(10, kAddrRowY);
  M5.Display.printf("Address: %u", Settings.spiAddress());
  M5.Display.drawRoundRect(kAddrMinusX, kAddrRowY - 6, kAddrStepperSize, kAddrStepperSize, 4, TFT_WHITE);
  M5.Display.drawRoundRect(kAddrPlusX, kAddrRowY - 6, kAddrStepperSize, kAddrStepperSize, 4, TFT_WHITE);
  M5.Display.setTextDatum(textdatum_t::middle_center);
  M5.Display.drawString("-", kAddrMinusX + kAddrStepperSize / 2, kAddrRowY - 6 + kAddrStepperSize / 2);
  M5.Display.drawString("+", kAddrPlusX + kAddrStepperSize / 2, kAddrRowY - 6 + kAddrStepperSize / 2);
  M5.Display.setTextDatum(textdatum_t::top_left);

  M5.Display.setCursor(10, 220);
  M5.Display.println("(tap elsewhere to go back)");
}

static void enterGatewaySettings()
{
  screenMode = ScreenMode::MENU_GATEWAY;
  drawGatewaySettings();
}

static void handleGatewaySettingsTouch(int32_t x, int32_t y)
{
  if (isInsideBox(x, y, kAddrMinusX, kAddrRowY - 6, kAddrStepperSize, kAddrStepperSize))
  {
    uint8_t addr = Settings.spiAddress();
    if (addr > 0x20)
      addr--;
    Settings.setSpiAddress(addr);
#if defined(ROLE_NODE)
    ActiveModel->begin(ActiveModel->devId(), addr); // takes effect immediately, no restart needed
#endif
    drawGatewaySettings();
  }
  else if (isInsideBox(x, y, kAddrPlusX, kAddrRowY - 6, kAddrStepperSize, kAddrStepperSize))
  {
    uint8_t addr = Settings.spiAddress();
    if (addr < 0xFF)
      addr++;
    Settings.setSpiAddress(addr);
#if defined(ROLE_NODE)
    ActiveModel->begin(ActiveModel->devId(), addr);
#endif
    drawGatewaySettings();
  }
  else if (y >= kEquipRowY - 10 && y < kEquipRowY - 10 + kGatewayRowH)
  {
    auto next = Settings.equipmentType() == DeviceSettings::EquipmentType::Dryer
                    ? DeviceSettings::EquipmentType::Crystallizer
                    : DeviceSettings::EquipmentType::Dryer;
    Settings.setEquipmentType(next);
    // Switching equipment type invalidates whatever Model string was
    // stored for the old type (e.g. "FD" isn't a valid crystallizer
    // model) - reset to that type's first option so SelectEquipmentModel()
    // below doesn't fall through to a default that doesn't match what's
    // displayed.
    Settings.setModel(next == DeviceSettings::EquipmentType::Dryer ? DeviceSettings::kDryerModels[0]
                                                                    : DeviceSettings::kCrystallizerModels[0]);
#if defined(ROLE_NODE)
    SelectEquipmentModel();
#endif
    drawGatewaySettings();
  }
  else if (y >= kModelRowY - 10 && y < kModelRowY - 10 + kGatewayRowH)
  {
    bool isDryer = Settings.equipmentType() == DeviceSettings::EquipmentType::Dryer;
    const char *const *options = isDryer ? DeviceSettings::kDryerModels : DeviceSettings::kCrystallizerModels;
    int count = isDryer ? DeviceSettings::kDryerModelCount : DeviceSettings::kCrystallizerModelCount;
    String current = Settings.model();
    int idx = 0;
    for (int i = 0; i < count; i++)
    {
      if (current == options[i])
      {
        idx = i;
        break;
      }
    }
    idx = (idx + 1) % count;
    Settings.setModel(options[idx]);
#if defined(ROLE_NODE)
    SelectEquipmentModel();
#endif
    drawGatewaySettings();
  }
  else
  {
    screenMode = ScreenMode::MENU;
    drawMenuList();
  }
}

static void checkTouchGestures()
{
  bool touchDown = M5.Touch.getCount() > 0;

  if (touchDown && !touchActive)
  {
    touchStartMs = millis();
    auto detail = M5.Touch.getDetail();
    touchStartX = detail.x;
    touchStartY = detail.y;
    lastActivityMs = touchStartMs;

    if (screenSaverActive)
    {
      // Wake-up tap: consumed entirely, doesn't fall through to any
      // gesture logic (so it can't also register as e.g. a gear tap).
      screenSaverActive = false;
      M5.Display.wakeup();
      drawHeaderBar();
      touchActive = touchDown;
      return;
    }
  }

  if (touchDown)
  {
    if (screenMode == ScreenMode::NORMAL && millis() - touchStartMs >= kHoldToConfirmMs)
    {
      enterPowerOffConfirm();
    }
    else if (screenMode == ScreenMode::POWER_OFF_ARMED)
    {
      updateSliderDrag();
    }
    // POWER_OFF_WAIT_RELEASE: ignore this touch entirely until it's released
  }

  if (!touchDown && touchActive)
  {
    if (screenMode == ScreenMode::POWER_OFF_WAIT_RELEASE)
    {
      // The touch that triggered the hold is gone - arm the slider so the
      // *next* touch is what actually drags it.
      screenMode = ScreenMode::POWER_OFF_ARMED;
    }
    else if (screenMode == ScreenMode::POWER_OFF_ARMED)
    {
      if (handleX >= sliderX1 - 4)
      {
        M5.Display.fillScreen(TFT_BLACK);
        M5.Display.setCursor(40, 100);
        M5.Display.println("Powering off...");
        delay(500);
        // A true PMIC power-off (M5.Power.powerOff()) kills the touch IC's
        // supply too, so nothing could ever notice a tap to turn it back
        // on - only the physical power button would work. Deep sleep with
        // the touch INT pin as a wake source keeps the touch IC powered,
        // so a tap wakes the chip (and re-runs setup(), same as a restart).
        M5.Display.sleep();
        esp_sleep_enable_ext0_wakeup(Pins::TOUCH_INT, 0);
        esp_deep_sleep_start();
      }
      else
      {
        exitPowerOffConfirmToNormal();
      }
    }
    else if (screenMode == ScreenMode::SETTINGS)
    {
      if (isInsideRestartButton(touchStartX, touchStartY))
      {
        Serial.println("Restart button pressed - restarting");
        delay(200);
        ESP.restart();
      }
      else
      {
        exitSettingsToNormal();
      }
    }
    else if (screenMode == ScreenMode::MENU)
    {
      switch (menuRowAt(touchStartY))
      {
      case 0:
        enterDeviceSettings();
        break;
      case 1:
        enterTimeSetup();
        break;
      case 2:
        enterWifiSettings();
        break;
      case 3:
        enterGatewaySettings();
        break;
      default:
        screenMode = ScreenMode::NORMAL;
        drawHeaderBar();
        break;
      }
    }
    else if (screenMode == ScreenMode::MENU_DEVICE)
    {
      handleDeviceSettingsTouch(touchStartX, touchStartY);
    }
    else if (screenMode == ScreenMode::MENU_TIME)
    {
      handleTimeSetupTouch(touchStartX, touchStartY);
    }
    else if (screenMode == ScreenMode::MENU_WIFI)
    {
      handleWifiSettingsTouch(touchStartX, touchStartY);
    }
    else if (screenMode == ScreenMode::MENU_GATEWAY)
    {
      handleGatewaySettingsTouch(touchStartX, touchStartY);
    }
    else if (isInsideGearIcon(touchStartX, touchStartY))
    {
      enterSettings();
    }
    else if (isInsideMenuIcon(touchStartX, touchStartY))
    {
      enterMenu();
    }
    // NORMAL screen, tapped elsewhere: no-op. Restart now lives behind a
    // deliberate button on the Settings page instead, so brushing the
    // screen can't reboot a unit mid-way through monitoring a dryer.
  }

  touchActive = touchDown;
}

// WiFi can drop and reconnect at runtime (see NetworkManager::loop()), so
// its header icon needs to track that - unlike the SD/ESP-NOW icons, which
// reflect a one-time boot check and never change.
static void refreshWifiIconIfNeeded()
{
  if (screenMode != ScreenMode::NORMAL)
    return;
  bool wifiConnected = Network.isConnected();
  if (wifiConnected != headerWifiIconState)
  {
    headerWifiIconState = wifiConnected;
    drawWifiIcon(kWifiIconX, kIconY, wifiConnected);
  }
}

void loop()
{
  ArduinoOTA.handle();
  if (otaActive)
  {
    delay(1);
    return;
  }

  M5.update();
  Network.loop();
  checkTouchGestures();
  refreshWifiIconIfNeeded();

  if (screenMode == ScreenMode::NORMAL && !screenSaverActive && Settings.screenSaverEnabled() &&
      millis() - lastActivityMs > static_cast<uint32_t>(Settings.screenSaverTimeoutSec()) * 1000UL)
  {
    screenSaverActive = true;
    M5.Display.sleep();
  }

#if defined(ROLE_NODE)
  Registers.task();
  Registers.processPendingWrite();
  DryerWeb.handleClient();
  rtsnowNodeLoop(); // no-op until rtsnowNodeBegin() has run

  // Runs every loop() (not gated by the 2s timer below) - a brute-force
  // scan is tens of thousands of attempts, so it needs every iteration
  // it can get. No-op when no scan is running.
  Discovery.task();

  // Normal round-robin polling pauses for the duration of a scan - keeps
  // the RS485 traffic and dashboard state unambiguous (only ever one
  // thing polling at a time), rather than relying on it being technically
  // harmless to interleave.
  static uint32_t lastUpdateMs = 0;
  if (!Discovery.isRunning() && millis() - lastUpdateMs > 2000)
  {
    lastUpdateMs = millis();
    updateDryerReadings();
  }
#endif

  delay(10);
}
