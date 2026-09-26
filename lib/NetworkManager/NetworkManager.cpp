#include "NetworkManager.h"

NetworkManager Network;

#if defined(BOARD_ATOMS3_POE)

// AtomS3 + Atomic PoE Base (W5500) - see pins.h's ETH_SCK/ETH_MISO/
// ETH_MOSI/ETH_CS. ESP-NOW/RTS-NOW still needs the WiFi radio powered on
// (WiFi.mode(WIFI_STA), done by EspNowLink::begin()) to pick a channel and
// transmit/receive, but this board never associates with any WiFi AP -
// every IP-level thing (Modbus TCP, the web dashboard) rides the wired
// Ethernet link instead. Uses M5Stack's M5_Ethernet (a polling-based fork
// of the classic Arduino Ethernet.h API) rather than the ESP32 core's
// native ETH.h - confirmed on real hardware (see pins.h's comment) that
// this board's W5500 has no RST/INT wired at all, and ETH.h's beginSPI()
// rejects boards without those.
#include <M5_Ethernet.h>
#include <SPI.h>
#include <utility/w5100.h>  // for W5100.init() directly - see begin()'s own comment on why hardwareStatus() alone can't trigger detection

#include "pins.h"

namespace {
uint8_t s_mac[6];  // filled from the chip's own WiFi MAC in begin() - unique per board, no per-unit config needed
constexpr uint32_t kMaintainIntervalMs = 1000;
uint32_t s_lastMaintainMs = 0;

// Independent, from-scratch cross-check of M5_Ethernet's own W5500
// detection - reads the chip's VERSIONR register (fixed address 0x0039 in
// the Common Register block, always reads back 0x04 on real hardware)
// via a hand-built SPI transaction, entirely bypassing Ethernet.init()/
// hardwareStatus(). Frame format per the W5500 datasheet: 2 address
// bytes, 1 control byte (block-select=Common(0)/read/variable-length
// mode -> 0x00), then N data bytes - here just 1. Run BEFORE
// Ethernet.init() below; that call reconfigures/reuses the same CS pin
// afterward, so there's no lasting conflict from probing it first.
uint8_t rawReadW5500Version() {
  pinMode(Pins::ETH_CS, OUTPUT);
  digitalWrite(Pins::ETH_CS, HIGH);
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Pins::ETH_CS, LOW);
  SPI.transfer(0x00);  // VERSIONR address, high byte
  SPI.transfer(0x39);  // VERSIONR address, low byte
  SPI.transfer(0x00);  // control byte: Common Register block, read, variable-length mode
  uint8_t result = SPI.transfer(0x00);  // dummy write while clocking in the response
  digitalWrite(Pins::ETH_CS, HIGH);
  SPI.endTransaction();
  return result;
}

// Writes `value` to the Mode Register (address 0x0000, Common block) and
// reads it straight back - isolates whether SPI WRITES actually take
// effect, independent of rawReadW5500Version() above (a pure read, no
// writes at all). M5_Ethernet's own isW5500()/softReset() do exactly this
// kind of write-then-verify as their very first step, before ever
// reaching a VERSIONR check - if that's where real hardware is failing,
// a pure VERSIONR read succeeding tells us nothing about it.
uint8_t rawWriteReadbackMR(uint8_t value) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Pins::ETH_CS, LOW);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x04);  // control byte: Common Register block, WRITE, variable-length mode
  SPI.transfer(value);
  digitalWrite(Pins::ETH_CS, HIGH);

  digitalWrite(Pins::ETH_CS, LOW);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);  // control byte: Common Register block, read, variable-length mode
  uint8_t readBack = SPI.transfer(0x00);
  digitalWrite(Pins::ETH_CS, HIGH);
  SPI.endTransaction();
  return readBack;
}

uint8_t rawReadMR() {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Pins::ETH_CS, LOW);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  uint8_t result = SPI.transfer(0x00);
  digitalWrite(Pins::ETH_CS, HIGH);
  SPI.endTransaction();
  return result;
}

// Replicates M5_Ethernet's own softReset() (utility/w5100.cpp) exactly:
// write the software-reset bit (0x80) to MR, then poll MR waiting for the
// chip to clear it back to 0 on its own once the reset completes -
// distinct from rawWriteReadbackMR() above, which writes an ordinary
// value (0x08) that the chip should just store as-is with no internal
// side effect. If THIS specific sequence never clears, the chip is
// accepting the write but not completing (or not completing in time)
// the reset it triggers - and the library's own softReset() bails out
// before ever reaching a VERSIONR check, regardless of whether plain
// reads/writes work fine.
uint32_t rawSoftResetTest(uint8_t *outFinalMr) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Pins::ETH_CS, LOW);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x04);  // write
  SPI.transfer(0x80);  // software reset bit
  digitalWrite(Pins::ETH_CS, HIGH);
  SPI.endTransaction();

  uint32_t start = millis();
  uint8_t mr = 0xFF;
  do {
    mr = rawReadMR();
    if (mr == 0x00) break;
    delay(1);
  } while (millis() - start < 20);
  *outFinalMr = mr;
  return millis() - start;
}

// Byte-for-byte replica of isW5500() (utility/w5100.cpp): one CONTINUOUS
// SPI transaction (matching init()'s own beginTransaction/endTransaction
// wrapping around the whole chip-detection dance, never closed between
// individual register accesses - unlike every raw*() function above,
// which each open and close their own transaction) doing softReset(),
// then writeMR(0x08)/verify, writeMR(0x10)/verify, writeMR(0x00)/verify,
// then reading VERSIONR. Every piece already proven to work in isolation
// (with real gaps between each, across separate boots) - this tests
// whether chaining them with zero gaps, inside one open transaction,
// changes anything. Returns the specific step that failed for
// diagnostics, 0 if every step passed.
uint8_t replicateIsW5500Exactly(uint8_t *outVersion) {
  auto writeReg = [](uint16_t addr, uint8_t value) {
    digitalWrite(Pins::ETH_CS, LOW);
    SPI.transfer(addr >> 8);
    SPI.transfer(addr & 0xFF);
    SPI.transfer(0x04);
    SPI.transfer(value);
    digitalWrite(Pins::ETH_CS, HIGH);
  };
  auto readReg = [](uint16_t addr) -> uint8_t {
    digitalWrite(Pins::ETH_CS, LOW);
    SPI.transfer(addr >> 8);
    SPI.transfer(addr & 0xFF);
    SPI.transfer(0x00);
    uint8_t v = SPI.transfer(0x00);
    digitalWrite(Pins::ETH_CS, HIGH);
    return v;
  };

  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  // softReset()
  writeReg(0x0000, 0x80);
  uint16_t count = 0;
  uint8_t mr;
  bool resetOk = false;
  do {
    mr = readReg(0x0000);
    if (mr == 0) { resetOk = true; break; }
    delay(1);
  } while (++count < 20);
  if (!resetOk) { SPI.endTransaction(); return 1; }

  writeReg(0x0000, 0x08);
  if (readReg(0x0000) != 0x08) { SPI.endTransaction(); return 2; }

  writeReg(0x0000, 0x10);
  if (readReg(0x0000) != 0x10) { SPI.endTransaction(); return 3; }

  writeReg(0x0000, 0x00);
  if (readReg(0x0000) != 0x00) { SPI.endTransaction(); return 4; }

  *outVersion = readReg(0x0039);
  SPI.endTransaction();
  return *outVersion == 0x04 ? 0 : 5;
}
}  // namespace

bool NetworkManager::begin(const char* ssid, const char* password, uint32_t timeoutMs) {
  (void)ssid;
  (void)password;

  // Deliberately checked in stages, with a Serial print after each one,
  // rather than going straight to Ethernet.begin(mac) (which does DHCP) -
  // that call blocks internally with its own long-ish timeout, and doing
  // it before confirming the W5500 chip even responds over SPI at all (or
  // that a cable is actually linked up) made an early real-hardware test
  // of this board look like a total hang, with zero Serial output the
  // whole time to tell a wiring problem apart from "no cable plugged in
  // yet, just waiting on DHCP".
  Serial.println("Ethernet: starting SPI + W5500 init...");
  // WiFi.macAddress() needs the WiFi driver initialized to reliably return
  // the chip's real factory MAC rather than all-zeros - this runs before
  // EspNow.begin() (the only other WiFi.mode() call site) in this board's
  // setup(), so it can't rely on that having already happened. A zero MAC
  // here would make the W5500 send every Ethernet frame from an invalid
  // source address - switches/DHCP servers can reject or mishandle that
  // outright, which looks exactly like "no link" from the switch's side.
  WiFi.mode(WIFI_STA);
  WiFi.macAddress(s_mac);
  SPI.begin(Pins::ETH_SCK, Pins::ETH_MISO, Pins::ETH_MOSI, -1);

  _lastRawVersionRead = rawReadW5500Version();
  Serial.printf("Ethernet: raw SPI VERSIONR read=0x%02X (%s)\n", _lastRawVersionRead,
                _lastRawVersionRead == 0x04 ? "matches real W5500" : "does NOT match - nothing answering on SPI");

  uint8_t mrReadback1 = rawWriteReadbackMR(0x08);
  uint8_t mrReadback2 = rawWriteReadbackMR(0x00);  // put MR back to its power-on default before Ethernet.init() runs its own sequence
  _lastRawWriteWorked = mrReadback1 == 0x08 && mrReadback2 == 0x00;
  Serial.printf("Ethernet: raw MR write/readback test: wrote 0x08 -> read 0x%02X, wrote 0x00 -> read 0x%02X (%s)\n",
                mrReadback1, mrReadback2, _lastRawWriteWorked ? "writes ARE taking effect" : "writes NOT taking effect");

  _lastSoftResetElapsedMs = rawSoftResetTest(&_lastSoftResetFinalMr);
  Serial.printf("Ethernet: raw softReset test: MR=0x%02X after %lums (%s)\n", _lastSoftResetFinalMr,
                static_cast<unsigned long>(_lastSoftResetElapsedMs),
                _lastSoftResetFinalMr == 0x00 ? "reset completed" : "reset did NOT complete in time");

  uint8_t replicaVersion = 0x00;
  _lastIsW5500ReplicaResult = replicateIsW5500Exactly(&replicaVersion);
  Serial.printf("Ethernet: isW5500() exact replica: step-result=%u, VERSIONR=0x%02X (%s)\n",
                _lastIsW5500ReplicaResult, replicaVersion,
                _lastIsW5500ReplicaResult == 0 ? "PASSED - should have been detected" : "FAILED");

  Ethernet.init(Pins::ETH_CS);  // EthernetClass::init() - only sets the CS pin (W5100.setSS()), does NOT probe the chip

  // The actual chip-detection call: EthernetClass::hardwareStatus() (below)
  // does nothing but return a CACHED value from W5100.getChip() - it never
  // triggers detection itself. That value is only ever populated by
  // W5100Class::init() (this call - a different, same-named function on a
  // different class, easy to conflate with the one right above), normally
  // invoked indirectly via EthernetClass::begin(). Every hardwareStatus()
  // check in this file, for this board's entire lifetime, was reading an
  // uninitialized/stale value because nothing had ever called this - not a
  // hardware fault, not a bug in the chip-detection sequence itself (both
  // conclusively ruled out via hand-rolled SPI probes during diagnosis).
  W5100.init();

  EthernetHardwareStatus hw = Ethernet.hardwareStatus();
  _hardwareDetected = hw == EthernetW5500;
  Serial.printf("Ethernet: hardwareStatus=%d (%s)\n", static_cast<int>(hw),
                hw == EthernetW5500 ? "W5500 detected" : "NOT detected - check wiring/seating");
  if (hw != EthernetW5500) {
    snprintf(_lastDiagnostic, sizeof(_lastDiagnostic),
             "W5500 chip NOT detected over SPI (hardwareStatus=%d, raw VERSIONR=0x%02X) - check the PoE "
             "Base's seating on the AtomS3, not the network cable/switch",
             static_cast<int>(hw), _lastRawVersionRead);
    _lastAttemptMs = millis();
    return false;
  }

  uint32_t start = millis();
  EthernetLinkStatus link;
  do {
    link = Ethernet.linkStatus();
    delay(100);
  } while (link != LinkON && millis() - start < timeoutMs);
  uint32_t linkWaitMs = millis() - start;
  Serial.printf("Ethernet: linkStatus=%d (%s) after %lums\n", static_cast<int>(link),
                link == LinkON ? "LinkON" : "not up - check cable", static_cast<unsigned long>(linkWaitMs));
  if (link != LinkON) {
    snprintf(_lastDiagnostic, sizeof(_lastDiagnostic),
             "W5500 chip detected OK, but no link after %lums - check the cable/switch port, not the PoE "
             "Base module itself",
             static_cast<unsigned long>(linkWaitMs));
    _lastAttemptMs = millis();
    return false;
  }

  Serial.println("Ethernet: link up, starting DHCP...");
  bool dhcpOk = Ethernet.begin(s_mac) != 0;
  Serial.printf("Ethernet: DHCP %s\n", dhcpOk ? "OK" : "FAILED");
  snprintf(_lastDiagnostic, sizeof(_lastDiagnostic), "W5500 detected, link up after %lums, DHCP %s",
           static_cast<unsigned long>(linkWaitMs), dhcpOk ? "OK" : "FAILED");

  _lastAttemptMs = millis();
  return isConnected();
}

bool NetworkManager::isConnected() const {
  return Ethernet.linkStatus() == LinkON && Ethernet.localIP() != IPAddress(0, 0, 0, 0);
}

IPAddress NetworkManager::localIP() const { return Ethernet.localIP(); }

void NetworkManager::loop() {
  if (millis() - s_lastMaintainMs < kMaintainIntervalMs) return;
  s_lastMaintainMs = millis();
  Ethernet.maintain();  // renews the DHCP lease as needed; no-op otherwise
}

#else

bool NetworkManager::begin(const char* ssid, const char* password, uint32_t timeoutMs) {
  _ssid = ssid;
  _password = password;

  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid.c_str(), _password.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
  }

  _lastAttemptMs = millis();
  return isConnected();
}

bool NetworkManager::isConnected() const { return WiFi.status() == WL_CONNECTED; }
IPAddress NetworkManager::localIP() const { return WiFi.localIP(); }

void NetworkManager::loop() {
  if (isConnected()) return;
  if (millis() - _lastAttemptMs < kRetryIntervalMs) return;

  _lastAttemptMs = millis();
  WiFi.disconnect();
  WiFi.begin(_ssid.c_str(), _password.c_str());
}

#endif
