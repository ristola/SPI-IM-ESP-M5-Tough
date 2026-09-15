#pragma once

#include <Arduino.h>

// SPI CCP (Society of the Plastics Industry, Committee on Communication
// Protocol) - an industrial polling protocol for reading/writing data on
// auxiliary equipment (dryers, crystallizers) from a primary machine.
// Despite the name, this has nothing to do with the SPI hardware bus: it's
// a half-duplex, Bisync-style (ANSI X3.28) protocol over RS-485, and runs
// on the existing RS485Bus hardware (Serial2). See
// DataSheets/SPI Protocol.pdf for the formal spec and Reference/*.py for
// Ristola Technical Services' prior working XBee/MicroPython gateway that
// this is ported from.
//
// A primary machine (this device) is the control station; auxiliary
// devices (dryers) are tributary stations, addressed by a DEVID (device
// type) + ADD (station address) pair. POLL reads data, SELECT writes it.
class SpiCcp
{
public:
  static constexpr uint8_t kNoError = 0x20; // ERR byte with no bits set beyond the mandatory bit5

  // Classifies the outcome of the most recent poll() call - lets a caller
  // that has no serial/log access (e.g. a remote Modbus client) see why
  // polling isn't working, by exposing lastOutcome() through a spare
  // holding register. Mirrors the distinct failure points already logged
  // to Serial inside poll() - see that function for what triggers each one.
  enum class PollOutcome : uint8_t
  {
    kNone = 0,            // no poll attempted yet since boot
    kEot = 1,             // tributary replied with a bare EOT - "nothing to send"
    kNoReply = 2,         // no bytes received before timeout
    kShortReply = 3,      // some bytes received, but fewer than a full frame
    kFramingMismatch = 4, // header bytes didn't match the request
    kMissingTrailer = 5,  // DLE/ETX trailer wasn't where expected
    kCrcMismatch = 6,
    kSuccess = 7,
  };

  void begin(uint32_t baud = 9600);

  uint32_t baud() const { return baud_; }

  // Sends a POLL (read) request and blocks for a reply. On success, fills
  // `data` (up to maxLen bytes) and `outLen`, and returns true. `err`
  // receives the tributary's ERR status byte when a reply was received at
  // all (valid whether or not this returns true).
  bool poll(uint8_t devId, uint8_t addr, uint8_t cmd1, uint8_t cmd2, uint8_t *data, size_t maxLen,
            size_t &outLen, uint8_t &err, uint32_t timeoutMs = 1000);

  // Same as poll(), but for exploring a command whose reply length isn't
  // known ahead of time (see the web dashboard's Raw Command Test) - reads
  // data until the first unescaped <DLE><ETX> trailer instead of requiring
  // exactly maxLen bytes first. `dataCap` bounds the buffer; a reply
  // longer than that is a framing error rather than silently truncated.
  bool pollRaw(uint8_t devId, uint8_t addr, uint8_t cmd1, uint8_t cmd2, uint8_t *data, size_t dataCap,
               size_t &outLen, uint8_t &err, uint32_t timeoutMs = 1000);

  PollOutcome lastOutcome() const { return lastOutcome_; }

  // Running count of CRC mismatches seen since boot (poll()/pollRaw()
  // combined) - feeds Modbus register 40009 ("SPI CRC Error" per
  // DataSheets/Modbus Registers V2.pdf). Distinct from kNoReply/kEot/etc:
  // this specifically means a reply arrived, had the right shape, but
  // failed the CRC check - a real link-quality signal (noise, wiring),
  // not "nothing answered" or "wrong command."
  uint32_t crcErrorCount() const { return crcErrorCount_; }

  // Zeroes the accumulated count - lets a technician clear an old count
  // (e.g. from a wiring problem since fixed) rather than it climbing
  // forever from boot. Doesn't affect lastOutcome()/lastTxHex()/etc.
  void resetCrcErrorCount() { crcErrorCount_ = 0; }

  // Raw wire capture of the most recent poll()/select() call - lets a
  // caller with no logic-analyzer access (e.g. the web dashboard) see
  // exactly what went out on RS485 and what, if anything, came back.
  String lastTxHex() const { return lastTxHex_; }
  String lastRxHex() const { return lastRxHex_; }

  // Sends a SELECT (write) request with the given data and blocks for the
  // tributary's acknowledgment. This is two round trips: the selection
  // sequence (tributary confirms it's ready to receive) then the data
  // block itself (tributary ACKs or NAKs it).
  bool select(uint8_t devId, uint8_t addr, uint8_t cmd1, uint8_t cmd2, const uint8_t *data, size_t dataLen,
              uint32_t timeoutMs = 1000);

private:
  static constexpr uint8_t SOH = 0x01;
  static constexpr uint8_t STX = 0x02;
  static constexpr uint8_t ETX = 0x03;
  static constexpr uint8_t EOT = 0x04;
  static constexpr uint8_t ENQ = 0x05;
  static constexpr uint8_t DLE = 0x10;
  static constexpr uint8_t NAK = 0x15;

  uint16_t crcTable_[256];
  PollOutcome lastOutcome_ = PollOutcome::kNone;
  uint32_t crcErrorCount_ = 0;
  uint32_t baud_ = 9600;
  String lastTxHex_;
  String lastRxHex_;

  void buildCrcTable();
  uint16_t crcUpdate(uint16_t crc, uint8_t b) const;

  size_t readWithTimeout(uint8_t *buf, size_t maxLen, uint32_t timeoutMs);
  bool readByte(uint8_t &b, uint32_t deadlineMs);
  void flushStaleRx();
  void sendSelectionSequence(uint8_t devId, uint8_t addr, uint8_t cmd1, uint8_t cmd2, bool isSelect);
  static String hexDump(const uint8_t *data, size_t len);
  static size_t stuffDle(uint8_t *out, size_t outCap, const uint8_t *in, size_t inLen);
};

extern SpiCcp SpiIm;
