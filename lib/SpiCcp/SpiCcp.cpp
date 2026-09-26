#include "SpiCcp.h"

#include <cstring>

#include "RS485Bus.h"

SpiCcp SpiIm;

void SpiCcp::begin(uint32_t baud)
{
  baud_ = baud;
  buildCrcTable();
  Bus485.begin(baud);
}

String SpiCcp::hexDump(const uint8_t *data, size_t len)
{
  String s;
  s.reserve(len * 3);
  for (size_t i = 0; i < len; i++)
  {
    if (i)
      s += ' ';
    char b[3];
    snprintf(b, sizeof(b), "%02X", data[i]);
    s += b;
  }
  return s;
}

// CRC-16, polynomial X^16+X^15+X^2+1 (0xA001 reflected) - verified to
// reproduce DataSheets/SPI Protocol.pdf's reference CRC table byte-for-byte
// (and matches Reference/modbus.py's hardcoded crc_lookup_table).
void SpiCcp::buildCrcTable()
{
  constexpr uint16_t kPoly = 0xA001;
  for (uint16_t i = 0; i < 256; i++)
  {
    uint16_t crc = i;
    for (uint8_t j = 0; j < 8; j++)
    {
      crc = (crc & 0x0001) ? (crc >> 1) ^ kPoly : (crc >> 1);
    }
    crcTable_[i] = crc;
  }
}

uint16_t SpiCcp::crcUpdate(uint16_t crc, uint8_t b) const
{
  uint8_t index = (crc ^ b) & 0xFF;
  return ((crc >> 8) & 0xFF) ^ crcTable_[index];
}

size_t SpiCcp::readWithTimeout(uint8_t *buf, size_t maxLen, uint32_t timeoutMs)
{
  size_t got = 0;
  uint32_t start = millis();
  while (got < maxLen && millis() - start < timeoutMs)
  {
    if (Bus485.available())
    {
      int b = Bus485.read();
      if (b >= 0)
        buf[got++] = static_cast<uint8_t>(b);
    }
  }
  return got;
}

bool SpiCcp::readByte(uint8_t &b, uint32_t deadlineMs)
{
  while (millis() < deadlineMs)
  {
    if (Bus485.available())
    {
      int v = Bus485.read();
      if (v >= 0)
      {
        b = static_cast<uint8_t>(v);
        return true;
      }
    }
  }
  return false;
}

// Bisync-style data transparency (see DataSheets/SPI Protocol.pdf): any
// literal 0x10 (DLE) byte occurring inside a DATA block must be doubled on
// the wire so it can't be mistaken for the DLE that introduces the block's
// terminator. Verified necessary against real hardware, not theoretical -
// a captured FD blanket-poll reply had two literal 0x10 data bytes back to
// back, which decoded correctly only once this stuffing/destuffing was
// implemented (confirmed by the reply's CRC matching once destuffed).
size_t SpiCcp::stuffDle(uint8_t *out, size_t outCap, const uint8_t *in, size_t inLen)
{
  size_t pos = 0;
  for (size_t i = 0; i < inLen; i++)
  {
    if (pos >= outCap)
      return pos;
    out[pos++] = in[i];
    if (in[i] == DLE)
    {
      if (pos >= outCap)
        return pos;
      out[pos++] = DLE;
    }
  }
  return pos;
}

// Discards anything already sitting in the RS-485 receive buffer before a
// new request goes out. Found necessary against real hardware: a stray
// leftover byte from a prior exchange (e.g. this station's own next-poll
// EOT arriving back through an imperfectly-isolated half-duplex
// transceiver, or simply not fully drained after a previous timeout)
// would otherwise become the first byte read as this poll's header,
// shifting everything after it by one byte and causing a false framing
// mismatch even though a perfectly valid reply followed right behind it.
void SpiCcp::flushStaleRx()
{
  while (Bus485.available())
  {
    Bus485.read();
  }
}

void SpiCcp::sendSelectionSequence(uint8_t devId, uint8_t addr, uint8_t cmd1, uint8_t cmd2, bool isSelect)
{
  uint8_t req[7] = {EOT, devId, addr, cmd1, static_cast<uint8_t>(isSelect ? (cmd2 | 0x01) : (cmd2 & ~0x01)),
                    kNoError, ENQ};
  lastTxHex_ = hexDump(req, sizeof(req));
  Bus485.write(req, sizeof(req));
}

bool SpiCcp::poll(uint8_t devId, uint8_t addr, uint8_t cmd1, uint8_t cmd2, uint8_t *data, size_t maxLen,
                  size_t &outLen, uint8_t &err, uint32_t timeoutMs)
{
  outLen = 0;
  err = 0;
  flushStaleRx();
  sendSelectionSequence(devId, addr, cmd1, cmd2, /*isSelect=*/false);

  uint32_t deadline = millis() + timeoutMs;

  // Fixed-size header, never DLE-stuffed (that only applies inside the
  // STX..ETX data block): <DLE><SOH>(devId)(addr)(cmd1)(cmd2)(RES)(ERR)
  // <DLE><STX>. If the tributary has nothing to send it replies with a
  // bare <EOT> instead, which reads as a 1-byte "header" here.
  uint8_t header[10];
  size_t headerGot = 0;
  while (headerGot < sizeof(header) && millis() < deadline)
  {
    if (Bus485.available())
    {
      int b = Bus485.read();
      if (b >= 0)
        header[headerGot++] = static_cast<uint8_t>(b);
    }
  }

  uint8_t rawCapture[192];
  size_t rawLen = 0;
  auto captureByte = [&](uint8_t b)
  {
    if (rawLen < sizeof(rawCapture))
      rawCapture[rawLen++] = b;
  };
  for (size_t i = 0; i < headerGot; i++)
    captureByte(header[i]);

  // Diagnostic-only: on any failure path below, keep draining whatever
  // continues to arrive for a bit longer before reporting lastRxHex_ - a
  // real reply that fails a framing/length check partway through is
  // usually still mid-transmission on the wire, and without this the
  // dashboard only ever showed the first few bytes read before the check
  // failed, not the actual full reply. Every valid message ends with
  // <DLE><ETX> followed by the 2-byte CRC and nothing else, so this stops
  // exactly there (a stuffed <DLE><DLE> pair is tracked and correctly not
  // mistaken for the trailer) rather than guessing from a quiet period.
  // Falls back to extraMs elapsed as a bound in case that trailer never
  // actually arrives. Doesn't affect outcome/CRC logic, purely extends
  // what gets captured for lastRxHex_.
  auto drainForDiagnostics = [&](uint32_t extraMs)
  {
    uint32_t drainDeadline = millis() + extraMs;
    bool sawDle = false;
    int afterTrailer = -1;
    while (millis() < drainDeadline)
    {
      if (!Bus485.available())
        continue;
      int v = Bus485.read();
      if (v < 0)
        continue;
      uint8_t b = static_cast<uint8_t>(v);
      captureByte(b);
      if (afterTrailer >= 0)
      {
        if (--afterTrailer <= 0)
          break;
        continue;
      }
      if (sawDle)
      {
        sawDle = false;
        if (b == ETX)
          afterTrailer = 2;
        // else: a stuffed <DLE><DLE> pair, or an unexpected byte - either
        // way this DLE is accounted for, keep scanning normally.
      }
      else if (b == DLE)
      {
        sawDle = true;
      }
    }
  };

  Serial.printf("SpiCcp: TX %s\n", lastTxHex_.c_str());

  if (headerGot == 1 && header[0] == EOT)
  {
    lastRxHex_ = hexDump(rawCapture, rawLen);
    Serial.printf("SpiCcp: poll devId=0x%02X addr=0x%02X cmd=0x%02X/0x%02X -> EOT (tributary has nothing to send)\n",
                  devId, addr, cmd1, cmd2);
    lastOutcome_ = PollOutcome::kEot;
    return false;
  }
  if (headerGot < sizeof(header))
  {
    drainForDiagnostics(150);
    lastRxHex_ = hexDump(rawCapture, rawLen);
    Serial.printf("SpiCcp: poll devId=0x%02X addr=0x%02X cmd=0x%02X/0x%02X -> %s (got %u/%u header bytes)\n", devId,
                  addr, cmd1, cmd2, headerGot == 0 ? "no reply" : "short reply/timeout",
                  static_cast<unsigned>(headerGot), static_cast<unsigned>(sizeof(header)));
    lastOutcome_ = headerGot == 0 ? PollOutcome::kNoReply : PollOutcome::kShortReply;
    return false;
  }

  if (header[0] != DLE || header[1] != SOH || header[2] != devId || header[3] != addr || header[4] != cmd1 ||
      header[5] != (cmd2 & ~0x01) || header[8] != DLE || header[9] != STX)
  {
    drainForDiagnostics(150);
    lastRxHex_ = hexDump(rawCapture, rawLen);
    Serial.printf(
        "SpiCcp: poll devId=0x%02X addr=0x%02X cmd=0x%02X/0x%02X -> framing/header mismatch "
        "(got %02X %02X devId=%02X addr=%02X cmd1=%02X cmd2=%02X)\n",
        devId, addr, cmd1, cmd2, header[0], header[1], header[2], header[3], header[4], header[5]);
    lastOutcome_ = PollOutcome::kFramingMismatch;
    return false;
  }
  err = header[7];

  // Data transparency: a literal 0x10 (DLE) byte inside the data block is
  // doubled on the wire (see stuffDle()'s comment) - destuff as we read,
  // stopping at the first unescaped <DLE><ETX>. maxLen is the known exact
  // logical (destuffed) length for this command, so overrunning it before
  // seeing the trailer is itself a framing error.
  if (maxLen > 128)
    return false;
  bool frameError = false;
  size_t dataGot = 0;
  while (dataGot < maxLen)
  {
    uint8_t b;
    if (!readByte(b, deadline))
    {
      drainForDiagnostics(150);
      lastRxHex_ = hexDump(rawCapture, rawLen);
      lastOutcome_ = PollOutcome::kShortReply;
      return false;
    }
    captureByte(b);
    if (b != DLE)
    {
      data[dataGot++] = b;
      continue;
    }
    uint8_t next;
    if (!readByte(next, deadline))
    {
      drainForDiagnostics(150);
      lastRxHex_ = hexDump(rawCapture, rawLen);
      lastOutcome_ = PollOutcome::kShortReply;
      return false;
    }
    captureByte(next);
    if (next == DLE)
    {
      data[dataGot++] = DLE;
      continue;
    }
    // Either the real <DLE><ETX> trailer (arriving before maxLen data bytes
    // were collected - the reply is shorter than expected for this
    // command) or an unexpected escape sequence. Either way, the data
    // doesn't match what this command was expecting.
    frameError = true;
    break;
  }

  if (!frameError)
  {
    uint8_t etxDle, etxByte;
    frameError = !readByte(etxDle, deadline) || !readByte(etxByte, deadline) || etxDle != DLE || etxByte != ETX;
    if (!frameError)
    {
      captureByte(etxDle);
      captureByte(etxByte);
    }
  }

  if (frameError)
  {
    drainForDiagnostics(150);
    lastRxHex_ = hexDump(rawCapture, rawLen);
    Serial.printf("SpiCcp: poll devId=0x%02X addr=0x%02X cmd=0x%02X/0x%02X -> missing DLE/ETX trailer\n", devId, addr,
                  cmd1, cmd2);
    lastOutcome_ = PollOutcome::kMissingTrailer;
    return false;
  }

  uint8_t crcBytes[2];
  if (!readByte(crcBytes[0], deadline) || !readByte(crcBytes[1], deadline))
  {
    drainForDiagnostics(150);
    lastRxHex_ = hexDump(rawCapture, rawLen);
    lastOutcome_ = PollOutcome::kShortReply;
    return false;
  }
  captureByte(crcBytes[0]);
  captureByte(crcBytes[1]);
  lastRxHex_ = hexDump(rawCapture, rawLen);
  Serial.printf("SpiCcp: RX %s\n", lastRxHex_.c_str());

  uint16_t rxCrc = (static_cast<uint16_t>(crcBytes[0]) << 8) | crcBytes[1];

  // CRC covers the header (devId..ERR), the STX byte itself (its DLE is
  // excluded, but since a header/SOH preceded it in this block, the STX is
  // included per the PDF's DLE sequence chart), the logical (destuffed)
  // data, and the ETX byte itself (its DLE excluded).
  uint16_t crc = 0;
  for (size_t i = 2; i <= 7; i++)
    crc = crcUpdate(crc, header[i]);
  crc = crcUpdate(crc, header[9]); // STX
  for (size_t i = 0; i < maxLen; i++)
    crc = crcUpdate(crc, data[i]);
  crc = crcUpdate(crc, ETX);

  if (crc != rxCrc)
  {
    Serial.printf("SpiCcp: poll devId=0x%02X addr=0x%02X cmd=0x%02X/0x%02X -> CRC mismatch (got 0x%04X, want 0x%04X)\n",
                  devId, addr, cmd1, cmd2, rxCrc, crc);
    lastOutcome_ = PollOutcome::kCrcMismatch;
    crcErrorCount_++;
    uint8_t nak[2] = {static_cast<uint8_t>(kNoError | 0x01), NAK}; // communication error bit set
    Bus485.write(nak, sizeof(nak));
    return false;
  }

  uint8_t ack[2] = {DLE, '1'}; // ACK1: affirmative reply to the first (and only) block
  Bus485.write(ack, sizeof(ack));

  outLen = maxLen;
  lastOutcome_ = PollOutcome::kSuccess;
  Serial.printf("SpiCcp: poll devId=0x%02X addr=0x%02X cmd=0x%02X/0x%02X -> OK (err=0x%02X, %u bytes)\n", devId, addr,
                cmd1, cmd2, err, static_cast<unsigned>(maxLen));
  return true;
}

// Same structure as poll() above (header read/validate, then destuff data
// until the trailer, then CRC-check) but for exploring a command whose
// reply length isn't known ahead of time - see the header comment. The
// only real difference is the data loop's stop condition: "until the
// trailer" instead of "until maxLen bytes, then the trailer must be
// next". Kept as a separate function rather than unifying with poll() to
// avoid touching that already hardware-validated implementation.
bool SpiCcp::pollRaw(uint8_t devId, uint8_t addr, uint8_t cmd1, uint8_t cmd2, uint8_t *data, size_t dataCap,
                     size_t &outLen, uint8_t &err, uint32_t timeoutMs)
{
  outLen = 0;
  err = 0;
  flushStaleRx();
  sendSelectionSequence(devId, addr, cmd1, cmd2, /*isSelect=*/false);

  uint32_t deadline = millis() + timeoutMs;

  uint8_t header[10];
  size_t headerGot = 0;
  while (headerGot < sizeof(header) && millis() < deadline)
  {
    if (Bus485.available())
    {
      int b = Bus485.read();
      if (b >= 0)
        header[headerGot++] = static_cast<uint8_t>(b);
    }
  }

  uint8_t rawCapture[192];
  size_t rawLen = 0;
  auto captureByte = [&](uint8_t b)
  {
    if (rawLen < sizeof(rawCapture))
      rawCapture[rawLen++] = b;
  };
  for (size_t i = 0; i < headerGot; i++)
    captureByte(header[i]);

  auto drainForDiagnostics = [&](uint32_t extraMs)
  {
    uint32_t drainDeadline = millis() + extraMs;
    bool sawDle = false;
    int afterTrailer = -1;
    while (millis() < drainDeadline)
    {
      if (!Bus485.available())
        continue;
      int v = Bus485.read();
      if (v < 0)
        continue;
      uint8_t b = static_cast<uint8_t>(v);
      captureByte(b);
      if (afterTrailer >= 0)
      {
        if (--afterTrailer <= 0)
          break;
        continue;
      }
      if (sawDle)
      {
        sawDle = false;
        if (b == ETX)
          afterTrailer = 2;
      }
      else if (b == DLE)
      {
        sawDle = true;
      }
    }
  };

  Serial.printf("SpiCcp: TX %s\n", lastTxHex_.c_str());

  if (headerGot == 1 && header[0] == EOT)
  {
    lastRxHex_ = hexDump(rawCapture, rawLen);
    lastOutcome_ = PollOutcome::kEot;
    return false;
  }
  if (headerGot < sizeof(header))
  {
    drainForDiagnostics(150);
    lastRxHex_ = hexDump(rawCapture, rawLen);
    lastOutcome_ = headerGot == 0 ? PollOutcome::kNoReply : PollOutcome::kShortReply;
    return false;
  }

  if (header[0] != DLE || header[1] != SOH || header[2] != devId || header[3] != addr || header[4] != cmd1 ||
      header[5] != (cmd2 & ~0x01) || header[8] != DLE || header[9] != STX)
  {
    drainForDiagnostics(150);
    lastRxHex_ = hexDump(rawCapture, rawLen);
    lastOutcome_ = PollOutcome::kFramingMismatch;
    return false;
  }
  err = header[7];

  // Read data (destuffing) until the first unescaped <DLE><ETX>, instead
  // of a predetermined length - this is the whole point of pollRaw().
  bool frameError = false;
  size_t dataGot = 0;
  bool sawTrailer = false;
  while (!sawTrailer)
  {
    uint8_t b;
    if (!readByte(b, deadline))
    {
      frameError = true;
      break;
    }
    captureByte(b);
    if (b != DLE)
    {
      if (dataGot >= dataCap)
      {
        frameError = true;
        break;
      }
      data[dataGot++] = b;
      continue;
    }
    uint8_t next;
    if (!readByte(next, deadline))
    {
      frameError = true;
      break;
    }
    captureByte(next);
    if (next == DLE)
    {
      if (dataGot >= dataCap)
      {
        frameError = true;
        break;
      }
      data[dataGot++] = DLE;
      continue;
    }
    if (next == ETX)
    {
      sawTrailer = true;
      continue;
    }
    frameError = true;
    break;
  }

  if (frameError)
  {
    drainForDiagnostics(150);
    lastRxHex_ = hexDump(rawCapture, rawLen);
    lastOutcome_ = PollOutcome::kMissingTrailer;
    return false;
  }

  uint8_t crcBytes[2];
  if (!readByte(crcBytes[0], deadline) || !readByte(crcBytes[1], deadline))
  {
    drainForDiagnostics(150);
    lastRxHex_ = hexDump(rawCapture, rawLen);
    lastOutcome_ = PollOutcome::kShortReply;
    return false;
  }
  captureByte(crcBytes[0]);
  captureByte(crcBytes[1]);
  lastRxHex_ = hexDump(rawCapture, rawLen);
  Serial.printf("SpiCcp: RX %s\n", lastRxHex_.c_str());

  uint16_t rxCrc = (static_cast<uint16_t>(crcBytes[0]) << 8) | crcBytes[1];

  uint16_t crc = 0;
  for (size_t i = 2; i <= 7; i++)
    crc = crcUpdate(crc, header[i]);
  crc = crcUpdate(crc, header[9]); // STX
  for (size_t i = 0; i < dataGot; i++)
    crc = crcUpdate(crc, data[i]);
  crc = crcUpdate(crc, ETX);

  if (crc != rxCrc)
  {
    lastOutcome_ = PollOutcome::kCrcMismatch;
    crcErrorCount_++;
    uint8_t nak[2] = {static_cast<uint8_t>(kNoError | 0x01), NAK};
    Bus485.write(nak, sizeof(nak));
    return false;
  }

  uint8_t ack[2] = {DLE, '1'};
  Bus485.write(ack, sizeof(ack));

  outLen = dataGot;
  lastOutcome_ = PollOutcome::kSuccess;
  return true;
}

bool SpiCcp::select(uint8_t devId, uint8_t addr, uint8_t cmd1, uint8_t cmd2, const uint8_t *data, size_t dataLen,
                    uint32_t timeoutMs)
{
  for (uint8_t attempt = 0; attempt <= kMaxSelectRetries; attempt++)
  {
    if (selectOnce(devId, addr, cmd1, cmd2, data, dataLen, timeoutMs))
      return true;
  }
  return false; // lastOutcome_/lastTxHex_/lastRxHex_ already reflect the final attempt
}

bool SpiCcp::selectOnce(uint8_t devId, uint8_t addr, uint8_t cmd1, uint8_t cmd2, const uint8_t *data, size_t dataLen,
                        uint32_t timeoutMs)
{
  flushStaleRx();
  sendSelectionSequence(devId, addr, cmd1, cmd2, /*isSelect=*/true);

  uint8_t buf[8];
  size_t got = readWithTimeout(buf, sizeof(buf), timeoutMs);
  lastRxHex_ = hexDump(buf, got);

  bool positiveAck = got >= 7 && buf[0] == devId && buf[1] == addr && buf[2] == cmd1 &&
                     buf[3] == (cmd2 | 0x01) && buf[5] == DLE && (buf[6] == '0' || buf[6] == '1');
  if (!positiveAck)
  {
    lastOutcome_ = got == 0 ? PollOutcome::kNoReply : PollOutcome::kFramingMismatch;
    return false;
  }

  // Control Station Message Format: <DLE><STX>(DATA)<DLE><ETX>(CRC-MSB)
  // (CRC-LSB). No header here (the selection sequence above already
  // established devId/addr/cmd1/cmd2), so per the PDF's DLE sequence
  // chart the STX itself is excluded from the CRC (only the data and the
  // ETX byte are covered). DATA is DLE-stuffed on the wire (see
  // stuffDle()) - any literal 0x10 byte in the value being written must be
  // doubled, same transparency rule the receive side has to undo.
  uint8_t frame[192];
  size_t pos = 0;
  frame[pos++] = DLE;
  frame[pos++] = STX;
  pos += stuffDle(frame + pos, sizeof(frame) - pos, data, dataLen);
  frame[pos++] = DLE;
  frame[pos++] = ETX;

  uint16_t crc = 0;
  for (size_t i = 0; i < dataLen; i++)
    crc = crcUpdate(crc, data[i]);
  crc = crcUpdate(crc, ETX);

  frame[pos++] = (crc >> 8) & 0xFF;
  frame[pos++] = crc & 0xFF;

  Bus485.write(frame, pos);
  lastTxHex_ = hexDump(frame, pos);  // the data block - the selection sequence overwrote lastTxHex_ already

  // Exactly the spec's ACK/NAK width. readWithTimeout() busy-waits for the
  // *entire* timeoutMs whenever the buffer isn't filled (no early-exit on
  // "nothing more is coming"), so a larger buffer here doesn't capture more
  // of a genuinely longer reply - it just guarantees every call blocks for
  // the full timeout, since every real tributary reply seen so far is 2
  // bytes. That combination briefly wedged the main loop when a Modbus
  // write triggered this synchronously (see DryerRegisters::
  // onSetSpiRegister) - keep this at 2 so a normal ACK returns as soon as
  // it arrives, same as every other read in this file.
  uint8_t ackBuf[2];
  got = readWithTimeout(ackBuf, sizeof(ackBuf), timeoutMs);
  lastRxHex_ = lastRxHex_ + " " + hexDump(ackBuf, got);  // append the final ACK/NAK to the confirm reply
  bool ok = got == 2 && ackBuf[0] == DLE && (ackBuf[1] == '0' || ackBuf[1] == '1');
  lastOutcome_ = ok ? PollOutcome::kSuccess : PollOutcome::kMissingTrailer;
  return ok;
}
