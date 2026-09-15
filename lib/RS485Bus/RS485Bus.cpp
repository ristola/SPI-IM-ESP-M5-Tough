#include "RS485Bus.h"

#include "pins.h"

RS485Bus Bus485;

void RS485Bus::begin(uint32_t baud) {
  _serial.begin(baud, SERIAL_8N1, Pins::RS485_RX, Pins::RS485_TX);
}

size_t RS485Bus::write(const uint8_t* data, size_t len) { return _serial.write(data, len); }

int RS485Bus::available() { return _serial.available(); }

int RS485Bus::read() { return _serial.read(); }

size_t RS485Bus::readBytes(uint8_t* buffer, size_t maxLen, uint32_t timeoutMs) {
  _serial.setTimeout(timeoutMs);
  return _serial.readBytes(buffer, maxLen);
}
