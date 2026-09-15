#pragma once

#include <HardwareSerial.h>

// Wraps the Tough Ext Board's built-in RS485 port (Serial2). The onboard
// transceiver switches direction automatically - no DE/RE pin to drive.
class RS485Bus {
 public:
  void begin(uint32_t baud = 115200);

  size_t write(const uint8_t* data, size_t len);
  int available();
  int read();
  size_t readBytes(uint8_t* buffer, size_t maxLen, uint32_t timeoutMs = 100);

 private:
  HardwareSerial& _serial = Serial2;
};

extern RS485Bus Bus485;
