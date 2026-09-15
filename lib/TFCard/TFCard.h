#pragma once

#include <SD.h>

// Wraps the microSD (TF) card slot, which shares the display's SPI bus.
class TFCard {
 public:
  bool begin(uint32_t spiFrequencyHz = 25000000);
  bool isMounted() const { return _mounted; }
  uint64_t cardSizeMB() const;

 private:
  bool _mounted = false;
};

extern TFCard SDCard;
