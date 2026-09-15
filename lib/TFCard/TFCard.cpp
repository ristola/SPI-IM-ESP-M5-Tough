#include "TFCard.h"

#include "pins.h"

TFCard SDCard;

bool TFCard::begin(uint32_t spiFrequencyHz) {
  _mounted = SD.begin(Pins::SD_CS, SPI, spiFrequencyHz);
  return _mounted;
}

uint64_t TFCard::cardSizeMB() const {
  if (!_mounted) return 0;
  return SD.cardSize() / (1024 * 1024);
}
