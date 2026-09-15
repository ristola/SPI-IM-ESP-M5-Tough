#pragma once

#include <cstddef>
#include <cstdint>

// Transparent-background PNG rendering of Assets/RTSLOGO.svg (200x200),
// embedded so it can be drawn at boot without needing an SD card present.
// Regenerate with: rsvg-convert -w 200 -h 200 -o logo.png Assets/RTSLOGO.svg
extern const uint8_t kBootLogoPng[];
extern const size_t kBootLogoPngLen;
