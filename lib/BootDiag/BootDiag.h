#pragma once

#include <Arduino.h>

// Captures why the current boot happened - reported by Discovery.cpp's user
// as "the device crashes/reboots mid-scan, losing all discovered results."
// esp_reset_reason() already tells us this for free (it's read from a
// hardware/ROM-set register at reset, not something we compute), but
// nothing in this codebase was reading or surfacing it - so every past
// reset (watchdog, panic, brownout, or just a deliberate restart) looked
// identical: gone, no trace. begin() must run first thing in setup(), for
// two reasons: it has to run before anything else has a chance to crash
// (defeating the whole point), and it captures the CURRENT boot's reason,
// so no persistence across boots is needed - DryerWebServer's Diagnostics
// tab just reads it live on every page load, and whichever boot is running
// when someone next checks the dashboard will report its own real cause.
namespace BootDiag
{
  void begin();

  // Human-readable reason this boot started (e.g. "Task watchdog (hang)",
  // "Panic (crash)", "Brownout (power sag)", "Power-on", "Software
  // (restart/OTA)").
  const char *currentResetReason();
}
