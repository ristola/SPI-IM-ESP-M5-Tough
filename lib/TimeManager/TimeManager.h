#pragma once

#include <Arduino.h>

// Wraps the Tough's onboard BM8563 RTC (via M5.Rtc) and NTP sync. The RTC
// keeps time across power loss; NTP (when WiFi is up) corrects drift and
// writes the result back to the RTC. Everything is kept in UTC.
class TimeManager {
 public:
  // Seeds the ESP32 system clock from the RTC. Call after M5.begin().
  bool begin();

  // Blocks up to timeoutMs syncing via NTP, then writes the result to the
  // RTC. Call only once WiFi is connected.
  bool syncFromNTP(const char* ntpServer = "pool.ntp.org", uint32_t timeoutMs = 10000);

  bool isRtcAvailable() const;

  // False means the RTC lost backup power at some point (BM8563 voltage-low
  // flag) and its date/time is just whatever it powered on with - not a
  // real time. True means it's kept ticking since it was last set and can
  // be trusted without an NTP sync.
  bool isRtcTrustworthy() const;

  String nowString() const;  // "YYYY-MM-DD HH:MM:SS" (UTC)
};

extern TimeManager Clock;
