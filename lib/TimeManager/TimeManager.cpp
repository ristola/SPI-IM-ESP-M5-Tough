#include "TimeManager.h"

#include <M5Unified.h>
#include <time.h>

TimeManager Clock;

bool TimeManager::begin() {
  if (!isRtcAvailable()) return false;

  auto dt = M5.Rtc.getDateTime();
  struct tm tmStruct = {};
  tmStruct.tm_year = dt.date.year - 1900;
  tmStruct.tm_mon = dt.date.month - 1;
  tmStruct.tm_mday = dt.date.date;
  tmStruct.tm_hour = dt.time.hours;
  tmStruct.tm_min = dt.time.minutes;
  tmStruct.tm_sec = dt.time.seconds;

  struct timeval tv = {mktime(&tmStruct), 0};
  settimeofday(&tv, nullptr);
  return true;
}

bool TimeManager::syncFromNTP(const char* ntpServer, uint32_t timeoutMs) {
  configTime(0, 0, ntpServer);  // UTC, no DST

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, timeoutMs)) return false;

  if (isRtcAvailable()) {
    time_t t = time(nullptr) + 1;  // advance one second
    while (t > time(nullptr)) {
    }  // wait for the second boundary
    M5.Rtc.setDateTime(gmtime(&t));
  }
  return true;
}

bool TimeManager::isRtcAvailable() const { return M5.Rtc.isEnabled(); }

bool TimeManager::isRtcTrustworthy() const {
  return isRtcAvailable() && !M5.Rtc.getVoltLow();
}

String TimeManager::nowString() const {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10)) return "unknown";
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}
