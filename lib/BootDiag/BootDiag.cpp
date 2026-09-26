#include "BootDiag.h"

#include <esp_system.h>

#include <cstring>

namespace BootDiag
{
  namespace
  {
    char currentReason_[32] = "Unknown";

    const char *reasonToText(esp_reset_reason_t reason)
    {
      switch (reason)
      {
      case ESP_RST_POWERON:
        return "Power-on";
      case ESP_RST_EXT:
        return "External pin";
      case ESP_RST_SW:
        return "Software (restart/OTA)";
      case ESP_RST_PANIC:
        return "Panic (crash)";
      case ESP_RST_INT_WDT:
        return "Interrupt watchdog";
      case ESP_RST_TASK_WDT:
        return "Task watchdog (hang)";
      case ESP_RST_WDT:
        return "Other watchdog";
      case ESP_RST_DEEPSLEEP:
        return "Deep sleep wake";
      case ESP_RST_BROWNOUT:
        return "Brownout (power sag)";
      case ESP_RST_SDIO:
        return "SDIO";
      default:
        return "Unknown";
      }
    }
  } // namespace

  void begin()
  {
    const char *text = reasonToText(esp_reset_reason());
    strncpy(currentReason_, text, sizeof(currentReason_) - 1);
    currentReason_[sizeof(currentReason_) - 1] = '\0';
    Serial.printf("BootDiag: reset reason = %s\n", currentReason_);
  }

  const char *currentResetReason() { return currentReason_; }

} // namespace BootDiag
