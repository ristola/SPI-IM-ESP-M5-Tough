#pragma once

// Serves a small HTML dashboard (port 80) showing equipment/model
// selection, the SPI-CCP poll queries the active model (see
// lib/EquipmentModel/ModelFactory) sends, and the register data last
// returned over RS485. ROLE_NODE only - depends on ActiveModel/SpiIm,
// which only exist in that build role (see src/main.cpp).
class DryerWebServer
{
public:
    void begin();

    // Non-blocking - call every loop() alongside Registers.task().
    void handleClient();
};

extern DryerWebServer DryerWeb;
