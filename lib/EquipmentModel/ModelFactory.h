#pragma once

#include "EquipmentModel.h"

// The model currently in use, selected by SelectEquipmentModel() below.
// Every other module (Modbus registers, ESP-NOW broadcast, the web
// dashboard) should read/poll through this pointer rather than reaching
// for a specific model's global directly - that's what makes "pick Dryer
// or Crystallizer, then pick FC/FD/FN/ADV/CD" a runtime choice instead of
// a compile-time one.
extern EquipmentModel* ActiveModel;

// Reads DeviceSettings' equipmentType()/model()/spiAddress(), selects the
// matching concrete model singleton (see each model's own lib for its
// SPI-CCP command table), calls begin() on it with the right DevID for
// that equipment/model combination, sets it as ActiveModel, and returns
// it. Call once from setup(), and again any time equipment type or model
// changes at runtime (e.g. from the web dashboard) to re-select and
// re-begin() the newly-chosen model.
EquipmentModel& SelectEquipmentModel();
