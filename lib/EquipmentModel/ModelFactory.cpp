#include "ModelFactory.h"

#include "DeviceSettings.h"
#include "DryerADV.h"
#include "DryerCD.h"
#include "DryerFC.h"
#include "DryerFD.h"
#include "DryerFN.h"
#include "Crystallizer.h"
#include "EthernetNode.h"

EquipmentModel* ActiveModel = nullptr;

EquipmentModel& SelectEquipmentModel() {
  DeviceSettings::EquipmentType equip = Settings.equipmentType();
  String model = Settings.model();
  uint8_t addr = Settings.spiAddress();

  EquipmentModel* selected;
  uint8_t devId;

  if (model == DeviceSettings::kEthernetModels[0]) {
    // ETHERNET is a model choice orthogonal to equipmentType() (see
    // DeviceSettings.h's kEthernetModels comment) - checked first so it
    // takes effect regardless of which of Dryer/Crystallizer is set.
    // No SPI-CCP equipment attached at all (see EthernetNode.h) - devId
    // is never actually used since pollNext() never sends anything, but
    // begin() still needs some value.
    devId = 0x00;
    selected = &EthernetEquipmentModel;
  } else if (equip == DeviceSettings::EquipmentType::Crystallizer) {
    // FC-Crystallizer and FN-Crystallizer share one command set (see
    // Crystallizer.h) - only DevID differs, per "DataSheets/SPI-CCP Notes
    // - Dryer and Crystallizer Polls.md" section 1.
    devId = (model == "FN-XTLR") ? 0x22 : 0x5C;  // default FC-XTLR
    selected = &XtlrCrystallizer;
  } else {
    devId = 0x22;  // every dryer model uses this DevID
    if (model == "FC")
      selected = &FcDryer;
    else if (model == "FN")
      selected = &FnDryer;
    else if (model == "ADV")
      selected = &AdvDryer;
    else if (model == "CD")
      selected = &CdDryer;
    else
      selected = &Dryer;  // default/fallback: FD
  }

  ActiveModel = selected;
  ActiveModel->begin(devId, addr);
  return *ActiveModel;
}
