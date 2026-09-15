#include "DryerCD.h"

DryerCD CdDryer;

void DryerCD::pollNext() {
  switch (queryIndex_) {
    case 0:
      pollProcessSetpoint(kCmd1);
      break;
    case 1:
      pollProcessDelta(kCmd1);
      break;
    case 2:
      pollProcessStatus(kCmd1);
      break;
    case 3:
      pollMachineStatus(kCmd1);
      break;
    case 4:
      pollDewTrigger(kCmd1);
      break;
    case 5:
      pollProcessTemp(kCmd1);
      break;
    case 6:
      pollReturnTemp(kCmd1);
      break;
  }
  queryIndex_ = (queryIndex_ + 1) % kQueryCount;
}

SpiCcpQueryInfo DryerCD::queryInfo(size_t index) const {
  static constexpr SpiCcpQueryInfo kQueries[kQueryCount] = {
      {"Process Setpoint", kCmd1, 0x30, "40010"}, {"Process Delta", kCmd1, 0x32, "40011"},
      {"Process Status", kCmd1, 0x40, "40013"},   {"Machine Status", kCmd1, 0x48, "40014"},
      {"Dew Trigger", kCmd1, 0x80, "40017"},      {"Process Temp", kCmd1, 0x70, "40012"},
      {"Return Temp", kCmd1, 0x72, "40015"},
  };
  return kQueries[index];
}
