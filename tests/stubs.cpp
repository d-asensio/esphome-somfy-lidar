// Minimal glue so the components link on the ESPHome host platform without the
// rest of remote_base (which drags in binary_sensor) or a real CC1101.
#include "esphome/components/cc1101/cc1101.h"
#include "esphome/components/remote_base/remote_base.h"

void loop() {}

namespace esphome::remote_base {
void RemoteTransmitterBase::send_(uint32_t send_times, uint32_t send_wait) {
  this->send_internal(send_times, send_wait);
}
}  // namespace esphome::remote_base

namespace esphome::cc1101 {
void CC1101Component::begin_tx() {}
void CC1101Component::set_idle() {}
}  // namespace esphome::cc1101
