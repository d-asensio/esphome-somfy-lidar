// Minimal glue so the components link on the ESPHome host platform without the
// rest of remote_base (which drags in binary_sensor).
#include "esphome/components/remote_base/remote_base.h"

void loop() {}

namespace esphome::remote_base {
void RemoteTransmitterBase::send_(uint32_t send_times, uint32_t send_wait) {
  this->send_internal(send_times, send_wait);
}
}  // namespace esphome::remote_base
