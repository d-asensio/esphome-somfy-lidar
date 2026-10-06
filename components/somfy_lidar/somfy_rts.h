#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/cc1101/cc1101.h"
#include "esphome/components/remote_base/remote_base.h"

namespace esphome::somfy_lidar {

/// Button codes of the 56-bit Somfy RTS protocol.
enum class Command : uint8_t {
  MY = 0x1,
  UP = 0x2,
  DOWN = 0x4,
  PROG = 0x8,
};

const char *command_to_string(Command command);

/// A virtual Somfy RTS remote with its own address and rolling code.
///
/// The frames are produced on a blocking remote_transmitter whose pin drives
/// GDO0 of a CC1101 in asynchronous serial (OOK) mode. The radio is switched
/// to TX for each transmission and kept idle otherwise. Frame layout and timings come from
/// ESPSomfy RTS (https://github.com/rstrouse/ESPSomfy-RTS).
class SomfyRTSRemote : public Component {
 public:
  void set_transmitter(remote_base::RemoteTransmitterBase *transmitter) { this->transmitter_ = transmitter; }
  void set_radio(cc1101::CC1101Component *radio) { this->radio_ = radio; }
  void set_address(uint32_t address) { this->address_ = address; }
  void set_initial_rolling_code(uint16_t code) { this->rolling_code_ = code; }
  void set_repeat(uint8_t repeat) { this->repeat_ = repeat; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void send_command(Command command) { this->send_command(command, this->repeat_); }
  void send_command(Command command, uint8_t repeat);

  uint16_t get_rolling_code() const { return this->rolling_code_; }

 protected:
  void encode_frame_(Command command, uint16_t rolling_code, uint8_t *frame) const;

  remote_base::RemoteTransmitterBase *transmitter_{nullptr};
  cc1101::CC1101Component *radio_{nullptr};
  uint32_t address_{0};
  uint16_t rolling_code_{1};
  uint8_t repeat_{2};
  ESPPreferenceObject pref_;
};

}  // namespace esphome::somfy_lidar
