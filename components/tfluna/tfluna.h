#pragma once

#include <array>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome::tfluna {

/// Benewake TF-Luna in its default UART mode (115200 baud, 100 Hz frames).
///
/// Every frame received between two updates is collected and the median of the
/// valid ones is published, so update_interval doubles as the smoothing window.
/// When no valid frame arrived in a window the distance is published as NAN.
class TFLunaComponent : public PollingComponent, public uart::UARTDevice {
 public:
  void set_distance_sensor(sensor::Sensor *s) { this->distance_sensor_ = s; }
  void set_signal_strength_sensor(sensor::Sensor *s) { this->signal_strength_sensor_ = s; }
  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_sensor_ = s; }
  void set_min_signal_strength(uint16_t v) { this->min_signal_strength_ = v; }

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void handle_frame_();

  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *signal_strength_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  uint16_t min_signal_strength_{100};

  std::array<uint8_t, 9> frame_{};
  uint8_t frame_pos_{0};

  std::vector<uint16_t> distances_;
  uint32_t strength_sum_{0};
  uint32_t strength_count_{0};
  float last_temperature_{NAN};
  uint32_t bad_checksums_{0};
  uint32_t weak_frames_{0};
  bool warned_{false};
};

}  // namespace esphome::tfluna
