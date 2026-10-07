#pragma once

#include "esphome/core/defines.h"
#ifdef USE_SOMFY_LIDAR_TFLUNA

#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome::somfy_lidar {

/// Benewake TF-Luna lidar in I2C mode (pin 5 tied to GND).
///
/// The latest measurement is polled every sample_interval. Every update the
/// median of the valid readings taken since the last update is published, so
/// update_interval doubles as the smoothing window. When no valid reading
/// arrived in a window the distance is published as NAN.
class TFLuna : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_distance_sensor(sensor::Sensor *s) { this->distance_sensor_ = s; }
  void set_signal_strength_sensor(sensor::Sensor *s) { this->signal_strength_sensor_ = s; }
  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_sensor_ = s; }
  void set_min_signal_strength(uint16_t v) { this->min_signal_strength_ = v; }
  void set_sample_interval(uint32_t ms) { this->sample_interval_ = ms; }

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *signal_strength_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  uint16_t min_signal_strength_{100};
  uint32_t sample_interval_{20};
  uint32_t last_sample_ms_{0};

  std::vector<uint16_t> distances_;
  uint32_t strength_sum_{0};
  uint32_t strength_count_{0};
  float last_temperature_{NAN};
  uint32_t read_errors_{0};
  uint32_t weak_readings_{0};
  bool warned_{false};
};

}  // namespace esphome::somfy_lidar

#endif  // USE_SOMFY_LIDAR_TFLUNA
