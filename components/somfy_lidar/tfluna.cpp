#include "tfluna.h"

#include <algorithm>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::somfy_lidar {

static const char *const TAG = "somfy_lidar.tfluna";

static const size_t MAX_SAMPLES = 64;
// Registers 0x00-0x05: DIST_L DIST_H AMP_L AMP_H TEMP_L TEMP_H (temperature in 0.01 °C).
static const uint8_t REG_DIST_LO = 0x00;
static const uint8_t REG_TEMP_HI = 0x05;
static const uint8_t REG_VERSION_REV = 0x0A;

void TFLuna::setup() {
  this->distances_.reserve(MAX_SAMPLES);
  uint8_t version[3];
  for (uint8_t i = 0; i < 3; i++) {
    if (this->read_register(REG_VERSION_REV + i, &version[i], 1) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "No TF-Luna at I2C address 0x%02X (is pin 5 tied to GND?)", this->address_);
      this->mark_failed();
      return;
    }
  }
  ESP_LOGD(TAG, "Firmware %u.%u.%u", version[2], version[1], version[0]);
}

void TFLuna::loop() {
  const uint32_t now = millis();
  if (now - this->last_sample_ms_ < this->sample_interval_)
    return;
  this->last_sample_ms_ = now;

  // The TF-Luna does not auto-increment reliably, so registers are read one at a time.
  uint8_t data[6];
  for (uint8_t reg = REG_DIST_LO; reg <= REG_TEMP_HI; reg++) {
    if (this->read_register(reg, &data[reg], 1) != i2c::ERROR_OK) {
      this->read_errors_++;
      return;
    }
  }
  uint16_t distance = encode_uint16(data[1], data[0]);
  uint16_t strength = encode_uint16(data[3], data[2]);
  this->last_temperature_ = static_cast<int16_t>(encode_uint16(data[5], data[4])) / 100.0f;

  // 65535 means the receiver is saturated (e.g. direct sunlight or a retroreflector).
  if (distance == 0 || strength < this->min_signal_strength_ || strength == 0xFFFF) {
    this->weak_readings_++;
    return;
  }
  if (this->distances_.size() < MAX_SAMPLES)
    this->distances_.push_back(distance);
  this->strength_sum_ += strength;
  this->strength_count_++;
}

void TFLuna::update() {
  if (this->distances_.empty()) {
    if (!this->warned_)
      ESP_LOGW(TAG, "No valid reading in the last window (read errors: %" PRIu32 ", weak/saturated: %" PRIu32 ")",
               this->read_errors_, this->weak_readings_);
    this->warned_ = true;
    this->distance_sensor_->publish_state(NAN);
  } else {
    if (this->warned_)
      ESP_LOGI(TAG, "Valid readings again");
    this->warned_ = false;
    auto mid = this->distances_.begin() + this->distances_.size() / 2;
    std::nth_element(this->distances_.begin(), mid, this->distances_.end());
    this->distance_sensor_->publish_state(*mid);
  }

  if (this->signal_strength_sensor_ != nullptr && this->strength_count_ > 0)
    this->signal_strength_sensor_->publish_state(static_cast<float>(this->strength_sum_) / this->strength_count_);
  if (this->temperature_sensor_ != nullptr && !std::isnan(this->last_temperature_))
    this->temperature_sensor_->publish_state(this->last_temperature_);

  this->distances_.clear();
  this->strength_sum_ = 0;
  this->strength_count_ = 0;
  this->read_errors_ = 0;
  this->weak_readings_ = 0;
}

void TFLuna::dump_config() {
  ESP_LOGCONFIG(TAG,
                "TF-Luna:\n"
                "  Sample interval: %" PRIu32 " ms\n"
                "  Min signal strength: %u",
                this->sample_interval_, this->min_signal_strength_);
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  if (this->is_failed())
    ESP_LOGE(TAG, "Communication with the TF-Luna failed");
}

}  // namespace esphome::somfy_lidar
