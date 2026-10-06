#include "tfluna.h"

#include <algorithm>

#include "esphome/core/log.h"

namespace esphome::tfluna {

static const char *const TAG = "tfluna";

// Frame: 0x59 0x59 DIST_L DIST_H AMP_L AMP_H TEMP_L TEMP_H CHECKSUM
static const uint8_t FRAME_HEADER = 0x59;
static const size_t MAX_SAMPLES = 64;

void TFLunaComponent::setup() { this->distances_.reserve(MAX_SAMPLES); }

void TFLunaComponent::loop() {
  uint8_t byte;
  while (this->available() && this->read_byte(&byte)) {
    if (this->frame_pos_ < 2 && byte != FRAME_HEADER) {
      this->frame_pos_ = 0;
      continue;
    }
    this->frame_[this->frame_pos_++] = byte;
    if (this->frame_pos_ == this->frame_.size()) {
      this->handle_frame_();
      this->frame_pos_ = 0;
    }
  }
}

void TFLunaComponent::handle_frame_() {
  uint8_t sum = 0;
  for (size_t i = 0; i < 8; i++)
    sum += this->frame_[i];
  if (sum != this->frame_[8]) {
    this->bad_checksums_++;
    return;
  }

  uint16_t distance = encode_uint16(this->frame_[3], this->frame_[2]);
  uint16_t strength = encode_uint16(this->frame_[5], this->frame_[4]);
  int16_t temp_raw = static_cast<int16_t>(encode_uint16(this->frame_[7], this->frame_[6]));
  this->last_temperature_ = temp_raw / 8.0f - 256.0f;

  // 65535 means the receiver is saturated (e.g. direct sunlight or a retroreflector).
  if (distance == 0 || strength < this->min_signal_strength_ || strength == 0xFFFF) {
    this->weak_frames_++;
    return;
  }
  if (this->distances_.size() < MAX_SAMPLES)
    this->distances_.push_back(distance);
  this->strength_sum_ += strength;
  this->strength_count_++;
}

void TFLunaComponent::update() {
  if (this->distances_.empty()) {
    if (!this->warned_)
      ESP_LOGW(TAG, "No valid reading in the last window (bad checksums: %" PRIu32 ", weak/saturated: %" PRIu32 ")",
               this->bad_checksums_, this->weak_frames_);
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
  this->bad_checksums_ = 0;
  this->weak_frames_ = 0;
}

void TFLunaComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "TF-Luna:\n  Min signal strength: %u", this->min_signal_strength_);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_SENSOR("  ", "Signal strength", this->signal_strength_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

}  // namespace esphome::tfluna
