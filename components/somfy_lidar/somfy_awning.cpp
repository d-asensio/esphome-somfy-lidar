#include "somfy_awning.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::somfy_lidar {

static const char *const TAG = "somfy_lidar.awning";

static const uint32_t CALIBRATION_MAGIC = 0x41574E31;  // "AWN1"
static const float MIN_SPAN_CM = 20.0f;
// Below this the awning counts as standing still. TF-Luna jitter over the 1s
// velocity window stays well under it; awnings move at roughly 3-10 cm/s.
static const float MOVING_CM_S = 2.0f;
static const uint32_t VELOCITY_WINDOW_MS = 1200;
static const uint32_t VELOCITY_MIN_SPAN_MS = 600;
static const uint32_t STILL_MS = 1500;
static const uint32_t CALIBRATION_STILL_MS = 2500;
// Ignore stillness right after a command while the motor spins up.
static const uint32_t START_GRACE_MS = 3000;
// Never send MY right after UP/DOWN: if the motor has not started yet it would
// take MY as "go to the favourite position".
static const uint32_t MIN_MY_DELAY_MS = 800;
static const uint32_t SENSOR_TIMEOUT_MS = 1500;
static const uint32_t STOPPING_TIMEOUT_MS = 8000;
static const float MIN_STOP_LATENCY_S = 0.1f;
static const float MAX_STOP_LATENCY_S = 3.0f;

void SomfyAwning::setup() {
  if (this->sensor_ == nullptr) {
    // Without a lidar the state is assumed: restore the last command's result.
    auto restore = this->restore_state_();
    if (restore.has_value()) {
      restore->apply(this);
    } else {
      this->position = cover::COVER_CLOSED;
    }
    this->current_operation = cover::COVER_OPERATION_IDLE;
    this->publish_state(false);
    this->disable_loop();
    return;
  }
  // Non-zero version keeps this apart from the cover's own restore-state slot.
  this->pref_ = this->make_entity_preference<Calibration>(0xA3A1C0DE);
  Calibration stored;
  if (this->pref_.load(&stored) && stored.magic == CALIBRATION_MAGIC) {
    this->cal_ = stored;
  }
  this->sensor_->add_on_state_callback([this](float cm) { this->on_distance_(cm); });
  if (this->stop_latency_sensor_ != nullptr)
    this->defer([this]() { this->stop_latency_sensor_->publish_state(this->cal_.stop_latency_s); });
}

void SomfyAwning::dump_config() {
  LOG_COVER("", "Somfy Awning", this);
  if (this->sensor_ == nullptr) {
    ESP_LOGCONFIG(TAG,
                  "  Open command: %s\n"
                  "  No lidar: open/close only, assumed state",
                  command_to_string(this->open_cmd_));
    return;
  }
  ESP_LOGCONFIG(TAG,
                "  Open command: %s\n"
                "  Calibrated: %s (closed %.1f cm, open %.1f cm)\n"
                "  Stop latency: %.2f s\n"
                "  Position tolerance: %.1f%%\n"
                "  Max travel time: %" PRIu32 " ms",
                command_to_string(this->open_cmd_), YESNO(this->is_calibrated()), this->cal_.closed_cm,
                this->cal_.open_cm, this->cal_.stop_latency_s, this->tolerance_ * 100.0f, this->max_travel_ms_);
}

cover::CoverTraits SomfyAwning::get_traits() {
  auto traits = cover::CoverTraits();
  // Without a lidar there is no position, and no stop: whether the motor is
  // still moving is unknown, and MY on a still motor goes to the favourite position.
  const bool has_lidar = this->sensor_ != nullptr;
  traits.set_supports_position(has_lidar);
  traits.set_supports_stop(has_lidar);
  traits.set_is_assumed_state(!has_lidar);
  return traits;
}

bool SomfyAwning::is_calibrated() const {
  return !std::isnan(this->cal_.closed_cm) && !std::isnan(this->cal_.open_cm) &&
         std::fabs(this->cal_.open_cm - this->cal_.closed_cm) >= MIN_SPAN_CM;
}

float SomfyAwning::position_for_(float cm) const {
  float pos = (cm - this->cal_.closed_cm) / (this->cal_.open_cm - this->cal_.closed_cm);
  return clamp(pos, 0.0f, 1.0f);
}

float SomfyAwning::velocity_cm_s_() const {
  // Least-squares slope over the samples in the window; far less noisy than
  // the difference between two readings.
  const size_t n = this->samples_.size();
  if (this->sample_count_ < 4)
    return NAN;
  const Sample &newest = this->samples_[(this->sample_head_ + n - 1) % n];
  float sum_t = 0, sum_d = 0, sum_tt = 0, sum_td = 0;
  uint8_t count = 0;
  uint32_t span = 0;
  for (uint8_t i = 1; i <= this->sample_count_; i++) {
    const Sample &s = this->samples_[(this->sample_head_ + n - i) % n];
    uint32_t age = newest.t - s.t;
    if (age > VELOCITY_WINDOW_MS)
      break;
    float t = -static_cast<float>(age) / 1000.0f;
    float d = s.cm - newest.cm;
    sum_t += t;
    sum_d += d;
    sum_tt += t * t;
    sum_td += t * d;
    count++;
    span = age;
  }
  if (count < 4 || span < VELOCITY_MIN_SPAN_MS)
    return NAN;
  float denom = count * sum_tt - sum_t * sum_t;
  if (denom <= 0)
    return NAN;
  return (count * sum_td - sum_t * sum_d) / denom;
}

bool SomfyAwning::is_still_for_(uint32_t ms) const {
  return this->still_since_ms_ != 0 && millis() - this->still_since_ms_ >= ms;
}

void SomfyAwning::set_mode_(Mode mode) {
  this->mode_ = mode;
  this->mode_start_ms_ = millis();
}

void SomfyAwning::move_(int8_t dir, Mode mode) {
  this->remote_->send_command(dir > 0 ? this->open_cmd_ : this->close_cmd_);
  this->dir_ = dir;
  this->move_cmd_ms_ = millis();
  this->still_since_ms_ = 0;
  this->set_mode_(mode);
  this->current_operation = dir > 0 ? cover::COVER_OPERATION_OPENING : cover::COVER_OPERATION_CLOSING;
  this->publish_(true);
}

void SomfyAwning::send_stop_(const char *reason) {
  ESP_LOGW(TAG, "Stopping: %s", reason);
  this->remote_->send_command(Command::MY);
  this->target_ = NAN;
  this->set_mode_(Mode::STOPPING);
}

void SomfyAwning::control(const cover::CoverCall &call) {
  if (this->sensor_ == nullptr) {
    this->control_without_lidar_(call);
    return;
  }
  if (call.get_stop()) {
    // Only trust what we commanded or what the sensor shows right now; MY on a
    // motor that is standing still would send it to the favourite position.
    float v = this->velocity_cm_s_();
    bool commanded = this->mode_ == Mode::TO_END || this->mode_ == Mode::SEEKING || this->mode_ == Mode::CAL_CLOSE ||
                     this->mode_ == Mode::CAL_OPEN || this->mode_ == Mode::CAL_RECLOSE;
    bool measured = !std::isnan(v) && std::fabs(v) >= MOVING_CM_S;
    if (!commanded && !measured) {
      ESP_LOGI(TAG, "Awning is not moving, not sending MY");
      return;
    }
    if (this->mode_ == Mode::CAL_CLOSE || this->mode_ == Mode::CAL_OPEN || this->mode_ == Mode::CAL_RECLOSE)
      ESP_LOGW(TAG, "Calibration aborted");
    this->remote_->send_command(Command::MY);
    this->target_ = NAN;
    this->set_mode_(Mode::STOPPING);
    return;
  }

  if (!call.get_position().has_value())
    return;
  float target = *call.get_position();

  if (this->mode_ == Mode::CAL_CLOSE || this->mode_ == Mode::CAL_OPEN || this->mode_ == Mode::CAL_RECLOSE) {
    ESP_LOGW(TAG, "Calibration in progress, ignoring command");
    return;
  }
  // The motor stops at its own limits, so the ends never need the sensor.
  if (target >= cover::COVER_OPEN - 0.001f) {
    this->target_ = NAN;
    this->move_(+1, Mode::TO_END);
    return;
  }
  if (target <= cover::COVER_CLOSED + 0.001f) {
    this->target_ = NAN;
    this->move_(-1, Mode::TO_END);
    return;
  }

  if (!this->is_calibrated()) {
    ESP_LOGW(TAG, "Not calibrated, only fully open and fully closed are possible");
    return;
  }
  if (std::isnan(this->last_cm_) || millis() - this->last_valid_ms_ > SENSOR_TIMEOUT_MS) {
    ESP_LOGW(TAG, "No valid distance reading, refusing to move to %.0f%%", target * 100.0f);
    return;
  }
  float current = this->position_for_(this->last_cm_);
  float delta = target - current;
  if (std::fabs(delta) < std::max(this->tolerance_, this->min_travel_)) {
    ESP_LOGD(TAG, "Already at %.0f%% (target %.0f%%)", current * 100.0f, target * 100.0f);
    this->publish_(true);
    return;
  }
  ESP_LOGD(TAG, "Moving from %.0f%% to %.0f%%", current * 100.0f, target * 100.0f);
  this->target_ = target;
  this->move_(delta > 0 ? +1 : -1, Mode::SEEKING);
}

void SomfyAwning::control_without_lidar_(const cover::CoverCall &call) {
  if (!call.get_position().has_value())
    return;
  // The motor runs to its own end limits; assume it gets there.
  bool open = *call.get_position() > cover::COVER_CLOSED;
  this->remote_->send_command(open ? this->open_cmd_ : this->close_cmd_);
  this->position = open ? cover::COVER_OPEN : cover::COVER_CLOSED;
  this->current_operation = cover::COVER_OPERATION_IDLE;
  this->publish_state();
}

void SomfyAwning::on_distance_(float cm) {
  if (std::isnan(cm))
    return;  // timeouts are handled in loop()
  if (this->reported_distance_ != nullptr)
    this->reported_distance_->publish_state(cm);
  const uint32_t now = millis();
  this->last_cm_ = cm;
  this->last_valid_ms_ = now;

  this->samples_[this->sample_head_] = {now, cm};
  this->sample_head_ = (this->sample_head_ + 1) % this->samples_.size();
  if (this->sample_count_ < this->samples_.size())
    this->sample_count_++;

  float v = this->velocity_cm_s_();
  if (!std::isnan(v)) {
    if (std::fabs(v) < MOVING_CM_S) {
      if (this->still_since_ms_ == 0)
        this->still_since_ms_ = now;
    } else {
      this->still_since_ms_ = 0;
    }
  }

  const bool calibrated = this->is_calibrated();
  const float span = this->cal_.open_cm - this->cal_.closed_cm;
  const bool settled = now - this->mode_start_ms_ >= START_GRACE_MS;

  switch (this->mode_) {
    case Mode::SEEKING: {
      float position = this->position_for_(cm);
      float remaining = (this->target_ - position) * this->dir_;
      float speed = std::isnan(v) ? 0.0f : std::fabs(v / span);
      float lead = speed * this->cal_.stop_latency_s;
      if (remaining <= lead && now - this->move_cmd_ms_ >= MIN_MY_DELAY_MS) {
        ESP_LOGD(TAG, "Sending MY at %.1f%% (target %.1f%%, speed %.2f%%/s, lead %.1f%%)", position * 100.0f,
                 this->target_ * 100.0f, speed * 100.0f, lead * 100.0f);
        this->stop_speed_ = speed;
        this->remote_->send_command(Command::MY);
        this->set_mode_(Mode::STOPPING);
      } else if (settled && this->is_still_for_(STILL_MS)) {
        // Stopped by the physical remote, a wind sensor or an end limit. Sending MY
        // now would move it to the favourite position, so just give up the target.
        ESP_LOGI(TAG, "Awning stopped before reaching %.0f%%, giving up the target", this->target_ * 100.0f);
        this->target_ = NAN;
        this->set_mode_(Mode::IDLE);
      }
      break;
    }
    case Mode::STOPPING:
      if (this->is_still_for_(1000)) {
        if (!std::isnan(this->target_) && calibrated && this->stop_speed_ > 0.002f) {
          // Learn the stop latency from how far we overshot (positive) or undershot.
          float position = this->position_for_(cm);
          float overshoot = (position - this->target_) * this->dir_;
          float adjust = overshoot / this->stop_speed_;
          ESP_LOGI(TAG, "Stopped at %.1f%% for target %.1f%% (error %+.1f%%)", position * 100.0f,
                   this->target_ * 100.0f, overshoot * 100.0f);
          if (std::fabs(adjust) < MAX_STOP_LATENCY_S) {
            this->cal_.stop_latency_s =
                clamp(this->cal_.stop_latency_s + 0.5f * adjust, MIN_STOP_LATENCY_S, MAX_STOP_LATENCY_S);
            ESP_LOGD(TAG, "Stop latency now %.2f s", this->cal_.stop_latency_s);
            this->save_calibration_();
          }
        }
        this->target_ = NAN;
        this->set_mode_(Mode::IDLE);
      }
      break;
    case Mode::TO_END:
      if (settled && this->is_still_for_(STILL_MS))
        this->set_mode_(Mode::IDLE);
      break;
    case Mode::CAL_CLOSE:
    case Mode::CAL_OPEN:
    case Mode::CAL_RECLOSE:
      if (settled && this->is_still_for_(CALIBRATION_STILL_MS))
        this->finish_calibration_phase_();
      break;
    case Mode::IDLE:
      break;
  }

  // Operation: what we commanded, otherwise what the sensor shows (e.g. the physical remote).
  if (this->mode_ == Mode::IDLE || this->mode_ == Mode::STOPPING) {
    if (calibrated && !std::isnan(v) && std::fabs(v) >= MOVING_CM_S) {
      this->current_operation = (v / span) > 0 ? cover::COVER_OPERATION_OPENING : cover::COVER_OPERATION_CLOSING;
    } else if (this->is_still_for_(STILL_MS)) {
      this->current_operation = cover::COVER_OPERATION_IDLE;
    }
  } else {
    this->current_operation = this->dir_ > 0 ? cover::COVER_OPERATION_OPENING : cover::COVER_OPERATION_CLOSING;
  }
  this->publish_(false);
}

void SomfyAwning::loop() {
  if (this->sensor_ == nullptr)
    return;
  const uint32_t now = millis();
  const bool stale = now - this->last_valid_ms_ > SENSOR_TIMEOUT_MS;
  const uint32_t elapsed = now - this->mode_start_ms_;

  switch (this->mode_) {
    case Mode::SEEKING:
      if (stale && now - this->move_cmd_ms_ >= MIN_MY_DELAY_MS) {
        this->send_stop_("lost the distance reading");
      } else if (elapsed > this->max_travel_ms_) {
        this->send_stop_("target not reached within max_travel_time");
      }
      break;
    case Mode::STOPPING:
      if (elapsed > STOPPING_TIMEOUT_MS) {
        this->target_ = NAN;
        this->set_mode_(Mode::IDLE);
        this->current_operation = cover::COVER_OPERATION_IDLE;
        this->publish_(true);
      }
      break;
    case Mode::TO_END:
      if (elapsed > this->max_travel_ms_ || (stale && elapsed > START_GRACE_MS)) {
        this->set_mode_(Mode::IDLE);
        this->current_operation = cover::COVER_OPERATION_IDLE;
        this->publish_(true);
      }
      break;
    case Mode::CAL_CLOSE:
    case Mode::CAL_OPEN:
    case Mode::CAL_RECLOSE:
      if (stale || elapsed > this->max_travel_ms_) {
        ESP_LOGE(TAG, "Calibration aborted: %s", stale ? "lost the distance reading" : "awning never stood still");
        this->set_mode_(Mode::IDLE);
        this->current_operation = cover::COVER_OPERATION_IDLE;
        this->publish_(true);
      }
      break;
    case Mode::IDLE:
      if (stale && this->current_operation != cover::COVER_OPERATION_IDLE) {
        this->current_operation = cover::COVER_OPERATION_IDLE;
        this->publish_(true);
      }
      break;
  }
}

void SomfyAwning::start_calibration() {
  if (std::isnan(this->last_cm_) || millis() - this->last_valid_ms_ > SENSOR_TIMEOUT_MS) {
    ESP_LOGE(TAG, "Cannot calibrate without a valid distance reading");
    return;
  }
  ESP_LOGI(TAG, "Calibration 1/3: closing");
  this->cal_closed_cm_ = NAN;
  this->cal_open_cm_ = NAN;
  this->target_ = NAN;
  this->move_(-1, Mode::CAL_CLOSE);
}

void SomfyAwning::finish_calibration_phase_() {
  switch (this->mode_) {
    case Mode::CAL_CLOSE:
      this->cal_closed_cm_ = this->last_cm_;
      ESP_LOGI(TAG, "Closed at %.1f cm. Calibration 2/3: opening", this->cal_closed_cm_);
      this->move_(+1, Mode::CAL_OPEN);
      break;
    case Mode::CAL_OPEN:
      this->cal_open_cm_ = this->last_cm_;
      if (std::fabs(this->cal_open_cm_ - this->cal_closed_cm_) < MIN_SPAN_CM) {
        ESP_LOGE(TAG, "Calibration failed: open (%.1f cm) and closed (%.1f cm) are less than %.0f cm apart",
                 this->cal_open_cm_, this->cal_closed_cm_, MIN_SPAN_CM);
        this->set_mode_(Mode::IDLE);
        break;
      }
      ESP_LOGI(TAG, "Open at %.1f cm. Calibration 3/3: closing", this->cal_open_cm_);
      this->move_(-1, Mode::CAL_RECLOSE);
      break;
    case Mode::CAL_RECLOSE:
      if (std::fabs(this->last_cm_ - this->cal_closed_cm_) > 5.0f)
        ESP_LOGW(TAG, "Closed position differs between runs (%.1f vs %.1f cm); check the sensor aim",
                 this->cal_closed_cm_, this->last_cm_);
      this->cal_.closed_cm = (this->cal_closed_cm_ + this->last_cm_) / 2.0f;
      this->cal_.open_cm = this->cal_open_cm_;
      this->save_calibration_();
      ESP_LOGI(TAG, "Calibration done: closed %.1f cm, open %.1f cm", this->cal_.closed_cm, this->cal_.open_cm);
      this->set_mode_(Mode::IDLE);
      break;
    default:
      break;
  }
  this->current_operation = this->mode_ == Mode::IDLE ? cover::COVER_OPERATION_IDLE : this->current_operation;
  this->publish_(true);
}

void SomfyAwning::set_closed_here() {
  if (std::isnan(this->last_cm_)) {
    ESP_LOGE(TAG, "No distance reading");
    return;
  }
  this->cal_.closed_cm = this->last_cm_;
  ESP_LOGI(TAG, "Closed position set to %.1f cm", this->last_cm_);
  this->save_calibration_();
  this->publish_(true);
}

void SomfyAwning::set_open_here() {
  if (std::isnan(this->last_cm_)) {
    ESP_LOGE(TAG, "No distance reading");
    return;
  }
  this->cal_.open_cm = this->last_cm_;
  ESP_LOGI(TAG, "Open position set to %.1f cm", this->last_cm_);
  this->save_calibration_();
  this->publish_(true);
}

void SomfyAwning::save_calibration_() {
  this->cal_.magic = CALIBRATION_MAGIC;
  this->pref_.save(&this->cal_);
  if (this->stop_latency_sensor_ != nullptr)
    this->stop_latency_sensor_->publish_state(this->cal_.stop_latency_s);
}

void AwningButton::press_action() {
  switch (this->action_) {
    case ButtonAction::CALIBRATE:
      this->parent_->start_calibration();
      break;
    case ButtonAction::SET_CLOSED_HERE:
      this->parent_->set_closed_here();
      break;
    case ButtonAction::SET_OPEN_HERE:
      this->parent_->set_open_here();
      break;
    case ButtonAction::PROG:
      // Long enough for the motor to register a new remote while in programming mode.
      this->parent_->send_command(Command::PROG, 7);
      break;
  }
}

void SomfyAwning::publish_(bool force) {
  if (!this->is_calibrated() || std::isnan(this->last_cm_))
    return;
  float position = this->position_for_(this->last_cm_);
  // Sensor noise should not keep the ends from reading as fully open/closed.
  if (position <= this->tolerance_) {
    position = cover::COVER_CLOSED;
  } else if (position >= 1.0f - this->tolerance_) {
    position = cover::COVER_OPEN;
  }
  const uint32_t now = millis();
  bool operation_changed = this->current_operation != this->published_operation_;
  bool position_changed =
      std::isnan(this->published_position_) || std::fabs(position - this->published_position_) >= 0.005f;
  if (!force && !operation_changed && (!position_changed || now - this->last_publish_ms_ < 250))
    return;
  this->position = position;
  this->published_position_ = position;
  this->published_operation_ = this->current_operation;
  this->last_publish_ms_ = now;
  this->publish_state(false);
}

}  // namespace esphome::somfy_lidar
