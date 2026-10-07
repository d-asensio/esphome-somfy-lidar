#pragma once

#include <array>

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/button/button.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"
#include "somfy_rts.h"

namespace esphome::somfy_lidar {

/// Stored in flash so calibration survives reboots and reflashes.
struct Calibration {
  float closed_cm;
  float open_cm;
  float stop_latency_s;
  uint32_t magic;
} __attribute__((packed));

/// Somfy RTS awning whose position comes from a distance sensor instead of timing.
///
/// Fully open/close just sends UP/DOWN and lets the motor run to its own limits.
/// Intermediate positions send UP/DOWN, watch the measured position and send MY
/// shortly before the target, using the measured speed times stop_latency as lead.
/// Movement started by the physical remote is picked up from the sensor too.
///
/// Without a distance sensor it is a plain assumed-state cover: open and close
/// only, each running the motor to its own limit.
class SomfyAwning : public cover::Cover, public Component {
 public:
  void set_remote(SomfyRTSRemote *remote) { this->remote_ = remote; }
  void set_distance_sensor(sensor::Sensor *sensor) { this->sensor_ = sensor; }
  /// Optional copy of the distance for Home Assistant, usually throttled by filters.
  void set_reported_distance_sensor(sensor::Sensor *sensor) { this->reported_distance_ = sensor; }
  void set_stop_latency_sensor(sensor::Sensor *sensor) { this->stop_latency_sensor_ = sensor; }
  void set_commands(Command open_cmd, Command close_cmd) {
    this->open_cmd_ = open_cmd;
    this->close_cmd_ = close_cmd;
  }
  void set_default_calibration(float closed_cm, float open_cm) {
    this->cal_.closed_cm = closed_cm;
    this->cal_.open_cm = open_cm;
  }
  void set_stop_latency(float seconds) { this->cal_.stop_latency_s = seconds; }
  void set_position_tolerance(float tolerance) { this->tolerance_ = tolerance; }
  void set_min_travel(float min_travel) { this->min_travel_ = min_travel; }
  void set_max_travel_time(uint32_t ms) { this->max_travel_ms_ = ms; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }
  cover::CoverTraits get_traits() override;

  /// Closes, opens and closes the awning, recording both end distances.
  void start_calibration();
  /// Manual calibration: store the current distance as the closed/open end.
  void set_closed_here();
  void set_open_here();
  bool is_calibrated() const;
  /// Sends a raw RTS command, e.g. MY to go to the motor's favourite position.
  void send_command(Command command, uint8_t repeat) { this->remote_->send_command(command, repeat); }
  float get_stop_latency() const { return this->cal_.stop_latency_s; }

 protected:
  enum class Mode : uint8_t {
    IDLE,         // not commanded by us; the remote may still move it
    TO_END,       // UP/DOWN sent, the motor stops at its own limit
    SEEKING,      // moving towards target_, MY will be sent before reaching it
    STOPPING,     // MY sent, waiting for the awning to stand still
    CAL_CLOSE,    // calibration, phase 1
    CAL_OPEN,     // calibration, phase 2
    CAL_RECLOSE,  // calibration, phase 3
  };
  struct Sample {
    uint32_t t;
    float cm;
  };

  void control(const cover::CoverCall &call) override;
  void control_without_lidar_(const cover::CoverCall &call);
  void on_distance_(float cm);
  void move_(int8_t dir, Mode mode);
  void send_stop_(const char *reason);
  void set_mode_(Mode mode);
  void finish_calibration_phase_();
  void save_calibration_();
  void publish_(bool force);

  float velocity_cm_s_() const;
  float position_for_(float cm) const;
  bool is_still_for_(uint32_t ms) const;

  SomfyRTSRemote *remote_{nullptr};
  sensor::Sensor *sensor_{nullptr};
  sensor::Sensor *reported_distance_{nullptr};
  sensor::Sensor *stop_latency_sensor_{nullptr};
  Command open_cmd_{Command::DOWN};
  Command close_cmd_{Command::UP};
  float tolerance_{0.02f};
  float min_travel_{0.03f};
  uint32_t max_travel_ms_{120000};

  Calibration cal_{NAN, NAN, 0.6f, 0};
  ESPPreferenceObject pref_;

  Mode mode_{Mode::IDLE};
  int8_t dir_{0};  // +1 opening, -1 closing, while commanded by us
  float target_{NAN};
  uint32_t mode_start_ms_{0};
  uint32_t move_cmd_ms_{0};
  float stop_speed_{NAN};  // position units per second when MY was sent

  std::array<Sample, 16> samples_{};
  uint8_t sample_head_{0};
  uint8_t sample_count_{0};
  float last_cm_{NAN};
  uint32_t last_valid_ms_{0};
  uint32_t still_since_ms_{0};

  float cal_closed_cm_{NAN};
  float cal_open_cm_{NAN};

  float published_position_{NAN};
  cover::CoverOperation published_operation_{cover::COVER_OPERATION_IDLE};
  uint32_t last_publish_ms_{0};
};

enum class ButtonAction : uint8_t { CALIBRATE, SET_CLOSED_HERE, SET_OPEN_HERE, PROG };

class AwningButton : public button::Button, public Parented<SomfyAwning> {
 public:
  void set_action(ButtonAction action) { this->action_ = action; }

 protected:
  void press_action() override;

  ButtonAction action_{ButtonAction::CALIBRATE};
};

template<typename... Ts> class SendCommandAction : public Action<Ts...>, public Parented<SomfyAwning> {
 public:
  void set_command(Command command) { this->command_ = command; }
  void set_repeat(uint8_t repeat) { this->repeat_ = repeat; }

  void play(const Ts &...x) override { this->parent_->send_command(this->command_, this->repeat_); }

 protected:
  Command command_{Command::MY};
  uint8_t repeat_{2};
};

}  // namespace esphome::somfy_lidar
