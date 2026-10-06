// Closed-loop simulation of somfy_awning against a model of a Somfy RTS motor.
//
// The motor reacts to the RTS frames the component actually transmits (decoded
// back from the raw timings), has start/stop delays, and goes to its favourite
// position if it receives MY while standing still, which the component must never cause.
// Run with ./run_sim.sh.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <thread>
#include "esphome/core/hal.h"
#include "esphome/components/somfy_lidar/somfy_awning.h"

using namespace esphome;

static const float CLOSED_CM = 250.0f, OPEN_CM = 30.0f;  // sensor gets closer as it opens
static const float SPEED = 0.10f;                        // fraction of travel per second
static const float MY_POS = 0.5f;                        // motor's favourite position

struct Motor {
  float ext = 0.0f;  // 0 closed .. 1 open
  int dir = 0;
  int pending = 0;  // command code waiting for the start delay
  uint32_t pending_at = 0;
  float goto_target = NAN;
  int my_while_still = 0;
  int commands = 0;
  void command(int cmd, uint32_t now) {
    commands++;
    const char *names[] = {"?", "MY", "UP", "?", "DOWN", "?", "?", "?", "PROG"};
    printf("  [motor t=%5.1fs] %-4s received at ext=%.3f (moving=%d)\n", now / 1000.0, names[cmd], ext, dir != 0);
    pending = cmd;
    pending_at = now + (cmd == 1 ? 150 : 300);
  }
  void step(uint32_t now, float dt) {
    if (pending && now >= pending_at) {
      int cmd = pending;
      pending = 0;
      if (cmd == 4) { dir = +1; goto_target = NAN; }       // DOWN extends
      else if (cmd == 2) { dir = -1; goto_target = NAN; }  // UP retracts
      else if (cmd == 1) {
        if (dir != 0) { dir = 0; goto_target = NAN; }
        else { my_while_still++; goto_target = MY_POS; dir = ext < MY_POS ? 1 : -1; }
      }
    }
    if (dir) {
      ext += dir * SPEED * dt;
      if (!std::isnan(goto_target) && (dir > 0 ? ext >= goto_target : ext <= goto_target)) { ext = goto_target; dir = 0; }
      if (ext >= 1) { ext = 1; dir = 0; }
      if (ext <= 0) { ext = 0; dir = 0; }
    }
  }
  float distance() const { return CLOSED_CM + ext * (OPEN_CM - CLOSED_CM); }
};
static Motor motor;

// Decodes the RTS command nibble back out of the raw timings.
static int decode_command(const remote_base::RawTimings &t) {
  size_t i = 0;
  while (i < t.size() && t[i] != 4850) i++;
  i++;
  std::vector<int> levels;
  for (; i < t.size() && t[i] > -20000; i++)
    for (int q = 0; q < std::abs(t[i]) / 640; q++) levels.push_back(t[i] > 0);
  levels.erase(levels.begin());
  if (levels.size() % 2) levels.push_back(0);
  uint8_t fr[7] = {};
  for (int b = 0; b < 56; b++) {
    int bit = levels[2 * b] == 0 && levels[2 * b + 1] == 1;
    fr[b / 8] |= bit << (7 - b % 8);
  }
  return (fr[1] ^ fr[0]) >> 4;
}

class SimTx : public remote_base::RemoteTransmitterBase {
 public:
  SimTx() : RemoteTransmitterBase(nullptr) {}
 protected:
  void send_internal(uint32_t, uint32_t) override { motor.command(decode_command(this->temp_.get_data()), millis()); }
};

static SimTx tx;
static somfy_lidar::SomfyRTSRemote remote;
static sensor::Sensor dist;
static somfy_lidar::SomfyAwning awning;
static std::mt19937 rng(42);
static std::normal_distribution<float> noise(0.0f, 0.7f);
static bool sensor_ok = true;
static uint32_t t0;

static void run(float seconds) {
  uint32_t end = millis() + seconds * 1000;
  uint32_t last = millis(), last_pub = 0;
  while (millis() < end) {
    uint32_t now = millis();
    motor.step(now, (now - last) / 1000.0f);
    last = now;
    if (now - last_pub >= 100) {
      last_pub = now;
      dist.publish_state(sensor_ok ? motor.distance() + noise(rng) : NAN);
    }
    awning.loop();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

static int failures = 0;
static void check(bool ok, const char *what) {
  printf("%s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) failures++;
}

static void go(float target) {
  printf("-- set_position(%.2f) from ext=%.3f\n", target, motor.ext);
  awning.make_call().set_position(target).perform();
}

void setup() {
  t0 = millis();
  remote.set_transmitter(&tx);
  remote.set_address(0x123456 + (rand() & 0xFFF));
  remote.set_repeat(1);
  remote.setup();
  awning.set_remote(&remote);
  awning.set_distance_sensor(&dist);
  awning.set_commands(somfy_lidar::Command::DOWN, somfy_lidar::Command::UP);
  awning.set_stop_latency(0.6f);
  awning.set_position_tolerance(0.02f);
  awning.set_min_travel(0.03f);
  awning.set_max_travel_time(30000);
  awning.setup();

  motor.ext = 0.3f;  // starts part-way open
  run(1.5);

  printf("== Calibration\n");
  awning.start_calibration();
  run(50);
  check(awning.is_calibrated(), "calibrated");
  check(motor.ext == 0.0f, "calibration ends closed");
  check(awning.current_operation == cover::COVER_OPERATION_IDLE, "idle after calibration");

  printf("== Positioning (stop latency learns)\n");
  float targets[] = {0.5f, 0.2f, 0.8f, 0.35f, 0.65f, 0.4f};
  for (float target : targets) {
    go(target);
    run(12);
    printf("   ended at ext=%.3f, cover position %.3f, error %+.1f%%, latency now %.2fs\n", motor.ext,
           awning.position, (motor.ext - target) * 100, awning.get_stop_latency());
  }
  check(std::fabs(motor.ext - 0.4f) <= 0.02f, "last positioning move within 2%");

  printf("== Stop while still must not send MY\n");
  int before = motor.commands;
  awning.make_call().set_command_stop().perform();
  run(1);
  check(motor.commands == before, "no command sent");

  printf("== Physical remote stops a positioning move\n");
  motor.my_while_still = 0;
  go(0.95f);
  run(2.0);
  motor.command(1, millis());  // someone presses MY on the real remote
  run(8);
  check(motor.my_while_still == 0, "never sent MY to a still motor");
  check(awning.current_operation == cover::COVER_OPERATION_IDLE, "idle afterwards");

  printf("== Physical remote moves it; position tracked\n");
  motor.command(2, millis());  // UP on the real remote
  run(2);
  check(awning.current_operation == cover::COVER_OPERATION_CLOSING, "closing detected from the sensor");
  run(10);
  check(awning.position == 0.0f && awning.current_operation == cover::COVER_OPERATION_IDLE, "closed and idle");

  printf("== Fully open goes to the limit\n");
  go(1.0f);
  run(13);
  check(motor.ext == 1.0f && awning.position == 1.0f, "fully open");

  printf("== Sensor dropout during a positioning move stops the awning\n");
  motor.my_while_still = 0;
  go(0.3f);
  run(2);
  sensor_ok = false;
  run(3);
  check(motor.dir == 0, "motor stopped");
  sensor_ok = true;
  run(2);
  check(motor.my_while_still == 0, "never sent MY to a still motor");

  printf("\n%d failure(s)\n", failures);
  fflush(stdout);
  exit(failures ? 1 : 0);
}
