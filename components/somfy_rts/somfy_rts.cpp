#include "somfy_rts.h"

#include "esphome/core/log.h"

namespace esphome::somfy_rts {

static const char *const TAG = "somfy_rts";

// Frame layout and timings are ported from ESPSomfy RTS by rstrouse
// (https://github.com/rstrouse/ESPSomfy-RTS, Somfy.cpp), where they were
// measured from real Somfy remotes. ESPSomfy RTS is released into the public domain.
static const uint32_t SYMBOL = 640;  // half a bit, in microseconds
static const uint32_t WAKEUP_HIGH = 10920;
static const uint32_t WAKEUP_LOW = 7357;
static const uint32_t SOFTWARE_SYNC = 4850;
static const uint32_t INTER_FRAME_GAP = 27434;
static const uint8_t HW_SYNC_FIRST = 2;
static const uint8_t HW_SYNC_REPEAT = 7;
static const uint8_t FRAME_BITS = 56;

const char *command_to_string(Command command) {
  switch (command) {
    case Command::MY:
      return "MY";
    case Command::UP:
      return "UP";
    case Command::DOWN:
      return "DOWN";
    case Command::PROG:
      return "PROG";
    default:
      return "UNKNOWN";
  }
}

/// Appends a level to the raw timings (marks positive, spaces negative), merging it into
/// the previous entry when the level does not change (e.g. a 0 followed by a 1).
static void append_level(remote_base::RawTimings &data, bool high, uint32_t usec) {
  int32_t value = high ? static_cast<int32_t>(usec) : -static_cast<int32_t>(usec);
  if (!data.empty() && (data.back() > 0) == high) {
    data.back() += value;
  } else {
    data.push_back(value);
  }
}

void SomfyRTSRemote::setup() {
  this->pref_ = global_preferences->make_preference<uint16_t>(0x50AF1E00 ^ this->address_, true);
  uint16_t stored;
  if (this->pref_.load(&stored) && stored != 0) {
    this->rolling_code_ = stored;
  }
}

void SomfyRTSRemote::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Somfy RTS remote:\n"
                "  Address: 0x%06" PRIX32 "\n"
                "  Next rolling code: %u\n"
                "  Repeat: %u",
                this->address_, this->rolling_code_, this->repeat_);
}

void SomfyRTSRemote::encode_frame_(Command command, uint16_t rolling_code, uint8_t *frame) const {
  frame[0] = 0xA0 | (rolling_code & 0x0F);  // encryption key, follows the rolling code
  frame[1] = static_cast<uint8_t>(command) << 4;
  frame[2] = rolling_code >> 8;
  frame[3] = rolling_code;
  frame[4] = this->address_ >> 16;
  frame[5] = this->address_ >> 8;
  frame[6] = this->address_;

  uint8_t checksum = 0;
  for (uint8_t i = 0; i < 7; i++)
    checksum ^= frame[i] ^ (frame[i] >> 4);
  frame[1] |= checksum & 0x0F;

  // Obfuscation
  for (uint8_t i = 1; i < 7; i++)
    frame[i] ^= frame[i - 1];
}

void SomfyRTSRemote::send_command(Command command, uint8_t repeat) {
  uint16_t code = this->rolling_code_;
  // Persist the next code before transmitting: if power is lost mid-send the
  // motor may have seen this code, so it must never be reused.
  this->rolling_code_ = code == 0xFFFF ? 1 : code + 1;
  this->pref_.save(&this->rolling_code_);
  global_preferences->sync();

  uint8_t frame[7];
  this->encode_frame_(command, code, frame);
  ESP_LOGD(TAG, "Sending %s (address 0x%06" PRIX32 ", rolling code %u, repeat %u)", command_to_string(command),
           this->address_, code, repeat);

  remote_base::RawTimings data;
  data.reserve((repeat + 1) * (FRAME_BITS * 2 + 2 * HW_SYNC_REPEAT + 6));

  for (uint8_t f = 0; f <= repeat; f++) {
    uint8_t hw_sync = HW_SYNC_REPEAT;
    if (f == 0) {
      // Only the first frame carries the wake-up pulse.
      append_level(data, true, WAKEUP_HIGH);
      append_level(data, false, WAKEUP_LOW);
      hw_sync = HW_SYNC_FIRST;
    }
    for (uint8_t i = 0; i < hw_sync; i++) {
      append_level(data, true, 4 * SYMBOL);
      append_level(data, false, 4 * SYMBOL);
    }
    append_level(data, true, SOFTWARE_SYNC);
    append_level(data, false, SYMBOL);
    // Manchester, most significant bit first: 1 is a rising edge, 0 a falling one.
    for (uint8_t i = 0; i < FRAME_BITS; i++) {
      bool bit = (frame[i / 8] >> (7 - (i % 8))) & 1;
      append_level(data, !bit, SYMBOL);
      append_level(data, bit, SYMBOL);
    }
    append_level(data, false, INTER_FRAME_GAP);
  }

  auto call = this->transmitter_->transmit();
  call.get_data()->set_carrier_frequency(0);
  call.get_data()->set_data(data);
  call.perform();
}

}  // namespace esphome::somfy_rts
