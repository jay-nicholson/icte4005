#include "analog_channel.h"

void AnalogChannel::begin(const Config& config) {
  config_ = config;
  zeroCount_ = 0;
  zeroSum_ = 0;
  zeroMin_ = 0xFFFF;
  zeroMax_ = 0;
  baseline_ = 0;
  filtered16_ = 0;
  railCount_ = 0;
  ready_ = false;
  active_ = false;
  fault_ = false;
}

uint16_t AnalogChannel::deviation() const {
  int32_t delta = filtered16_ - (int32_t)baseline_ * 16;
  if (delta < 0) delta = -delta;
  return (uint16_t)(delta / 16);
}

void AnalogChannel::sample(uint16_t raw) {
  // Health: a reading pinned to either rail means a broken wire or a short. Latched.
  if (raw <= RAIL_LOW || raw >= RAIL_HIGH) {
    if (railCount_ < RAIL_SAMPLES) railCount_++;
    if (railCount_ >= RAIL_SAMPLES) fault_ = true;
  } else {
    railCount_ = 0;
  }

  if (!ready_) {
    zeroSum_ += raw;
    if (raw < zeroMin_) zeroMin_ = raw;
    if (raw > zeroMax_) zeroMax_ = raw;
    zeroCount_++;
    filtered16_ = (int32_t)raw * 16;
    if (zeroCount_ >= config_.zeroSamples) {
      baseline_ = (uint16_t)(zeroSum_ / zeroCount_);
      filtered16_ = (int32_t)baseline_ * 16;
      ready_ = true;
      // A resting level outside the plausible band, or a jumpy one, means the sensor is not what we
      // think it is (unplugged, floating, wrong pin) or a puck was sitting on it at power-up.
      if (baseline_ < config_.baselineMin || baseline_ > config_.baselineMax ||
          (uint16_t)(zeroMax_ - zeroMin_) > config_.maxZeroSpread) {
        fault_ = true;
      }
    }
    return;
  }

  // Low-pass: move 1/2^shift of the way toward the new reading. Division (not >>) keeps the
  // behaviour defined for negative differences.
  int32_t target = (int32_t)raw * 16;
  filtered16_ += (target - filtered16_) / (int32_t)(1 << config_.filterShift);

  if (fault_) {
    active_ = false;
    return;
  }

  uint16_t dev = deviation();
  if (!active_ && dev >= config_.onDelta) {
    active_ = true;
  } else if (active_ && dev <= config_.offDelta) {
    active_ = false;
  }
}
