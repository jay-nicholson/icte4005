#include "analog_channel.h"

void AnalogChannel::begin(const Config& config) {
  config_ = config;
  zeroCount_ = 0;
  zeroSum_ = 0;
  zeroMin_ = 0xFFFF;
  zeroMax_ = 0;
  baseline_ = 0;
  filteredTimes16_ = 0;
  railCount_ = 0;
  ready_ = false;
  active_ = false;
  fault_ = false;
}

uint16_t AnalogChannel::deviation() const {
  int32_t delta = filteredTimes16_ - (int32_t)baseline_ * 16;
  if (delta < 0) delta = -delta;
  return (uint16_t)(delta / 16);
}

void AnalogChannel::sample(uint16_t raw) {
  if (raw <= RAIL_LOW_COUNTS || raw >= RAIL_HIGH_COUNTS) {
    if (railCount_ < RAIL_SAMPLES_TO_FAULT) railCount_++;
    if (railCount_ >= RAIL_SAMPLES_TO_FAULT) fault_ = true;
  } else {
    railCount_ = 0;
  }

  if (!ready_) {
    zeroSum_ += raw;
    if (raw < zeroMin_) zeroMin_ = raw;
    if (raw > zeroMax_) zeroMax_ = raw;
    zeroCount_++;
    filteredTimes16_ = (int32_t)raw * 16;
    if (zeroCount_ >= config_.zeroSamples) {
      baseline_ = (uint16_t)(zeroSum_ / zeroCount_);
      filteredTimes16_ = (int32_t)baseline_ * 16;
      ready_ = true;
      if (baseline_ < config_.baselineMin || baseline_ > config_.baselineMax ||
          (uint16_t)(zeroMax_ - zeroMin_) > config_.maxZeroSpread) {
        fault_ = true;
      }
    }
    return;
  }

  // Division rather than a shift so a negative difference behaves as intended.
  int32_t target = (int32_t)raw * 16;
  filteredTimes16_ += (target - filteredTimes16_) / (int32_t)(1 << config_.filterShift);

  if (fault_) {
    active_ = false;
    return;
  }

  uint16_t currentDeviation = deviation();
  if (!active_ && currentDeviation >= config_.onDelta) {
    active_ = true;
  } else if (active_ && currentDeviation <= config_.offDelta) {
    active_ = false;
  }
}
