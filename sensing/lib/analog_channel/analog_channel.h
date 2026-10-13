#pragma once
#include <stdint.h>

// raw ADC -> boot baseline -> low-pass filter -> |filtered - baseline| -> hysteresis -> active()
class AnalogChannel {
 public:
  struct Config {
    uint16_t zeroSamples;
    uint8_t filterShift;
    uint16_t onDelta;
    uint16_t offDelta;
    uint16_t baselineMin;
    uint16_t baselineMax;
    uint16_t maxZeroSpread;
  };

  void begin(const Config& config);
  void sample(uint16_t raw);

  bool ready() const { return ready_; }
  bool active() const { return active_; }
  bool fault() const { return fault_; }
  uint16_t baseline() const { return baseline_; }
  uint16_t filtered() const { return (uint16_t)(filteredTimes16_ / 16); }
  uint16_t deviation() const;

 private:
  static const uint16_t RAIL_LOW_COUNTS = 3;
  static const uint16_t RAIL_HIGH_COUNTS = 1020;
  static const uint16_t RAIL_SAMPLES_TO_FAULT = 50;

  Config config_;
  uint16_t zeroCount_;
  uint32_t zeroSum_;
  uint16_t zeroMin_;
  uint16_t zeroMax_;
  uint16_t baseline_;
  int32_t filteredTimes16_;
  uint16_t railCount_;
  bool ready_;
  bool active_;
  bool fault_;
};

inline bool fusedClosed(bool magnetic, bool presence, bool gateEnabled) {
  return magnetic && (!gateEnabled || presence);
}
