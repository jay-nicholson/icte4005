#pragma once
#include <stdint.h>

// Turns one noisy analog sensor into a clean "event present" bit.
//
//   raw ADC -> auto-zero baseline -> low-pass filter -> |filtered - baseline| -> hysteresis -> active()
//
// Why each stage exists (all of them came from the reed-switch failures):
//  - Auto-zero:   the sensor's resting level differs per part, per supply and per mounting, so we
//                 measure it at boot instead of trusting a datasheet number.
//  - Filter:      ADC noise and motor-coil pickup must not look like an event.
//  - |deviation|: a linear Hall sensor swings up or down with magnet polarity. Using the absolute
//                 deviation makes puck orientation irrelevant.
//  - Hysteresis:  separate on/off thresholds stop a marginal signal chattering.
//  - Health:      a disconnected or shorted sensor is detected and latched, which a normally-open
//                 reed cannot do (it just looks "open").
//
// Pure C++ (no Arduino calls) so the same code runs on the Uno and in the host tests.
class AnalogChannel {
 public:
  struct Config {
    uint16_t zeroSamples;    // readings averaged at boot to set the baseline (belt must be clear)
    uint8_t filterShift;     // low-pass strength: each sample moves the filter 1/2^shift of the way
    uint16_t onDelta;        // ADC counts of deviation that turn the channel on
    uint16_t offDelta;       // ADC counts of deviation below which it turns off (< onDelta)
    uint16_t baselineMin;    // plausible resting range; outside it the sensor is faulted
    uint16_t baselineMax;
    uint16_t maxZeroSpread;  // max (highest - lowest) reading allowed while zeroing
  };

  void begin(const Config& config);
  void sample(uint16_t raw);

  bool ready() const { return ready_; }
  bool active() const { return active_; }
  bool fault() const { return fault_; }
  uint16_t baseline() const { return baseline_; }
  uint16_t filtered() const { return (uint16_t)(filtered16_ / 16); }
  uint16_t deviation() const;

 private:
  static const uint16_t RAIL_LOW = 3;
  static const uint16_t RAIL_HIGH = 1020;
  static const uint16_t RAIL_SAMPLES = 50;

  Config config_;
  uint16_t zeroCount_;
  uint32_t zeroSum_;
  uint16_t zeroMin_;
  uint16_t zeroMax_;
  uint16_t baseline_;
  int32_t filtered16_;  // filtered reading x16, to keep fractional resolution without floats
  uint16_t railCount_;
  bool ready_;
  bool active_;
  bool fault_;
};

// Sensor fusion for the stop-point channel: a magnetic reading only counts as a puck if the optical
// channel agrees a puck is physically there. Rejects stray fields (stepper coils, a dropped magnet)
// that the Hall sensor alone cannot tell apart from a puck. With the gate disabled it is a no-op.
inline bool fusedClosed(bool magnetic, bool presence, bool gateEnabled) {
  return magnetic && (!gateEnabled || presence);
}
