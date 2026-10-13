#pragma once
#include <stdint.h>
#include "contract.h"

class SensingLogic {
 public:
  void begin(uint16_t debounceMs);
  void sample(uint32_t now, bool channel1Closed, bool channel2Closed);
  void serviceWatchdog(bool contactLost);
  void setLevels(uint16_t channel1Level, uint16_t channel2Level);
  void raiseLocalFault();
  void encode(uint8_t* bytes, uint32_t uptime) const;

  uint8_t state() const { return state_; }
  uint8_t fault() const { return fault_; }
  uint8_t live() const { return live_; }
  uint16_t channel1Edges() const { return channel1Edges_; }
  uint16_t channel2Edges() const { return channel2Edges_; }

 private:
  struct Debounce {
    bool stable;
    bool candidate;
    uint32_t candidateSinceMs;
  };
  bool debounce(Debounce& input, uint32_t now, bool raw);

  uint16_t debounceMs_;
  Debounce channel1_;
  Debounce channel2_;
  uint8_t state_;
  uint8_t fault_;
  uint8_t live_;
  uint16_t channel1Edges_;
  uint16_t channel2Edges_;
  uint16_t channel1Level_;
  uint16_t channel2Level_;
};
