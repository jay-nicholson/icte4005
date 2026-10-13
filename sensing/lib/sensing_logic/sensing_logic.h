#pragma once
#include <stdint.h>
#include "contract.h"

class SensingLogic {
 public:
  void begin(uint16_t debounceMs);
  void sample(uint32_t now, bool reed1Closed, bool reed2Closed);
  void serviceWatchdog(bool contactLost);
  // Latches FAULT with reason LOCAL (a sensor fault this board detected itself). Keeps the first
  // fault reason if the board is already in FAULT, as the contract requires.
  void raiseLocalFault();
  void encode(uint8_t* bytes, uint32_t uptime) const;

  uint8_t state() const { return state_; }
  uint8_t fault() const { return fault_; }
  uint8_t live() const { return live_; }
  uint16_t reed1Edges() const { return reed1Edges_; }
  uint16_t reed2Edges() const { return reed2Edges_; }
  bool greenOn() const { return live_ & LIVE_REED1; }
  bool amberOn() const { return live_ & LIVE_REED2; }
  bool redOn() const { return state_ == SENSING_FAULT; }

 private:
  struct Debounce {
    bool stable;
    bool candidate;
    uint32_t candidateSinceMs;
  };
  bool debounce(Debounce& input, uint32_t now, bool raw);

  uint16_t debounceMs_;
  Debounce reed1_;
  Debounce reed2_;
  uint8_t state_;
  uint8_t fault_;
  uint8_t live_;
  uint16_t reed1Edges_;
  uint16_t reed2Edges_;
};
