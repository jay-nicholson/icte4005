#include "sensing_logic.h"

void SensingLogic::begin(uint16_t debounceMs) {
  debounceMs_ = debounceMs;
  reed1_ = {false, false, 0};
  reed2_ = {false, false, 0};
  state_ = SENSING_OK;
  fault_ = FAULT_NONE;
  live_ = 0;
  reed1Edges_ = 0;
  reed2Edges_ = 0;
}

// Returns true on a closing edge of the debounced level.
bool SensingLogic::debounce(Debounce& input, uint32_t now, bool raw) {
  if (raw != input.candidate) {
    input.candidate = raw;
    input.candidateSinceMs = now;
    return false;
  }
  if (input.candidate == input.stable) return false;
  if (now - input.candidateSinceMs < debounceMs_) return false;
  input.stable = input.candidate;
  return input.stable;
}

void SensingLogic::sample(uint32_t now, bool reed1Closed, bool reed2Closed) {
  if (debounce(reed1_, now, reed1Closed)) reed1Edges_++;
  if (debounce(reed2_, now, reed2Closed)) reed2Edges_++;
  live_ = (reed1_.stable ? LIVE_REED1 : 0) | (reed2_.stable ? LIVE_REED2 : 0);
}

void SensingLogic::serviceWatchdog(bool contactLost) {
  if (contactLost && state_ != SENSING_FAULT) {
    state_ = SENSING_FAULT;
    fault_ = FAULT_CONTACT_LOST;
  }
}

void SensingLogic::raiseLocalFault() {
  if (state_ != SENSING_FAULT) {
    state_ = SENSING_FAULT;
    fault_ = FAULT_LOCAL;
  }
}

void SensingLogic::encode(uint8_t* bytes, uint32_t uptime) const {
  bytes[OFFSET_STATE] = state_;
  bytes[OFFSET_FAULT] = fault_;
  writeUint32LittleEndian(bytes + OFFSET_UPTIME, uptime);
  bytes[OFFSET_SENSING_LIVE_MASK] = live_;
  writeUint16LittleEndian(bytes + OFFSET_SENSING_REED1_EDGES, reed1Edges_);
  writeUint16LittleEndian(bytes + OFFSET_SENSING_REED2_EDGES, reed2Edges_);
}
