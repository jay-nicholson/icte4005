#include "sensing_logic.h"

void SensingLogic::begin(uint16_t debounceMs) {
  debounceMs_ = debounceMs;
  channel1_ = {false, false, 0};
  channel2_ = {false, false, 0};
  state_ = SENSING_OK;
  fault_ = FAULT_NONE;
  live_ = 0;
  channel1Edges_ = 0;
  channel2Edges_ = 0;
  channel1Level_ = 0;
  channel2Level_ = 0;
  localFault_ = false;
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

void SensingLogic::sample(uint32_t now, bool channel1Closed, bool channel2Closed) {
  if (debounce(channel1_, now, channel1Closed)) channel1Edges_++;
  if (debounce(channel2_, now, channel2Closed)) channel2Edges_++;
  live_ = (channel1_.stable ? LIVE_CHANNEL1 : 0) | (channel2_.stable ? LIVE_CHANNEL2 : 0);
}

void SensingLogic::serviceWatchdog(bool contactLost) {
  if (contactLost && state_ != SENSING_FAULT) {
    state_ = SENSING_FAULT;
    fault_ = FAULT_CONTACT_LOST;
  } else if (!contactLost && state_ == SENSING_FAULT && fault_ == FAULT_CONTACT_LOST) {
    if (localFault_) {
      fault_ = FAULT_LOCAL;
    } else {
      state_ = SENSING_OK;
      fault_ = FAULT_NONE;
    }
  }
}

void SensingLogic::raiseLocalFault() {
  localFault_ = true;
  if (state_ != SENSING_FAULT) {
    state_ = SENSING_FAULT;
    fault_ = FAULT_LOCAL;
  }
}

void SensingLogic::setLevels(uint16_t channel1Level, uint16_t channel2Level) {
  channel1Level_ = channel1Level;
  channel2Level_ = channel2Level;
}

void SensingLogic::encode(uint8_t* bytes, uint32_t uptime) const {
  bytes[OFFSET_STATE] = state_;
  bytes[OFFSET_FAULT] = fault_;
  writeUint32LittleEndian(bytes + OFFSET_UPTIME, uptime);
  bytes[OFFSET_SENSING_LIVE_MASK] = live_;
  writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL1_EDGES, channel1Edges_);
  writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL2_EDGES, channel2Edges_);
  writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL1_LEVEL, channel1Level_);
  writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL2_LEVEL, channel2Level_);
}
