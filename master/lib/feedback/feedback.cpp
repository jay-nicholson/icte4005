#include "feedback.h"

const char* decodeResultName(DecodeResult result) {
  switch (result) {
    case DECODE_OK: return "OK";
    case DECODE_BAD_LENGTH: return "BAD_LENGTH";
    case DECODE_BAD_STATE: return "BAD_STATE";
    case DECODE_BAD_FAULT: return "BAD_FAULT";
    case DECODE_FAULT_MISMATCH: return "FAULT_MISMATCH";
    case DECODE_BAD_MASK: return "BAD_MASK";
    case DECODE_COUNTER_DECREASED: return "COUNTER_DECREASED";
    default: return "?";
  }
}

static DecodeResult checkFaultPair(uint8_t state, uint8_t fault, uint8_t faultState) {
  if (fault > FAULT_LOCAL) return DECODE_BAD_FAULT;
  bool isFault = (state == faultState);
  if (isFault != (fault != FAULT_NONE)) return DECODE_FAULT_MISMATCH;
  return DECODE_OK;
}

DecodeResult decodeConveyor(const uint8_t* bytes, uint8_t length, SlaveView& view) {
  if (length != CONVEYOR_REPLY_LENGTH) return DECODE_BAD_LENGTH;
  uint8_t state = bytes[OFFSET_STATE];
  uint8_t fault = bytes[OFFSET_FAULT];
  if (state > CONVEYOR_FAULT) return DECODE_BAD_STATE;
  DecodeResult result = checkFaultPair(state, fault, CONVEYOR_FAULT);
  if (result != DECODE_OK) return result;
  view.state = state;
  view.fault = fault;
  view.uptime = readUint32LittleEndian(bytes + OFFSET_UPTIME);
  view.steps = readUint32LittleEndian(bytes + OFFSET_CONVEYOR_STEPS);
  return DECODE_OK;
}

DecodeResult decodeSensing(const uint8_t* bytes, uint8_t length, SlaveView& view) {
  if (length != SENSING_REPLY_LENGTH) return DECODE_BAD_LENGTH;
  uint8_t state = bytes[OFFSET_STATE];
  uint8_t fault = bytes[OFFSET_FAULT];
  uint8_t live = bytes[OFFSET_SENSING_LIVE_MASK];
  if (state > SENSING_FAULT) return DECODE_BAD_STATE;
  DecodeResult result = checkFaultPair(state, fault, SENSING_FAULT);
  if (result != DECODE_OK) return result;
  if (live & ~LIVE_MASK_ALL) return DECODE_BAD_MASK;
  view.state = state;
  view.fault = fault;
  view.uptime = readUint32LittleEndian(bytes + OFFSET_UPTIME);
  view.live = live;
  view.reed1Edges = readUint16LittleEndian(bytes + OFFSET_SENSING_REED1_EDGES);
  view.reed2Edges = readUint16LittleEndian(bytes + OFFSET_SENSING_REED2_EDGES);
  return DECODE_OK;
}

void SlaveView::begin(SlaveKind slaveKind) {
  kind = slaveKind;
  everValid = false;
  fresh = false;
  repliedThisPoll = false;
  lastValidReplyMs = 0;
  ageMs = 0;
  invalid = false;
  lastError = DECODE_OK;
  uptimeStalled = false;
  rebootPending = false;
  state = 0;
  fault = 0;
  uptime = 0;
  steps = 0;
  live = 0;
  reed1Edges = 0;
  reed2Edges = 0;
  lastUptimeChangeMs = 0;
}

static bool decreased32(uint32_t previous, uint32_t current) { return (current - previous) >= 0x80000000UL; }
static bool decreased16(uint16_t previous, uint16_t current) { return (uint16_t)(current - previous) >= 0x8000; }

DecodeResult SlaveView::noteReply(uint32_t now, const uint8_t* bytes, uint8_t length) {
  if (length == 0) {
    noteNoReply(now);
    return DECODE_BAD_LENGTH;
  }
  SlaveView previous = *this;
  DecodeResult result =
      (kind == SLAVE_CONVEYOR) ? decodeConveyor(bytes, length, *this) : decodeSensing(bytes, length, *this);
  bool rebooted = false;
  if (result == DECODE_OK && previous.everValid) {
    rebooted = decreased32(previous.uptime, uptime);
    bool counterDecreased = (kind == SLAVE_CONVEYOR)
        ? decreased32(previous.steps, steps)
        : (decreased16(previous.reed1Edges, reed1Edges) || decreased16(previous.reed2Edges, reed2Edges));
    if (counterDecreased && !rebooted) result = DECODE_COUNTER_DECREASED;
  }
  if (result != DECODE_OK) {
    *this = previous;
    invalid = true;
    lastError = result;
    repliedThisPoll = false;
    ageMs = now - lastValidReplyMs;
    fresh = everValid && (ageMs < FEEDBACK_STALE_MS);
    return result;
  }
  invalid = false;
  lastError = DECODE_OK;
  if (previous.everValid) {
    uint32_t uptimeDelta = uptime - previous.uptime;
    if (uptimeDelta == 0) {
      if (now - lastUptimeChangeMs >= UPTIME_STALL_MS) uptimeStalled = true;
    } else {
      lastUptimeChangeMs = now;
      uptimeStalled = false;
      if (rebooted) rebootPending = true;
    }
  } else {
    lastUptimeChangeMs = now;
  }
  everValid = true;
  fresh = true;
  repliedThisPoll = true;
  lastValidReplyMs = now;
  ageMs = 0;
  return DECODE_OK;
}

void SlaveView::noteNoReply(uint32_t now) {
  invalid = false;
  repliedThisPoll = false;
  ageMs = now - lastValidReplyMs;
  fresh = everValid && (ageMs < FEEDBACK_STALE_MS);
}

bool SlaveView::inFault() const {
  uint8_t faultState = (kind == SLAVE_CONVEYOR) ? (uint8_t)CONVEYOR_FAULT : (uint8_t)SENSING_FAULT;
  return fresh && !invalid && state == faultState;
}
