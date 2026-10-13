#pragma once
#include <stdint.h>
#include "contract.h"

enum DecodeResult {
  DECODE_OK = 0,
  DECODE_BAD_LENGTH,
  DECODE_BAD_STATE,
  DECODE_BAD_FAULT,
  DECODE_FAULT_MISMATCH,
  DECODE_BAD_MASK,
  DECODE_COUNTER_DECREASED
};

const char* decodeResultName(DecodeResult result);

enum SlaveKind { SLAVE_CONVEYOR, SLAVE_SENSING };

struct SlaveView {
  SlaveKind kind;

  bool everValid;
  bool fresh;
  bool repliedThisPoll;
  uint32_t lastValidReplyMs;
  uint32_t ageMs;

  bool invalid;
  DecodeResult lastError;
  bool uptimeStalled;
  bool rebootPending;

  uint8_t state;
  uint8_t fault;
  uint32_t uptime;
  uint32_t steps;
  uint8_t live;
  uint16_t channel1Edges;
  uint16_t channel2Edges;
  uint16_t channel1Level;
  uint16_t channel2Level;

  uint32_t lastUptimeChangeMs;

  void begin(SlaveKind slaveKind);
  DecodeResult noteReply(uint32_t now, const uint8_t* bytes, uint8_t length);
  void noteNoReply(uint32_t now);
  bool inFault() const;
};

DecodeResult decodeConveyor(const uint8_t* bytes, uint8_t length, SlaveView& view);
DecodeResult decodeSensing(const uint8_t* bytes, uint8_t length, SlaveView& view);
