// Numbers from CONTRACT.md v3. Change only together with that file.
#pragma once
#include <stdint.h>

static const uint8_t CONTRACT_VERSION = 3;

static const uint8_t CONVEYOR_ADDRESS = 0x10;
static const uint8_t SENSING_ADDRESS = 0x11;

static const uint8_t CONVEYOR_REPLY_LENGTH = 10;
static const uint8_t SENSING_REPLY_LENGTH = 15;
static const uint8_t MAX_REPLY_LENGTH = 15;

static const char COMMAND_GO = 'g';
static const char COMMAND_BACK = 'b';
static const char COMMAND_STOP = 's';
static const char COMMAND_EJECT = 'e';

static const uint8_t OFFSET_STATE = 0;
static const uint8_t OFFSET_FAULT = 1;
static const uint8_t OFFSET_UPTIME = 2;
static const uint8_t OFFSET_CONVEYOR_STEPS = 6;
static const uint8_t OFFSET_SENSING_LIVE_MASK = 6;
static const uint8_t OFFSET_SENSING_CHANNEL1_EDGES = 7;
static const uint8_t OFFSET_SENSING_CHANNEL2_EDGES = 9;
static const uint8_t OFFSET_SENSING_CHANNEL1_LEVEL = 11;
static const uint8_t OFFSET_SENSING_CHANNEL2_LEVEL = 13;

enum ConveyorState {
  CONVEYOR_STOPPED = 0,
  CONVEYOR_FORWARD = 1,
  CONVEYOR_REVERSE = 2,
  CONVEYOR_FAULT = 3,
  CONVEYOR_SWEEPING = 4
};

enum SensingState {
  SENSING_OK = 0,
  SENSING_FAULT = 1
};

enum FaultCode {
  FAULT_NONE = 0,
  FAULT_CONTACT_LOST = 1,
  FAULT_LOCAL = 2
};

static const uint8_t LIVE_CHANNEL1 = 0x01;
static const uint8_t LIVE_CHANNEL2 = 0x02;
static const uint8_t LIVE_MASK_ALL = LIVE_CHANNEL1 | LIVE_CHANNEL2;

static const uint16_t POLL_INTERVAL_MS = 100;
static const uint16_t CONFIRM_TIMEOUT_MS = 300;
static const uint16_t FEEDBACK_STALE_MS = 300;
static const uint16_t UPTIME_STALL_MS = 300;
static const uint16_t SNAPSHOT_REFRESH_MS = 20;
static const uint16_t CONTACT_WATCHDOG_MS = 1000;
static const uint16_t CHANNEL_CLEAR_GRACE_MS = 300;
static const uint16_t SWEEP_TIMEOUT_MS = 6000;

inline const char* conveyorStateName(uint8_t state) {
  switch (state) {
    case CONVEYOR_STOPPED: return "STOPPED";
    case CONVEYOR_FORWARD: return "FORWARD";
    case CONVEYOR_REVERSE: return "REVERSE";
    case CONVEYOR_FAULT: return "FAULT";
    case CONVEYOR_SWEEPING: return "SWEEPING";
    default: return "INVALID";
  }
}

inline const char* sensingStateName(uint8_t state) {
  switch (state) {
    case SENSING_OK: return "OK";
    case SENSING_FAULT: return "FAULT";
    default: return "INVALID";
  }
}

inline const char* faultCodeName(uint8_t faultCode) {
  switch (faultCode) {
    case FAULT_NONE: return "NONE";
    case FAULT_CONTACT_LOST: return "CONTACT_LOST";
    case FAULT_LOCAL: return "LOCAL";
    default: return "INVALID";
  }
}

inline void writeUint32LittleEndian(uint8_t* bytes, uint32_t value) {
  bytes[0] = (uint8_t)value;
  bytes[1] = (uint8_t)(value >> 8);
  bytes[2] = (uint8_t)(value >> 16);
  bytes[3] = (uint8_t)(value >> 24);
}

inline void writeUint16LittleEndian(uint8_t* bytes, uint16_t value) {
  bytes[0] = (uint8_t)value;
  bytes[1] = (uint8_t)(value >> 8);
}

inline uint32_t readUint32LittleEndian(const uint8_t* bytes) {
  return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

inline uint16_t readUint16LittleEndian(const uint8_t* bytes) {
  return (uint16_t)(bytes[0] | (bytes[1] << 8));
}
