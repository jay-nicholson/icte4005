#include "conveyor_logic.h"

void ConveyorLogic::begin() {
  state_ = CONVEYOR_STOPPED;
  fault_ = FAULT_NONE;
  steps_ = 0;
}

void ConveyorLogic::command(char command) {
  if (state_ == CONVEYOR_FAULT) return;
  switch (command) {
    case COMMAND_GO:
      if (state_ == CONVEYOR_STOPPED) state_ = CONVEYOR_FORWARD;
      return;
    case COMMAND_BACK:
      if (state_ == CONVEYOR_STOPPED) state_ = CONVEYOR_REVERSE;
      return;
    case COMMAND_STOP:
      state_ = CONVEYOR_STOPPED;
      return;
    default:
      return;
  }
}

void ConveyorLogic::serviceWatchdog(bool contactLost) {
  if (contactLost && state_ != CONVEYOR_FAULT) {
    state_ = CONVEYOR_FAULT;
    fault_ = FAULT_CONTACT_LOST;
  } else if (!contactLost && state_ == CONVEYOR_FAULT && fault_ == FAULT_CONTACT_LOST) {
    state_ = CONVEYOR_STOPPED;
    fault_ = FAULT_NONE;
  }
}

void ConveyorLogic::countStep() { steps_++; }

MotorRequest ConveyorLogic::motor() const {
  if (state_ == CONVEYOR_FORWARD) return MOTOR_FORWARD;
  if (state_ == CONVEYOR_REVERSE) return MOTOR_REVERSE;
  return MOTOR_OFF;
}

void ConveyorLogic::encode(uint8_t* bytes, uint32_t uptime) const {
  bytes[OFFSET_STATE] = state_;
  bytes[OFFSET_FAULT] = fault_;
  writeUint32LittleEndian(bytes + OFFSET_UPTIME, uptime);
  writeUint32LittleEndian(bytes + OFFSET_CONVEYOR_STEPS, steps_);
}
