#pragma once
#include <stdint.h>
#include "contract.h"

enum MotorRequest { MOTOR_OFF, MOTOR_FORWARD, MOTOR_REVERSE };

class ConveyorLogic {
 public:
  void begin();
  void command(char command);
  void serviceWatchdog(bool contactLost);
  void countStep();
  void sweepComplete();
  void encode(uint8_t* bytes, uint32_t uptime) const;

  uint8_t state() const { return state_; }
  uint8_t fault() const { return fault_; }
  uint32_t steps() const { return steps_; }
  MotorRequest motor() const;
  bool sweeping() const { return state_ == CONVEYOR_SWEEPING; }

 private:
  uint8_t state_;
  uint8_t fault_;
  uint32_t steps_;
};
