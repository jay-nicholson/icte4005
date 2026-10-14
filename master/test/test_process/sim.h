// Scripted slaves: each test sets the reply bytes. Nothing here models slave behaviour.
#pragma once
#include "process.h"

struct Sim {
  uint32_t now;
  SlaveView conveyor;
  SlaveView sensing;
  Process process;
  ProcessOutput output;

  uint8_t conveyorState;
  uint8_t conveyorFault;
  uint32_t conveyorSteps;
  uint32_t conveyorUptime;
  bool conveyorReplies;
  uint32_t stepsPerPoll;

  uint8_t sensingState;
  uint8_t sensingFault;
  uint8_t sensingLive;
  uint16_t channel1Edges;
  uint16_t channel2Edges;
  uint32_t sensingUptime;
  bool sensingReplies;

  void begin(uint32_t channelSpacing = 1000, uint32_t paddleSteps = 300) {
    now = 1000;
    conveyor.begin(SLAVE_CONVEYOR);
    sensing.begin(SLAVE_SENSING);
    Calibration calibration = {channelSpacing, paddleSteps};
    process.begin(calibration);
    conveyorState = CONVEYOR_STOPPED; conveyorFault = 0; conveyorSteps = 0; conveyorUptime = 500;
    conveyorReplies = true; stepsPerPoll = 50;
    sensingState = SENSING_OK; sensingFault = 0; sensingLive = 0; channel1Edges = 0; channel2Edges = 0;
    sensingUptime = 700; sensingReplies = true;
  }

  void step(OperatorKind kind = OPERATOR_NONE) {
    OperatorCommand operatorCommand = {kind, 0, 0};
    stepWith(operatorCommand);
  }

  void stepWith(const OperatorCommand& operatorCommand) {
    now += POLL_INTERVAL_MS;
    conveyorUptime += POLL_INTERVAL_MS;
    sensingUptime += POLL_INTERVAL_MS;
    if (conveyorState == CONVEYOR_FORWARD || conveyorState == CONVEYOR_REVERSE) conveyorSteps += stepsPerPoll;
    uint8_t bytes[MAX_REPLY_LENGTH];
    if (sensingReplies) {
      bytes[OFFSET_STATE] = sensingState;
      bytes[OFFSET_FAULT] = sensingFault;
      writeUint32LittleEndian(bytes + OFFSET_UPTIME, sensingUptime);
      bytes[OFFSET_SENSING_LIVE_MASK] = sensingLive;
      writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL1_EDGES, channel1Edges);
      writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL2_EDGES, channel2Edges);
      writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL1_LEVEL, 0);
      writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL2_LEVEL, 0);
      sensing.noteReply(now, bytes, SENSING_REPLY_LENGTH);
    } else {
      sensing.noteNoReply(now);
    }
    if (conveyorReplies) {
      bytes[OFFSET_STATE] = conveyorState;
      bytes[OFFSET_FAULT] = conveyorFault;
      writeUint32LittleEndian(bytes + OFFSET_UPTIME, conveyorUptime);
      writeUint32LittleEndian(bytes + OFFSET_CONVEYOR_STEPS, conveyorSteps);
      conveyor.noteReply(now, bytes, CONVEYOR_REPLY_LENGTH);
    } else {
      conveyor.noteNoReply(now);
    }
    process.tick(now, conveyor, sensing, operatorCommand, output);
  }

  bool sent(uint8_t address, char command) const {
    for (uint8_t i = 0; i < output.requestCount; i++) {
      if (output.requests[i].address == address && output.requests[i].command == command) return true;
    }
    return false;
  }

  bool sentAnything() const { return output.requestCount > 0; }
  MasterState state() const { return process.state(); }

  void bringToReady() {
    step();
    step();
  }

  // Scripted replies: the test sets the conveyor state after seeing the command, as the real board would.
  void applyConveyorCommands() {
    if (sent(CONVEYOR_ADDRESS, COMMAND_GO) && conveyorState == CONVEYOR_STOPPED) conveyorState = CONVEYOR_FORWARD;
    if (sent(CONVEYOR_ADDRESS, COMMAND_BACK) && conveyorState == CONVEYOR_STOPPED) conveyorState = CONVEYOR_REVERSE;
    if (sent(CONVEYOR_ADDRESS, COMMAND_STOP) && conveyorState != CONVEYOR_FAULT && conveyorState != CONVEYOR_SWEEPING) {
      conveyorState = CONVEYOR_STOPPED;
    }
    if (sent(CONVEYOR_ADDRESS, COMMAND_EJECT) && (conveyorState == CONVEYOR_STOPPED || conveyorState == CONVEYOR_FORWARD)) {
      conveyorState = CONVEYOR_SWEEPING;
    }
  }
};
