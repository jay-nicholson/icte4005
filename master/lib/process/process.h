#pragma once
#include <stdint.h>
#include "contract.h"
#include "feedback.h"

enum MasterState {
  MASTER_INITIALISING = 0,
  MASTER_READY,
  MASTER_RUNNING,
  MASTER_STOPPING,
  MASTER_EJECTING,
  MASTER_PAUSED,
  MASTER_BENCH,
  MASTER_FAULT
};

enum MasterFault {
  MASTER_FAULT_NONE = 0,
  MASTER_FAULT_FEEDBACK_LOST,
  MASTER_FAULT_FEEDBACK_STALE,
  MASTER_FAULT_INVALID_REPLY,
  MASTER_FAULT_SLAVE_FAULT,
  MASTER_FAULT_UNEXPECTED_MOTION,
  MASTER_FAULT_CONFIRM_TIMEOUT,
  MASTER_FAULT_STOP_UNCONFIRMED,
  MASTER_FAULT_SWEEP_TIMEOUT,
  MASTER_FAULT_SLAVE_REBOOT,
  MASTER_FAULT_QUEUE_OVERFLOW
};

enum OperatorKind {
  OPERATOR_NONE = 0,
  OPERATOR_START,
  OPERATOR_STOP,
  OPERATOR_RESUME,
  OPERATOR_BENCH,
  OPERATOR_SEND
};

struct OperatorCommand {
  OperatorKind kind;
  uint8_t address;
  char command;
};

enum Notice {
  NOTICE_NONE = 0,
  NOTICE_START_NOT_CALIBRATED,
  NOTICE_START_NOT_READY,
  NOTICE_IGNORED,
  NOTICE_INIT_WAITING,
  NOTICE_BENCH_ENTERED,
  NOTICE_RAW_SENT,
  NOTICE_STOP_ALREADY,
  NOTICE_SWEEP_IN_PROGRESS,
  NOTICE_RESUMED
};

enum AndonColour { ANDON_OFF, ANDON_GREEN, ANDON_AMBER, ANDON_AMBER_BLINK, ANDON_RED };

struct Calibration {
  uint32_t channelSpacingSteps;  // presence sensor to magnetic sensor, plus margin
  uint32_t paddleSteps;          // magnetic sensor to paddle
  bool ready() const {
    return channelSpacingSteps > 0 && paddleSteps > 0 && channelSpacingSteps < 0x80000000UL &&
           paddleSteps < 0x80000000UL;
  }
};

struct Counters {
  uint16_t safe;
  uint16_t magnetic;
  uint16_t unmatchedMagnetic;
  uint16_t ejected;
};

struct Request {
  uint8_t address;
  char command;
};

struct ProcessOutput {
  Request requests[4];
  uint8_t requestCount;
  Notice notice;
  void clear() { requestCount = 0; notice = NOTICE_NONE; }
  void add(uint8_t address, char command) {
    if (requestCount < 4) {
      requests[requestCount].address = address;
      requests[requestCount].command = command;
      requestCount++;
    }
  }
};

static const uint8_t QUEUE_CAPACITY = 16;

const char* masterStateName(MasterState state);
const char* masterFaultName(MasterFault fault);

class Process {
 public:
  void begin(const Calibration& calibration);
  void tick(uint32_t now, SlaveView& conveyor, SlaveView& sensing, const OperatorCommand& operatorCommand,
            ProcessOutput& output);

  MasterState state() const { return state_; }
  MasterFault fault() const { return fault_; }
  bool stopUnconfirmed() const { return stopUnconfirmed_; }
  bool calibrated() const { return calibration_.ready(); }
  const Counters& counters() const { return counters_; }
  uint8_t pendingPucks() const { return windowCount_; }
  uint8_t pendingEjects() const { return ejectCount_; }
  bool nextEject(uint32_t steps, uint32_t& remaining) const;
  uint8_t conveyorTarget() const;
  AndonColour andon() const;

 private:
  enum ConfirmResult { CONFIRM_IDLE, CONFIRM_WAITING, CONFIRM_DONE, CONFIRM_TIMED_OUT };
  struct Pending {
    bool active;
    char command;
    uint8_t expectedState;
    bool retryEachPoll;
    uint32_t sentMs;
    uint32_t deadline;
  };

  bool inCycle() const;
  void sendMotion(ProcessOutput& output, char command, uint8_t expectedState, bool retryEachPoll, uint32_t now);
  ConfirmResult confirm(const SlaveView& conveyor, ProcessOutput& output, uint32_t now);
  void enterFault(MasterFault reason, ProcessOutput& output, uint32_t now);
  void enterInitialising();
  void enterStopping(ProcessOutput& output, uint32_t now);
  void resetTracking();
  bool checkFeedback(SlaveView& conveyor, SlaveView& sensing, ProcessOutput& output, uint32_t now);
  bool checkMotion(const SlaveView& conveyor, ProcessOutput& output, uint32_t now);
  void trackPucks(const SlaveView& conveyor, const SlaveView& sensing, ProcessOutput& output, uint32_t now);
  void expireWindows(uint32_t steps);
  bool pushWindow(uint32_t steps);
  void popWindow();
  bool pushEject(uint32_t target);
  void popEject();
  void startBelt(ProcessOutput& output, uint32_t now);
  void startEject(ProcessOutput& output, uint32_t now);
  void tickEjecting(const SlaveView& conveyor, ProcessOutput& output, uint32_t now);

  MasterState state_;
  MasterState resumeTo_;
  MasterFault fault_;
  Calibration calibration_;
  Pending pending_;
  bool initialStopRequested_;
  bool stopUnconfirmed_;
  bool sweepSeen_;
  uint32_t ejectSentMs_;

  bool tracking_;
  uint16_t lastChannel1Edges_;
  uint16_t lastChannel2Edges_;
  uint32_t windows_[QUEUE_CAPACITY];
  uint8_t windowCount_;
  uint32_t ejects_[QUEUE_CAPACITY];
  uint8_t ejectCount_;
  Counters counters_;
};
