#include "process.h"

const char* masterStateName(MasterState state) {
  switch (state) {
    case MASTER_INITIALISING: return "INITIALISING";
    case MASTER_READY: return "READY";
    case MASTER_RUNNING: return "RUNNING";
    case MASTER_STOPPING: return "STOPPING";
    case MASTER_EJECTING: return "EJECTING";
    case MASTER_PAUSED: return "PAUSED";
    case MASTER_BENCH: return "BENCH";
    case MASTER_FAULT: return "FAULT";
    default: return "?";
  }
}

const char* masterFaultName(MasterFault fault) {
  switch (fault) {
    case MASTER_FAULT_NONE: return "NONE";
    case MASTER_FAULT_FEEDBACK_LOST: return "FEEDBACK_LOST";
    case MASTER_FAULT_FEEDBACK_STALE: return "FEEDBACK_STALE";
    case MASTER_FAULT_INVALID_REPLY: return "INVALID_REPLY";
    case MASTER_FAULT_SLAVE_FAULT: return "SLAVE_FAULT";
    case MASTER_FAULT_UNEXPECTED_MOTION: return "UNEXPECTED_MOTION";
    case MASTER_FAULT_CONFIRM_TIMEOUT: return "CONFIRM_TIMEOUT";
    case MASTER_FAULT_STOP_UNCONFIRMED: return "STOP_UNCONFIRMED";
    case MASTER_FAULT_SWEEP_TIMEOUT: return "SWEEP_TIMEOUT";
    case MASTER_FAULT_SLAVE_REBOOT: return "SLAVE_REBOOT";
    case MASTER_FAULT_QUEUE_OVERFLOW: return "QUEUE_OVERFLOW";
    default: return "?";
  }
}

static bool repliedAfter(const SlaveView& view, uint32_t sentMs) {
  return view.repliedThisPoll && (int32_t)(view.lastValidReplyMs - sentMs) > 0;
}

bool Process::inCycle() const {
  return state_ == MASTER_RUNNING || state_ == MASTER_STOPPING || state_ == MASTER_EJECTING || state_ == MASTER_PAUSED;
}

void Process::begin(const Calibration& calibration) {
  calibration_ = calibration;
  fault_ = MASTER_FAULT_NONE;
  stopUnconfirmed_ = false;
  counters_ = {0, 0, 0, 0};
  enterInitialising();
}

void Process::enterInitialising() {
  state_ = MASTER_INITIALISING;
  resumeTo_ = MASTER_RUNNING;
  pending_.active = false;
  initialStopRequested_ = false;
  resetTracking();
}

void Process::resetTracking() {
  tracking_ = false;
  windowCount_ = 0;
  ejectCount_ = 0;
  sweepSeen_ = false;
}

void Process::sendMotion(ProcessOutput& output, char command, uint8_t expectedState, bool retryEachPoll,
                         uint32_t now) {
  pending_.active = true;
  pending_.command = command;
  pending_.expectedState = expectedState;
  pending_.retryEachPoll = retryEachPoll;
  pending_.sentMs = now;
  pending_.deadline = now + CONFIRM_TIMEOUT_MS;
  output.add(CONVEYOR_ADDRESS, command);
}

// Only a reply received after the command was sent can confirm it.
Process::ConfirmResult Process::confirm(const SlaveView& conveyor, ProcessOutput& output, uint32_t now) {
  if (!pending_.active) return CONFIRM_IDLE;
  if (repliedAfter(conveyor, pending_.sentMs) && conveyor.state == pending_.expectedState) {
    pending_.active = false;
    return CONFIRM_DONE;
  }
  if ((int32_t)(now - pending_.deadline) >= 0) {
    pending_.active = false;
    return CONFIRM_TIMED_OUT;
  }
  if (pending_.retryEachPoll) output.add(CONVEYOR_ADDRESS, pending_.command);
  return CONFIRM_WAITING;
}

void Process::enterFault(MasterFault reason, ProcessOutput& output, uint32_t now) {
  if (state_ == MASTER_FAULT) return;
  state_ = MASTER_FAULT;
  fault_ = reason;
  pending_.active = false;
  stopUnconfirmed_ = true;
  pending_.sentMs = now;
  output.add(CONVEYOR_ADDRESS, COMMAND_STOP);
}

void Process::enterStopping(ProcessOutput& output, uint32_t now) {
  state_ = MASTER_STOPPING;
  sendMotion(output, COMMAND_STOP, CONVEYOR_STOPPED, true, now);
}

bool Process::checkFeedback(SlaveView& conveyor, SlaveView& sensing, ProcessOutput& output, uint32_t now) {
  bool idle = !inCycle();
  SlaveView* views[2] = {&conveyor, &sensing};
  for (uint8_t i = 0; i < 2; i++) {
    SlaveView& view = *views[i];
    if (view.invalid && !idle) { enterFault(MASTER_FAULT_INVALID_REPLY, output, now); return true; }
    if (view.inFault() && !(idle && view.fault == FAULT_CONTACT_LOST)) {
      enterFault(MASTER_FAULT_SLAVE_FAULT, output, now);
      return true;
    }
    if (view.uptimeStalled) { enterFault(MASTER_FAULT_FEEDBACK_STALE, output, now); return true; }
    if (view.rebootPending) {
      view.rebootPending = false;
      if (!idle) { enterFault(MASTER_FAULT_SLAVE_REBOOT, output, now); return true; }
      enterInitialising();
    }
    if (!view.fresh && !idle) { enterFault(MASTER_FAULT_FEEDBACK_LOST, output, now); return true; }
  }
  return false;
}

// Belt state must match the phase whenever no command is in flight.
bool Process::checkMotion(const SlaveView& conveyor, ProcessOutput& output, uint32_t now) {
  if (pending_.active || !conveyor.fresh || conveyor.invalid) return false;
  bool asExpected;
  switch (state_) {
    case MASTER_RUNNING: asExpected = conveyor.state == CONVEYOR_FORWARD; break;
    case MASTER_READY:
    case MASTER_PAUSED: asExpected = conveyor.state == CONVEYOR_STOPPED; break;
    default: return false;
  }
  if (!asExpected) {
    enterFault(MASTER_FAULT_UNEXPECTED_MOTION, output, now);
    return true;
  }
  return false;
}

bool Process::pushWindow(uint32_t steps) {
  if (windowCount_ >= QUEUE_CAPACITY) return false;
  windows_[windowCount_++] = steps;
  return true;
}

void Process::popWindow() {
  for (uint8_t i = 1; i < windowCount_; i++) windows_[i - 1] = windows_[i];
  if (windowCount_ > 0) windowCount_--;
}

bool Process::pushEject(uint32_t target) {
  if (ejectCount_ >= QUEUE_CAPACITY) return false;
  ejects_[ejectCount_++] = target;
  return true;
}

void Process::popEject() {
  for (uint8_t i = 1; i < ejectCount_; i++) ejects_[i - 1] = ejects_[i];
  if (ejectCount_ > 0) ejectCount_--;
}

// Windows are in arrival order, so the oldest expires first.
void Process::expireWindows(uint32_t steps) {
  while (windowCount_ > 0 && steps - windows_[0] > calibration_.channelSpacingSteps) {
    counters_.safe++;
    popWindow();
  }
}

// Every channel 1 edge opens a presence window. A channel 2 edge inside the oldest window is a magnetic puck;
// one with no window open is still magnetic, still ejected, and counted as unmatched.
void Process::trackPucks(const SlaveView& conveyor, const SlaveView& sensing, ProcessOutput& output, uint32_t now) {
  if (!conveyor.repliedThisPoll || !sensing.repliedThisPoll) return;
  if (!tracking_) {
    tracking_ = true;
    lastChannel1Edges_ = sensing.channel1Edges;
    lastChannel2Edges_ = sensing.channel2Edges;
    return;
  }
  uint16_t arrivals = (uint16_t)(sensing.channel1Edges - lastChannel1Edges_);
  uint16_t magnetics = (uint16_t)(sensing.channel2Edges - lastChannel2Edges_);
  lastChannel1Edges_ = sensing.channel1Edges;
  lastChannel2Edges_ = sensing.channel2Edges;
  uint32_t steps = conveyor.steps;
  for (uint16_t i = 0; i < arrivals; i++) {
    if (!pushWindow(steps)) { enterFault(MASTER_FAULT_QUEUE_OVERFLOW, output, now); return; }
  }
  for (uint16_t i = 0; i < magnetics; i++) {
    if (windowCount_ > 0) popWindow();
    else counters_.unmatchedMagnetic++;
    counters_.magnetic++;
    if (!pushEject(steps + calibration_.paddleSteps)) { enterFault(MASTER_FAULT_QUEUE_OVERFLOW, output, now); return; }
  }
  expireWindows(steps);
}

bool Process::nextEject(uint32_t steps, uint32_t& remaining) const {
  if (ejectCount_ == 0) return false;
  uint32_t target = ejects_[0];
  remaining = (int32_t)(target - steps) > 0 ? target - steps : 0;
  return true;
}

void Process::startBelt(ProcessOutput& output, uint32_t now) {
  state_ = MASTER_RUNNING;
  sendMotion(output, COMMAND_GO, CONVEYOR_FORWARD, false, now);
}

void Process::startEject(ProcessOutput& output, uint32_t now) {
  state_ = MASTER_EJECTING;
  sweepSeen_ = false;
  ejectSentMs_ = now;
  sendMotion(output, COMMAND_EJECT, CONVEYOR_SWEEPING, false, now);
}

void Process::tickEjecting(const SlaveView& conveyor, ProcessOutput& output, uint32_t now) {
  ConfirmResult result = confirm(conveyor, output, now);
  if (result == CONFIRM_TIMED_OUT) { enterFault(MASTER_FAULT_CONFIRM_TIMEOUT, output, now); return; }
  if (result == CONFIRM_DONE) sweepSeen_ = true;
  if (result == CONFIRM_WAITING) return;
  bool stopped = conveyor.fresh && conveyor.state == CONVEYOR_STOPPED;
  bool sweeping = conveyor.fresh && conveyor.state == CONVEYOR_SWEEPING;
  if (stopped && sweepSeen_) {
    popEject();
    counters_.ejected++;
    if (resumeTo_ == MASTER_PAUSED) state_ = MASTER_PAUSED;
    else startBelt(output, now);
    return;
  }
  if (now - ejectSentMs_ >= SWEEP_TIMEOUT_MS) { enterFault(MASTER_FAULT_SWEEP_TIMEOUT, output, now); return; }
  if (!sweeping && !stopped) enterFault(MASTER_FAULT_UNEXPECTED_MOTION, output, now);
}

void Process::tick(uint32_t now, SlaveView& conveyor, SlaveView& sensing, const OperatorCommand& operatorCommand,
                   ProcessOutput& output) {
  output.clear();
  OperatorKind operatorKind = operatorCommand.kind;

  if (state_ == MASTER_FAULT) {
    bool moving = conveyor.fresh && (conveyor.state == CONVEYOR_FORWARD || conveyor.state == CONVEYOR_REVERSE);
    if (moving && !stopUnconfirmed_) {
      stopUnconfirmed_ = true;
      pending_.sentMs = now;
    }
    if (stopUnconfirmed_) {
      bool atRest = conveyor.state == CONVEYOR_STOPPED || conveyor.state == CONVEYOR_FAULT;
      if (repliedAfter(conveyor, pending_.sentMs) && atRest) stopUnconfirmed_ = false;
      else output.add(CONVEYOR_ADDRESS, COMMAND_STOP);
    }
    if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
    return;
  }

  if (state_ == MASTER_BENCH) {
    if (operatorKind == OPERATOR_SEND) {
      output.add(operatorCommand.address, operatorCommand.command);
      output.notice = NOTICE_RAW_SENT;
    } else if (operatorKind == OPERATOR_STOP) {
      output.add(CONVEYOR_ADDRESS, COMMAND_STOP);
    } else if (operatorKind != OPERATOR_NONE) {
      output.notice = NOTICE_IGNORED;
    }
    return;
  }

  if (checkFeedback(conveyor, sensing, output, now)) return;
  if (inCycle()) {
    trackPucks(conveyor, sensing, output, now);
    if (state_ == MASTER_FAULT) return;
  }
  if (checkMotion(conveyor, output, now)) return;

  bool conveyorFresh = conveyor.fresh && !conveyor.invalid && conveyor.state != CONVEYOR_FAULT;
  bool sensingFresh = sensing.fresh && !sensing.invalid && sensing.state != SENSING_FAULT;
  bool conveyorStopped = conveyorFresh && conveyor.state == CONVEYOR_STOPPED;

  switch (state_) {
    case MASTER_INITIALISING: {
      if (operatorKind == OPERATOR_BENCH) {
        state_ = MASTER_BENCH;
        output.notice = NOTICE_BENCH_ENTERED;
        return;
      }
      if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      if (!initialStopRequested_ || !conveyorFresh || !sensingFresh) {
        initialStopRequested_ = true;
        output.add(CONVEYOR_ADDRESS, COMMAND_STOP);
        if (output.notice == NOTICE_NONE) output.notice = NOTICE_INIT_WAITING;
        return;
      }
      if (!conveyorStopped) {
        output.add(CONVEYOR_ADDRESS, COMMAND_STOP);
        return;
      }
      state_ = MASTER_READY;
      return;
    }

    case MASTER_READY: {
      if (operatorKind == OPERATOR_BENCH) {
        state_ = MASTER_BENCH;
        output.notice = NOTICE_BENCH_ENTERED;
        return;
      }
      if (operatorKind == OPERATOR_START) {
        if (!calibration_.ready()) { output.notice = NOTICE_START_NOT_CALIBRATED; return; }
        if (!conveyorFresh || !sensingFresh) { output.notice = NOTICE_START_NOT_READY; return; }
        resetTracking();
        resumeTo_ = MASTER_RUNNING;
        startBelt(output, now);
        return;
      }
      if (operatorKind == OPERATOR_STOP) { output.notice = NOTICE_STOP_ALREADY; return; }
      if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      return;
    }

    case MASTER_RUNNING: {
      if (operatorKind == OPERATOR_STOP) { resumeTo_ = MASTER_PAUSED; enterStopping(output, now); return; }
      if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      if (confirm(conveyor, output, now) == CONFIRM_TIMED_OUT) {
        enterFault(MASTER_FAULT_CONFIRM_TIMEOUT, output, now);
        return;
      }
      uint32_t remaining;
      if (conveyorFresh && nextEject(conveyor.steps, remaining) && remaining == 0) startEject(output, now);
      return;
    }

    case MASTER_STOPPING: {
      if (operatorKind == OPERATOR_STOP) output.notice = NOTICE_STOP_ALREADY;
      else if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      ConfirmResult result = confirm(conveyor, output, now);
      if (result == CONFIRM_TIMED_OUT) { enterFault(MASTER_FAULT_STOP_UNCONFIRMED, output, now); return; }
      if (result == CONFIRM_DONE) state_ = MASTER_PAUSED;
      return;
    }

    case MASTER_EJECTING: {
      if (operatorKind == OPERATOR_STOP) { resumeTo_ = MASTER_PAUSED; output.notice = NOTICE_SWEEP_IN_PROGRESS; }
      else if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      tickEjecting(conveyor, output, now);
      return;
    }

    case MASTER_PAUSED: {
      if (operatorKind == OPERATOR_RESUME) {
        if (!conveyorStopped || !sensingFresh) { output.notice = NOTICE_IGNORED; return; }
        output.notice = NOTICE_RESUMED;
        resumeTo_ = MASTER_RUNNING;
        uint32_t remaining;
        if (nextEject(conveyor.steps, remaining) && remaining == 0) startEject(output, now);
        else startBelt(output, now);
        return;
      }
      if (operatorKind == OPERATOR_STOP) output.notice = NOTICE_STOP_ALREADY;
      else if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      return;
    }

    default:
      return;
  }
}

uint8_t Process::conveyorTarget() const {
  switch (state_) {
    case MASTER_RUNNING: return CONVEYOR_FORWARD;
    case MASTER_EJECTING: return CONVEYOR_SWEEPING;
    default: return CONVEYOR_STOPPED;
  }
}

AndonColour Process::andon() const {
  switch (state_) {
    case MASTER_FAULT: return ANDON_RED;
    case MASTER_INITIALISING: return ANDON_AMBER_BLINK;
    case MASTER_PAUSED: return ANDON_AMBER;
    case MASTER_RUNNING:
    case MASTER_STOPPING:
    case MASTER_EJECTING: return ANDON_GREEN;
    default: return ANDON_OFF;
  }
}
