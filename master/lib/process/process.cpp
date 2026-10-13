#include "process.h"

const char* masterStateName(MasterState state) {
  switch (state) {
    case MASTER_INITIALISING: return "INITIALISING";
    case MASTER_READY: return "READY";
    case MASTER_FORWARD: return "FORWARD";
    case MASTER_TRACKING: return "TRACKING";
    case MASTER_STOPPING: return "STOPPING";
    case MASTER_WAIT_ACK: return "WAIT_ACK";
    case MASTER_RETURNING: return "RETURNING";
    case MASTER_OVERRUN: return "OVERRUN";
    case MASTER_CLEARING: return "CLEARING";
    case MASTER_PAUSED: return "PAUSED";
    case MASTER_BENCH: return "BENCH";
    case MASTER_FAULT: return "FAULT";
    default: return "?";
  }
}

const char* masterFaultName(MasterFault fault) {
  switch (fault) {
    case MASTER_FAULT_NONE: return "NONE";
    case MASTER_FAULT_MISSING_FORWARD: return "MISSING_FORWARD";
    case MASTER_FAULT_MISSING_RETURN: return "MISSING_RETURN";
    case MASTER_FAULT_REED_ORDER: return "REED_ORDER";
    case MASTER_FAULT_FEEDBACK_LOST: return "FEEDBACK_LOST";
    case MASTER_FAULT_FEEDBACK_STALE: return "FEEDBACK_STALE";
    case MASTER_FAULT_INVALID_REPLY: return "INVALID_REPLY";
    case MASTER_FAULT_SLAVE_FAULT: return "SLAVE_FAULT";
    case MASTER_FAULT_UNEXPECTED_MOTION: return "UNEXPECTED_MOTION";
    case MASTER_FAULT_CONFIRM_TIMEOUT: return "CONFIRM_TIMEOUT";
    case MASTER_FAULT_STOP_UNCONFIRMED: return "STOP_UNCONFIRMED";
    case MASTER_FAULT_SLAVE_REBOOT: return "SLAVE_REBOOT";
    case MASTER_FAULT_REED_HELD: return "REED_HELD";
    default: return "?";
  }
}

static bool repliedAfter(const SlaveView& view, uint32_t sentMs) {
  return view.repliedThisPoll && (int32_t)(view.lastValidReplyMs - sentMs) > 0;
}

static bool isForwardPhase(MasterState state) { return state == MASTER_FORWARD || state == MASTER_TRACKING; }
static bool isReturnPhase(MasterState state) { return state == MASTER_RETURNING || state == MASTER_OVERRUN; }

MasterState Process::activePhase() const {
  return (state_ == MASTER_STOPPING || state_ == MASTER_PAUSED) ? savedPhase_ : state_;
}

bool Process::inForwardPhase() const { return isForwardPhase(activePhase()); }
bool Process::inReturnPhase() const { return isReturnPhase(activePhase()); }

bool Process::inCycle() const {
  return state_ != MASTER_INITIALISING && state_ != MASTER_READY && state_ != MASTER_BENCH && state_ != MASTER_FAULT;
}

void Process::begin(const Calibration& calibration) {
  calibration_ = calibration;
  fault_ = MASTER_FAULT_NONE;
  stopUnconfirmed_ = false;
  enterInitialising();
}

void Process::enterInitialising() {
  state_ = MASTER_INITIALISING;
  pending_.active = false;
  initialStopRequested_ = false;
  resetCycle();
}

void Process::resetCycle() {
  savedPhase_ = MASTER_INITIALISING;
  hasReedBaseline_ = false;
  hasEntryBaseline_ = false;
  hasReturnBaseline_ = false;
  hasOverrunBaseline_ = false;
  clearingWaitStarted_ = false;
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

void Process::enterStopping(MasterState saved, ProcessOutput& output, uint32_t now) {
  savedPhase_ = saved;
  state_ = MASTER_STOPPING;
  sendMotion(output, COMMAND_STOP, CONVEYOR_STOPPED, true, now);
}

bool Process::checkFeedback(SlaveView& conveyor, SlaveView& sensing, ProcessOutput& output, uint32_t now) {
  bool idle = !inCycle();
  SlaveView* views[2] = {&conveyor, &sensing};
  for (uint8_t i = 0; i < 2; i++) {
    SlaveView& view = *views[i];
    if (view.invalid) { enterFault(MASTER_FAULT_INVALID_REPLY, output, now); return true; }
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

// Step baselines pair a reed event with the conveyor reply from the same poll.
void Process::updateBaselines(const SlaveView& conveyor, const SlaveView& sensing) {
  if (!conveyor.repliedThisPoll || !sensing.repliedThisPoll) return;
  if (hasReedBaseline_ && !hasEntryBaseline_ && (uint16_t)(sensing.reed1Edges - reed1Start_) > 0) {
    hasEntryBaseline_ = true;
    entryBaseline_ = conveyor.steps;
  }
  if (hasReturnBaseline_ && !hasOverrunBaseline_ && (uint16_t)(sensing.reed1Edges - reed1ReturnStart_) > 0) {
    hasOverrunBaseline_ = true;
    overrunBaseline_ = conveyor.steps;
  }
}

Observation Process::observe(const SlaveView& conveyor, const SlaveView& sensing) const {
  Observation seen;
  seen.conveyorFresh = conveyor.fresh && conveyor.state != CONVEYOR_FAULT;
  seen.sensingFresh = sensing.fresh && sensing.state != SENSING_FAULT;
  seen.conveyorStopped = conveyor.fresh && conveyor.state == CONVEYOR_STOPPED;
  seen.conveyorForward = conveyor.fresh && conveyor.state == CONVEYOR_FORWARD;
  seen.conveyorReverse = conveyor.fresh && conveyor.state == CONVEYOR_REVERSE;
  seen.reedsClear = sensing.fresh && sensing.live == 0;
  seen.reed1Closed = sensing.fresh && (sensing.live & LIVE_REED1);
  bool reed1Rose = hasReedBaseline_ && (uint16_t)(sensing.reed1Edges - reed1Start_) > 0;
  bool reed2Rose = hasReedBaseline_ && (uint16_t)(sensing.reed2Edges - reed2Start_) > 0;
  seen.arrived = reed1Rose;
  seen.magnetic = reed1Rose && reed2Rose;
  seen.reedOrderBad = reed2Rose && !reed1Rose;
  seen.returned = hasReturnBaseline_ && (uint16_t)(sensing.reed1Edges - reed1ReturnStart_) > 0;
  seen.forwardOverdue = hasEntryBaseline_ && !seen.magnetic &&
                        (conveyor.steps - entryBaseline_) > calibration_.forwardAllowanceSteps;
  seen.returnOverdue = hasReturnBaseline_ && !seen.returned &&
                       (conveyor.steps - returnBaseline_) > calibration_.returnAllowanceSteps;
  seen.overrunDone = hasOverrunBaseline_ && (conveyor.steps - overrunBaseline_) >= calibration_.dischargeOverrunSteps;
  return seen;
}

// Runs in every cycle state, including STOPPING and PAUSED, before any operator command.
bool Process::checkProgress(const Observation& seen, ProcessOutput& output, uint32_t now) {
  if (inForwardPhase()) {
    if (seen.reedOrderBad) { enterFault(MASTER_FAULT_REED_ORDER, output, now); return true; }
    if (seen.forwardOverdue) { enterFault(MASTER_FAULT_MISSING_FORWARD, output, now); return true; }
  }
  if (inReturnPhase() && seen.returnOverdue) {
    enterFault(MASTER_FAULT_MISSING_RETURN, output, now);
    return true;
  }
  if (pending_.active || !seen.conveyorFresh || state_ == MASTER_STOPPING) return false;
  bool expectedForward = isForwardPhase(state_);
  bool expectedReverse = isReturnPhase(state_);
  bool asExpected = expectedForward ? seen.conveyorForward : expectedReverse ? seen.conveyorReverse : seen.conveyorStopped;
  if (!asExpected) {
    enterFault(MASTER_FAULT_UNEXPECTED_MOTION, output, now);
    return true;
  }
  return false;
}

void Process::startForward(const Observation& seen, ProcessOutput& output, uint32_t now) {
  state_ = seen.arrived ? MASTER_TRACKING : MASTER_FORWARD;
  sendMotion(output, COMMAND_GO, CONVEYOR_FORWARD, false, now);
}

void Process::startReverse(const SlaveView& conveyor, const SlaveView& sensing, const Observation& seen,
                           ProcessOutput& output, uint32_t now) {
  if (!hasReturnBaseline_) {
    hasReturnBaseline_ = true;
    reed1ReturnStart_ = sensing.reed1Edges;
    returnBaseline_ = conveyor.steps;
  }
  state_ = seen.returned ? MASTER_OVERRUN : MASTER_RETURNING;
  sendMotion(output, COMMAND_BACK, CONVEYOR_REVERSE, false, now);
}

void Process::finishStop(const Observation& seen, ProcessOutput& output, uint32_t now) {
  if (inForwardPhase()) {
    if (seen.arrived) savedPhase_ = MASTER_TRACKING;
    state_ = seen.magnetic ? MASTER_WAIT_ACK : MASTER_PAUSED;
    return;
  }
  if (seen.returned) savedPhase_ = MASTER_OVERRUN;
  if (seen.returned && seen.overrunDone) {
    state_ = MASTER_CLEARING;
    clearingWaitStarted_ = false;
    tickClearing(seen, output, now);
    return;
  }
  state_ = MASTER_PAUSED;
}

void Process::resume(const SlaveView& conveyor, const SlaveView& sensing, const Observation& seen,
                     ProcessOutput& output, uint32_t now) {
  if (!seen.conveyorStopped || !seen.sensingFresh) {
    output.notice = NOTICE_IGNORED;
    return;
  }
  output.notice = NOTICE_RESUMED;
  if (inForwardPhase()) {
    if (seen.magnetic) state_ = MASTER_WAIT_ACK;
    else startForward(seen, output, now);
    return;
  }
  if (seen.returned && seen.overrunDone) {
    state_ = MASTER_CLEARING;
    clearingWaitStarted_ = false;
    tickClearing(seen, output, now);
    return;
  }
  startReverse(conveyor, sensing, seen, output, now);
}

void Process::tickClearing(const Observation& seen, ProcessOutput& output, uint32_t now) {
  if (!seen.reedsClear) {
    if (!clearingWaitStarted_) {
      clearingWaitStarted_ = true;
      clearingWaitStartMs_ = now;
    }
    if (now - clearingWaitStartMs_ >= REED_CLEAR_GRACE_MS) enterFault(MASTER_FAULT_REED_HELD, output, now);
    return;
  }
  state_ = MASTER_READY;
  resetCycle();
}

void Process::tick(uint32_t now, SlaveView& conveyor, SlaveView& sensing, const OperatorCommand& operatorCommand,
                   ProcessOutput& output) {
  output.clear();
  OperatorKind operatorKind = operatorCommand.kind;

  if (state_ == MASTER_FAULT) {
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
  if (inCycle()) updateBaselines(conveyor, sensing);
  Observation seen = observe(conveyor, sensing);
  if (inCycle() && checkProgress(seen, output, now)) return;

  switch (state_) {
    case MASTER_INITIALISING: {
      if (operatorKind == OPERATOR_BENCH) {
        state_ = MASTER_BENCH;
        output.notice = NOTICE_BENCH_ENTERED;
        return;
      }
      if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      if (!initialStopRequested_ || !seen.conveyorFresh || !seen.sensingFresh) {
        initialStopRequested_ = true;
        output.add(CONVEYOR_ADDRESS, COMMAND_STOP);
        if (output.notice == NOTICE_NONE) output.notice = NOTICE_INIT_WAITING;
        return;
      }
      if (!seen.conveyorStopped) {
        output.add(CONVEYOR_ADDRESS, COMMAND_STOP);
        return;
      }
      if (!seen.reedsClear) {
        if (output.notice == NOTICE_NONE) output.notice = NOTICE_INIT_REEDS_ACTIVE;
        return;
      }
      state_ = MASTER_READY;
      resetCycle();
      return;
    }

    case MASTER_READY: {
      if (seen.conveyorFresh && !seen.conveyorStopped) {
        enterFault(MASTER_FAULT_UNEXPECTED_MOTION, output, now);
        return;
      }
      if (operatorKind == OPERATOR_BENCH) {
        state_ = MASTER_BENCH;
        output.notice = NOTICE_BENCH_ENTERED;
        return;
      }
      if (operatorKind == OPERATOR_START) {
        if (!calibration_.ready()) { output.notice = NOTICE_START_NOT_CALIBRATED; return; }
        if (!seen.conveyorFresh || !seen.sensingFresh) { output.notice = NOTICE_START_NOT_READY; return; }
        if (!seen.reedsClear) { output.notice = NOTICE_START_REED_ACTIVE; return; }
        resetCycle();
        hasReedBaseline_ = true;
        reed1Start_ = sensing.reed1Edges;
        reed2Start_ = sensing.reed2Edges;
        startForward(observe(conveyor, sensing), output, now);
        return;
      }
      if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      return;
    }

    case MASTER_FORWARD:
    case MASTER_TRACKING: {
      if (seen.magnetic) { enterStopping(state_, output, now); return; }
      if (operatorKind == OPERATOR_STOP) { enterStopping(state_, output, now); return; }
      if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      if (confirm(conveyor, output, now) == CONFIRM_TIMED_OUT) {
        enterFault(MASTER_FAULT_CONFIRM_TIMEOUT, output, now);
        return;
      }
      if (seen.arrived) state_ = MASTER_TRACKING;
      return;
    }

    case MASTER_STOPPING: {
      if (operatorKind == OPERATOR_STOP) output.notice = NOTICE_STOP_ALREADY;
      else if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      ConfirmResult result = confirm(conveyor, output, now);
      if (result == CONFIRM_TIMED_OUT) { enterFault(MASTER_FAULT_STOP_UNCONFIRMED, output, now); return; }
      if (result == CONFIRM_DONE) finishStop(seen, output, now);
      return;
    }

    case MASTER_WAIT_ACK: {
      if (operatorKind == OPERATOR_ACK) {
        if (seen.reed1Closed) { output.notice = NOTICE_ACK_REED1_ACTIVE; return; }
        startReverse(conveyor, sensing, seen, output, now);
        return;
      }
      if (operatorKind == OPERATOR_STOP) output.notice = NOTICE_STOP_ALREADY;
      else if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      return;
    }

    case MASTER_RETURNING:
    case MASTER_OVERRUN: {
      if (operatorKind == OPERATOR_STOP) { enterStopping(state_, output, now); return; }
      if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      if (confirm(conveyor, output, now) == CONFIRM_TIMED_OUT) {
        enterFault(MASTER_FAULT_CONFIRM_TIMEOUT, output, now);
        return;
      }
      if (seen.returned) state_ = MASTER_OVERRUN;
      if (seen.overrunDone) enterStopping(MASTER_OVERRUN, output, now);
      return;
    }

    case MASTER_CLEARING: {
      if (operatorKind == OPERATOR_STOP) output.notice = NOTICE_STOP_ALREADY;
      else if (operatorKind != OPERATOR_NONE) output.notice = NOTICE_IGNORED;
      tickClearing(seen, output, now);
      return;
    }

    case MASTER_PAUSED: {
      if (operatorKind == OPERATOR_RESUME) { resume(conveyor, sensing, seen, output, now); return; }
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
    case MASTER_FORWARD:
    case MASTER_TRACKING: return CONVEYOR_FORWARD;
    case MASTER_RETURNING:
    case MASTER_OVERRUN: return CONVEYOR_REVERSE;
    default: return CONVEYOR_STOPPED;
  }
}

AndonColour Process::andon() const {
  switch (state_) {
    case MASTER_FAULT: return ANDON_RED;
    case MASTER_WAIT_ACK:
    case MASTER_PAUSED: return ANDON_AMBER;
    case MASTER_FORWARD:
    case MASTER_TRACKING:
    case MASTER_STOPPING:
    case MASTER_RETURNING:
    case MASTER_OVERRUN:
    case MASTER_CLEARING: return ANDON_GREEN;
    default: return ANDON_OFF;
  }
}

bool Process::travel(uint32_t steps, uint32_t& delta, uint32_t& limit) const {
  MasterState phase = activePhase();
  if (phase == MASTER_TRACKING && hasEntryBaseline_) {
    delta = steps - entryBaseline_;
    limit = calibration_.forwardAllowanceSteps;
    return true;
  }
  if (phase == MASTER_RETURNING && hasReturnBaseline_) {
    delta = steps - returnBaseline_;
    limit = calibration_.returnAllowanceSteps;
    return true;
  }
  if (phase == MASTER_OVERRUN && hasOverrunBaseline_) {
    delta = steps - overrunBaseline_;
    limit = calibration_.dischargeOverrunSteps;
    return true;
  }
  return false;
}
