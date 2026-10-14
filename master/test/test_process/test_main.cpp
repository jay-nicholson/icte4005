#include <unity.h>
#include "sim.h"

void setUp() {}
void tearDown() {}

#define ASSERT_STATE(sim, s) TEST_ASSERT_EQUAL_STRING(masterStateName(s), masterStateName((sim).state()))
#define ASSERT_FAULT(sim, f) TEST_ASSERT_EQUAL_STRING(masterFaultName(f), masterFaultName((sim).process.fault()))

static void startAndConfirmRunning(Sim& sim) {
  sim.bringToReady();
  sim.step(OPERATOR_START);
  ASSERT_STATE(sim, MASTER_RUNNING);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_RUNNING);
}

static void runUntilEject(Sim& sim) {
  while (sim.state() == MASTER_RUNNING) sim.step();
  ASSERT_STATE(sim, MASTER_EJECTING);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_EJECT));
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_EJECTING);
  TEST_ASSERT_EQUAL(CONVEYOR_SWEEPING, sim.conveyorState);
}

static void finishSweep(Sim& sim) {
  sim.conveyorState = CONVEYOR_STOPPED;
  sim.step();
}

void test_initialising_sends_stop_then_ready() {
  Sim sim; sim.begin();
  sim.step();
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  ASSERT_STATE(sim, MASTER_INITIALISING);
  sim.step();
  ASSERT_STATE(sim, MASTER_READY);
  TEST_ASSERT_EQUAL(ANDON_OFF, sim.process.andon());
}

void test_initialising_waits_for_absent_slaves() {
  Sim sim; sim.begin();
  sim.conveyorReplies = false; sim.sensingReplies = false;
  for (int i = 0; i < 20; i++) sim.step();
  ASSERT_STATE(sim, MASTER_INITIALISING);
  TEST_ASSERT_EQUAL(ANDON_AMBER_BLINK, sim.process.andon());
  sim.conveyorReplies = true; sim.sensingReplies = true;
  sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_READY);
}

void test_start_refused_without_calibration() {
  Sim sim; sim.begin(0, 0);
  sim.bringToReady();
  sim.step(OPERATOR_START);
  ASSERT_STATE(sim, MASTER_READY);
  TEST_ASSERT_EQUAL(NOTICE_START_NOT_CALIBRATED, sim.output.notice);
  TEST_ASSERT_FALSE(sim.sentAnything());
}

void test_safe_puck_is_counted_and_belt_keeps_running() {
  Sim sim; sim.begin(300, 500);
  startAndConfirmRunning(sim);
  sim.channel1Edges++;
  sim.step();
  TEST_ASSERT_EQUAL(1, sim.process.pendingPucks());
  for (int i = 0; i < 6; i++) sim.step();
  TEST_ASSERT_EQUAL(1, sim.process.pendingPucks());
  sim.step();
  TEST_ASSERT_EQUAL(0, sim.process.pendingPucks());
  TEST_ASSERT_EQUAL(1, sim.process.counters().safe);
  TEST_ASSERT_EQUAL(0, sim.process.counters().magnetic);
  ASSERT_STATE(sim, MASTER_RUNNING);
  TEST_ASSERT_EQUAL(ANDON_GREEN, sim.process.andon());
}

void test_magnetic_puck_is_counted_queued_and_ejected_then_belt_resumes() {
  Sim sim; sim.begin(300, 500);
  startAndConfirmRunning(sim);
  sim.channel1Edges++;
  sim.step();
  sim.step();
  sim.channel2Edges++;
  sim.step();
  TEST_ASSERT_EQUAL(0, sim.process.pendingPucks());
  TEST_ASSERT_EQUAL(1, sim.process.pendingEjects());
  TEST_ASSERT_EQUAL(1, sim.process.counters().magnetic);
  TEST_ASSERT_EQUAL(0, sim.process.counters().safe);
  uint32_t ejectAt = sim.conveyorSteps + 500;
  runUntilEject(sim);
  TEST_ASSERT_TRUE(sim.conveyorSteps >= ejectAt);
  TEST_ASSERT_TRUE(sim.conveyorSteps < ejectAt + sim.stepsPerPoll);
  TEST_ASSERT_EQUAL(ANDON_GREEN, sim.process.andon());
  sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_EJECTING);
  finishSweep(sim);
  ASSERT_STATE(sim, MASTER_RUNNING);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
  TEST_ASSERT_EQUAL(1, sim.process.counters().ejected);
  TEST_ASSERT_EQUAL(0, sim.process.pendingEjects());
}

void test_several_pucks_queue_in_order() {
  Sim sim; sim.begin(300, 2000);
  startAndConfirmRunning(sim);
  sim.channel1Edges++; sim.channel2Edges++;
  sim.step();
  sim.step(); sim.step();
  sim.channel1Edges++;
  sim.step();
  sim.step(); sim.step();
  sim.channel1Edges++; sim.channel2Edges++;
  sim.step();
  TEST_ASSERT_EQUAL(2, sim.process.pendingEjects());
  TEST_ASSERT_EQUAL(1, sim.process.pendingPucks());
  for (int i = 0; i < 7; i++) sim.step();
  TEST_ASSERT_EQUAL(1, sim.process.counters().safe);
  TEST_ASSERT_EQUAL(2, sim.process.counters().magnetic);
  runUntilEject(sim);
  finishSweep(sim);
  ASSERT_STATE(sim, MASTER_RUNNING);
  TEST_ASSERT_EQUAL(1, sim.process.pendingEjects());
  sim.applyConveyorCommands();
  runUntilEject(sim);
  finishSweep(sim);
  TEST_ASSERT_EQUAL(2, sim.process.counters().ejected);
  TEST_ASSERT_EQUAL(0, sim.process.pendingEjects());
}

void test_magnetic_without_presence_is_unmatched_but_still_ejected() {
  Sim sim; sim.begin(300, 200);
  startAndConfirmRunning(sim);
  sim.channel2Edges++;
  sim.step();
  TEST_ASSERT_EQUAL(1, sim.process.counters().magnetic);
  TEST_ASSERT_EQUAL(1, sim.process.counters().unmatchedMagnetic);
  TEST_ASSERT_EQUAL(1, sim.process.pendingEjects());
  ASSERT_STATE(sim, MASTER_RUNNING);
}

void test_two_close_pucks_do_not_fault() {
  Sim sim; sim.begin(300, 2000);
  startAndConfirmRunning(sim);
  sim.channel1Edges += 2;
  sim.step();
  TEST_ASSERT_EQUAL(2, sim.process.pendingPucks());
  sim.channel2Edges++;
  sim.step();
  TEST_ASSERT_EQUAL(1, sim.process.pendingPucks());
  for (int i = 0; i < 8; i++) sim.step();
  TEST_ASSERT_EQUAL(1, sim.process.counters().safe);
  TEST_ASSERT_EQUAL(1, sim.process.counters().magnetic);
  ASSERT_STATE(sim, MASTER_RUNNING);
}

void test_stop_pauses_and_resume_keeps_queue() {
  Sim sim; sim.begin(300, 1000);
  startAndConfirmRunning(sim);
  sim.channel1Edges++; sim.channel2Edges++;
  sim.step();
  uint32_t ejectAt = sim.conveyorSteps + 1000;
  sim.step(OPERATOR_STOP);
  ASSERT_STATE(sim, MASTER_STOPPING);
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_PAUSED);
  TEST_ASSERT_EQUAL(ANDON_AMBER, sim.process.andon());
  for (int i = 0; i < 30; i++) sim.step();
  ASSERT_STATE(sim, MASTER_PAUSED);
  TEST_ASSERT_EQUAL(1, sim.process.pendingEjects());
  sim.step(OPERATOR_RESUME);
  ASSERT_STATE(sim, MASTER_RUNNING);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
  sim.applyConveyorCommands();
  runUntilEject(sim);
  TEST_ASSERT_TRUE(sim.conveyorSteps >= ejectAt);
}

void test_stop_during_sweep_pauses_after_it() {
  Sim sim; sim.begin(300, 100);
  startAndConfirmRunning(sim);
  sim.channel2Edges++;
  sim.step();
  runUntilEject(sim);
  sim.step(OPERATOR_STOP);
  ASSERT_STATE(sim, MASTER_EJECTING);
  TEST_ASSERT_EQUAL(NOTICE_SWEEP_IN_PROGRESS, sim.output.notice);
  finishSweep(sim);
  ASSERT_STATE(sim, MASTER_PAUSED);
  TEST_ASSERT_FALSE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
}

void test_sweep_timeout_and_unconfirmed_eject_fault() {
  Sim sim; sim.begin(300, 100);
  startAndConfirmRunning(sim);
  sim.channel2Edges++;
  sim.step();
  runUntilEject(sim);
  for (int i = 0; i < 70 && sim.state() == MASTER_EJECTING; i++) sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_SWEEP_TIMEOUT);
  Sim other; other.begin(300, 100);
  startAndConfirmRunning(other);
  other.channel2Edges++;
  other.step();
  while (other.state() == MASTER_RUNNING) other.step();
  ASSERT_STATE(other, MASTER_EJECTING);
  other.step(); other.step(); other.step();
  ASSERT_FAULT(other, MASTER_FAULT_CONFIRM_TIMEOUT);
}

void test_stale_stopped_reply_cannot_confirm_stop() {
  Sim sim; sim.begin();
  sim.bringToReady();
  sim.step(OPERATOR_START);
  sim.conveyorReplies = false;
  sim.step();
  sim.step(OPERATOR_STOP);
  ASSERT_STATE(sim, MASTER_STOPPING);
  sim.step();
  TEST_ASSERT_NOT_EQUAL(MASTER_PAUSED, sim.state());
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_FEEDBACK_LOST);
}

void test_stop_retried_then_unconfirmed() {
  Sim sim; sim.begin();
  startAndConfirmRunning(sim);
  sim.step(OPERATOR_STOP);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.step();
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.step();
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_STOP_UNCONFIRMED);
}

void test_feedback_lost_in_motion_faults_and_keeps_requesting_stop() {
  Sim sim; sim.begin();
  startAndConfirmRunning(sim);
  sim.conveyorReplies = false;
  sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_RUNNING);
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_FEEDBACK_LOST);
  TEST_ASSERT_TRUE(sim.process.stopUnconfirmed());
  sim.step();
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.conveyorReplies = true;
  sim.conveyorState = CONVEYOR_STOPPED;
  sim.step();
  TEST_ASSERT_FALSE(sim.process.stopUnconfirmed());
  ASSERT_STATE(sim, MASTER_FAULT);
  TEST_ASSERT_EQUAL(ANDON_RED, sim.process.andon());
}

void test_invalid_reply_faults_only_in_cycle() {
  Sim sim; sim.begin();
  sim.bringToReady();
  sim.conveyorState = 9;
  sim.step();
  ASSERT_STATE(sim, MASTER_READY);
  sim.conveyorState = CONVEYOR_STOPPED;
  startAndConfirmRunning(sim);
  sim.conveyorState = 9;
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_INVALID_REPLY);
}

void test_slave_faults_and_contact_loss() {
  Sim sim; sim.begin();
  sim.bringToReady();
  sim.sensingState = SENSING_FAULT; sim.sensingFault = FAULT_CONTACT_LOST;
  sim.step();
  ASSERT_STATE(sim, MASTER_READY);
  sim.sensingFault = FAULT_LOCAL;
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_SLAVE_FAULT);
  Sim other; other.begin();
  startAndConfirmRunning(other);
  other.sensingState = SENSING_FAULT; other.sensingFault = FAULT_CONTACT_LOST;
  other.step();
  ASSERT_FAULT(other, MASTER_FAULT_SLAVE_FAULT);
}

void test_reboot_in_ready_reinitialises_but_in_cycle_faults() {
  Sim sim; sim.begin();
  sim.bringToReady();
  sim.conveyorUptime = 10;
  sim.step();
  ASSERT_STATE(sim, MASTER_INITIALISING);
  sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_READY);
  sim.step(OPERATOR_START);
  sim.sensingUptime = 5;
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_SLAVE_REBOOT);
}

void test_unexpected_motion_faults() {
  Sim sim; sim.begin();
  sim.bringToReady();
  sim.conveyorState = CONVEYOR_FORWARD;
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_UNEXPECTED_MOTION);
  Sim other; other.begin();
  startAndConfirmRunning(other);
  other.conveyorState = CONVEYOR_STOPPED;
  other.step();
  ASSERT_FAULT(other, MASTER_FAULT_UNEXPECTED_MOTION);
}

void test_counters_survive_pause_and_stop_in_ready() {
  Sim sim; sim.begin(300, 100);
  startAndConfirmRunning(sim);
  sim.channel1Edges++;
  for (int i = 0; i < 9; i++) sim.step();
  TEST_ASSERT_EQUAL(1, sim.process.counters().safe);
  sim.step(OPERATOR_STOP);
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_PAUSED);
  TEST_ASSERT_EQUAL(1, sim.process.counters().safe);
}

void test_bench_raw_sends_and_plain_stop() {
  Sim sim; sim.begin();
  sim.bringToReady();
  sim.step(OPERATOR_BENCH);
  ASSERT_STATE(sim, MASTER_BENCH);
  OperatorCommand raw = {OPERATOR_SEND, CONVEYOR_ADDRESS, COMMAND_GO};
  sim.stepWith(raw);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
  sim.conveyorState = 9;
  sim.step();
  ASSERT_STATE(sim, MASTER_BENCH);
  sim.conveyorState = CONVEYOR_FORWARD;
  sim.step(OPERATOR_STOP);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
}

void test_queue_overflow_faults() {
  Sim sim; sim.begin(100000, 100000);
  startAndConfirmRunning(sim);
  sim.channel1Edges += QUEUE_CAPACITY;
  sim.step();
  ASSERT_STATE(sim, MASTER_RUNNING);
  sim.channel1Edges++;
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_QUEUE_OVERFLOW);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_initialising_sends_stop_then_ready);
  RUN_TEST(test_initialising_waits_for_absent_slaves);
  RUN_TEST(test_start_refused_without_calibration);
  RUN_TEST(test_safe_puck_is_counted_and_belt_keeps_running);
  RUN_TEST(test_magnetic_puck_is_counted_queued_and_ejected_then_belt_resumes);
  RUN_TEST(test_several_pucks_queue_in_order);
  RUN_TEST(test_magnetic_without_presence_is_unmatched_but_still_ejected);
  RUN_TEST(test_two_close_pucks_do_not_fault);
  RUN_TEST(test_stop_pauses_and_resume_keeps_queue);
  RUN_TEST(test_stop_during_sweep_pauses_after_it);
  RUN_TEST(test_sweep_timeout_and_unconfirmed_eject_fault);
  RUN_TEST(test_stale_stopped_reply_cannot_confirm_stop);
  RUN_TEST(test_stop_retried_then_unconfirmed);
  RUN_TEST(test_feedback_lost_in_motion_faults_and_keeps_requesting_stop);
  RUN_TEST(test_invalid_reply_faults_only_in_cycle);
  RUN_TEST(test_slave_faults_and_contact_loss);
  RUN_TEST(test_reboot_in_ready_reinitialises_but_in_cycle_faults);
  RUN_TEST(test_unexpected_motion_faults);
  RUN_TEST(test_counters_survive_pause_and_stop_in_ready);
  RUN_TEST(test_bench_raw_sends_and_plain_stop);
  RUN_TEST(test_queue_overflow_faults);
  return UNITY_END();
}
