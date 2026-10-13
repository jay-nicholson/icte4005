#include <unity.h>
#include "sim.h"

void setUp() {}
void tearDown() {}

#define ASSERT_STATE(sim, s) TEST_ASSERT_EQUAL_STRING(masterStateName(s), masterStateName((sim).state()))
#define ASSERT_FAULT(sim, f) TEST_ASSERT_EQUAL_STRING(masterFaultName(f), masterFaultName((sim).process.fault()))

static void startAndConfirmForward(Sim& sim) {
  sim.bringToReady();
  sim.step(OPERATOR_START);
  ASSERT_STATE(sim, MASTER_FORWARD);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_FORWARD);
}

static void runToWaitAck(Sim& sim) {
  startAndConfirmForward(sim);
  sim.channel1Edges++;
  sim.step();
  ASSERT_STATE(sim, MASTER_TRACKING);
  sim.channel2Edges++;
  sim.sensingLive = LIVE_CHANNEL2;
  sim.step();
  ASSERT_STATE(sim, MASTER_STOPPING);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_WAIT_ACK);
}

static void ackAndConfirmApproach(Sim& sim) {
  sim.step(OPERATOR_ACK);
  ASSERT_STATE(sim, MASTER_APPROACH);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_APPROACH);
}

static void runApproachToEject(Sim& sim) {
  while (sim.state() == MASTER_APPROACH) sim.step();
  ASSERT_STATE(sim, MASTER_EJECTING);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_EJECT));
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_EJECTING);
  TEST_ASSERT_EQUAL(CONVEYOR_SWEEPING, sim.conveyorState);
}

void test_initialising_sends_stop_then_ready() {
  Sim sim; sim.begin();
  sim.step();
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  ASSERT_STATE(sim, MASTER_INITIALISING);
  sim.step();
  ASSERT_STATE(sim, MASTER_READY);
}

void test_initialising_waits_for_absent_slaves() {
  Sim sim; sim.begin();
  sim.conveyorReplies = false; sim.sensingReplies = false;
  for (int i = 0; i < 20; i++) sim.step();
  ASSERT_STATE(sim, MASTER_INITIALISING);
  sim.conveyorReplies = true; sim.sensingReplies = true;
  sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_READY);
}

void test_start_refusals() {
  Sim sim; sim.begin(0, 0);
  sim.bringToReady();
  sim.step(OPERATOR_START);
  ASSERT_STATE(sim, MASTER_READY);
  TEST_ASSERT_EQUAL(NOTICE_START_NOT_CALIBRATED, sim.output.notice);
  Sim sim2; sim2.begin();
  sim2.bringToReady();
  sim2.sensingLive = LIVE_CHANNEL1;
  sim2.step(OPERATOR_START);
  ASSERT_STATE(sim2, MASTER_READY);
  TEST_ASSERT_EQUAL(NOTICE_START_CHANNEL_ACTIVE, sim2.output.notice);
  TEST_ASSERT_FALSE(sim2.sentAnything());
}

void test_full_cycle() {
  Sim sim; sim.begin(1000, 300);
  runToWaitAck(sim);
  TEST_ASSERT_EQUAL(ANDON_AMBER, sim.process.andon());
  ackAndConfirmApproach(sim);
  uint32_t approachStart = sim.conveyorSteps - sim.stepsPerPoll;
  runApproachToEject(sim);
  TEST_ASSERT_TRUE(sim.conveyorSteps - approachStart >= 300);
  sim.sensingLive = 0;
  sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_EJECTING);
  sim.conveyorState = CONVEYOR_STOPPED;
  sim.step();
  ASSERT_STATE(sim, MASTER_READY);
  TEST_ASSERT_EQUAL(ANDON_OFF, sim.process.andon());
}

void test_sweep_timeout_faults() {
  Sim sim; sim.begin(1000, 100);
  runToWaitAck(sim);
  ackAndConfirmApproach(sim);
  runApproachToEject(sim);
  for (int i = 0; i < 60; i++) { sim.step(); if (sim.state() != MASTER_EJECTING) break; }
  ASSERT_STATE(sim, MASTER_FAULT);
  ASSERT_FAULT(sim, MASTER_FAULT_SWEEP_TIMEOUT);
}

void test_eject_not_confirmed_faults() {
  Sim sim; sim.begin(1000, 100);
  runToWaitAck(sim);
  ackAndConfirmApproach(sim);
  while (sim.state() == MASTER_APPROACH) sim.step();
  ASSERT_STATE(sim, MASTER_EJECTING);
  sim.step(); sim.step(); sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_CONFIRM_TIMEOUT);
}

void test_stop_during_approach_pauses_and_resumes_from_same_baseline() {
  Sim sim; sim.begin(1000, 400);
  runToWaitAck(sim);
  ackAndConfirmApproach(sim);
  sim.step(OPERATOR_STOP);
  ASSERT_STATE(sim, MASTER_STOPPING);
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_PAUSED);
  TEST_ASSERT_EQUAL(MASTER_APPROACH, sim.process.savedPhase());
  uint32_t delta, limit;
  TEST_ASSERT_TRUE(sim.process.travel(sim.conveyorSteps, delta, limit));
  TEST_ASSERT_EQUAL(400, limit);
  sim.step(OPERATOR_RESUME);
  ASSERT_STATE(sim, MASTER_APPROACH);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
  uint32_t deltaAfter;
  TEST_ASSERT_TRUE(sim.process.travel(sim.conveyorSteps, deltaAfter, limit));
  TEST_ASSERT_EQUAL(delta, deltaAfter);
}

void test_stop_during_ejecting_is_refused() {
  Sim sim; sim.begin(1000, 100);
  runToWaitAck(sim);
  ackAndConfirmApproach(sim);
  runApproachToEject(sim);
  sim.step(OPERATOR_STOP);
  ASSERT_STATE(sim, MASTER_EJECTING);
  TEST_ASSERT_EQUAL(NOTICE_SWEEP_IN_PROGRESS, sim.output.notice);
  TEST_ASSERT_FALSE(sim.sentAnything());
}

void test_both_reeds_in_same_poll_is_magnetic() {
  Sim sim; sim.begin();
  startAndConfirmForward(sim);
  sim.channel1Edges++; sim.channel2Edges++;
  sim.step();
  ASSERT_STATE(sim, MASTER_STOPPING);
}

void test_reed2_before_reed1_faults() {
  Sim sim; sim.begin();
  startAndConfirmForward(sim);
  sim.channel2Edges++;
  sim.step();
  ASSERT_STATE(sim, MASTER_FAULT);
  ASSERT_FAULT(sim, MASTER_FAULT_CHANNEL_ORDER);
}

void test_missing_forward_faults_and_keeps_requesting_stop() {
  Sim sim; sim.begin(300, 200);
  startAndConfirmForward(sim);
  sim.channel1Edges++;
  sim.step();
  ASSERT_STATE(sim, MASTER_TRACKING);
  for (int i = 0; i < 6; i++) sim.step();
  ASSERT_STATE(sim, MASTER_TRACKING);
  sim.step();
  ASSERT_STATE(sim, MASTER_FAULT);
  ASSERT_FAULT(sim, MASTER_FAULT_MISSING_FORWARD);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  TEST_ASSERT_TRUE(sim.process.stopUnconfirmed());
  TEST_ASSERT_EQUAL(ANDON_RED, sim.process.andon());
  sim.conveyorReplies = false;
  sim.step();
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.step();
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.conveyorReplies = true;
  sim.conveyorState = CONVEYOR_STOPPED;
  sim.step();
  TEST_ASSERT_FALSE(sim.process.stopUnconfirmed());
  TEST_ASSERT_FALSE(sim.sentAnything());
  sim.step(OPERATOR_RESUME);
  ASSERT_STATE(sim, MASTER_FAULT);
}

void test_event_wins_over_limit_in_same_poll() {
  Sim sim; sim.begin(300, 200);
  startAndConfirmForward(sim);
  sim.channel1Edges++;
  sim.step();
  for (int i = 0; i < 6; i++) sim.step();
  sim.channel2Edges++;
  sim.step();
  ASSERT_STATE(sim, MASTER_STOPPING);
}

void test_pause_in_tracking_keeps_baseline() {
  Sim sim; sim.begin(500, 200);
  startAndConfirmForward(sim);
  sim.channel1Edges++;
  sim.step();
  sim.step(); sim.step();
  sim.step(OPERATOR_STOP);
  ASSERT_STATE(sim, MASTER_STOPPING);
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_PAUSED);
  TEST_ASSERT_EQUAL(ANDON_AMBER, sim.process.andon());
  for (int i = 0; i < 50; i++) sim.step();
  ASSERT_STATE(sim, MASTER_PAUSED);
  sim.step(OPERATOR_RESUME);
  ASSERT_STATE(sim, MASTER_TRACKING);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
  sim.applyConveyorCommands();
  for (int i = 0; i < 8; i++) sim.step();
  ASSERT_STATE(sim, MASTER_FAULT);
  ASSERT_FAULT(sim, MASTER_FAULT_MISSING_FORWARD);
}

void test_magnetic_during_stop_for_pause_goes_to_wait_ack() {
  Sim sim; sim.begin();
  startAndConfirmForward(sim);
  sim.step(OPERATOR_STOP);
  ASSERT_STATE(sim, MASTER_STOPPING);
  sim.channel1Edges++; sim.channel2Edges++;
  sim.step();
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_WAIT_ACK);
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
  ASSERT_STATE(sim, MASTER_FAULT);
  ASSERT_FAULT(sim, MASTER_FAULT_FEEDBACK_LOST);
}

void test_stop_retried_then_unconfirmed() {
  Sim sim; sim.begin();
  startAndConfirmForward(sim);
  sim.step(OPERATOR_STOP);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.step();
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.step();
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.step();
  ASSERT_STATE(sim, MASTER_FAULT);
  ASSERT_FAULT(sim, MASTER_FAULT_STOP_UNCONFIRMED);
}

void test_feedback_lost_in_motion_faults_and_stays() {
  Sim sim; sim.begin();
  startAndConfirmForward(sim);
  sim.conveyorReplies = false;
  sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_FORWARD);
  sim.step();
  ASSERT_STATE(sim, MASTER_FAULT);
  ASSERT_FAULT(sim, MASTER_FAULT_FEEDBACK_LOST);
  sim.conveyorReplies = true;
  sim.conveyorState = CONVEYOR_STOPPED;
  sim.step();
  ASSERT_STATE(sim, MASTER_FAULT);
}

void test_invalid_reply_slave_fault_and_confirm_timeout() {
  Sim sim; sim.begin();
  sim.bringToReady();
  sim.conveyorState = 9;
  sim.step();
  ASSERT_STATE(sim, MASTER_READY);
  sim.conveyorState = CONVEYOR_STOPPED;
  startAndConfirmForward(sim);
  sim.conveyorState = 9;
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_INVALID_REPLY);
  Sim sim2; sim2.begin();
  sim2.bringToReady();
  sim2.sensingState = SENSING_FAULT; sim2.sensingFault = FAULT_LOCAL;
  sim2.step();
  ASSERT_FAULT(sim2, MASTER_FAULT_SLAVE_FAULT);
  Sim sim3; sim3.begin();
  sim3.bringToReady();
  sim3.step(OPERATOR_START);
  sim3.step(); sim3.step();
  ASSERT_STATE(sim3, MASTER_FORWARD);
  sim3.step();
  ASSERT_FAULT(sim3, MASTER_FAULT_CONFIRM_TIMEOUT);
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

void test_held_reed_after_eject_faults() {
  Sim sim; sim.begin(1000, 100);
  runToWaitAck(sim);
  ackAndConfirmApproach(sim);
  runApproachToEject(sim);
  sim.conveyorState = CONVEYOR_STOPPED;
  sim.sensingLive = LIVE_CHANNEL2;
  sim.step();
  ASSERT_STATE(sim, MASTER_CLEARING);
  sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_CLEARING);
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_CHANNEL_HELD);
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
  ASSERT_STATE(sim, MASTER_BENCH);
}

void test_raw_send_outside_bench_is_ignored() {
  Sim sim; sim.begin();
  sim.bringToReady();
  OperatorCommand raw = {OPERATOR_SEND, CONVEYOR_ADDRESS, COMMAND_GO};
  sim.stepWith(raw);
  TEST_ASSERT_FALSE(sim.sentAnything());
}

void test_lost_detection_history_faults() {
  Sim sim; sim.begin(500, 200);
  startAndConfirmForward(sim);
  sim.channel1Edges++;
  sim.step();
  ASSERT_STATE(sim, MASTER_TRACKING);
  sim.step(OPERATOR_STOP);
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_PAUSED);
  sim.channel1Edges--;
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_INVALID_REPLY);
}

void test_limit_exceeded_during_stopping_faults_before_resume() {
  Sim sim; sim.begin(300, 200);
  startAndConfirmForward(sim);
  sim.channel1Edges++;
  sim.step();
  sim.step(OPERATOR_STOP);
  ASSERT_STATE(sim, MASTER_STOPPING);
  sim.applyConveyorCommands();
  sim.conveyorSteps += 400;
  sim.step();
  ASSERT_STATE(sim, MASTER_FAULT);
  ASSERT_FAULT(sim, MASTER_FAULT_MISSING_FORWARD);
  TEST_ASSERT_FALSE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
}

void test_unexpected_motion_faults() {
  Sim sim; sim.begin();
  sim.bringToReady();
  sim.conveyorState = CONVEYOR_FORWARD;
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_UNEXPECTED_MOTION);
  Sim sim2; sim2.begin();
  runToWaitAck(sim2);
  sim2.conveyorState = CONVEYOR_FORWARD;
  sim2.step();
  ASSERT_FAULT(sim2, MASTER_FAULT_UNEXPECTED_MOTION);
}

void test_contact_lost_slave_only_blocks_idle_states() {
  Sim sim; sim.begin();
  sim.conveyorState = CONVEYOR_FAULT; sim.conveyorFault = FAULT_CONTACT_LOST;
  sim.sensingState = SENSING_FAULT; sim.sensingFault = FAULT_CONTACT_LOST;
  sim.step(); sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_INITIALISING);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_STOP));
  sim.conveyorState = CONVEYOR_STOPPED; sim.conveyorFault = FAULT_NONE;
  sim.sensingState = SENSING_OK; sim.sensingFault = FAULT_NONE;
  sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_READY);
  sim.sensingState = SENSING_FAULT; sim.sensingFault = FAULT_CONTACT_LOST;
  sim.step(OPERATOR_START);
  ASSERT_STATE(sim, MASTER_READY);
  TEST_ASSERT_EQUAL(NOTICE_START_NOT_READY, sim.output.notice);
  sim.sensingState = SENSING_FAULT; sim.sensingFault = FAULT_LOCAL;
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_SLAVE_FAULT);
}

void test_contact_lost_slave_during_cycle_faults() {
  Sim sim; sim.begin();
  startAndConfirmForward(sim);
  sim.sensingState = SENSING_FAULT; sim.sensingFault = FAULT_CONTACT_LOST;
  sim.step();
  ASSERT_FAULT(sim, MASTER_FAULT_SLAVE_FAULT);
}

void test_arrival_during_stop_promotes_saved_phase() {
  Sim sim; sim.begin();
  startAndConfirmForward(sim);
  sim.step(OPERATOR_STOP);
  sim.channel1Edges++;
  sim.applyConveyorCommands();
  sim.step();
  ASSERT_STATE(sim, MASTER_PAUSED);
  TEST_ASSERT_EQUAL(MASTER_TRACKING, sim.process.savedPhase());
  uint32_t delta, limit;
  TEST_ASSERT_TRUE(sim.process.travel(sim.conveyorSteps, delta, limit));
}

void test_single_channel_bench_treats_channel1_as_magnetic() {
  Sim sim; sim.begin();
  sim.process.begin(Calibration{1000, 300}, Options{0, false, true});
  startAndConfirmForward(sim);
  sim.channel1Edges++;
  sim.step();
  ASSERT_STATE(sim, MASTER_STOPPING);
}

void test_auto_ack_after_delay() {
  Sim sim; sim.begin(1000, 300, 500, false);
  runToWaitAck(sim);
  sim.step(); sim.step(); sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_WAIT_ACK);
  sim.step();
  ASSERT_STATE(sim, MASTER_APPROACH);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
}

void test_auto_restart_after_eject() {
  Sim sim; sim.begin(1000, 100, 0, true);
  runToWaitAck(sim);
  ackAndConfirmApproach(sim);
  runApproachToEject(sim);
  sim.conveyorState = CONVEYOR_STOPPED;
  sim.sensingLive = 0;
  sim.step();
  ASSERT_STATE(sim, MASTER_READY);
  sim.step();
  ASSERT_STATE(sim, MASTER_FORWARD);
  TEST_ASSERT_TRUE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
  sim.applyConveyorCommands();
  sim.step();
  sim.channel1Edges++; sim.channel2Edges++;
  sim.step();
  ASSERT_STATE(sim, MASTER_STOPPING);
}

void test_stop_in_ready_cancels_pending_restart() {
  Sim sim; sim.begin(1000, 100, 0, true);
  runToWaitAck(sim);
  ackAndConfirmApproach(sim);
  runApproachToEject(sim);
  sim.conveyorState = CONVEYOR_STOPPED;
  sim.sensingLive = LIVE_CHANNEL1;
  sim.step();
  ASSERT_STATE(sim, MASTER_CLEARING);
  sim.sensingLive = 0;
  sim.step();
  ASSERT_STATE(sim, MASTER_READY);
  sim.step(OPERATOR_STOP);
  ASSERT_STATE(sim, MASTER_READY);
  sim.step(); sim.step();
  ASSERT_STATE(sim, MASTER_READY);
  TEST_ASSERT_FALSE(sim.sent(CONVEYOR_ADDRESS, COMMAND_GO));
}

void test_targets_and_andon() {
  Sim sim; sim.begin();
  startAndConfirmForward(sim);
  TEST_ASSERT_EQUAL(CONVEYOR_FORWARD, sim.process.conveyorTarget());
  TEST_ASSERT_EQUAL(ANDON_GREEN, sim.process.andon());
  Sim sim2; sim2.begin();
  runToWaitAck(sim2);
  TEST_ASSERT_EQUAL(CONVEYOR_STOPPED, sim2.process.conveyorTarget());
  TEST_ASSERT_EQUAL(ANDON_AMBER, sim2.process.andon());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_initialising_sends_stop_then_ready);
  RUN_TEST(test_initialising_waits_for_absent_slaves);
  RUN_TEST(test_start_refusals);
  RUN_TEST(test_full_cycle);
  RUN_TEST(test_sweep_timeout_faults);
  RUN_TEST(test_eject_not_confirmed_faults);
  RUN_TEST(test_stop_during_approach_pauses_and_resumes_from_same_baseline);
  RUN_TEST(test_stop_during_ejecting_is_refused);
  RUN_TEST(test_both_reeds_in_same_poll_is_magnetic);
  RUN_TEST(test_reed2_before_reed1_faults);
  RUN_TEST(test_missing_forward_faults_and_keeps_requesting_stop);
  RUN_TEST(test_event_wins_over_limit_in_same_poll);
  RUN_TEST(test_pause_in_tracking_keeps_baseline);
  RUN_TEST(test_magnetic_during_stop_for_pause_goes_to_wait_ack);
  RUN_TEST(test_stale_stopped_reply_cannot_confirm_stop);
  RUN_TEST(test_stop_retried_then_unconfirmed);
  RUN_TEST(test_feedback_lost_in_motion_faults_and_stays);
  RUN_TEST(test_invalid_reply_slave_fault_and_confirm_timeout);
  RUN_TEST(test_reboot_in_ready_reinitialises_but_in_cycle_faults);
  RUN_TEST(test_held_reed_after_eject_faults);
  RUN_TEST(test_bench_raw_sends_and_plain_stop);
  RUN_TEST(test_raw_send_outside_bench_is_ignored);
  RUN_TEST(test_lost_detection_history_faults);
  RUN_TEST(test_limit_exceeded_during_stopping_faults_before_resume);
  RUN_TEST(test_unexpected_motion_faults);
  RUN_TEST(test_contact_lost_slave_only_blocks_idle_states);
  RUN_TEST(test_contact_lost_slave_during_cycle_faults);
  RUN_TEST(test_arrival_during_stop_promotes_saved_phase);
  RUN_TEST(test_single_channel_bench_treats_channel1_as_magnetic);
  RUN_TEST(test_auto_ack_after_delay);
  RUN_TEST(test_auto_restart_after_eject);
  RUN_TEST(test_stop_in_ready_cancels_pending_restart);
  RUN_TEST(test_targets_and_andon);
  return UNITY_END();
}
