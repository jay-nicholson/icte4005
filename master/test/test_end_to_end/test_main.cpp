// Runs the real master process against the real slave logic through contract bytes.
// The belt, puck and reeds are a small physical model; the hardware hooks are stubbed.
#include <unity.h>
#include "conveyor_logic.h"
#include "process.h"
#include "sensing_logic.h"

static const float STEPS_PER_MM = 10;
static const float REED1_MM = 100;
static const float REED2_MM = 160;
static const float REED_HALF_WINDOW_MM = 5;
static const uint16_t SENSING_SAMPLE_MS = 2;
static const uint16_t DEBOUNCE_MS = 20;

struct World {
  uint32_t now;
  ConveyorLogic conveyorLogic;
  SensingLogic sensingLogic;
  SlaveView conveyorView;
  SlaveView sensingView;
  Process master;
  ProcessOutput output;
  float puckMm;
  bool puckMagnetic;
  bool masterPolls;
  uint32_t lastReadMs;
  uint32_t lastSampleMs;
  uint32_t lastPollMs;

  void begin(float startMm, bool magnetic, uint32_t forward = 900, uint32_t ret = 1500, uint32_t overrun = 300) {
    now = 0;
    conveyorLogic.begin();
    sensingLogic.begin(DEBOUNCE_MS);
    conveyorView.begin(SLAVE_CONVEYOR);
    sensingView.begin(SLAVE_SENSING);
    Calibration calibration = {forward, ret, overrun};
    master.begin(calibration);
    puckMm = startMm;
    puckMagnetic = magnetic;
    masterPolls = true;
    lastReadMs = 0;
    lastSampleMs = 0;
    lastPollMs = 0;
    output.clear();
  }

  bool reedClosed(float reedMm) const {
    if (!puckMagnetic) return false;
    float distance = puckMm - reedMm;
    if (distance < 0) distance = -distance;
    return distance < REED_HALF_WINDOW_MM;
  }

  void poll(OperatorKind kind) {
    uint8_t bytes[MAX_REPLY_LENGTH];
    sensingLogic.encode(bytes, now);
    sensingView.noteReply(now, bytes, SENSING_REPLY_LENGTH);
    conveyorLogic.encode(bytes, now);
    conveyorView.noteReply(now, bytes, CONVEYOR_REPLY_LENGTH);
    lastReadMs = now;
    OperatorCommand operatorCommand = {kind, 0, 0};
    master.tick(now, conveyorView, sensingView, operatorCommand, output);
    for (uint8_t i = 0; i < output.requestCount; i++) {
      if (output.requests[i].address == CONVEYOR_ADDRESS) conveyorLogic.command(output.requests[i].command);
    }
  }

  // One millisecond of the world: one motor step when running, sensing sample, master poll.
  void millisecond(OperatorKind kind = OPERATOR_NONE) {
    now++;
    MotorRequest motor = conveyorLogic.motor();
    if (motor != MOTOR_OFF) {
      conveyorLogic.countStep();
      puckMm += (motor == MOTOR_FORWARD ? 1.0f : -1.0f) / STEPS_PER_MM;
    }
    if (now - lastSampleMs >= SENSING_SAMPLE_MS) {
      lastSampleMs = now;
      sensingLogic.sample(now, reedClosed(REED1_MM), reedClosed(REED2_MM));
    }
    bool contactLost = lastReadMs > 0 && (now - lastReadMs) >= CONTACT_WATCHDOG_MS;
    conveyorLogic.serviceWatchdog(contactLost);
    sensingLogic.serviceWatchdog(contactLost);
    if (masterPolls && now - lastPollMs >= POLL_INTERVAL_MS) {
      lastPollMs = now;
      poll(kind);
    }
  }

  void operate(OperatorKind kind) {
    while (now - lastPollMs < POLL_INTERVAL_MS - 1) millisecond();
    millisecond(kind);
  }

  bool runUntil(MasterState target, uint32_t maxMs) {
    uint32_t deadline = now + maxMs;
    while (now < deadline) {
      millisecond();
      if (master.state() == target) return true;
    }
    return false;
  }

  void runFor(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) millisecond();
  }
};

void setUp() {}
void tearDown() {}

#define ASSERT_STATE(w, s) TEST_ASSERT_EQUAL_STRING(masterStateName(s), masterStateName((w).master.state()))

void test_magnetic_puck_full_cycle() {
  World world; world.begin(20, true);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_READY, 1000));
  world.operate(OPERATOR_START);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_TRACKING, 2000));
  TEST_ASSERT_TRUE(world.runUntil(MASTER_WAIT_ACK, 2000));
  TEST_ASSERT_EQUAL(CONVEYOR_STOPPED, world.conveyorLogic.state());
  TEST_ASSERT_TRUE(world.puckMm > REED2_MM - REED_HALF_WINDOW_MM);
  TEST_ASSERT_EQUAL(LIVE_REED2, world.sensingLogic.live());
  world.operate(OPERATOR_ACK);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_OVERRUN, 3000));
  TEST_ASSERT_TRUE(world.runUntil(MASTER_READY, 3000));
  TEST_ASSERT_EQUAL(CONVEYOR_STOPPED, world.conveyorLogic.state());
  TEST_ASSERT_TRUE(world.puckMm < REED1_MM - REED_HALF_WINDOW_MM);
  TEST_ASSERT_EQUAL(0, world.sensingLogic.live());
  TEST_ASSERT_EQUAL(MASTER_FAULT_NONE, world.master.fault());
}

void test_pause_and_resume_while_tracking() {
  World world; world.begin(20, true);
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_TRACKING, 2000));
  world.operate(OPERATOR_STOP);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_PAUSED, 1000));
  TEST_ASSERT_EQUAL(CONVEYOR_STOPPED, world.conveyorLogic.state());
  world.runFor(5000);
  ASSERT_STATE(world, MASTER_PAUSED);
  world.operate(OPERATOR_RESUME);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_WAIT_ACK, 2000));
  TEST_ASSERT_EQUAL(MASTER_FAULT_NONE, world.master.fault());
}

void test_non_magnetic_puck_runs_until_operator_stop() {
  World world; world.begin(20, false);
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  world.runFor(20000);
  ASSERT_STATE(world, MASTER_FORWARD);
  TEST_ASSERT_EQUAL(CONVEYOR_FORWARD, world.conveyorLogic.state());
  world.operate(OPERATOR_STOP);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_PAUSED, 1000));
  TEST_ASSERT_EQUAL(CONVEYOR_STOPPED, world.conveyorLogic.state());
}

void test_slaves_fault_when_master_stops_polling() {
  World world; world.begin(20, true);
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  world.runFor(300);
  TEST_ASSERT_EQUAL(CONVEYOR_FORWARD, world.conveyorLogic.state());
  world.masterPolls = false;
  world.runFor(1200);
  TEST_ASSERT_EQUAL(CONVEYOR_FAULT, world.conveyorLogic.state());
  TEST_ASSERT_EQUAL(FAULT_CONTACT_LOST, world.conveyorLogic.fault());
  TEST_ASSERT_EQUAL(MOTOR_OFF, world.conveyorLogic.motor());
  world.masterPolls = true;
  world.runFor(300);
  ASSERT_STATE(world, MASTER_FAULT);
}

void test_short_forward_allowance_faults_with_real_slaves() {
  World world; world.begin(20, true, 200, 1500, 300);
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_FAULT, 4000));
  TEST_ASSERT_EQUAL(MASTER_FAULT_MISSING_FORWARD, world.master.fault());
  world.runFor(300);
  TEST_ASSERT_EQUAL(CONVEYOR_STOPPED, world.conveyorLogic.state());
  TEST_ASSERT_FALSE(world.master.stopUnconfirmed());
}

void test_master_reset_recovers_without_touching_slaves() {
  World world; world.begin(20, true);
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  world.runFor(300);
  world.masterPolls = false;
  world.runFor(1500);
  TEST_ASSERT_EQUAL(CONVEYOR_FAULT, world.conveyorLogic.state());
  TEST_ASSERT_EQUAL(SENSING_FAULT, world.sensingLogic.state());
  Calibration calibration = {900, 1500, 300};
  world.master.begin(calibration);
  world.conveyorView.begin(SLAVE_CONVEYOR);
  world.sensingView.begin(SLAVE_SENSING);
  world.masterPolls = true;
  TEST_ASSERT_TRUE(world.runUntil(MASTER_READY, 2000));
  TEST_ASSERT_EQUAL(CONVEYOR_STOPPED, world.conveyorLogic.state());
  TEST_ASSERT_EQUAL(SENSING_OK, world.sensingLogic.state());
  TEST_ASSERT_EQUAL(MASTER_FAULT_NONE, world.master.fault());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_magnetic_puck_full_cycle);
  RUN_TEST(test_pause_and_resume_while_tracking);
  RUN_TEST(test_non_magnetic_puck_runs_until_operator_stop);
  RUN_TEST(test_slaves_fault_when_master_stops_polling);
  RUN_TEST(test_short_forward_allowance_faults_with_real_slaves);
  RUN_TEST(test_master_reset_recovers_without_touching_slaves);
  return UNITY_END();
}
