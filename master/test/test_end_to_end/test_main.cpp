// Runs the real master process against the real slave logic through contract bytes on a simulated belt.
#include <unity.h>
#include "conveyor_logic.h"
#include "process.h"
#include "sensing_logic.h"

static const float STEPS_PER_MM = 10;
static const float LIGHT_MM = 100;
static const float HALL_MM = 130;
static const float PADDLE_MM = 280;
static const float SENSOR_HALF_WINDOW_MM = 5;
static const uint16_t SENSING_SAMPLE_MS = 2;
static const uint16_t DEBOUNCE_MS = 20;
static const uint32_t SWEEP_DURATION_MS = 2800;
static const uint8_t MAX_PUCKS = 6;

struct Puck {
  float mm;
  bool magnetic;
  bool ejected;
};

struct World {
  uint32_t now;
  ConveyorLogic conveyorLogic;
  SensingLogic sensingLogic;
  SlaveView conveyorView;
  SlaveView sensingView;
  Process master;
  ProcessOutput output;
  Puck pucks[MAX_PUCKS];
  uint8_t puckCount;
  bool masterPolls;
  uint32_t lastReadMs;
  uint32_t lastSampleMs;
  uint32_t lastPollMs;
  uint32_t sweepStartedMs;

  void begin(uint32_t spacingSteps = 450, uint32_t paddleSteps = 1500) {
    now = 0;
    conveyorLogic.begin();
    sensingLogic.begin(DEBOUNCE_MS);
    conveyorView.begin(SLAVE_CONVEYOR);
    sensingView.begin(SLAVE_SENSING);
    Calibration calibration = {spacingSteps, paddleSteps};
    master.begin(calibration);
    puckCount = 0;
    masterPolls = true;
    lastReadMs = 0;
    lastSampleMs = 0;
    lastPollMs = 0;
    sweepStartedMs = 0;
    output.clear();
  }

  void addPuck(float mm, bool magnetic) {
    pucks[puckCount].mm = mm;
    pucks[puckCount].magnetic = magnetic;
    pucks[puckCount].ejected = false;
    puckCount++;
  }

  bool anyPuckWithin(float sensorMm, bool magneticOnly) const {
    for (uint8_t i = 0; i < puckCount; i++) {
      const Puck& puck = pucks[i];
      if (puck.ejected || (magneticOnly && !puck.magnetic)) continue;
      float distance = puck.mm - sensorMm;
      if (distance < 0) distance = -distance;
      if (distance < SENSOR_HALF_WINDOW_MM) return true;
    }
    return false;
  }

  void poll() {
    uint8_t bytes[MAX_REPLY_LENGTH];
    sensingLogic.encode(bytes, now);
    sensingView.noteReply(now, bytes, SENSING_REPLY_LENGTH);
    conveyorLogic.encode(bytes, now);
    conveyorView.noteReply(now, bytes, CONVEYOR_REPLY_LENGTH);
    lastReadMs = now;
    OperatorCommand none = {OPERATOR_NONE, 0, 0};
    master.tick(now, conveyorView, sensingView, pendingCommand.kind == OPERATOR_NONE ? none : pendingCommand, output);
    pendingCommand.kind = OPERATOR_NONE;
    for (uint8_t i = 0; i < output.requestCount; i++) {
      if (output.requests[i].address == CONVEYOR_ADDRESS) conveyorLogic.command(output.requests[i].command);
    }
  }

  OperatorCommand pendingCommand = {OPERATOR_NONE, 0, 0};

  void millisecond() {
    now++;
    MotorRequest motor = conveyorLogic.motor();
    if (motor != MOTOR_OFF) {
      conveyorLogic.countStep();
      for (uint8_t i = 0; i < puckCount; i++) pucks[i].mm += (motor == MOTOR_FORWARD ? 1.0f : -1.0f) / STEPS_PER_MM;
    }
    if (conveyorLogic.sweeping()) {
      if (sweepStartedMs == 0) sweepStartedMs = now;
      if (now - sweepStartedMs >= SWEEP_DURATION_MS) {
        for (uint8_t i = 0; i < puckCount; i++) {
          float distance = pucks[i].mm - PADDLE_MM;
          if (distance < 0) distance = -distance;
          if (!pucks[i].ejected && distance < 15) pucks[i].ejected = true;
        }
        conveyorLogic.sweepComplete();
        sweepStartedMs = 0;
      }
    }
    if (now - lastSampleMs >= SENSING_SAMPLE_MS) {
      lastSampleMs = now;
      sensingLogic.sample(now, anyPuckWithin(LIGHT_MM, false), anyPuckWithin(HALL_MM, true));
    }
    bool contactLost = lastReadMs > 0 && (now - lastReadMs) >= CONTACT_WATCHDOG_MS;
    conveyorLogic.serviceWatchdog(contactLost);
    sensingLogic.serviceWatchdog(contactLost);
    if (masterPolls && now - lastPollMs >= POLL_INTERVAL_MS) {
      lastPollMs = now;
      poll();
    }
  }

  void operate(OperatorKind kind) {
    pendingCommand.kind = kind;
    while (pendingCommand.kind != OPERATOR_NONE) millisecond();
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

  uint8_t ejectedCount() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < puckCount; i++) if (pucks[i].ejected) count++;
    return count;
  }
};

void setUp() {}
void tearDown() {}

#define ASSERT_STATE(w, s) TEST_ASSERT_EQUAL_STRING(masterStateName(s), masterStateName((w).master.state()))

void test_mixed_pucks_are_sorted_and_counted() {
  World world; world.begin();
  world.addPuck(20, true);
  world.addPuck(-40, false);
  world.addPuck(-100, true);
  world.addPuck(-160, false);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_READY, 1000));
  world.operate(OPERATOR_START);
  world.runFor(45000);
  ASSERT_STATE(world, MASTER_RUNNING);
  TEST_ASSERT_EQUAL(2, world.master.counters().magnetic);
  TEST_ASSERT_EQUAL(2, world.master.counters().safe);
  TEST_ASSERT_EQUAL(2, world.master.counters().ejected);
  TEST_ASSERT_EQUAL(0, world.master.counters().unmatchedMagnetic);
  TEST_ASSERT_EQUAL(2, world.ejectedCount());
  TEST_ASSERT_TRUE(world.pucks[0].ejected);
  TEST_ASSERT_FALSE(world.pucks[1].ejected);
  TEST_ASSERT_TRUE(world.pucks[2].ejected);
  TEST_ASSERT_FALSE(world.pucks[3].ejected);
  TEST_ASSERT_EQUAL(0, world.master.pendingEjects());
  TEST_ASSERT_EQUAL(MASTER_FAULT_NONE, world.master.fault());
}

void test_pause_and_resume_keeps_the_queue() {
  World world; world.begin();
  world.addPuck(20, true);
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  world.runFor(2000);
  TEST_ASSERT_EQUAL(1, world.master.pendingEjects());
  world.operate(OPERATOR_STOP);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_PAUSED, 1000));
  world.runFor(5000);
  ASSERT_STATE(world, MASTER_PAUSED);
  world.operate(OPERATOR_RESUME);
  world.runFor(20000);
  TEST_ASSERT_TRUE(world.pucks[0].ejected);
  TEST_ASSERT_EQUAL(1, world.master.counters().ejected);
  ASSERT_STATE(world, MASTER_RUNNING);
}

void test_safe_puck_only_is_counted_and_never_ejected() {
  World world; world.begin();
  world.addPuck(20, false);
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  world.runFor(30000);
  TEST_ASSERT_EQUAL(1, world.master.counters().safe);
  TEST_ASSERT_EQUAL(0, world.master.counters().magnetic);
  TEST_ASSERT_FALSE(world.pucks[0].ejected);
  ASSERT_STATE(world, MASTER_RUNNING);
}

void test_slaves_fault_when_master_stops_polling() {
  World world; world.begin();
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  world.runFor(300);
  world.masterPolls = false;
  world.runFor(1200);
  TEST_ASSERT_EQUAL(CONVEYOR_FAULT, world.conveyorLogic.state());
  TEST_ASSERT_EQUAL(MOTOR_OFF, world.conveyorLogic.motor());
  world.masterPolls = true;
  world.runFor(300);
  ASSERT_STATE(world, MASTER_FAULT);
}

void test_master_reset_recovers_without_touching_slaves() {
  World world; world.begin();
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  world.runFor(300);
  world.masterPolls = false;
  world.runFor(1500);
  TEST_ASSERT_EQUAL(CONVEYOR_FAULT, world.conveyorLogic.state());
  Calibration calibration = {450, 1500};
  world.master.begin(calibration);
  world.conveyorView.begin(SLAVE_CONVEYOR);
  world.sensingView.begin(SLAVE_SENSING);
  world.masterPolls = true;
  TEST_ASSERT_TRUE(world.runUntil(MASTER_READY, 2000));
  TEST_ASSERT_EQUAL(CONVEYOR_STOPPED, world.conveyorLogic.state());
  TEST_ASSERT_EQUAL(MASTER_FAULT_NONE, world.master.fault());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_mixed_pucks_are_sorted_and_counted);
  RUN_TEST(test_pause_and_resume_keeps_the_queue);
  RUN_TEST(test_safe_puck_only_is_counted_and_never_ejected);
  RUN_TEST(test_slaves_fault_when_master_stops_polling);
  RUN_TEST(test_master_reset_recovers_without_touching_slaves);
  return UNITY_END();
}
