// Runs Jay's unchanged master and conveyor logic against the Hall/LDR sensing front end.
// The belt and puck are the same physical model as master/test/test_end_to_end; the only difference
// is that the sensing board sees noisy ADC readings instead of ideal reed contacts. This is the proof
// that swapping the reeds for Hall sensors needs no change to the master or to CONTRACT.md.
#include <math.h>
#include <unity.h>
#include "analog_channel.h"
#include "conveyor_logic.h"
#include "process.h"
#include "sensing_logic.h"

static const float STEPS_PER_MM = 10;
static const float CH1_MM = 100;
static const float CH2_MM = 160;
static const float HALL_SIGMA_MM = 4.7f;   // field bump width: about +/-6 mm above the threshold
static const float HALL_PEAK = 60;         // counts at the sensor's closest approach
static const float LDR_REACH_MM = 15;      // the puck shades the LDR within this distance of ch2
static const uint16_t SENSING_SAMPLE_MS = 2;
static const uint16_t DEBOUNCE_MS = 20;
static const uint16_t REST = 512;
static const uint16_t LDR_REST = 600;

// Same values as sensing/src/main.cpp.
static const AnalogChannel::Config HALL_CONFIG = {128, 3, 12, 6, 400, 624, 20};
static const AnalogChannel::Config LDR_CONFIG = {128, 3, 60, 30, 30, 990, 40};

static uint32_t rngState;
static int noise(int amplitude) {
  if (amplitude == 0) return 0;
  rngState ^= rngState << 13; rngState ^= rngState >> 17; rngState ^= rngState << 5;
  return (int)(rngState % (2 * amplitude + 1)) - amplitude;
}
static uint16_t clampAdc(float v) { return v < 0 ? 0 : (v > 1023 ? 1023 : (uint16_t)v); }

struct World {
  uint32_t now;
  ConveyorLogic conveyorLogic;
  SensingLogic sensingLogic;
  AnalogChannel hall1, hall2, ldr;
  SlaveView conveyorView, sensingView;
  Process master;
  ProcessOutput output;
  float puckMm;
  bool puckMagnetic;
  int polarity;           // +1 / -1: which pole faces the sensor
  bool useGate;
  int noiseAmp;
  bool masterPolls;
  bool hall1Disconnected;
  float strayCounts;      // extra field on Hall 2 with no puck near (e.g. stepper coil pickup)
  uint32_t lastReadMs, lastSampleMs, lastPollMs;

  void begin(float startMm, bool magnetic, int pol = 1, bool gate = false, int noiseAmplitude = 2, uint32_t seed = 1) {
    rngState = seed;
    now = 0;
    conveyorLogic.begin();
    sensingLogic.begin(DEBOUNCE_MS);
    hall1.begin(HALL_CONFIG); hall2.begin(HALL_CONFIG); ldr.begin(LDR_CONFIG);
    conveyorView.begin(SLAVE_CONVEYOR);
    sensingView.begin(SLAVE_SENSING);
    Calibration calibration = {900, 1500, 300};
    master.begin(calibration);
    puckMm = startMm; puckMagnetic = magnetic; polarity = pol; useGate = gate; noiseAmp = noiseAmplitude;
    masterPolls = true; hall1Disconnected = false; strayCounts = 0;
    lastReadMs = lastSampleMs = lastPollMs = 0;
    output.clear();
  }

  uint16_t hallRaw(float sensorMm, float extra) const {
    float value = REST + extra;
    if (puckMagnetic) {
      float d = (puckMm - sensorMm) / HALL_SIGMA_MM;
      value += polarity * HALL_PEAK * expf(-d * d);
    }
    return clampAdc(value + noise(noiseAmp));
  }
  uint16_t ldrRaw() const {
    float shade = fabsf(puckMm - CH2_MM) < LDR_REACH_MM ? -150 : 0;  // any puck, magnetic or not
    return clampAdc(LDR_REST + shade + noise(noiseAmp));
  }

  void sampleSensing() {
    hall1.sample(hall1Disconnected ? 0 : hallRaw(CH1_MM, 0));
    hall2.sample(hallRaw(CH2_MM, strayCounts));
    ldr.sample(ldrRaw());
    sensingLogic.sample(now, hall1.active(), fusedClosed(hall2.active(), ldr.active(), useGate));
    if (hall1.fault() || hall2.fault() || (useGate && ldr.fault())) sensingLogic.raiseLocalFault();
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

  void millisecond(OperatorKind kind = OPERATOR_NONE) {
    now++;
    MotorRequest motor = conveyorLogic.motor();
    if (motor != MOTOR_OFF) {
      conveyorLogic.countStep();
      puckMm += (motor == MOTOR_FORWARD ? 1.0f : -1.0f) / STEPS_PER_MM;
    }
    if (now - lastSampleMs >= SENSING_SAMPLE_MS) { lastSampleMs = now; sampleSensing(); }
    bool contactLost = lastReadMs > 0 && (now - lastReadMs) >= CONTACT_WATCHDOG_MS;
    conveyorLogic.serviceWatchdog(contactLost);
    sensingLogic.serviceWatchdog(contactLost);
    if (masterPolls && now - lastPollMs >= POLL_INTERVAL_MS) { lastPollMs = now; poll(kind); }
  }

  void operate(OperatorKind kind) {
    while (now - lastPollMs < POLL_INTERVAL_MS - 1) millisecond();
    millisecond(kind);
  }
  bool runUntil(MasterState target, uint32_t maxMs) {
    uint32_t deadline = now + maxMs;
    while (now < deadline) { millisecond(); if (master.state() == target) return true; }
    return false;
  }
  void runFor(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) millisecond(); }
};

void setUp() {}
void tearDown() {}

#define ASSERT_STATE(w, s) TEST_ASSERT_EQUAL_STRING(masterStateName(s), masterStateName((w).master.state()))

static void runFullCycle(World& world) {
  TEST_ASSERT_TRUE(world.runUntil(MASTER_READY, 1000));
  world.operate(OPERATOR_START);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_TRACKING, 2000));
  TEST_ASSERT_TRUE(world.runUntil(MASTER_WAIT_ACK, 2000));
  TEST_ASSERT_EQUAL(CONVEYOR_STOPPED, world.conveyorLogic.state());
  TEST_ASSERT_EQUAL(1, world.sensingLogic.reed2Edges());
  world.operate(OPERATOR_ACK);
  TEST_ASSERT_TRUE(world.runUntil(MASTER_OVERRUN, 3000));
  TEST_ASSERT_TRUE(world.runUntil(MASTER_READY, 3000));
  TEST_ASSERT_EQUAL(CONVEYOR_STOPPED, world.conveyorLogic.state());
  TEST_ASSERT_EQUAL(MASTER_FAULT_NONE, world.master.fault());
  TEST_ASSERT_EQUAL(SENSING_OK, world.sensingLogic.state());
}

void test_hall_front_end_full_cycle() {
  World world; world.begin(20, true);
  runFullCycle(world);
}

void test_reversed_magnet_polarity_full_cycle() {
  World world; world.begin(20, true, -1);
  runFullCycle(world);
}

void test_full_cycle_with_noise_across_seeds() {
  for (uint32_t seed = 1; seed <= 8; seed++) {
    World world; world.begin(20, true, (seed % 2) ? 1 : -1, false, 4, seed * 7919);
    runFullCycle(world);
  }
}

void test_full_cycle_with_ldr_gate_enabled() {
  World world; world.begin(20, true, 1, true);
  runFullCycle(world);
}

void test_non_magnetic_puck_runs_until_operator_stop() {
  World world; world.begin(20, false);
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  world.runFor(20000);
  ASSERT_STATE(world, MASTER_FORWARD);
  TEST_ASSERT_EQUAL(0, world.sensingLogic.reed1Edges());
  TEST_ASSERT_EQUAL(0, world.sensingLogic.reed2Edges());
  TEST_ASSERT_EQUAL(MASTER_FAULT_NONE, world.master.fault());
}

// The case that justifies the fusion gate: a brief stray field on Hall 2 while no puck is at the
// station (stepper coil pickup, a dropped magnet). The same pulse is used with and without the gate.
// Limit: the gate cannot reject a stray field that coincides with a non-magnetic puck at the station,
// because the LDR sees "a puck" either way. It narrows the false-trigger window; it does not close it.
static void injectTransientStrayField(World& world) {
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  world.runFor(500);          // puck is still well short of the station
  world.strayCounts = 40;
  world.runFor(300);
  world.strayCounts = 0;
  world.runFor(3000);         // the non-magnetic puck then passes the station normally
}

void test_stray_field_faults_master_without_gate() {
  World world; world.begin(20, false, 1, false);
  injectTransientStrayField(world);
  ASSERT_STATE(world, MASTER_FAULT);
  TEST_ASSERT_EQUAL(MASTER_FAULT_REED_ORDER, world.master.fault());
}

void test_stray_field_ignored_with_gate() {
  World world; world.begin(20, false, 1, true);
  injectTransientStrayField(world);
  ASSERT_STATE(world, MASTER_FORWARD);
  TEST_ASSERT_EQUAL(0, world.sensingLogic.reed2Edges());
  TEST_ASSERT_EQUAL(MASTER_FAULT_NONE, world.master.fault());
}

// A reed that falls off its wire just looks open. A Hall sensor that falls off its wire does not.
void test_unplugged_hall_is_detected_and_stops_the_belt() {
  World world; world.begin(20, true);
  world.runUntil(MASTER_READY, 1000);
  world.operate(OPERATOR_START);
  world.runFor(300);
  world.hall1Disconnected = true;
  world.runFor(600);
  TEST_ASSERT_EQUAL(SENSING_FAULT, world.sensingLogic.state());
  TEST_ASSERT_EQUAL(FAULT_LOCAL, world.sensingLogic.fault());
  TEST_ASSERT_TRUE(world.sensingLogic.redOn());
  ASSERT_STATE(world, MASTER_FAULT);
  TEST_ASSERT_EQUAL(MASTER_FAULT_SLAVE_FAULT, world.master.fault());
  world.runFor(300);
  TEST_ASSERT_EQUAL(CONVEYOR_STOPPED, world.conveyorLogic.state());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_hall_front_end_full_cycle);
  RUN_TEST(test_reversed_magnet_polarity_full_cycle);
  RUN_TEST(test_full_cycle_with_noise_across_seeds);
  RUN_TEST(test_full_cycle_with_ldr_gate_enabled);
  RUN_TEST(test_non_magnetic_puck_runs_until_operator_stop);
  RUN_TEST(test_stray_field_faults_master_without_gate);
  RUN_TEST(test_stray_field_ignored_with_gate);
  RUN_TEST(test_unplugged_hall_is_detected_and_stops_the_belt);
  return UNITY_END();
}
