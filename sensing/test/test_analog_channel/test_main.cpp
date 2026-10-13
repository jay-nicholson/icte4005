// Unit tests for the Hall/LDR front end. A puck pass is modelled as a smooth bump on the ADC reading.
#include <math.h>
#include <unity.h>
#include "analog_channel.h"
#include "sensing_logic.h"

static const AnalogChannel::Config HALL = {128, 3, 12, 6, 400, 624, 20};
static const uint16_t REST = 512;

static uint32_t rngState = 12345;
static int noise(int amplitude) {  // deterministic, uniform in [-amplitude, +amplitude]
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return (int)(rngState % (2 * amplitude + 1)) - amplitude;
}

static uint16_t clampAdc(int value) { return value < 0 ? 0 : (value > 1023 ? 1023 : value); }

static void zeroChannel(AnalogChannel& channel, int noiseAmp = 2) {
  for (int i = 0; i < HALL.zeroSamples; i++) channel.sample(clampAdc(REST + noise(noiseAmp)));
}

// Runs one puck pass through `channel` -> SensingLogic and returns the closing edges counted.
// peak is signed (the sign is the magnet polarity). speedMmS is belt speed; sample period is 2 ms.
static int passEdges(AnalogChannel& channel, SensingLogic& logic, int peak, float speedMmS, float sigmaMm = 4.7f) {
  uint32_t now = 1000;
  uint16_t before = logic.reed1Edges();
  float span = 6 * sigmaMm;
  float travelled = -span;
  while (travelled < span + 40) {  // run on past the sensor so the channel can release
    float bump = peak * expf(-(travelled / sigmaMm) * (travelled / sigmaMm));
    channel.sample(clampAdc((int)(REST + bump + noise(2))));
    logic.sample(now, channel.active(), false);
    now += 2;
    travelled += speedMmS * 0.002f;
  }
  return logic.reed1Edges() - before;
}

void setUp() { rngState = 12345; }
void tearDown() {}

void test_baseline_captured_and_ready() {
  AnalogChannel channel; channel.begin(HALL);
  TEST_ASSERT_FALSE(channel.ready());
  zeroChannel(channel);
  TEST_ASSERT_TRUE(channel.ready());
  TEST_ASSERT_UINT16_WITHIN(2, REST, channel.baseline());
  TEST_ASSERT_FALSE(channel.active());
  TEST_ASSERT_FALSE(channel.fault());
}

void test_noise_alone_never_triggers() {
  AnalogChannel channel; channel.begin(HALL); zeroChannel(channel);
  for (int i = 0; i < 50000; i++) {
    channel.sample(clampAdc(REST + noise(5)));
    TEST_ASSERT_FALSE(channel.active());
  }
}

void test_either_polarity_triggers_once() {
  AnalogChannel channel; channel.begin(HALL); zeroChannel(channel);
  SensingLogic logic; logic.begin(20);
  TEST_ASSERT_EQUAL(1, passEdges(channel, logic, +60, 100));
  TEST_ASSERT_EQUAL(1, passEdges(channel, logic, -60, 100));
}

void test_one_edge_per_pass_across_belt_speeds() {
  const float speeds[] = {10, 25, 50, 100, 150};
  for (float speed : speeds) {
    AnalogChannel channel; channel.begin(HALL); zeroChannel(channel);
    SensingLogic logic; logic.begin(20);
    TEST_ASSERT_EQUAL_MESSAGE(1, passEdges(channel, logic, 60, speed), "edge count at this speed");
  }
}

void test_signal_below_threshold_is_ignored() {
  AnalogChannel channel; channel.begin(HALL); zeroChannel(channel);
  SensingLogic logic; logic.begin(20);
  TEST_ASSERT_EQUAL(0, passEdges(channel, logic, 8, 50));  // weaker than onDelta: deliberately invisible
}

void test_hysteresis_stops_chatter() {
  AnalogChannel channel; channel.begin(HALL); zeroChannel(channel, 0);
  // Hovering between the off and on thresholds must not toggle the output either way.
  for (int i = 0; i < 400; i++) { channel.sample(REST + 9 + (i % 2)); TEST_ASSERT_FALSE(channel.active()); }
  for (int i = 0; i < 100; i++) channel.sample(REST + 40);
  TEST_ASSERT_TRUE(channel.active());
  for (int i = 0; i < 400; i++) { channel.sample(REST + 9 + (i % 2)); TEST_ASSERT_TRUE(channel.active()); }
  for (int i = 0; i < 100; i++) channel.sample(REST + 1);
  TEST_ASSERT_FALSE(channel.active());
}

void test_unplausible_baseline_faults() {
  AnalogChannel low; low.begin(HALL);
  for (int i = 0; i < HALL.zeroSamples; i++) low.sample(150);  // unplugged / pulled low
  TEST_ASSERT_TRUE(low.ready());
  TEST_ASSERT_TRUE(low.fault());
  AnalogChannel jumpy; jumpy.begin(HALL);
  for (int i = 0; i < HALL.zeroSamples; i++) jumpy.sample(i % 2 ? REST + 30 : REST - 30);  // spread 60 > 20
  TEST_ASSERT_TRUE(jumpy.fault());
}

void test_stuck_at_rail_latches_fault() {
  AnalogChannel channel; channel.begin(HALL); zeroChannel(channel);
  for (int i = 0; i < 49; i++) channel.sample(1023);
  TEST_ASSERT_FALSE(channel.fault());  // a brief spike is not a fault
  channel.sample(REST);
  for (int i = 0; i < 60; i++) channel.sample(1023);
  TEST_ASSERT_TRUE(channel.fault());
  for (int i = 0; i < 200; i++) channel.sample(REST);
  TEST_ASSERT_TRUE(channel.fault());   // latched
  TEST_ASSERT_FALSE(channel.active());
}

void test_local_fault_is_reported_and_keeps_first_reason() {
  SensingLogic logic; logic.begin(20);
  logic.raiseLocalFault();
  uint8_t bytes[SENSING_REPLY_LENGTH];
  logic.encode(bytes, 5000);
  TEST_ASSERT_EQUAL(SENSING_FAULT, bytes[OFFSET_STATE]);
  TEST_ASSERT_EQUAL(FAULT_LOCAL, bytes[OFFSET_FAULT]);
  TEST_ASSERT_TRUE(logic.redOn());
  logic.serviceWatchdog(true);  // a later contact loss must not overwrite the first reason
  TEST_ASSERT_EQUAL(FAULT_LOCAL, logic.fault());

  SensingLogic other; other.begin(20);
  other.serviceWatchdog(true);
  other.raiseLocalFault();
  TEST_ASSERT_EQUAL(FAULT_CONTACT_LOST, other.fault());
}

void test_wire_format_is_unchanged() {
  SensingLogic logic; logic.begin(20);
  AnalogChannel channel; channel.begin(HALL); zeroChannel(channel);
  passEdges(channel, logic, 60, 100);
  uint8_t bytes[SENSING_REPLY_LENGTH];
  logic.encode(bytes, 0x01020304);
  TEST_ASSERT_EQUAL(0, bytes[OFFSET_STATE]);
  TEST_ASSERT_EQUAL(0, bytes[OFFSET_FAULT]);
  TEST_ASSERT_EQUAL(0x04, bytes[2]);  // little-endian uptime
  TEST_ASSERT_EQUAL(0x01, bytes[5]);
  TEST_ASSERT_EQUAL(0, bytes[OFFSET_SENSING_LIVE_MASK] & ~LIVE_MASK_ALL);
  TEST_ASSERT_EQUAL(1, readUint16LittleEndian(bytes + OFFSET_SENSING_REED1_EDGES));
  TEST_ASSERT_EQUAL(0, readUint16LittleEndian(bytes + OFFSET_SENSING_REED2_EDGES));
}

void test_fusion_truth_table() {
  TEST_ASSERT_FALSE(fusedClosed(false, false, true));
  TEST_ASSERT_FALSE(fusedClosed(true, false, true));   // stray field, no puck: rejected
  TEST_ASSERT_FALSE(fusedClosed(false, true, true));   // non-magnetic puck: not a magnetic event
  TEST_ASSERT_TRUE(fusedClosed(true, true, true));     // magnet and puck agree
  TEST_ASSERT_TRUE(fusedClosed(true, false, false));   // gate off: magnetic alone counts
  TEST_ASSERT_FALSE(fusedClosed(false, true, false));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_baseline_captured_and_ready);
  RUN_TEST(test_noise_alone_never_triggers);
  RUN_TEST(test_either_polarity_triggers_once);
  RUN_TEST(test_one_edge_per_pass_across_belt_speeds);
  RUN_TEST(test_signal_below_threshold_is_ignored);
  RUN_TEST(test_hysteresis_stops_chatter);
  RUN_TEST(test_unplausible_baseline_faults);
  RUN_TEST(test_stuck_at_rail_latches_fault);
  RUN_TEST(test_local_fault_is_reported_and_keeps_first_reason);
  RUN_TEST(test_wire_format_is_unchanged);
  RUN_TEST(test_fusion_truth_table);
  return UNITY_END();
}
