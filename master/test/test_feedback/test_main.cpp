#include <unity.h>
#include "feedback.h"

static void conveyorBytes(uint8_t* bytes, uint8_t state, uint8_t fault, uint32_t uptime, uint32_t steps) {
  bytes[OFFSET_STATE] = state;
  bytes[OFFSET_FAULT] = fault;
  writeUint32LittleEndian(bytes + OFFSET_UPTIME, uptime);
  writeUint32LittleEndian(bytes + OFFSET_CONVEYOR_STEPS, steps);
}

static void sensingBytes(uint8_t* bytes, uint8_t state, uint8_t fault, uint32_t uptime, uint8_t live, uint16_t channel1,
                         uint16_t channel2) {
  bytes[OFFSET_STATE] = state;
  bytes[OFFSET_FAULT] = fault;
  writeUint32LittleEndian(bytes + OFFSET_UPTIME, uptime);
  bytes[OFFSET_SENSING_LIVE_MASK] = live;
  writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL1_EDGES, channel1);
  writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL2_EDGES, channel2);
  writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL1_LEVEL, 0);
  writeUint16LittleEndian(bytes + OFFSET_SENSING_CHANNEL2_LEVEL, 0);
}

void setUp() {}
void tearDown() {}

void test_conveyor_decodes_little_endian() {
  SlaveView view; view.begin(SLAVE_CONVEYOR);
  uint8_t bytes[10]; conveyorBytes(bytes, CONVEYOR_FORWARD, FAULT_NONE, 0x01020304, 0xA1B2C3D4);
  TEST_ASSERT_EQUAL(DECODE_OK, view.noteReply(100, bytes, 10));
  TEST_ASSERT_EQUAL(CONVEYOR_FORWARD, view.state);
  TEST_ASSERT_EQUAL_HEX32(0x01020304, view.uptime);
  TEST_ASSERT_EQUAL_HEX32(0xA1B2C3D4, view.steps);
  TEST_ASSERT_TRUE(view.fresh);
  TEST_ASSERT_TRUE(view.repliedThisPoll);
}

void test_sensing_decodes_counts() {
  SlaveView view; view.begin(SLAVE_SENSING);
  uint8_t bytes[15]; sensingBytes(bytes, SENSING_OK, FAULT_NONE, 5, LIVE_CHANNEL2, 0x1234, 0xBEEF);
  TEST_ASSERT_EQUAL(DECODE_OK, view.noteReply(100, bytes, 15));
  TEST_ASSERT_EQUAL_HEX16(0x1234, view.channel1Edges);
  TEST_ASSERT_EQUAL_HEX16(0xBEEF, view.channel2Edges);
  TEST_ASSERT_EQUAL(LIVE_CHANNEL2, view.live);
}

void test_wrong_length_is_invalid_and_keeps_last_good() {
  SlaveView view; view.begin(SLAVE_SENSING);
  uint8_t bytes[15]; sensingBytes(bytes, SENSING_OK, 0, 1, 0, 7, 0);
  view.noteReply(100, bytes, 15);
  sensingBytes(bytes, SENSING_OK, 0, 2, 0, 9, 0);
  TEST_ASSERT_EQUAL(DECODE_BAD_LENGTH, view.noteReply(200, bytes, 14));
  TEST_ASSERT_TRUE(view.invalid);
  TEST_ASSERT_FALSE(view.repliedThisPoll);
  TEST_ASSERT_EQUAL(7, view.channel1Edges);
}

void test_fault_state_needs_reason() {
  SlaveView view; view.begin(SLAVE_CONVEYOR);
  uint8_t bytes[10];
  conveyorBytes(bytes, CONVEYOR_FAULT, FAULT_NONE, 1, 0);
  TEST_ASSERT_EQUAL(DECODE_FAULT_MISMATCH, decodeConveyor(bytes, 10, view));
  conveyorBytes(bytes, CONVEYOR_STOPPED, FAULT_LOCAL, 1, 0);
  TEST_ASSERT_EQUAL(DECODE_FAULT_MISMATCH, decodeConveyor(bytes, 10, view));
  conveyorBytes(bytes, CONVEYOR_STOPPED, 9, 1, 0);
  TEST_ASSERT_EQUAL(DECODE_BAD_FAULT, decodeConveyor(bytes, 10, view));
  conveyorBytes(bytes, 5, 0, 1, 0);
  TEST_ASSERT_EQUAL(DECODE_BAD_STATE, decodeConveyor(bytes, 10, view));
}

void test_sensing_reserved_bits() {
  SlaveView view; view.begin(SLAVE_SENSING);
  uint8_t bytes[15];
  sensingBytes(bytes, SENSING_OK, 0, 1, 0x04, 0, 0);
  TEST_ASSERT_EQUAL(DECODE_BAD_MASK, decodeSensing(bytes, 15, view));
  sensingBytes(bytes, 2, 0, 1, 0, 0, 0);
  TEST_ASSERT_EQUAL(DECODE_BAD_STATE, decodeSensing(bytes, 15, view));
}

void test_counter_decrease_without_reboot_is_invalid() {
  SlaveView view; view.begin(SLAVE_SENSING);
  uint8_t bytes[15];
  sensingBytes(bytes, SENSING_OK, 0, 1000, 0, 5, 2); view.noteReply(100, bytes, 15);
  sensingBytes(bytes, SENSING_OK, 0, 1100, 0, 4, 2);
  TEST_ASSERT_EQUAL(DECODE_COUNTER_DECREASED, view.noteReply(200, bytes, 15));
  TEST_ASSERT_EQUAL(5, view.channel1Edges);
}

void test_counter_decrease_with_reboot_is_reboot() {
  SlaveView view; view.begin(SLAVE_CONVEYOR);
  uint8_t bytes[10];
  conveyorBytes(bytes, CONVEYOR_STOPPED, 0, 9000, 500); view.noteReply(100, bytes, 10);
  conveyorBytes(bytes, CONVEYOR_STOPPED, 0, 50, 0);
  TEST_ASSERT_EQUAL(DECODE_OK, view.noteReply(200, bytes, 10));
  TEST_ASSERT_TRUE(view.rebootPending);
}

void test_freshness_decays_without_reply() {
  SlaveView view; view.begin(SLAVE_CONVEYOR);
  uint8_t bytes[10]; conveyorBytes(bytes, CONVEYOR_STOPPED, 0, 1000, 0);
  view.noteReply(1000, bytes, 10);
  view.noteNoReply(1200);
  TEST_ASSERT_TRUE(view.fresh);
  TEST_ASSERT_FALSE(view.repliedThisPoll);
  view.noteNoReply(1300);
  TEST_ASSERT_FALSE(view.fresh);
  TEST_ASSERT_EQUAL(300, view.ageMs);
}

void test_uptime_stall() {
  SlaveView view; view.begin(SLAVE_CONVEYOR);
  uint8_t bytes[10];
  for (uint32_t t = 100; t <= 300; t += 100) { conveyorBytes(bytes, CONVEYOR_STOPPED, 0, 5000, 0); view.noteReply(t, bytes, 10); }
  TEST_ASSERT_FALSE(view.uptimeStalled);
  conveyorBytes(bytes, CONVEYOR_STOPPED, 0, 5000, 0); view.noteReply(400, bytes, 10);
  TEST_ASSERT_TRUE(view.uptimeStalled);
  conveyorBytes(bytes, CONVEYOR_STOPPED, 0, 5100, 0); view.noteReply(500, bytes, 10);
  TEST_ASSERT_FALSE(view.uptimeStalled);
}

void test_uptime_rollover_is_not_reboot() {
  SlaveView view; view.begin(SLAVE_CONVEYOR);
  uint8_t bytes[10];
  conveyorBytes(bytes, CONVEYOR_STOPPED, 0, 0xFFFFFFF0, 0); view.noteReply(100, bytes, 10);
  conveyorBytes(bytes, CONVEYOR_STOPPED, 0, 0x00000010, 0); view.noteReply(200, bytes, 10);
  TEST_ASSERT_FALSE(view.rebootPending);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_conveyor_decodes_little_endian);
  RUN_TEST(test_sensing_decodes_counts);
  RUN_TEST(test_wrong_length_is_invalid_and_keeps_last_good);
  RUN_TEST(test_fault_state_needs_reason);
  RUN_TEST(test_sensing_reserved_bits);
  RUN_TEST(test_counter_decrease_without_reboot_is_invalid);
  RUN_TEST(test_counter_decrease_with_reboot_is_reboot);
  RUN_TEST(test_freshness_decays_without_reply);
  RUN_TEST(test_uptime_stall);
  RUN_TEST(test_uptime_rollover_is_not_reboot);
  return UNITY_END();
}
