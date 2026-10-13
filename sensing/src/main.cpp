#include <Arduino.h>
#include <Wire.h>
#include "analog_channel.h"
#include "sensing_logic.h"
#include "slave_link.h"

// Sensing node: two linear Hall sensors (replacing the reeds 1:1) and an optional LDR presence gate.
// The I2C contract is unchanged. To the master this board still reports two debounced channels and
// their closing-edge counts; it neither knows nor cares what produced them.
//
// Channel 1: puck arrival at the load/discharge end   (Hall 1)
// Channel 2: magnetic detection at the stop point     (Hall 2, optionally gated by the LDR)

// ---- Wiring. A4/A5 are the I2C bus, so the analog inputs start at A0 and avoid them. ----
static const uint8_t HALL1_PIN = A0;
static const uint8_t HALL2_PIN = A1;
static const uint8_t LDR_PIN = A2;
static const uint8_t LED_GREEN_PIN = 5;
static const uint8_t LED_AMBER_PIN = 6;
static const uint8_t LED_RED_PIN = 7;

// ---- Tuning. STARTING VALUES ONLY: replace them with numbers from the calibrate sketch. ----
// Fusion gate: set true only after the LDR has been bench-tested in the demo lighting.
static const bool USE_LDR_GATE = false;
// Print live readings on Serial at 9600 baud (5 Hz). Harmless to the I2C link; set 0 to silence.
#define SENSING_DEBUG 1

static const uint16_t DEBOUNCE_MS = 20;
static const uint16_t SAMPLE_INTERVAL_MS = 2;

// UGN3503UA rests near half supply (~512 counts) and moves about 1.3 mV/gauss (~0.27 counts/G).
static const AnalogChannel::Config HALL_CONFIG = {
    128,  // zeroSamples: about 0.26 s of readings at boot. Keep the belt clear while it runs.
    3,    // filterShift: time constant of about 8 samples (16 ms)
    12,   // onDelta: counts of deviation that turn the channel on
    6,    // offDelta: counts below which it turns off
    400,  // baselineMin
    624,  // baselineMax
    20};  // maxZeroSpread

// The LDR module's resting level depends on room light, so its limits are wide.
static const AnalogChannel::Config LDR_CONFIG = {128, 3, 60, 30, 30, 990, 40};

static AnalogChannel hall1;
static AnalogChannel hall2;
static AnalogChannel ldr;
static SensingLogic logic;
static SlaveLink link;
static uint32_t lastSampleMs = 0;
static uint32_t lastPublishMs = 0;
#if SENSING_DEBUG
static uint32_t lastDebugMs = 0;
#endif

// The Uno's single ADC is shared between pins. The first reading after switching pins can carry
// charge from the previous one, so read twice and keep the second.
static uint16_t readAdc(uint8_t pin) {
  analogRead(pin);
  return analogRead(pin);
}

static void setLeds(bool green, bool amber, bool red) {
  digitalWrite(LED_GREEN_PIN, green);
  digitalWrite(LED_AMBER_PIN, amber);
  digitalWrite(LED_RED_PIN, red);
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_AMBER_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  // Lamp test: all three on briefly so a dead LED or wrong pin shows up before the demo.
  setLeds(true, true, true);
  delay(300);
  setLeds(false, false, false);

  hall1.begin(HALL_CONFIG);
  hall2.begin(HALL_CONFIG);
  ldr.begin(LDR_CONFIG);
  logic.begin(DEBOUNCE_MS);
  link.begin(SENSING_ADDRESS, SENSING_REPLY_LENGTH);
  Serial.print(F("SENSING contract v"));
  Serial.print(CONTRACT_VERSION);
  Serial.println(F(" (Hall + LDR front end)"));
}

void loop() {
  uint32_t now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    hall1.sample(readAdc(HALL1_PIN));
    hall2.sample(readAdc(HALL2_PIN));
    ldr.sample(readAdc(LDR_PIN));
    bool channel1 = hall1.active();
    bool channel2 = fusedClosed(hall2.active(), ldr.active(), USE_LDR_GATE);
    logic.sample(now, channel1, channel2);
    // A broken or shorted sensor is a LOCAL fault. The LDR only matters while the gate depends on it.
    if (hall1.fault() || hall2.fault() || (USE_LDR_GATE && ldr.fault())) logic.raiseLocalFault();
  }
  logic.serviceWatchdog(link.contactLost(now));
  setLeds(logic.greenOn(), logic.amberOn(), logic.redOn());
  if (now - lastPublishMs >= SNAPSHOT_REFRESH_MS) {
    lastPublishMs = now;
    uint8_t bytes[SENSING_REPLY_LENGTH];
    logic.encode(bytes, now);
    link.publish(bytes);
  }
#if SENSING_DEBUG
  if (now - lastDebugMs >= 200) {
    lastDebugMs = now;
    Serial.print(F("dev H1="));
    Serial.print(hall1.deviation());
    Serial.print(F(" H2="));
    Serial.print(hall2.deviation());
    Serial.print(F(" LDR="));
    Serial.print(ldr.deviation());
    Serial.print(F(" | live="));
    Serial.print(logic.live());
    Serial.print(F(" edges="));
    Serial.print(logic.reed1Edges());
    Serial.print('/');
    Serial.print(logic.reed2Edges());
    Serial.print(F(" state="));
    Serial.print(sensingStateName(logic.state()));
    Serial.print(F(" fault="));
    Serial.println(faultCodeName(logic.fault()));
  }
#endif
}
