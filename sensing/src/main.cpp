#include <Arduino.h>
#include <Wire.h>
#include "analog_channel.h"
#include "sensing_logic.h"
#include "slave_link.h"

// Set to the assembled wiring. A4 and A5 are the I2C bus.
// Bench only, not committed: channel 2 reads the same sensor as channel 1 until the second Hall is fitted.
static const uint8_t HALL1_PIN = A0;
static const uint8_t HALL2_PIN = A0;
#define HALL2_FITTED 1
static const uint8_t LIGHT_SENSOR_PIN = A2;

// Starting values only. Measure with the calibrate environment; see README.
static const bool USE_LIGHT_GATE = false;
#define SENSING_DEBUG 1

static const uint16_t DEBOUNCE_MS = 20;
static const uint16_t SAMPLE_INTERVAL_MS = 2;

static const AnalogChannel::Config HALL_CONFIG = {128, 3, 12, 6, 400, 624, 20};
static const AnalogChannel::Config LIGHT_SENSOR_CONFIG = {128, 3, 60, 30, 30, 990, 40};

static AnalogChannel hall1;
static AnalogChannel hall2;
static AnalogChannel lightSensor;
static SensingLogic logic;
static SlaveLink link;
static uint32_t lastSampleMs = 0;
static uint32_t lastPublishMs = 0;
#if SENSING_DEBUG
static uint32_t lastDebugMs = 0;
#endif

// The Uno shares one ADC between pins; the first reading after a pin change carries charge from the last pin.
static uint16_t readAdc(uint8_t pin) {
  analogRead(pin);
  return analogRead(pin);
}

void setup() {
  Serial.begin(9600);
  hall1.begin(HALL_CONFIG);
  hall2.begin(HALL_CONFIG);
  lightSensor.begin(LIGHT_SENSOR_CONFIG);
  logic.begin(DEBOUNCE_MS);
  link.begin(SENSING_ADDRESS, SENSING_REPLY_LENGTH);
  Serial.print(F("SENSING contract v"));
  Serial.print(CONTRACT_VERSION);
  Serial.println(F(" hall front end"));
}

void loop() {
  uint32_t now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    hall1.sample(readAdc(HALL1_PIN));
    if (HALL2_FITTED) hall2.sample(readAdc(HALL2_PIN));
    lightSensor.sample(readAdc(LIGHT_SENSOR_PIN));
    bool channel1 = hall1.active();
    bool channel2 = HALL2_FITTED && fusedClosed(hall2.active(), lightSensor.active(), USE_LIGHT_GATE);
    logic.sample(now, channel1, channel2);
    logic.setLevels(hall1.deviation(), hall2.deviation());
    bool hallFault = hall1.fault() || (HALL2_FITTED && hall2.fault());
    if (hallFault || (USE_LIGHT_GATE && lightSensor.fault())) logic.raiseLocalFault();
  }
  logic.serviceWatchdog(link.contactLost(now));
  if (now - lastPublishMs >= SNAPSHOT_REFRESH_MS) {
    lastPublishMs = now;
    uint8_t bytes[SENSING_REPLY_LENGTH];
    logic.encode(bytes, now);
    link.publish(bytes);
  }
#if SENSING_DEBUG
  if (now - lastDebugMs >= 200) {
    lastDebugMs = now;
    Serial.print(F("hall1="));
    Serial.print(hall1.deviation());
    Serial.print(F(" hall2="));
    Serial.print(hall2.deviation());
    Serial.print(F(" light="));
    Serial.print(lightSensor.deviation());
    Serial.print(F(" live="));
    Serial.print(logic.live());
    Serial.print(F(" edges="));
    Serial.print(logic.channel1Edges());
    Serial.print('/');
    Serial.print(logic.channel2Edges());
    Serial.print(F(" state="));
    Serial.print(sensingStateName(logic.state()));
    Serial.print(F(" fault="));
    Serial.println(faultCodeName(logic.fault()));
  }
#endif
}
