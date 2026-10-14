#include <Arduino.h>
#include <Wire.h>
#include "analog_channel.h"
#include "sensing_logic.h"
#include "slave_link.h"

// Set to the assembled wiring. A4 and A5 are the I2C bus.
// Channel 1 is the light sensor, about 30 mm ahead of the Hall; channel 2 is the Hall. One Hall is fitted.
static const uint8_t HALL_PIN = A0;
static const uint8_t LIGHT_SENSOR_PIN = A2;

// Starting values only. Measure with the calibrate environment; see README.
#define SENSING_DEBUG 1

static const uint16_t DEBOUNCE_MS = 20;
static const uint16_t SAMPLE_INTERVAL_MS = 2;

static const AnalogChannel::Config HALL_CONFIG = {128, 3, 12, 6, 400, 624, 20};
static const AnalogChannel::Config LIGHT_SENSOR_CONFIG = {128, 3, 60, 30, 30, 990, 40};

static AnalogChannel hall;
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
  hall.begin(HALL_CONFIG);
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
    hall.sample(readAdc(HALL_PIN));
    lightSensor.sample(readAdc(LIGHT_SENSOR_PIN));
    logic.sample(now, lightSensor.active(), hall.active());
    logic.setLevels(lightSensor.deviation(), hall.deviation());
    if (hall.fault() || lightSensor.fault()) logic.raiseLocalFault();
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
    Serial.print(F("light="));
    Serial.print(lightSensor.deviation());
    Serial.print(F(" hall="));
    Serial.print(hall.deviation());
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
