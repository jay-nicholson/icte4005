// Bench tool for the sensing node. Build: pio run -d sensing -e calibrate -t upload. Usage in README.
#include <Arduino.h>
#include "analog_channel.h"

static const uint8_t HALL1_PIN = A0;
static const uint8_t HALL2_PIN = A1;
static const uint8_t LIGHT_SENSOR_PIN = A2;

// Thresholds unreachable and health limits wide open: this build only measures.
static const AnalogChannel::Config MEASURE_ONLY = {128, 3, 1023, 1023, 0, 1023, 1023};

static AnalogChannel hall1;
static AnalogChannel hall2;
static AnalogChannel lightSensor;
static uint16_t peakHall1 = 0;
static uint16_t peakHall2 = 0;
static uint16_t peakLight = 0;
static uint32_t lastSampleMs = 0;
static uint32_t lastPrintMs = 0;

static uint16_t readAdc(uint8_t pin) {
  analogRead(pin);
  return analogRead(pin);
}

static void zero() {
  hall1.begin(MEASURE_ONLY);
  hall2.begin(MEASURE_ONLY);
  lightSensor.begin(MEASURE_ONLY);
  peakHall1 = peakHall2 = peakLight = 0;
}

static void keepPeak(uint16_t& peak, uint16_t value) {
  if (value > peak) peak = value;
}

static void printValue(const __FlashStringHelper* label, uint16_t value) {
  Serial.print(label);
  Serial.print(value);
  Serial.print(' ');
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("calibrate: zeroing, keep the belt clear"));
  zero();
}

void loop() {
  uint32_t now = millis();
  if (now - lastSampleMs >= 2) {
    lastSampleMs = now;
    hall1.sample(readAdc(HALL1_PIN));
    hall2.sample(readAdc(HALL2_PIN));
    lightSensor.sample(readAdc(LIGHT_SENSOR_PIN));
    if (hall1.ready()) {
      keepPeak(peakHall1, hall1.deviation());
      keepPeak(peakHall2, hall2.deviation());
      keepPeak(peakLight, lightSensor.deviation());
    }
  }
  while (Serial.available()) {
    char key = Serial.read();
    if (key == 'r') peakHall1 = peakHall2 = peakLight = 0;
    if (key == 'z') zero();
  }
  if (now - lastPrintMs >= 100 && hall1.ready()) {
    lastPrintMs = now;
    printValue(F("hall1:"), hall1.deviation());
    printValue(F("hall2:"), hall2.deviation());
    printValue(F("light:"), lightSensor.deviation());
    printValue(F("peakHall1:"), peakHall1);
    printValue(F("peakHall2:"), peakHall2);
    printValue(F("peakLight:"), peakLight);
    printValue(F("baseline1:"), hall1.baseline());
    printValue(F("baseline2:"), hall2.baseline());
    printValue(F("baselineLight:"), lightSensor.baseline());
    Serial.println();
  }
}
