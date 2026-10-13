#include <Arduino.h>
#include "analog_channel.h"

// Calibration helper for the sensing node. Build with:  pio run -d sensing -e calibrate -t upload
// (or open calibrate_standalone/calibrate_standalone.ino in the Arduino IDE).
//
// It reuses the real AnalogChannel so the numbers you read here are exactly what the firmware sees.
// Output is `label:value` pairs at 10 Hz: readable in the Serial Monitor, graphable in the Serial
// Plotter. Serial 9600 baud. Keep the belt clear and magnets away from the sensors while it boots.
//
//   H1 H2 LDR    live deviation from the boot baseline, in ADC counts
//   pk1 pk2 pkL  the largest deviation since the last 'r'
//   b1 b2 bL     the baselines captured at boot
//
// Keys:  r = reset the peak values   z = re-zero the baselines (belt clear first)

static const uint8_t HALL1_PIN = A0;
static const uint8_t HALL2_PIN = A1;
static const uint8_t LDR_PIN = A2;

// Thresholds are set unreachable and the health limits wide open: this sketch only measures.
static const AnalogChannel::Config MEASURE_ONLY = {128, 3, 1023, 1023, 0, 1023, 1023};

static AnalogChannel hall1;
static AnalogChannel hall2;
static AnalogChannel ldr;
static uint16_t peak1 = 0, peak2 = 0, peakL = 0;
static uint32_t lastSampleMs = 0;
static uint32_t lastPrintMs = 0;

static uint16_t readAdc(uint8_t pin) {
  analogRead(pin);
  return analogRead(pin);
}

static void zero() {
  hall1.begin(MEASURE_ONLY);
  hall2.begin(MEASURE_ONLY);
  ldr.begin(MEASURE_ONLY);
  peak1 = peak2 = peakL = 0;
}

static void keep(uint16_t& peak, uint16_t value) {
  if (value > peak) peak = value;
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("Calibrate: zeroing, keep the belt clear..."));
  zero();
}

void loop() {
  uint32_t now = millis();
  if (now - lastSampleMs >= 2) {
    lastSampleMs = now;
    hall1.sample(readAdc(HALL1_PIN));
    hall2.sample(readAdc(HALL2_PIN));
    ldr.sample(readAdc(LDR_PIN));
    if (hall1.ready()) {
      keep(peak1, hall1.deviation());
      keep(peak2, hall2.deviation());
      keep(peakL, ldr.deviation());
    }
  }
  while (Serial.available()) {
    char key = Serial.read();
    if (key == 'r') peak1 = peak2 = peakL = 0;
    if (key == 'z') zero();
  }
  if (now - lastPrintMs >= 100 && hall1.ready()) {
    lastPrintMs = now;
    Serial.print(F("H1:")); Serial.print(hall1.deviation());
    Serial.print(F(" H2:")); Serial.print(hall2.deviation());
    Serial.print(F(" LDR:")); Serial.print(ldr.deviation());
    Serial.print(F(" pk1:")); Serial.print(peak1);
    Serial.print(F(" pk2:")); Serial.print(peak2);
    Serial.print(F(" pkL:")); Serial.print(peakL);
    Serial.print(F(" b1:")); Serial.print(hall1.baseline());
    Serial.print(F(" b2:")); Serial.print(hall2.baseline());
    Serial.print(F(" bL:")); Serial.println(ldr.baseline());
  }
}
