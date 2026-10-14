#include <Arduino.h>
#include <Wire.h>
#include "conveyor_logic.h"
#include "slave_link.h"

// Motor section: 28BYJ-48 on a ULN2003, half-stepped directly, no library. Set to the assembled wiring.
static const uint8_t BELT_MOTOR_PINS[4] = {4, 5, 6, 7};
static const float MM_PER_HALF_STEP = 0.0307f;
static const float BELT_SPEED_MM_S = 15.0f;
static const float MAX_HALF_STEPS_PER_SECOND = 1000.0f;
static const uint32_t CALIBRATION_HALF_STEPS = 4096;
static const int8_t FORWARD_DIRECTION = 1;

#ifndef EJECTOR_ENABLED
#define EJECTOR_ENABLED 1
#endif

static const uint8_t HALF_STEP_SEQUENCE[8][4] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}};

struct HalfStepMotor {
  const uint8_t* pins;
  uint8_t phase;

  void begin() {
    for (uint8_t i = 0; i < 4; i++) pinMode(pins[i], OUTPUT);
    release();
  }
  void step(int8_t direction) {
    phase = (direction > 0) ? ((phase + 1) & 7) : ((phase + 7) & 7);
    for (uint8_t i = 0; i < 4; i++) digitalWrite(pins[i], HALF_STEP_SEQUENCE[phase][i]);
  }
  void release() {
    for (uint8_t i = 0; i < 4; i++) digitalWrite(pins[i], LOW);
  }
};

static HalfStepMotor beltMotor = {BELT_MOTOR_PINS, 0};
static uint32_t stepIntervalMicros;
static uint32_t lastStepMicros = 0;
static bool beltEnergised = false;
static uint32_t calibrationRemaining = 0;

static uint32_t stepIntervalFor(float speedMmS) {
  float maxSpeedMmS = MAX_HALF_STEPS_PER_SECOND * MM_PER_HALF_STEP;
  if (speedMmS > maxSpeedMmS) speedMmS = maxSpeedMmS;
  return (uint32_t)(1000000.0f * MM_PER_HALF_STEP / speedMmS);
}

static void motorService(MotorRequest request, ConveyorLogic& logic) {
  uint32_t now = micros();
  if (request == MOTOR_OFF) {
    lastStepMicros = now;
    calibrationRemaining = 0;
    if (beltEnergised) {
      beltMotor.release();
      beltEnergised = false;
    }
    return;
  }
  if (now - lastStepMicros < stepIntervalMicros) return;
  lastStepMicros = now;
  beltMotor.step(request == MOTOR_FORWARD ? FORWARD_DIRECTION : -FORWARD_DIRECTION);
  beltEnergised = true;
  logic.countStep();
  if (calibrationRemaining > 0 && --calibrationRemaining == 0) {
    logic.command(COMMAND_STOP);
    Serial.println(F("calibration done: MM_PER_HALF_STEP = travel_mm / 4096"));
  }
}

#if EJECTOR_ENABLED
static const uint8_t EJECTOR_MOTOR_PINS[4] = {11, 10, 9, 8};
static const int SWEEP_HALF_STEPS = 1024;
static const float SWEEP_HALF_STEPS_PER_SECOND = 800.0f;
static const uint32_t SWEEP_DWELL_MS = 150;
static const int8_t SWEEP_DIRECTION = 1;
static const uint32_t SWEEP_INTERVAL_MICROS = (uint32_t)(1000000.0f / SWEEP_HALF_STEPS_PER_SECOND);

enum SweepState { SWEEP_IDLE, SWEEP_OUT, SWEEP_DWELL, SWEEP_BACK };

static HalfStepMotor ejectorMotor = {EJECTOR_MOTOR_PINS, 0};
static SweepState sweepState = SWEEP_IDLE;
static int sweepPosition = 0;
static uint32_t lastSweepStepMicros = 0;
static uint32_t dwellStartMs = 0;

static void startSweep() {
  if (sweepState != SWEEP_IDLE) return;
  sweepState = SWEEP_OUT;
  lastSweepStepMicros = micros();
}

static void ejectorService(ConveyorLogic& logic) {
  if (logic.sweeping() && sweepState == SWEEP_IDLE && sweepPosition == 0) startSweep();
  if (sweepState == SWEEP_IDLE) return;
  uint32_t now = micros();
  if (sweepState == SWEEP_DWELL) {
    if (millis() - dwellStartMs >= SWEEP_DWELL_MS) {
      sweepState = SWEEP_BACK;
      lastSweepStepMicros = now;
    }
    return;
  }
  if (now - lastSweepStepMicros < SWEEP_INTERVAL_MICROS) return;
  lastSweepStepMicros = now;
  if (sweepState == SWEEP_OUT) {
    ejectorMotor.step(SWEEP_DIRECTION);
    if (++sweepPosition >= SWEEP_HALF_STEPS) {
      sweepState = SWEEP_DWELL;
      dwellStartMs = millis();
    }
  } else {
    ejectorMotor.step(-SWEEP_DIRECTION);
    if (--sweepPosition <= 0) {
      sweepState = SWEEP_IDLE;
      ejectorMotor.release();
      logic.sweepComplete();
    }
  }
}
#endif

static ConveyorLogic logic;
static SlaveLink link;
static uint32_t lastPublishMs = 0;
static uint32_t lastPrintMs = 0;

// Bench keys pass the same guards as master commands. Do not type them during a cycle.
static void serviceBenchSerial() {
  while (Serial.available()) {
    char key = (char)Serial.read();
    if (key == COMMAND_GO || key == COMMAND_BACK || key == COMMAND_STOP || key == COMMAND_EJECT) {
      Serial.print(F("serial key: "));
      Serial.println(key);
      logic.command(key);
    } else if (key == 'c') {
      calibrationRemaining = CALIBRATION_HALF_STEPS;
      logic.command(COMMAND_GO);
      Serial.println(F("calibrating: mark the belt, it moves 4096 half-steps"));
    }
  }
}

void setup() {
  Serial.begin(9600);
  logic.begin();
  beltMotor.begin();
#if EJECTOR_ENABLED
  ejectorMotor.begin();
#endif
  stepIntervalMicros = stepIntervalFor(BELT_SPEED_MM_S);
  link.begin(CONVEYOR_ADDRESS, CONVEYOR_REPLY_LENGTH);
  Serial.print(F("CONVEYOR contract v"));
  Serial.print(CONTRACT_VERSION);
  Serial.println(F("  bench keys: g b s e c"));
}

void loop() {
  uint32_t now = millis();
  char command = link.takeCommand();
  if (command) {
    Serial.print(F("bus command: "));
    Serial.println(command);
    logic.command(command);
  }
  serviceBenchSerial();
  logic.serviceWatchdog(link.contactLost(now));
  motorService(logic.motor(), logic);
#if EJECTOR_ENABLED
  ejectorService(logic);
#else
  if (logic.sweeping()) logic.sweepComplete();
#endif
  if (now - lastPublishMs >= SNAPSHOT_REFRESH_MS) {
    lastPublishMs = now;
    uint8_t bytes[CONVEYOR_REPLY_LENGTH];
    logic.encode(bytes, now);
    link.publish(bytes);
  }
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    Serial.print(conveyorStateName(logic.state()));
    Serial.print(F(" steps="));
    Serial.print(logic.steps());
    Serial.print(F(" pos="));
    Serial.print(logic.steps() * MM_PER_HALF_STEP, 1);
    Serial.println(F(" mm"));
  }
}
