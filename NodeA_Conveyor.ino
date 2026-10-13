/*
  Node A: conveyor and sweeper
  Stands in for M1 Conveyor drive and M5 Diverter.

  Board:   Arduino Uno, I2C slave at address 8 (SDA = A4, SCL = A5)
  Motors:  two 28BYJ-48 steppers on ULN2003 driver boards
           Conveyor IN1..IN4 -> D4, D5, D6, D7
           Sweeper  IN1..IN4 -> D11, D10, D9, D8
  Optional motor-power sense on D2 (see USE_POWER_SENSE below)

  No libraries to install: the steppers are driven directly (half-step),
  without blocking, so I2C stays responsive and the belt runs smoothly.

  I2C messages (agree these with Node C)
    Node C -> Node A (Wire.beginTransmission(8) ... write ...)
      'R', v   run the belt at v mm/s (v = 0 keeps the current speed)
      'S'      stop the belt
      'E'      eject the item that has just cleared Node B's beam
    Node A -> Node C (Wire.requestFrom(8, 7))
      bytes 0-1  belt speed, 0.1 mm/s units (uint16, low byte first)
      bytes 2-5  belt position, 0.1 mm units (uint32, low byte first)
      byte  6    status bits: bit0 running (commanded), bit1 sweeping,
                              bit2 motor power present, bit3 time-based eject mode
    Node C should watch bit2 to detect the E-stop, NOT the speed:
    the belt legitimately pauses at zero speed while the paddle sweeps.

  Serial monitor (115200 baud) for testing Node A on its own:
    r run      s stop      e eject (put an item at Node B's beam position first)
    m switch eject mode (step-counted / time-based)
    + / -      speed up / slow down by 2 mm/s
    c calibrate: runs exactly 4096 half-steps, then measure how far the belt moved
*/

#include <Wire.h>

// ---------------- Settings to measure and tune ----------------
const float MM_PER_STEP        = 0.0307;  // belt travel per half-step. Use 'c', then travel / 4096
const float BEAM_TO_PADDLE_MM  = 150.0;   // distance from Node B's IR beam to the paddle, along the belt
const float DEFAULT_SPEED_MM_S = 15.0;    // belt speed at start-up
const float NOMINAL_SPEED_MM_S = 15.0;    // the speed the time-based eject mode assumes
const int   SWEEP_STEPS        = 1024;    // half-steps for one sweep out (1024 is about 90 degrees)
const float SWEEP_RATE         = 800.0;   // sweeper half-steps per second
const unsigned long SWEEP_DWELL_MS = 150; // pause at full sweep before returning
const int8_t CONVEYOR_DIR      = 1;       // change to -1 if the belt runs backwards
const int8_t SWEEP_DIR         = 1;       // change to -1 if the paddle sweeps the wrong way
const bool  PAUSE_BELT_WHILE_SWEEPING = true;
const float MAX_STEP_RATE      = 1000.0;  // a 28BYJ-48 at 5 V stalls above roughly this

// Set to 1 once Node C's relay is in the motor supply.
// Wiring: motor 5 V after the relay -> 1 kOhm -> D2, and 10 kOhm from D2 to GND.
#define USE_POWER_SENSE 0
const uint8_t POWER_SENSE_PIN  = 2;

const uint8_t I2C_ADDRESS = 8;

// ---------------- Stepper driver (half-step, IN1..IN4 order) ----------------
const uint8_t HALF_STEP[8][4] = {
  {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
  {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

struct Stepper28 {
  uint8_t pin[4];
  uint8_t phase;

  void begin() {
    for (uint8_t i = 0; i < 4; i++) pinMode(pin[i], OUTPUT);
    release();
  }
  void step(int8_t dir) {
    phase = (dir > 0) ? ((phase + 1) & 7) : ((phase + 7) & 7);
    for (uint8_t i = 0; i < 4; i++) digitalWrite(pin[i], HALF_STEP[phase][i]);
  }
  void release() {  // de-energise the coils (saves current and heat)
    for (uint8_t i = 0; i < 4; i++) digitalWrite(pin[i], LOW);
  }
};

Stepper28 conveyor = {{4, 5, 6, 7}, 0};
Stepper28 sweeper  = {{11, 10, 9, 8}, 0};

// ---------------- Belt state ----------------
float speedMmS = DEFAULT_SPEED_MM_S;
unsigned long stepIntervalUs = 2000;
bool running = false;         // commanded to run
bool powerOk = true;          // motor supply present (always true without power sense)
bool beltEnergised = false;
long beltSteps = 0;           // half-steps travelled since start-up = belt position
unsigned long lastBeltStepUs = 0;
long calibrateRemaining = 0;  // > 0 while a calibration run is in progress

long stepsAtLastSample = 0;
unsigned long lastSampleMs = 0;
float measuredSpeed = 0.0;    // from steps actually issued, so 0 when paused or stopped

// ---------------- Eject queue ----------------
struct Eject {
  bool timeBased;
  long target;                // belt step count, or millis() time
};
const uint8_t QUEUE_SIZE = 4;
Eject queue[QUEUE_SIZE];
uint8_t qHead = 0;
uint8_t qCount = 0;
bool timeBasedMode = false;

// ---------------- Sweeper state ----------------
enum SweepState : uint8_t { SW_IDLE, SW_OUT, SW_DWELL, SW_BACK };
SweepState sweepState = SW_IDLE;
int sweepPos = 0;
unsigned long lastSweepStepUs = 0;
unsigned long dwellStartMs = 0;
const unsigned long SWEEP_INTERVAL_US = (unsigned long)(1000000.0 / SWEEP_RATE);

// ---------------- I2C ----------------
volatile char pendingCmd = 0;
volatile uint8_t pendingArg = 0;
volatile uint8_t pendingEjects = 0;
uint8_t txBuf[7];

unsigned long lastTxMs = 0;
unsigned long lastPrintMs = 0;

// ---------------- Helpers ----------------
void setSpeed(float v) {
  float maxSpeed = MAX_STEP_RATE * MM_PER_STEP;
  if (v > maxSpeed) {
    v = maxSpeed;
    Serial.println(F("Speed limited to the stepper maximum"));
  }
  if (v < 1.0) v = 1.0;
  speedMmS = v;
  stepIntervalUs = (unsigned long)(1000000.0 * MM_PER_STEP / v);
}

void queueEject() {
  if (qCount == QUEUE_SIZE) {
    Serial.println(F("Eject queue full, item dropped"));
    return;
  }
  Eject &e = queue[(qHead + qCount) % QUEUE_SIZE];
  e.timeBased = timeBasedMode;
  if (timeBasedMode) {
    e.target = (long)(millis() + (unsigned long)(BEAM_TO_PADDLE_MM / NOMINAL_SPEED_MM_S * 1000.0));
  } else {
    e.target = beltSteps + (long)(BEAM_TO_PADDLE_MM / MM_PER_STEP);
  }
  qCount++;
  Serial.print(F("Eject queued ("));
  Serial.print(timeBasedMode ? F("time-based") : F("step-counted"));
  Serial.print(F(") at belt position "));
  Serial.print(beltSteps * MM_PER_STEP, 1);
  Serial.println(F(" mm"));
}

bool headReached() {
  if (qCount == 0) return false;
  const Eject &e = queue[qHead];
  if (e.timeBased) return (long)(millis() - (unsigned long)e.target) >= 0;
  return beltSteps >= e.target;
}

void handleCommand(char c, uint8_t arg) {
  switch (c) {
    case 'R': case 'r':
      if (arg > 0) setSpeed(arg);
      if (powerOk) {
        running = true;
        lastBeltStepUs = micros();
      } else {
        Serial.println(F("Run refused: no motor power"));
      }
      break;
    case 'S': case 's':
      running = false;
      calibrateRemaining = 0;
      break;
    case 'E': case 'e':
      queueEject();
      break;
    case 'm':
      timeBasedMode = !timeBasedMode;
      Serial.print(F("Eject mode: "));
      Serial.println(timeBasedMode ? F("time-based") : F("step-counted"));
      break;
    case '+':
      setSpeed(speedMmS + 2.0);
      break;
    case '-':
      setSpeed(speedMmS - 2.0);
      break;
    case 'c':
      calibrateRemaining = 4096;
      if (powerOk) {
        running = true;
        lastBeltStepUs = micros();
      }
      Serial.println(F("Calibrating: mark the belt, it will move 4096 half-steps"));
      break;
    default:
      break;
  }
}

void updateTx() {
  uint16_t v = (uint16_t)(measuredSpeed * 10.0 + 0.5);
  uint32_t p = (uint32_t)(beltSteps * MM_PER_STEP * 10.0);
  uint8_t status = (running ? 0x01 : 0) | (sweepState != SW_IDLE ? 0x02 : 0) |
                   (powerOk ? 0x04 : 0) | (timeBasedMode ? 0x08 : 0);
  noInterrupts();
  txBuf[0] = v & 0xFF;
  txBuf[1] = (v >> 8) & 0xFF;
  txBuf[2] = p & 0xFF;
  txBuf[3] = (p >> 8) & 0xFF;
  txBuf[4] = (p >> 16) & 0xFF;
  txBuf[5] = (p >> 24) & 0xFF;
  txBuf[6] = status;
  interrupts();
}

// ---------------- I2C callbacks (run in interrupt context: keep short) ----------------
void onI2CReceive(int n) {
  if (n < 1) return;
  char c = (char)Wire.read();
  uint8_t arg = Wire.available() ? (uint8_t)Wire.read() : 0;
  while (Wire.available()) Wire.read();
  if (c == 'E') {
    if (pendingEjects < 255) pendingEjects++;
  } else {
    pendingCmd = c;
    pendingArg = arg;
  }
}

void onI2CRequest() {
  Wire.write(txBuf, sizeof(txBuf));
}

// ---------------- Setup and loop ----------------
void setup() {
  Serial.begin(115200);
  conveyor.begin();
  sweeper.begin();
#if USE_POWER_SENSE
  pinMode(POWER_SENSE_PIN, INPUT);
#endif
  setSpeed(DEFAULT_SPEED_MM_S);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  updateTx();
  Serial.println(F("Node A ready. r run, s stop, e eject, m mode, +/- speed, c calibrate"));
}

void loop() {
  // 1. Commands from Node C (I2C) and from the serial monitor
  noInterrupts();
  char c = pendingCmd;
  uint8_t a = pendingArg;
  pendingCmd = 0;
  uint8_t ej = pendingEjects;
  pendingEjects = 0;
  interrupts();
  if (c) handleCommand(c, a);
  while (ej--) queueEject();

  while (Serial.available()) {
    char s = (char)Serial.read();
    if (s == '\n' || s == '\r' || s == ' ') continue;
    handleCommand(s, 0);
  }

  // 2. Motor power after Node C's relay (E-stop). A stepper keeps "counting"
  //    even with its power cut, so stop issuing steps when power is lost.
#if USE_POWER_SENSE
  bool p = (digitalRead(POWER_SENSE_PIN) == HIGH);
  if (!p && powerOk) {
    running = false;
    calibrateRemaining = 0;
    Serial.println(F("Motor power lost: stopped. Needs a new run command."));
  }
  powerOk = p;
#endif

  unsigned long nowUs = micros();

  // 3. Belt
  bool beltMay = running && powerOk &&
                 !(PAUSE_BELT_WHILE_SWEEPING && sweepState != SW_IDLE);
  if (beltMay) {
    if (nowUs - lastBeltStepUs >= stepIntervalUs) {
      lastBeltStepUs += stepIntervalUs;
      if (nowUs - lastBeltStepUs >= stepIntervalUs) lastBeltStepUs = nowUs;  // fell behind: resync, no burst
      conveyor.step(CONVEYOR_DIR);
      beltEnergised = true;
      beltSteps++;
      if (calibrateRemaining > 0) {
        calibrateRemaining--;
        if (calibrateRemaining == 0) {
          running = false;
          Serial.println(F("Calibration done. Measure the travel: MM_PER_STEP = travel_mm / 4096"));
        }
      }
    }
  } else {
    lastBeltStepUs = nowUs;
    if (beltEnergised) {
      conveyor.release();
      beltEnergised = false;
    }
  }

  // 4. Start a sweep when the next flagged item reaches the paddle
  if (sweepState == SW_IDLE && headReached()) {
    qHead = (qHead + 1) % QUEUE_SIZE;
    qCount--;
    sweepState = SW_OUT;
    lastSweepStepUs = nowUs;
    Serial.print(F("Sweep at belt position "));
    Serial.print(beltSteps * MM_PER_STEP, 1);
    Serial.println(F(" mm"));
  }

  // 5. Sweeper motion (stops with the line)
  bool sweepMay = running && powerOk;
  if (sweepState != SW_IDLE && sweepMay) {
    if (sweepState == SW_DWELL) {
      if (millis() - dwellStartMs >= SWEEP_DWELL_MS) {
        sweepState = SW_BACK;
        lastSweepStepUs = nowUs;
      }
    } else if (nowUs - lastSweepStepUs >= SWEEP_INTERVAL_US) {
      lastSweepStepUs = nowUs;
      if (sweepState == SW_OUT) {
        sweeper.step(SWEEP_DIR);
        sweepPos++;
        if (sweepPos >= SWEEP_STEPS) {
          sweepState = SW_DWELL;
          dwellStartMs = millis();
        }
      } else {
        sweeper.step(-SWEEP_DIR);
        sweepPos--;
        if (sweepPos <= 0) {
          sweepState = SW_IDLE;
          sweeper.release();
        }
      }
    }
  }

  // 6. Speed measurement, I2C reply buffer, status print
  unsigned long nowMs = millis();
  if (nowMs - lastSampleMs >= 250) {
    measuredSpeed = (beltSteps - stepsAtLastSample) * MM_PER_STEP * 1000.0 / (nowMs - lastSampleMs);
    stepsAtLastSample = beltSteps;
    lastSampleMs = nowMs;
  }
  if (nowMs - lastTxMs >= 20) {
    lastTxMs = nowMs;
    updateTx();
  }
  if (nowMs - lastPrintMs >= 1000) {
    lastPrintMs = nowMs;
    Serial.print(F("pos "));
    Serial.print(beltSteps * MM_PER_STEP, 1);
    Serial.print(F(" mm  speed "));
    Serial.print(measuredSpeed, 1);
    Serial.print(F(" mm/s  run "));
    Serial.print(running);
    Serial.print(F("  power "));
    Serial.print(powerOk);
    Serial.print(F("  queue "));
    Serial.println(qCount);
  }
}
