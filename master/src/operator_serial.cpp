#include "operator_serial.h"
#include <Arduino.h>
#include <string.h>
#include "config.h"

static char line[24];
static uint8_t lineLength = 0;

void serialBegin(uint32_t baud) {
  Serial.begin(baud);
  lineLength = 0;
}

static bool equals(const char* text, const char* expected) { return strcmp(text, expected) == 0; }

static SerialAction parseLine(char* text, OperatorCommand& operatorCommand) {
  for (char* character = text; *character; character++) *character = (char)tolower(*character);
  operatorCommand.kind = OPERATOR_NONE;
  if (equals(text, "start")) operatorCommand.kind = OPERATOR_START;
  else if (equals(text, "stop")) operatorCommand.kind = OPERATOR_STOP;
  else if (equals(text, "resume")) operatorCommand.kind = OPERATOR_RESUME;
  else if (equals(text, "ack")) operatorCommand.kind = OPERATOR_ACK;
  else if (equals(text, "bench")) operatorCommand.kind = OPERATOR_BENCH;
  else if (equals(text, "status")) return SERIAL_STATUS;
  else if (equals(text, "help") || equals(text, "?")) return SERIAL_HELP;
  else if (strncmp(text, "send ", 5) == 0 && strlen(text) == 8 && text[6] == ' ') {
    if (text[5] == 'c') operatorCommand.address = CONVEYOR_ADDRESS;
    else if (text[5] == 's') operatorCommand.address = SENSING_ADDRESS;
    else return SERIAL_UNKNOWN;
    operatorCommand.kind = OPERATOR_SEND;
    operatorCommand.command = text[7];
  } else {
    return SERIAL_UNKNOWN;
  }
  return SERIAL_PROCESS_COMMAND;
}

SerialAction serialPoll(OperatorCommand& operatorCommand) {
  while (Serial.available()) {
    char character = (char)Serial.read();
    if (character == '\r') continue;
    if (character == '\n') {
      line[lineLength] = 0;
      uint8_t length = lineLength;
      lineLength = 0;
      if (length == 0) return SERIAL_NONE;
      return parseLine(line, operatorCommand);
    }
    if (lineLength < sizeof(line) - 1) line[lineLength++] = character;
  }
  return SERIAL_NONE;
}

void printHelp() {
  Serial.println(F("commands: start | stop | resume | ack | status | bench | send <c|s> <char> | help"));
  Serial.println(F("bench: raw sends for slave testing and calibration, stop still works, leave by reset"));
}

void printNotice(Notice notice) {
  switch (notice) {
    case NOTICE_NONE: return;
    case NOTICE_START_NOT_CALIBRATED: Serial.println(F("start refused: not calibrated, set FORWARD_ALLOWANCE_STEPS and PADDLE_STEPS in config.h")); return;
    case NOTICE_START_NOT_READY: Serial.println(F("start refused: slave feedback not fresh")); return;
    case NOTICE_START_CHANNEL_ACTIVE: Serial.println(F("start refused: a channel is closed, move the puck before the sensors")); return;
    case NOTICE_IGNORED: Serial.println(F("command ignored in this state")); return;
    case NOTICE_INIT_WAITING: Serial.println(F("initialising: waiting for both slaves")); return;
    case NOTICE_INIT_CHANNELS_ACTIVE: Serial.println(F("initialising: clear the sensors")); return;
    case NOTICE_BENCH_ENTERED: Serial.println(F("bench mode: raw sends enabled, reset to leave")); return;
    case NOTICE_RAW_SENT: return;
    case NOTICE_STOP_ALREADY: Serial.println(F("already stopped")); return;
    case NOTICE_SWEEP_IN_PROGRESS: Serial.println(F("sweep in progress, wait for it to finish")); return;
    case NOTICE_RESUMED: Serial.println(F("resumed")); return;
  }
}

static void printSlaveName(uint8_t address) {
  Serial.print(address == CONVEYOR_ADDRESS ? F("conveyor") : F("sensing"));
}

void printSendResult(uint8_t address, char command, bool acknowledged) {
  Serial.print(F("sent '"));
  Serial.print(command);
  Serial.print(F("' to "));
  printSlaveName(address);
  Serial.println(acknowledged ? F(": ack") : F(": NO ACK"));
}

static void printBits(uint8_t value, uint8_t bitCount) {
  for (int8_t bit = bitCount - 1; bit >= 0; bit--) Serial.print((value >> bit) & 1);
}

void printChannelEvent(const SlaveView& sensing, const SlaveView& conveyor) {
  Serial.print(F("channel live="));
  printBits(sensing.live, 2);
  Serial.print(F(" edges="));
  Serial.print(sensing.channel1Edges);
  Serial.print('/');
  Serial.print(sensing.channel2Edges);
  Serial.print(F(" steps="));
  Serial.println(conveyor.steps);
}

static void printAge(const SlaveView& view) {
  if (!view.everValid) { Serial.print(F(" age=never")); return; }
  Serial.print(F(" age="));
  Serial.print(view.ageMs);
  if (!view.fresh) Serial.print(F(" UNKNOWN"));
  if (view.invalid) { Serial.print(F(" INVALID:")); Serial.print(decodeResultName(view.lastError)); }
}

void printReport(uint32_t now, const Process& process, const SlaveView& conveyor, const SlaveView& sensing) {
  Serial.print(F("t="));
  Serial.print(now);
  Serial.print(F(" MASTER "));
  Serial.print(masterStateName(process.state()));
  if (process.state() == MASTER_FAULT) {
    Serial.print(F(" reason="));
    Serial.print(masterFaultName(process.fault()));
    if (process.stopUnconfirmed()) Serial.print(F(" STOP_UNCONFIRMED"));
  }
  if (process.state() == MASTER_PAUSED || process.state() == MASTER_STOPPING) {
    Serial.print(F(" phase="));
    Serial.print(masterStateName(process.savedPhase()));
  }
  if (!process.calibrated()) Serial.print(F(" NOT_CALIBRATED"));

  Serial.print(F(" | CONV target="));
  Serial.print(conveyorStateName(process.conveyorTarget()));
  Serial.print(F(" cur="));
  Serial.print(conveyor.everValid ? conveyorStateName(conveyor.state) : "-");
  if (conveyor.fault) { Serial.print(F(" fault=")); Serial.print(faultCodeName(conveyor.fault)); }
  printAge(conveyor);
  Serial.print(F(" steps="));
  Serial.print(conveyor.steps);

  Serial.print(F(" | SENS cur="));
  Serial.print(sensing.everValid ? sensingStateName(sensing.state) : "-");
  if (sensing.fault) { Serial.print(F(" fault=")); Serial.print(faultCodeName(sensing.fault)); }
  printAge(sensing);
  Serial.print(F(" live="));
  printBits(sensing.live, 2);
  Serial.print(F(" edges="));
  Serial.print(sensing.channel1Edges);
  Serial.print('/');
  Serial.print(sensing.channel2Edges);
  Serial.print(F(" level="));
  Serial.print(sensing.channel1Level);
  Serial.print('/');
  Serial.print(sensing.channel2Level);

  uint32_t delta, limit;
  if (process.travel(conveyor.steps, delta, limit)) {
    Serial.print(F(" | travel~"));
    Serial.print(delta);
    Serial.print('/');
    Serial.print(limit);
    Serial.print(F(" steps"));
    if (STEPS_PER_MM > 0) {
      Serial.print(F(" ("));
      Serial.print(delta / STEPS_PER_MM, 0);
      Serial.print(F(" mm)"));
    }
  }
  Serial.println();
}
