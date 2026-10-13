#include <Arduino.h>
#include "config.h"
#include "contract.h"
#include "feedback.h"
#include "operator_serial.h"
#include "process.h"
#include "wire_link.h"

static SlaveView conveyor;
static SlaveView sensing;
static Process process;
static bool stopRequested;
static OperatorCommand pendingOperatorCommand;
static Notice lastNotice;
static uint32_t lastPollMs;
static uint32_t lastReportMs;

static MasterState shownMasterState;
static uint8_t shownConveyorState;
static uint8_t shownSensingState;
static uint8_t shownLive;
static uint16_t shownReed1Edges;
static uint16_t shownReed2Edges;
static bool shownConveyorFresh;
static bool shownSensingFresh;

static void pollSlave(SlaveView& view, uint8_t address, uint8_t replyLength, uint32_t now) {
  uint8_t bytes[MAX_REPLY_LENGTH];
  uint8_t received = wireRead(address, bytes, replyLength);
  if (received == 0) view.noteNoReply(now);
  else view.noteReply(now, bytes, received);
}

static void queueOperatorCommand(const OperatorCommand& operatorCommand) {
  if (operatorCommand.kind == OPERATOR_STOP) {
    stopRequested = true;
    pendingOperatorCommand.kind = OPERATOR_NONE;
    return;
  }
  if (pendingOperatorCommand.kind != OPERATOR_NONE) Serial.println(F("previous command replaced"));
  pendingOperatorCommand = operatorCommand;
}

static OperatorCommand takeOperatorCommand() {
  OperatorCommand taken = {OPERATOR_NONE, 0, 0};
  if (stopRequested) {
    stopRequested = false;
    pendingOperatorCommand.kind = OPERATOR_NONE;
    taken.kind = OPERATOR_STOP;
    return taken;
  }
  taken = pendingOperatorCommand;
  pendingOperatorCommand.kind = OPERATOR_NONE;
  return taken;
}

static void printNoticeOnChange(Notice notice, bool stateChanged) {
  if (stateChanged) lastNotice = NOTICE_NONE;
  if (notice == lastNotice) return;
  lastNotice = notice;
  printNotice(notice);
}

static void showAndon(AndonColour colour) {
  digitalWrite(ANDON_GREEN_PIN, colour == ANDON_GREEN);
  digitalWrite(ANDON_AMBER_PIN, colour == ANDON_AMBER);
  digitalWrite(ANDON_RED_PIN, colour == ANDON_RED);
}

static bool reportDue(uint32_t now) {
  if (process.state() != shownMasterState) return true;
  if (conveyor.state != shownConveyorState || sensing.state != shownSensingState) return true;
  if (conveyor.fresh != shownConveyorFresh || sensing.fresh != shownSensingFresh) return true;
  return now - lastReportMs >= REPORT_HEARTBEAT_MS;
}

static void rememberShown(uint32_t now) {
  shownMasterState = process.state();
  shownConveyorState = conveyor.state;
  shownSensingState = sensing.state;
  shownConveyorFresh = conveyor.fresh;
  shownSensingFresh = sensing.fresh;
  lastReportMs = now;
}

void setup() {
  serialBegin(SERIAL_BAUD);
  wireLinkBegin(WIRE_TIMEOUT_MICROSECONDS);
  pinMode(ANDON_GREEN_PIN, OUTPUT);
  pinMode(ANDON_AMBER_PIN, OUTPUT);
  pinMode(ANDON_RED_PIN, OUTPUT);
  conveyor.begin(SLAVE_CONVEYOR);
  sensing.begin(SLAVE_SENSING);
  Calibration calibration = {FORWARD_ALLOWANCE_STEPS, RETURN_ALLOWANCE_STEPS, DISCHARGE_OVERRUN_STEPS};
  process.begin(calibration);
  stopRequested = false;
  pendingOperatorCommand.kind = OPERATOR_NONE;
  lastNotice = NOTICE_NONE;
  lastPollMs = millis();
  shownMasterState = MASTER_FAULT;
  Serial.print(F("MASTER contract v"));
  Serial.println(CONTRACT_VERSION);
  printHelp();
}

void loop() {
  uint32_t now = millis();

  OperatorCommand operatorCommand;
  switch (serialPoll(operatorCommand)) {
    case SERIAL_PROCESS_COMMAND: queueOperatorCommand(operatorCommand); break;
    case SERIAL_STATUS: printReport(now, process, conveyor, sensing); break;
    case SERIAL_HELP: printHelp(); break;
    case SERIAL_UNKNOWN: Serial.println(F("unknown command, type help")); break;
    case SERIAL_NONE: break;
  }

  if (now - lastPollMs < POLL_INTERVAL_MS) return;
  lastPollMs = now;

  pollSlave(sensing, SENSING_ADDRESS, SENSING_REPLY_LENGTH, now);
  pollSlave(conveyor, CONVEYOR_ADDRESS, CONVEYOR_REPLY_LENGTH, now);

  OperatorCommand taken = takeOperatorCommand();
  MasterState stateBefore = process.state();
  ProcessOutput output;
  process.tick(now, conveyor, sensing, taken, output);
  showAndon(process.andon());

  for (uint8_t i = 0; i < output.requestCount; i++) {
    bool acknowledged = wireSend(output.requests[i].address, output.requests[i].command);
    if (taken.kind == OPERATOR_SEND || !acknowledged) {
      printSendResult(output.requests[i].address, output.requests[i].command, acknowledged);
    }
  }
  printNoticeOnChange(output.notice, process.state() != stateBefore || taken.kind != OPERATOR_NONE);

  if (sensing.fresh && (sensing.live != shownLive || sensing.reed1Edges != shownReed1Edges ||
                        sensing.reed2Edges != shownReed2Edges)) {
    printReedEvent(sensing, conveyor);
    shownLive = sensing.live;
    shownReed1Edges = sensing.reed1Edges;
    shownReed2Edges = sensing.reed2Edges;
  }
  if (reportDue(now)) {
    printReport(now, process, conveyor, sensing);
    rememberShown(now);
  }
}
