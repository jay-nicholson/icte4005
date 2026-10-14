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
static bool conveyorAckFailed;
static bool sensingAckFailed;
static uint32_t lastPollMs;
static uint32_t lastReportMs;

static MasterState shownMasterState;
static uint8_t shownConveyorState;
static uint8_t shownSensingState;
static uint8_t shownLive;
static uint16_t shownChannel1Edges;
static uint16_t shownChannel2Edges;
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

static void printSendResultOnChange(uint8_t address, char command, bool acknowledged, bool rawSend) {
  bool& failed = (address == CONVEYOR_ADDRESS) ? conveyorAckFailed : sensingAckFailed;
  bool repeatedFailure = !acknowledged && failed;
  if (rawSend || !repeatedFailure) printSendResult(address, command, acknowledged);
  failed = !acknowledged;
}

static void printNoticeOnChange(Notice notice, bool stateChanged) {
  if (stateChanged) lastNotice = NOTICE_NONE;
  if (notice == lastNotice) return;
  lastNotice = notice;
  printNotice(notice);
}

static void showAndon(AndonColour colour, uint32_t now) {
  bool blinkOn = (now % ANDON_BLINK_PERIOD_MS) < ANDON_BLINK_PERIOD_MS / 2;
  bool amber = colour == ANDON_AMBER || (colour == ANDON_AMBER_BLINK && blinkOn);
  digitalWrite(ANDON_GREEN_PIN, colour == ANDON_GREEN);
  digitalWrite(ANDON_AMBER_PIN, amber);
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
  Calibration calibration = {FORWARD_ALLOWANCE_STEPS, PADDLE_STEPS};
  Options options = {AUTO_ACK_MS, AUTO_RESTART, SINGLE_CHANNEL_BENCH};
  process.begin(calibration, options);
  stopRequested = false;
  pendingOperatorCommand.kind = OPERATOR_NONE;
  lastNotice = NOTICE_NONE;
  conveyorAckFailed = false;
  sensingAckFailed = false;
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

  showAndon(process.andon(), now);
  if (now - lastPollMs < POLL_INTERVAL_MS) return;
  lastPollMs = now;

  pollSlave(sensing, SENSING_ADDRESS, SENSING_REPLY_LENGTH, now);
  pollSlave(conveyor, CONVEYOR_ADDRESS, CONVEYOR_REPLY_LENGTH, now);

  OperatorCommand taken = takeOperatorCommand();
  MasterState stateBefore = process.state();
  ProcessOutput output;
  process.tick(now, conveyor, sensing, taken, output);

  for (uint8_t i = 0; i < output.requestCount; i++) {
    bool acknowledged = wireSend(output.requests[i].address, output.requests[i].command);
    printSendResultOnChange(output.requests[i].address, output.requests[i].command, acknowledged,
                            taken.kind == OPERATOR_SEND);
  }
  printNoticeOnChange(output.notice, process.state() != stateBefore || taken.kind != OPERATOR_NONE);

  if (sensing.fresh && (sensing.live != shownLive || sensing.channel1Edges != shownChannel1Edges ||
                        sensing.channel2Edges != shownChannel2Edges)) {
    printChannelEvent(sensing, conveyor);
    shownLive = sensing.live;
    shownChannel1Edges = sensing.channel1Edges;
    shownChannel2Edges = sensing.channel2Edges;
  }
  if (reportDue(now)) {
    printReport(now, process, conveyor, sensing);
    rememberShown(now);
  }
}
