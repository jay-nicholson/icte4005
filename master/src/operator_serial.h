#pragma once
#include <stdint.h>
#include "feedback.h"
#include "process.h"

enum SerialAction { SERIAL_NONE, SERIAL_PROCESS_COMMAND, SERIAL_STATUS, SERIAL_HELP, SERIAL_UNKNOWN };

void serialBegin(uint32_t baud);
SerialAction serialPoll(OperatorCommand& operatorCommand);
void printHelp();
void printNotice(Notice notice);
void printSendResult(uint8_t address, char command, bool acknowledged);
void printChannelEvent(const SlaveView& sensing, const SlaveView& conveyor);
void printReport(uint32_t now, const Process& process, const SlaveView& conveyor, const SlaveView& sensing);
