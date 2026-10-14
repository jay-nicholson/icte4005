#include "wire_link.h"
#include <Arduino.h>
#include <Wire.h>

void wireLinkBegin(uint32_t timeoutMicroseconds) {
  Wire.begin();
  Wire.setWireTimeout(timeoutMicroseconds, true);
}

static bool timedOut() {
  if (!Wire.getWireTimeoutFlag()) return false;
  Wire.clearWireTimeoutFlag();
  return true;
}

bool wireSend(uint8_t address, char command) {
  Wire.beginTransmission(address);
  Wire.write((uint8_t)command);
  Wire.write((uint8_t)~command);
  uint8_t returnCode = Wire.endTransmission(true);
  if (timedOut()) return false;
  return returnCode == 0;
}

uint8_t wireRead(uint8_t address, uint8_t* bytes, uint8_t length) {
  Wire.requestFrom(address, length, (uint8_t)true);
  uint8_t received = 0;
  while (Wire.available()) {
    int value = Wire.read();
    if (received < length) bytes[received] = (uint8_t)value;
    received++;
  }
  if (timedOut()) return 0;
  return received;
}
