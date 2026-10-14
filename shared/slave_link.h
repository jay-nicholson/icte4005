// Wire slave boilerplate shared by both slave boards. Include from main.cpp only.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "contract.h"

class SlaveLink {
 public:
  void begin(uint8_t address, uint8_t replyLength) {
    instance_ = this;
    replyLength_ = replyLength;
    memset(reply_, 0, sizeof(reply_));
    pendingCommand_ = 0;
    everRead_ = false;
    lastReadMs_ = 0;
    Wire.begin(address);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
  }

  // Returns the queued command byte, or 0 when there is none.
  char takeCommand() {
    noInterrupts();
    char command = pendingCommand_;
    pendingCommand_ = 0;
    interrupts();
    return command;
  }

  void publish(const uint8_t* bytes) {
    noInterrupts();
    memcpy(reply_, bytes, replyLength_);
    interrupts();
  }

  bool masterInContact(uint32_t now) const {
    noInterrupts();
    bool everRead = everRead_;
    interrupts();
    return everRead && !contactLost(now);
  }

  bool contactLost(uint32_t now) const {
    noInterrupts();
    bool everRead = everRead_;
    uint32_t lastReadMs = lastReadMs_;
    interrupts();
    return everRead && (now - lastReadMs) >= CONTACT_WATCHDOG_MS;
  }

 private:
  // A command is the ASCII byte followed by its bitwise complement; anything else is ignored.
  static void onReceive(int count) {
    uint8_t first = 0;
    uint8_t second = 0;
    uint8_t index = 0;
    while (Wire.available()) {
      uint8_t value = (uint8_t)Wire.read();
      if (index == 0) first = value;
      if (index == 1) second = value;
      index++;
    }
    if (count == 2 && second == (uint8_t)~first) instance_->pendingCommand_ = (char)first;
  }

  static void onRequest() {
    Wire.write(instance_->reply_, instance_->replyLength_);
    instance_->lastReadMs_ = millis();
    instance_->everRead_ = true;
  }

  static SlaveLink* instance_;
  uint8_t reply_[MAX_REPLY_LENGTH];
  uint8_t replyLength_;
  volatile char pendingCommand_;
  volatile bool everRead_;
  volatile uint32_t lastReadMs_;
};

SlaveLink* SlaveLink::instance_ = nullptr;
