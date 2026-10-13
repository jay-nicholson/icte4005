#pragma once
#include <stdint.h>

static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t WIRE_TIMEOUT_MICROSECONDS = 25000;
static const uint16_t REPORT_HEARTBEAT_MS = 1000;

// Master andon. Set to the assembled wiring.
static const uint8_t ANDON_GREEN_PIN = 2;
static const uint8_t ANDON_AMBER_PIN = 3;
static const uint8_t ANDON_RED_PIN = 4;
static const uint16_t ANDON_BLINK_PERIOD_MS = 1000;

// Demo behaviour. AUTO_ACK_MS 0 keeps the acknowledgement manual; AUTO_RESTART false returns to READY after each eject.
static const uint32_t AUTO_ACK_MS = 2000;
static const bool AUTO_RESTART = true;

// Measured in bench mode on the assembled belt. 0 means not calibrated; Start is refused.
static const uint32_t FORWARD_ALLOWANCE_STEPS = 2000;  // bench placeholder, channel 2 shares the channel 1 sensor tonight
static const uint32_t PADDLE_STEPS = 4886;             // bench placeholder, 150 mm at 0.0307 mm per half-step
static const bool SINGLE_CHANNEL_BENCH = true;         // bench only: one Hall sensor fitted

// Display only. 0 shows steps without millimetres.
static const float STEPS_PER_MM = 0;
