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


// Measured on the assembled belt. 0 means not calibrated; Start is refused.
static const uint32_t CHANNEL_SPACING_STEPS = 1300;  // light sensor to Hall, about 30 mm plus margin
static const uint32_t PADDLE_STEPS = 4886;           // Hall to paddle, placeholder for 150 mm

// Display only. 0 shows steps without millimetres.
static const float STEPS_PER_MM = 0;
