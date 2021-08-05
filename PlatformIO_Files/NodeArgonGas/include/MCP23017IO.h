#pragma once

#include <Wire.h>
#include "Adafruit_MCP23017.h"

extern bool resetButtonPressed;

extern bool panicButtonPressed;

void setup_MCP23017();

void signalSafeLampOn();

void signalSafeLampOff();

void signalActiveLampOn();

void signalActiveLampOff();

void greenLedOn();

void greenLedOff();

void blueLedOn();

void blueLedOff();

void buzzerOn();

void buzzerOff();

void loop_MCP23017();
