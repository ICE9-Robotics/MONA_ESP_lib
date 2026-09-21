#pragma once

#include <stdint.h>

extern bool logging;
extern uint32_t loopGapMaxMs;

void printHelp();
void beginStatusLog();
void serviceSerial();
void serviceStatusLog(uint32_t now);
