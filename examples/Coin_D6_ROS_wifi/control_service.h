#pragma once

#include <WiFi.h>
#include <stdint.h>

extern WiFiServer controlServer;
extern WiFiClient controlClient;
extern bool controlHasClient;
extern bool motorsArmed;
extern uint32_t odDrops;
extern uint32_t odSent;

void initMotors();
void stopMotors();
void setSpeed(int left_tps, int right_tps);
void serviceControlClient();
