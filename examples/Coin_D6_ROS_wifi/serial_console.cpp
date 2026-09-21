#include "serial_console.h"

#include "config.h"
#include "control_service.h"
#include "lidar_service.h"
#include "motor_speed.h"
#include <stdio.h>
#include <string.h>

bool logging = true;
uint32_t loopGapMaxMs = 0;

static char lineBuf[kLineMax];
static size_t lineLen = 0;
static uint32_t lastStatMs = 0;

static void printStats() {
  motor_speed_state_t st;
  motor_speed_get_state(&st);
  Serial.print("gap_max_ms=");
  Serial.print(loopGapMaxMs);
  Serial.print(" lidar_pkts=");
  Serial.print(lidarStream.parsed());
  Serial.print(" lidar_drop=");
  Serial.print(lidarStream.dropped());
  Serial.print(" lidar_sent=");
  Serial.print(lidarStream.sent());
  Serial.print(" od_drop=");
  Serial.print(odDrops);
  Serial.print(" od_sent=");
  Serial.print(odSent);
  Serial.print(" L_tgt=");
  Serial.print(st.left_target_tps);
  Serial.print(" L_meas=");
  Serial.print(st.left_meas_tps);
  Serial.print(" R_tgt=");
  Serial.print(st.right_target_tps);
  Serial.print(" R_meas=");
  Serial.println(st.right_meas_tps);
}

static void resetPeriodCounters() {
  loopGapMaxMs = 0;
  odDrops = 0;
  odSent = 0;
  lidarStream.resetCounters();
}

void printHelp() {
  Serial.println("# Coin_D6_ROS_wifi  USB serial test, ROS TCP optional");
  Serial.println("# help | s left right | stop | stat | log 0|1");
  Serial.println("# gap_max_ms on the 1 Hz line is the stall detector");
}

void beginStatusLog() {
  lastStatMs = millis();
}

static void handleLine(char* line) {
  if (strcmp(line, "help") == 0) {
    printHelp();
    return;
  }
  if (strcmp(line, "stop") == 0) {
    stopMotors();
    Serial.println("# stop");
    return;
  }
  if (strcmp(line, "stat") == 0) {
    printStats();
    return;
  }

  int a = 0;
  int b = 0;
  if (sscanf(line, "s %d %d", &a, &b) == 2) {
    if (controlHasClient) {
      Serial.println("# ROS control client owns motors; disconnect it first");
      return;
    }
    setSpeed(a, b);
    Serial.print("# s ");
    Serial.print(a);
    Serial.print(" ");
    Serial.println(b);
    return;
  }
  if (sscanf(line, "log %d", &a) == 1) {
    logging = a != 0;
    Serial.print("# log ");
    Serial.println(logging ? 1 : 0);
    return;
  }

  Serial.print("# unknown: ");
  Serial.println(line);
}

void serviceSerial() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
        lineLen = 0;
      }
      continue;
    }
    if (lineLen + 1 < sizeof(lineBuf)) {
      lineBuf[lineLen++] = ch;
    } else {
      lineLen = 0;
    }
  }
}

void serviceStatusLog(uint32_t now) {
  if (logging && (now - lastStatMs) >= STAT_PERIOD_MS) {
    lastStatMs = now;
    printStats();
    resetPeriodCounters();
  }
}
