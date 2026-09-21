#include "serial_console.h"

#include "config.h"
#include "control_service.h"
#include "lidar_service.h"
#include "motor_speed.h"
#include "ver.h"
#include <WiFi.h>
#include <stdio.h>
#include <string.h>

bool logging = false;
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
  Serial.println("#################################");
  Serial.println("# Commands:");
  Serial.println("# help - print this help");
  Serial.println("# s <left> <right> - set motor speeds");
  Serial.println("# stop - stop motors");
  Serial.println("# stat - print status");
  Serial.println("# reset_stat - reset period counters");
  Serial.println("# info - print info about the robot");
  Serial.println("# log 0|1 - enable/disable screen logging");
  Serial.println("#################################");
}

void printInfo() {
  Serial.println("#################################");
  Serial.println("# Info:");
  Serial.print("# Firmware: ");
  Serial.println(FIRMWARE_NAME);
  Serial.print("# Version: ");
  Serial.println(VERSION);
  Serial.print("# Build date: ");
  Serial.println(BUILD_DATE);
  Serial.print("# Build time: ");
  Serial.println(BUILD_TIME);
  Serial.print("# IP address: ");
  if (WIFI_HOTSPOT_MODE) {
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println(WiFi.localIP());
  }
  Serial.print("# Lidar TCP port: ");
  Serial.println(LIDAR_TCP_PORT);
  Serial.print("# Control TCP port: ");
  Serial.println(CONTROL_TCP_PORT);
  Serial.print("# Hotspot mode: ");
  Serial.println(WIFI_HOTSPOT_MODE ? "true" : "false");
  Serial.print("# WiFi SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("# WiFi password: ");
  Serial.println(WIFI_PASSWORD);
  Serial.print("# WiFi stations: ");
  Serial.println(WIFI_HOTSPOT_MODE ? WiFi.softAPgetStationNum() : 0);
  Serial.print("# Lidar client: ");
  if (lidarHasClient && lidarClient.connected()) {
    Serial.println(lidarClient.remoteIP());
  } else {
    Serial.println("none");
  }
  Serial.print("# Control client: ");
  if (controlHasClient && controlClient.connected()) {
    Serial.println(controlClient.remoteIP());
  } else {
    Serial.println("none");
  }
  Serial.println("#################################");
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
  if (strcmp(line, "reset_stat") == 0) {
    resetPeriodCounters();
    return;
  }
  if (strcmp(line, "info") == 0) {
    printInfo();
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
