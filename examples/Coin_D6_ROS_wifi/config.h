#pragma once

#include <Arduino.h>

static const int LIDAR_RX_PIN = 26;
static const int LIDAR_TX_PIN = 27;

#define LIDAR_SERIAL Serial1

static const char* WIFI_SSID = "NetworkForMonaESP";
static const char* WIFI_PASSWORD = "WeLoveMONA123";
static const bool WIFI_HOTSPOT_MODE = true;
static const uint16_t LIDAR_TCP_PORT = 8888;
static const uint16_t CONTROL_TCP_PORT = 8889;
static const uint32_t ODOM_PERIOD_MS = 20;
static const uint32_t MOTOR_WATCHDOG_MS = 300;
static const uint32_t STAT_PERIOD_MS = 1000;
static const int kLineMax = 80;
