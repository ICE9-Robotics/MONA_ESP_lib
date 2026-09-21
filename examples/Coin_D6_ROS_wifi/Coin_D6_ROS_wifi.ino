/*
  Coin_D6_ROS_wifi.ino

  Stream COIN-D6 raw scan packets to a ROS 2 PC and accept wheel speed
  commands (ticks/s) while publishing encoder ticks.

  PC launch (ROS 2 Jazzy):
    ros2 launch mona_bringup mona.launch.py

  Serial via micro-USB: USB Serial 115200.
    Commands:
     - "help" to print commands
     - "s <left> <right>" to set motor speeds
     - "stop" to stop motors
     - "stat" to print status
     - "log 0|1" to enable/disable logging

  TCP ports:
    8888  lidar  'LR' + u16 size + scan packet bytes
    8889  control  PC->ESP 'MW' + i16 left ticks/s + i16 right ticks/s
                   ESP->PC 'OD' + u32 millis + i32 left ticks + i32 right ticks

  Wiring (cross TX/RX):
    LIDAR TX -> ESP32 GPIO26 (RX)
    LIDAR RX -> ESP32 GPIO27 (TX)
    LIDAR GND -> ESP32 GND
    LIDAR VCC -> 5V (typical 240 mA, peak ~800 mA)
*/
#include "Mona_ESP_lib.h"
#include "control_service.h"
#include "lidar_service.h"
#include "serial_console.h"
#include "wifi_setup.h"
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  printHelp();
  Serial.println("PC: set host in yaml, then ros2 launch mona_bringup mona.launch.py");

  Mona_ESP_init();
  initMotors();
  connectWifi();

  if (!startLidarScan()) {
    while (true) {
      delay(1000);
    }
  }

  lidarServer.begin();
  controlServer.begin();
  beginStatusLog();
  Serial.println("Ready. Type help, or wait for ROS TCP clients.");
}

void loop() {
  const uint32_t now = millis();
  static uint32_t lastLoopMs = 0;
  if (lastLoopMs != 0) {
    const uint32_t gap = now - lastLoopMs;
    if (gap > loopGapMaxMs) {
      loopGapMaxMs = gap;
    }
  }
  lastLoopMs = now;

  serviceSerial();
  serviceLidarClient();
  serviceControlClient();
  lidar.pumpScan();
  lidarStream.flush();
  serviceStatusLog(now);

  delay(1);
}
