/*
  Coin_D6_ROS_wifi.ino

  Stream COIN-D6 raw scan packets to a ROS 2 PC and accept wheel speed
  commands (ticks/s) while publishing encoder ticks.

  Serial via micro-USB: 
    USB Serial for debugging and testing. 
    Baud rate 115200. 
    Type "help" for commands.

  TCP ports:
    8888: lidar
          packet format:
          - Out: 'LR' + uint16 size + scan packet bytes
    8889: control
          packet format: 
          - Out: 'OD' + uint32 millis + int32 left ticks + int32 right ticks
          - In: 'MW' + int16 left ticks/s + int16 right ticks/s
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

  Mona_ESP_init();
  initMotors();
  wifiSetup();
  beginLidarUart();

  controlServer.begin();
  beginStatusLog();
  printInfo();
  Serial.println("Ready. Type 'help' for commands, or wait for ROS TCP clients.");
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
  serviceControlClient();
  lidar.pumpScan();
  pollLidar(now);
  serviceLidarClient();
  lidarStream.flush();
  serviceStatusLog(now);

  delay(1);
}
