#include "RPLidar_C1.h"

// Wiring (cross TX/RX):
//   LIDAR TX (yellow) -> ESP32 GPIO26 (RX)
//   LIDAR RX (green)  -> ESP32 GPIO27 (TX)
//   LIDAR GND (black) -> ESP32 GND
//   LIDAR VCC (red)   -> 5V (needs ~250mA; aim for 5.0V, not 4.7V)
static const int LIDAR_RX_PIN = 26;
static const int LIDAR_TX_PIN = 27;

#define LIDAR_SERIAL Serial1

RPLidarC1 lidar(LIDAR_SERIAL);

void printDeviceInfo(const LidarDeviceInfo& info) {
  Serial.print("Model: 0x");
  Serial.println(info.model, HEX);
  Serial.print("Firmware: ");
  Serial.print(info.firmware_version >> 8);
  Serial.print(".");
  if ((info.firmware_version & 0xFF) < 10) {
    Serial.print("0");
  }
  Serial.println(info.firmware_version & 0xFF);
  Serial.print("Hardware: ");
  Serial.println(info.hardware_version);
  Serial.print("Serial: ");
  for (int i = 0; i < 16; ++i) {
    if (info.serialnum[i] < 16) {
      Serial.print("0");
    }
    Serial.print(info.serialnum[i], HEX);
  }
  Serial.println();
}

void printHealth(const LidarHealth& health) {
  Serial.print("Health status: ");
  switch (health.status) {
    case 0:
      Serial.println("OK");
      break;
    case 1:
      Serial.println("WARNING");
      break;
    case 2:
      Serial.println("ERROR");
      break;
    default:
      Serial.println("UNKNOWN");
      break;
  }
  if (health.error_code != 0) {
    Serial.print("Error code: 0x");
    Serial.println(health.error_code, HEX);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("RPLIDAR C1 info");

  lidar.begin(LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(500);

  if (!lidar.connect(LIDAR_RX_PIN, LIDAR_TX_PIN)) {
    Serial.println("Failed to connect to lidar.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Connected.");

  LidarDeviceInfo info;
  if (!lidar.getDeviceInfo(info)) {
    Serial.println("Failed to read device info.");
    while (true) {
      delay(1000);
    }
  }
  printDeviceInfo(info);

  LidarHealth health;
  if (!lidar.getHealth(health)) {
    Serial.println("Failed to read device health.");
    while (true) {
      delay(1000);
    }
  }
  printHealth(health);
}

void loop() {
}
