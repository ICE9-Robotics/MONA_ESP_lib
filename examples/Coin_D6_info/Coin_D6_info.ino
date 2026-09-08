#include "Coin_D6.h"

// Wiring (cross TX/RX):
//   LIDAR TX -> ESP32 GPIO26 (RX)
//   LIDAR RX -> ESP32 GPIO27 (TX)
//   LIDAR GND -> ESP32 GND
//   LIDAR VCC -> 5V (typical 240 mA, peak ~800 mA)
static const int LIDAR_RX_PIN = 26;
static const int LIDAR_TX_PIN = 27;

#define LIDAR_SERIAL Serial1

CoinD6 lidar(LIDAR_SERIAL);

void printDeviceInfo(const CoinD6DeviceInfo& info) {
  Serial.print("Model: ");
  Serial.println(info.model);
  Serial.print("Software: Rev ");
  Serial.println(info.software_version);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("COIN-D6 info");

  lidar.begin(LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(500);

  if (!lidar.connect(LIDAR_RX_PIN, LIDAR_TX_PIN)) {
    Serial.println("Failed to connect to lidar.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Connected.");

  CoinD6DeviceInfo info;
  if (!lidar.getDeviceInfo(info)) {
    Serial.println("Device info: (not received; power-cycle the lidar with the ESP32).");
    return;
  }
  printDeviceInfo(info);
}

void loop() {
}
