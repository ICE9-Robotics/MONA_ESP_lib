#include "Coin_D6.h"
#include <WiFi.h>

// Wiring (cross TX/RX):
//   LIDAR TX -> ESP32 GPIO26 (RX)
//   LIDAR RX -> ESP32 GPIO27 (TX)
//   LIDAR GND -> ESP32 GND
//   LIDAR VCC -> 5V (typical 240 mA, peak ~800 mA)
static const int LIDAR_RX_PIN = 26;
static const int LIDAR_TX_PIN = 27;

#define LIDAR_SERIAL Serial1

// Set your WiFi credentials before uploading.
static const char* WIFI_SSID = "NetworkForMonaESP";
static const char* WIFI_PASSWORD = "WeLoveMONA123";
static const uint16_t LIDAR_TCP_PORT = 8888;

CoinD6 lidar(LIDAR_SERIAL);
WiFiServer lidarServer(LIDAR_TCP_PORT);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("COIN-D6 raw packet stream over WiFi");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Connecting to WiFi ");
    Serial.print(WIFI_SSID);
    Serial.println("...");
  }

  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Raw packet TCP port: ");
  Serial.println(LIDAR_TCP_PORT);
  Serial.println("Frame format: 'LR' + u16 size + scan packet bytes");

  lidar.begin(LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(500);

  if (!lidar.connect(LIDAR_RX_PIN, LIDAR_TX_PIN)) {
    Serial.println("Failed to connect to lidar.");
    while (true) {
      delay(1000);
    }
  }

  if (!lidar.startScan()) {
    Serial.println("Failed to start scan.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("Scan stream type 0x");
  Serial.print(lidar.scanAnswerType(), HEX);
  Serial.print(", packet size ");
  Serial.println(lidar.capsuleSize());

  lidarServer.begin();
  Serial.println("Waiting for TCP client...");
}

void loop() {
  WiFiClient client = lidarServer.available();
  if (!client) {
    lidar.pumpScan();
    return;
  }

  Serial.print("Client connected from ");
  Serial.println(client.remoteIP());

  LidarRawCapsuleStream rawStream(client);
  lidar.setRawCapsuleStream(&rawStream);

  while (client.connected()) {
    lidar.pumpScan();
    delay(1);
  }

  lidar.setRawCapsuleStream(nullptr);
  client.stop();
  Serial.println("Client disconnected");
  Serial.println("Waiting for TCP client...");
}
