#include "RPLidar_C1.h"
#include <WiFi.h>

// Wiring (cross TX/RX):
//   LIDAR TX (yellow) -> ESP32 GPIO26 (RX)
//   LIDAR RX (green)  -> ESP32 GPIO27 (TX)
//   LIDAR GND (black) -> ESP32 GND
//   LIDAR VCC (red)   -> 5V (needs ~250mA; aim for 5.0V, not 4.7V)
static const int LIDAR_RX_PIN = 26;
static const int LIDAR_TX_PIN = 27;

#define LIDAR_SERIAL Serial1

// Set your WiFi credentials before uploading.
static const char* WIFI_SSID = "NetworkForMonaESP";
static const char* WIFI_PASSWORD = "WeLoveMONA123";
static const uint16_t LIDAR_TCP_PORT = 8888;

static const LidarPointFormat POINT_OUTPUT = LidarPointFormat::Cartesian;

RPLidarC1 lidar(LIDAR_SERIAL);
WiFiServer lidarServer(LIDAR_TCP_PORT);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("RPLIDAR C1 point stream over WiFi");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Connecting to WiFi ");
    Serial.print(WIFI_SSID);
    Serial.println("...");
  }

  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Point stream TCP port: ");
  Serial.println(LIDAR_TCP_PORT);
  Serial.println("Frame format: 'LD' + int16 pairs + 'EE' + u16 count");

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
  Serial.print(", capsule size ");
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

  LidarBinaryStream pointStream(client, POINT_OUTPUT);

  while (client.connected()) {
    lidar.getPoints(2000, &pointStream);
  }

  client.stop();
  Serial.println("Client disconnected");
  Serial.println("Waiting for TCP client...");
}
