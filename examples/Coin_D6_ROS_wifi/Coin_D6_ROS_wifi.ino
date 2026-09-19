/*
  Coin_D6_ROS_wifi.ino

  Stream COIN-D6 raw scan packets to a ROS 2 PC and accept wheel speed
  commands (ticks/s) while publishing encoder ticks.

  PC launch (ROS 2 Jazzy):
    ros2 launch mona_bringup mona.launch.py

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
#include "Coin_D6.h"
#include "Mona_ESP_lib.h"
#include "motor_speed.h"
#include <WiFi.h>
#include <Wire.h>
#include <string.h>

static const int LIDAR_RX_PIN = 26;
static const int LIDAR_TX_PIN = 27;

#define LIDAR_SERIAL Serial1

static const char* WIFI_SSID = "NetworkForMonaESP";
static const char* WIFI_PASSWORD = "WeLoveMONA123";
static const uint16_t LIDAR_TCP_PORT = 8888;
static const uint16_t CONTROL_TCP_PORT = 8889;
static const uint32_t ODOM_PERIOD_MS = 20;
static const uint32_t MOTOR_WATCHDOG_MS = 300;

CoinD6 lidar(LIDAR_SERIAL);
WiFiServer lidarServer(LIDAR_TCP_PORT);
WiFiServer controlServer(CONTROL_TCP_PORT);

WiFiClient lidarClient;
WiFiClient controlClient;
LidarRawCapsuleStream lidarStream(lidarClient);

uint8_t controlBuf[32];
size_t controlBufLen = 0;
uint32_t lastMwMs = 0;
uint32_t lastOdMs = 0;
bool motorsArmed = false;
bool lidarHasClient = false;
bool controlHasClient = false;

static void stopMotors() {
  motor_set_left(0);
  motor_set_right(0);
  motorsArmed = false;
}

static int16_t clampTicksPerSec(int32_t value) {
  if (value > 10000) {
    return 10000;
  }
  if (value < -10000) {
    return -10000;
  }
  return static_cast<int16_t>(value);
}

static int16_t readI16Le(const uint8_t* data) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
}

static void writeU32Le(uint8_t* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
  data[2] = static_cast<uint8_t>(value >> 16);
  data[3] = static_cast<uint8_t>(value >> 24);
}

static void writeI32Le(uint8_t* data, int32_t value) {
  writeU32Le(data, static_cast<uint32_t>(value));
}

static void applyMwFrame(const uint8_t* data) {
  const int16_t left = clampTicksPerSec(readI16Le(data));
  const int16_t right = clampTicksPerSec(readI16Le(data + 2));
  motor_set_left(left);
  motor_set_right(right);
  lastMwMs = millis();
  motorsArmed = true;
}

static void parseControlBytes() {
  size_t offset = 0;
  while (controlBufLen - offset >= 6) {
    if (controlBuf[offset] != 'M' || controlBuf[offset + 1] != 'W') {
      offset += 1;
      continue;
    }
    applyMwFrame(controlBuf + offset + 2);
    offset += 6;
  }
  if (offset > 0) {
    controlBufLen -= offset;
    if (controlBufLen > 0) {
      memmove(controlBuf, controlBuf + offset, controlBufLen);
    }
  }
}

static void serviceLidarClient() {
  if (lidarHasClient && lidarClient.connected()) {
    return;
  }

  if (lidarHasClient) {
    lidar.setRawCapsuleStream(nullptr);
    lidarClient.stop();
    lidarHasClient = false;
    Serial.println("Lidar client disconnected");
  }

  WiFiClient incoming = lidarServer.available();
  if (!incoming) {
    return;
  }
  lidarClient = incoming;
  lidarHasClient = true;
  lidar.setRawCapsuleStream(&lidarStream);
  Serial.print("Lidar client connected from ");
  Serial.println(lidarClient.remoteIP());
}

static void sendOdometry() {
  const uint32_t now = millis();
  if (now - lastOdMs < ODOM_PERIOD_MS) {
    return;
  }
  lastOdMs = now;

  uint8_t frame[14];
  frame[0] = 'O';
  frame[1] = 'D';
  writeU32Le(frame + 2, now);
  writeI32Le(frame + 6, Encoder_left());
  writeI32Le(frame + 10, Encoder_right());
  controlClient.write(frame, sizeof(frame));
}

static void serviceControlClient() {
  if (!(controlHasClient && controlClient.connected())) {
    if (controlHasClient) {
      controlClient.stop();
      controlHasClient = false;
      controlBufLen = 0;
      stopMotors();
      Serial.println("Control client disconnected");
    }

    WiFiClient incoming = controlServer.available();
    if (!incoming) {
      return;
    }
    controlClient = incoming;
    controlHasClient = true;
    controlBufLen = 0;
    lastMwMs = millis();
    lastOdMs = 0;
    Serial.print("Control client connected from ");
    Serial.println(controlClient.remoteIP());
  }

  while (controlClient.available() > 0) {
    if (controlBufLen >= sizeof(controlBuf)) {
      controlBufLen = 0;
    }
    const int incoming = controlClient.read();
    if (incoming < 0) {
      break;
    }
    controlBuf[controlBufLen++] = static_cast<uint8_t>(incoming);
    parseControlBytes();
  }

  if (motorsArmed && (millis() - lastMwMs) >= MOTOR_WATCHDOG_MS) {
    stopMotors();
  }

  sendOdometry();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("COIN-D6 ROS WiFi: lidar + motors + odometry");
  Serial.println("PC: set host in yaml, then ros2 launch mona_bringup mona.launch.py");

  Mona_ESP_init();
  Encoder_reset();
  stopMotors();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Connecting to WiFi ");
    Serial.print(WIFI_SSID);
    Serial.println("...");
  }

  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Lidar TCP port: ");
  Serial.println(LIDAR_TCP_PORT);
  Serial.print("Control TCP port: ");
  Serial.println(CONTROL_TCP_PORT);

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
  controlServer.begin();
  Serial.println("Waiting for ROS TCP clients...");
}

void loop() {
  serviceLidarClient();
  serviceControlClient();
  lidar.pumpScan();
  delay(1);
}
