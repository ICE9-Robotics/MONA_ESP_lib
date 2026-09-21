#include "control_service.h"

#include "Mona_ESP_lib.h"
#include "config.h"
#include "motor_speed.h"
#include "tcp_util.h"
#include <string.h>

WiFiServer controlServer(CONTROL_TCP_PORT);
WiFiClient controlClient;
bool controlHasClient = false;
bool motorsArmed = false;
uint32_t odDrops = 0;
uint32_t odSent = 0;

static uint8_t controlBuf[32];
static size_t controlBufLen = 0;
static uint32_t lastMwMs = 0;
static uint32_t lastOdMs = 0;

void initMotors() {
  Encoder_reset();
  stopMotors();
}

void stopMotors() {
  motor_set_left(0);
  motor_set_right(0);
  motorsArmed = false;
}

void setSpeed(int left_tps, int right_tps) {
  motor_set_left(left_tps);
  motor_set_right(right_tps);
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
  if (!socketHasSpace(controlClient, sizeof(frame))) {
    odDrops++;
    return;
  }
  const size_t wrote = controlClient.write(frame, sizeof(frame));
  if (wrote != sizeof(frame)) {
    odDrops++;
    return;
  }
  odSent++;
}

void serviceControlClient() {
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
    configureTcpClient(controlClient);
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
