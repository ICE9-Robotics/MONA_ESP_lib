#include "lidar_service.h"

#include "config.h"
#include "tcp_util.h"
#include <string.h>

CoinD6 lidar(LIDAR_SERIAL);
WiFiServer lidarServer(LIDAR_TCP_PORT);
WiFiClient lidarClient;
QueuedRawCapsuleStream lidarStream(lidarClient);
bool lidarHasClient = false;

QueuedRawCapsuleStream::QueuedRawCapsuleStream(WiFiClient& client)
    : LidarRawCapsuleStream(client), _client(client) {}

void QueuedRawCapsuleStream::setSinkEnabled(bool enabled) {
  _enabled = enabled;
  if (!enabled) {
    clear();
  }
}

void QueuedRawCapsuleStream::onCapsule(const uint8_t* data, size_t len) {
  _parsed++;
  if (!_enabled) {
    return;
  }
  if (len + 4 > kMaxFrame) {
    _dropped++;
    return;
  }
  if (_count >= kQueueDepth) {
    _dropped++;
    return;
  }
  uint8_t* dst = _frames[_tail];
  dst[0] = FRAME_MAGIC[0];
  dst[1] = FRAME_MAGIC[1];
  dst[2] = static_cast<uint8_t>(len);
  dst[3] = static_cast<uint8_t>(len >> 8);
  memcpy(dst + 4, data, len);
  _lens[_tail] = static_cast<uint16_t>(len + 4);
  _tail = (_tail + 1) % kQueueDepth;
  _count++;
}

void QueuedRawCapsuleStream::flush() {
  if (!_enabled || !_client.connected()) {
    return;
  }
  while (_count > 0) {
    const uint16_t need = _lens[_head];
    if (!socketHasSpace(_client, need)) {
      return;
    }
    const size_t wrote = _client.write(_frames[_head], need);
    if (wrote != need) {
      if (wrote != 0) {
        popHead();
        _dropped++;
      }
      return;
    }
    _sent++;
    popHead();
  }
}

void QueuedRawCapsuleStream::clear() {
  _head = 0;
  _tail = 0;
  _count = 0;
}

void QueuedRawCapsuleStream::resetCounters() {
  _parsed = 0;
  _dropped = 0;
  _sent = 0;
}

void QueuedRawCapsuleStream::popHead() {
  _head = (_head + 1) % kQueueDepth;
  _count--;
}

void serviceLidarClient() {
  if (lidarHasClient && lidarClient.connected()) {
    return;
  }

  if (lidarHasClient) {
    lidarClient.stop();
    lidarHasClient = false;
    lidarStream.setSinkEnabled(false);
    Serial.println("Lidar client disconnected");
  }

  WiFiClient incoming = lidarServer.available();
  if (!incoming) {
    return;
  }
  lidarClient = incoming;
  configureTcpClient(lidarClient);
  lidarHasClient = true;
  lidarStream.setSinkEnabled(true);
  Serial.print("Lidar client connected from ");
  Serial.println(lidarClient.remoteIP());
}

bool startLidarScan() {
  lidar.begin(LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(500);

  if (!lidar.connect(LIDAR_RX_PIN, LIDAR_TX_PIN)) {
    Serial.println("Failed to connect to lidar.");
    return false;
  }

  if (!lidar.startScan()) {
    Serial.println("Failed to start scan.");
    return false;
  }

  Serial.print("Scan stream type 0x");
  Serial.print(lidar.scanAnswerType(), HEX);
  Serial.print(", packet size ");
  Serial.println(lidar.capsuleSize());

  lidar.setRawCapsuleStream(&lidarStream);
  return true;
}
