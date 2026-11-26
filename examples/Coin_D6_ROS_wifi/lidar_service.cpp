#include "lidar_service.h"

#include "config.h"
#include "tcp_util.h"
#include <string.h>

CoinD6 lidar(LIDAR_SERIAL);
WiFiServer lidarServer(LIDAR_TCP_PORT);
WiFiClient lidarClient;
QueuedRawCapsuleStream lidarStream(lidarClient);
bool lidarHasClient = false;
bool lidarReady = false;

static uint32_t lastLidarProbeMs = 0;
static uint32_t lidarWaitSinceMs = 0;
static bool lidarWaitAnnounced = false;

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

void beginLidarUart() {
  lidar.begin(LIDAR_RX_PIN, LIDAR_TX_PIN);
  lidar.setRawCapsuleStream(&lidarStream);
}

void pollLidar(uint32_t now) {
  if (lidarReady) {
    return;
  }
  if (lidar.capsuleSize() != 0) {
    lidarReady = true;
    lidarServer.begin();
    Serial.print("Lidar scan up, type 0x");
    Serial.print(lidar.scanAnswerType(), HEX);
    Serial.print(", packet size ");
    Serial.println(lidar.capsuleSize());
    Serial.println("Lidar server started.");
    return;
  }
  if (lidarWaitSinceMs == 0) {
    lidarWaitSinceMs = now;
  }
  if (lastLidarProbeMs == 0 || (now - lastLidarProbeMs) >= LIDAR_RETRY_MS) {
    lastLidarProbeMs = now;
    lidar.requestScan();
  }
  if (!lidarWaitAnnounced && (now - lidarWaitSinceMs) >= LIDAR_RETRY_MS) {
    lidarWaitAnnounced = true;
    Serial.println(
        "Lidar not ready. Control server is up; lidar server starts when the sensor responds.");
  }
}

void serviceLidarClient() {
  if (!lidarReady) {
    return;
  }
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

