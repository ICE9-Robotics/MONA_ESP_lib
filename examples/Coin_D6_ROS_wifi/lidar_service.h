#pragma once

#include "Coin_D6.h"
#include <WiFi.h>
#include <stddef.h>
#include <stdint.h>

class QueuedRawCapsuleStream : public LidarRawCapsuleStream {
 public:
  static const size_t kMaxFrame = 160;
  static const size_t kQueueDepth = 16;

  explicit QueuedRawCapsuleStream(WiFiClient& client);

  void setSinkEnabled(bool enabled);
  void onCapsule(const uint8_t* data, size_t len) override;
  void flush();
  void clear();

  uint32_t parsed() const { return _parsed; }
  uint32_t dropped() const { return _dropped; }
  uint32_t sent() const { return _sent; }
  void resetCounters();

 private:
  WiFiClient& _client;
  uint8_t _frames[kQueueDepth][kMaxFrame];
  uint16_t _lens[kQueueDepth] = {};
  size_t _head = 0;
  size_t _tail = 0;
  size_t _count = 0;
  uint32_t _parsed = 0;
  uint32_t _dropped = 0;
  uint32_t _sent = 0;
  bool _enabled = false;

  void popHead();
};

extern CoinD6 lidar;
extern WiFiServer lidarServer;
extern WiFiClient lidarClient;
extern QueuedRawCapsuleStream lidarStream;
extern bool lidarHasClient;

void serviceLidarClient();
bool startLidarScan();
