#pragma once

#include <WiFi.h>
#include <stdint.h>

inline bool socketHasSpace(WiFiClient& client, size_t need) {
  const int space = client.availableForWrite();
  // Arduino Client defaults to 0 when the socket does not report buffer space.
  return space <= 0 || space >= static_cast<int>(need);
}

inline void configureTcpClient(WiFiClient& client) {
  client.setNoDelay(true);
  client.setTimeout(1);
}

inline int16_t readI16Le(const uint8_t* data) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
}

inline void writeU32Le(uint8_t* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
  data[2] = static_cast<uint8_t>(value >> 16);
  data[3] = static_cast<uint8_t>(value >> 24);
}

inline void writeI32Le(uint8_t* data, int32_t value) {
  writeU32Le(data, static_cast<uint32_t>(value));
}
