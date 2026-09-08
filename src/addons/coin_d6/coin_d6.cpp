#include "coin_d6.h"

#include <string.h>

namespace {

const uint8_t kStartScan[] = {0xAA, 0x55, 0xF0, 0x0F};
const uint8_t kStopScan[] = {0xAA, 0x55, 0xF5, 0x0A};

bool expired(uint32_t deadline_ms) {
  return static_cast<int32_t>(deadline_ms - millis()) <= 0;
}

}  // namespace

CoinD6::CoinD6(HardwareSerial& serial) : _serial(serial) {}

void CoinD6::flushInput() {
  while (_serial.available()) {
    _serial.read();
  }
}

bool CoinD6::reopen(uint32_t baud) {
  _serial.end();
  delay(50);
  _serial.setRxBufferSize(4096);
  _serial.begin(baud, SERIAL_8N1, _rx_pin, _tx_pin);
  _baud = baud;
  delay(100);
  return true;
}

void CoinD6::sendCommand(const uint8_t* cmd, size_t len) {
  _serial.write(cmd, len);
  _serial.flush();
}

void CoinD6::resetParser() {
  _parse = ParseState::Hunt;
  _hdr0 = 0;
  _pos = 0;
  _frame_len = 0;
}

bool CoinD6::begin(int8_t rx_pin, int8_t tx_pin, uint32_t baud) {
  _rx_pin = rx_pin;
  _tx_pin = tx_pin;
  reopen(baud);
  return true;
}

bool CoinD6::connect(int8_t rx_pin, int8_t tx_pin) {
  _rx_pin = rx_pin;
  _tx_pin = tx_pin;
  reopen(COIN_D6_BAUD);
  flushInput();
  resetParser();
  _queue_head = 0;
  _queue_tail = 0;
  delay(200);

  sendCommand(kStartScan, sizeof(kStartScan));
  const uint32_t start = millis();
  while (millis() - start < 3000) {
    pumpScan();
    if (_queue_head != _queue_tail || _info.valid) {
      break;
    }
    delay(0);
  }
  const bool ok = (_queue_head != _queue_tail) || _info.valid;
  stop();
  return ok;
}

bool CoinD6::stop() {
  sendCommand(kStopScan, sizeof(kStopScan));
  delay(20);
  flushInput();
  resetParser();
  _queue_head = 0;
  _queue_tail = 0;
  _scanning = false;
  _last_frame_len = 0;
  return true;
}

bool CoinD6::getDeviceInfo(CoinD6DeviceInfo& info, uint32_t timeout_ms) {
  if (_info.valid) {
    info = _info;
    return true;
  }

  const uint32_t deadline = millis() + timeout_ms;
  while (!expired(deadline)) {
    pumpScan();
    if (_info.valid) {
      info = _info;
      return true;
    }
    delay(0);
  }
  info = _info;
  return _info.valid;
}

bool CoinD6::startScan() {
  resetParser();
  _queue_head = 0;
  _queue_tail = 0;
  _scanning = true;
  sendCommand(kStartScan, sizeof(kStartScan));
  const uint32_t start = millis();
  while (millis() - start < 3000) {
    pumpScan();
    if (_queue_head != _queue_tail) {
      return true;
    }
    delay(0);
  }
  _scanning = false;
  return false;
}

void CoinD6::pumpScan() {
  while (_serial.available()) {
    feedByte(static_cast<uint8_t>(_serial.read()));
  }
}

bool CoinD6::resyncScanStream() {
  resetParser();
  bool decoded = false;
  while (_serial.available()) {
    if (feedByte(static_cast<uint8_t>(_serial.read()))) {
      decoded = true;
    }
  }
  return decoded;
}

bool CoinD6::readPoint(LidarPoint& point, uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    pumpScan();
    if (dequeuePoint(point)) {
      return true;
    }
    delay(0);
  }

  if (resyncScanStream() && dequeuePoint(point)) {
    return true;
  }
  return false;
}

LidarScan& CoinD6::getPoints(uint32_t point_timeout_ms, LidarPointStream* stream) {
  _scan.clear();
  if (stream) {
    stream->beginRotation();
  }

  while (true) {
    LidarPoint point;
    if (!readPoint(point, point_timeout_ms)) {
      resyncScanStream();
      continue;
    }

    if (point.sync && _scan.count() > 0) {
      break;
    }

    if (_scan.add(point) && stream) {
      stream->onPoint(point);
    }
  }

  if (stream) {
    stream->endRotation(_scan.count());
  }

  pumpScan();
  return _scan;
}

bool CoinD6::feedByte(uint8_t byte) {
  if (_parse == ParseState::Hunt) {
    if (_hdr0 == 0) {
      if (byte == 0xAA || byte == 0xA5) {
        _hdr0 = byte;
      }
      return false;
    }
    if (_hdr0 == 0xAA) {
      if (byte == 0x55) {
        _buf[0] = 0xAA;
        _buf[1] = 0x55;
        _pos = 2;
        _frame_len = kHeaderLen;
        _parse = ParseState::Scan;
      } else if (byte == 0xAA) {
        _hdr0 = 0xAA;
      } else if (byte == 0xA5) {
        _hdr0 = 0xA5;
      } else {
        _hdr0 = 0;
      }
      return false;
    }
    if (byte == 0x5A) {
      _buf[0] = 0xA5;
      _buf[1] = 0x5A;
      _pos = 2;
      _frame_len = 7;
      _parse = ParseState::Ctrl;
    } else if (byte == 0xAA) {
      _hdr0 = 0xAA;
    } else if (byte == 0xA5) {
      _hdr0 = 0xA5;
    } else {
      _hdr0 = 0;
    }
    return false;
  }

  if (_pos >= sizeof(_buf)) {
    resetParser();
    return false;
  }
  _buf[_pos++] = byte;

  if (_parse == ParseState::Scan) {
    if (_pos == 4) {
      const uint8_t lsn = _buf[3];
      if (lsn == 0 || lsn > kMaxLsn) {
        resetParser();
        return false;
      }
      _frame_len = kHeaderLen + static_cast<size_t>(lsn) * 3;
    }
    if (_pos >= _frame_len && _pos >= kHeaderLen) {
      return finishScanPacket();
    }
    return false;
  }

  if (_pos == 7) {
    const uint16_t length = read_u16_le(_buf + 2);
    if (length > kMaxCtrlData) {
      resetParser();
      return false;
    }
    _frame_len = 7 + length;
  }
  if (_pos >= _frame_len && _pos >= 7) {
    finishCtrlFrame();
  }
  return false;
}

bool CoinD6::finishScanPacket() {
  const uint8_t lsn = _buf[3];
  const bool ok = checksum(_buf, lsn) == read_u16_le(_buf + 8);
  if (ok) {
    _last_frame_len = _frame_len;
    if (_raw_stream) {
      _raw_stream->onCapsule(_buf, _frame_len);
    }
    decodePacket(_buf, lsn);
  }
  resetParser();
  return ok;
}

bool CoinD6::finishCtrlFrame() {
  const uint16_t length = read_u16_le(_buf + 2);
  const uint8_t type = _buf[6];
  const uint8_t* data = _buf + 7;
  if (type == 0x01 && length >= 20) {
    memcpy(_info.model, data, 12);
    _info.model[12] = '\0';
    _info.software_version = data[19];
    _info.valid = true;
  }
  resetParser();
  return true;
}

uint16_t CoinD6::checksum(const uint8_t* raw, uint8_t lsn) const {
  uint16_t cs = 0x55AA;
  cs ^= read_u16_le(raw + 4);
  const uint8_t* sample = raw + kHeaderLen;
  for (uint8_t i = 0; i < lsn; ++i) {
    cs ^= sample[0];
    cs ^= static_cast<uint16_t>(sample[1] | (static_cast<uint16_t>(sample[2]) << 8));
    sample += 3;
  }
  cs ^= static_cast<uint16_t>(raw[2] | (static_cast<uint16_t>(raw[3]) << 8));
  cs ^= read_u16_le(raw + 6);
  return cs;
}

void CoinD6::decodePacket(const uint8_t* raw, uint8_t lsn) {
  const uint8_t m_and_t = raw[2];
  const bool is_start = (m_and_t & 0x01) != 0;
  if (is_start) {
    _scan_hz = (m_and_t >> 1) / 10.0f;
  }

  const uint16_t fsa = read_u16_le(raw + 4);
  const uint16_t lsa = read_u16_le(raw + 6);
  float start_deg = (fsa >> 1) / 64.0f;
  float end_deg = (lsa >> 1) / 64.0f;
  float step = 0.0f;
  if (lsn > 1) {
    float delta = end_deg - start_deg;
    if (delta < 0.0f) {
      delta += 360.0f;
    }
    step = delta / static_cast<float>(lsn - 1);
  }

  const uint8_t* sample = raw + kHeaderLen;
  for (uint8_t i = 0; i < lsn; ++i) {
    float angle = start_deg + step * static_cast<float>(i);
    if (angle >= 360.0f) {
      angle -= 360.0f;
    }
    LidarPoint point;
    point.angle_deg = angle;
    point.distance_mm = static_cast<float>((sample[2] * 64) + (sample[1] >> 2));
    point.quality = static_cast<uint8_t>(((sample[1] & 0x03) * 64) + (sample[0] >> 2));
    point.sync = is_start && i == 0;
    enqueuePoint(point);
    sample += 3;
  }
}

bool CoinD6::dropOldestNonSyncPoint() {
  if (_queue_head == _queue_tail) {
    return false;
  }

  size_t idx = _queue_head;
  while (idx != _queue_tail) {
    if (!_point_queue[idx].sync) {
      size_t next = (idx + 1) % POINT_QUEUE_SIZE;
      while (next != _queue_tail) {
        _point_queue[idx] = _point_queue[next];
        idx = next;
        next = (next + 1) % POINT_QUEUE_SIZE;
      }
      _queue_tail = (_queue_tail + POINT_QUEUE_SIZE - 1) % POINT_QUEUE_SIZE;
      return true;
    }
    idx = (idx + 1) % POINT_QUEUE_SIZE;
  }
  return false;
}

void CoinD6::enqueuePoint(const LidarPoint& point) {
  while (((_queue_tail + 1) % POINT_QUEUE_SIZE) == _queue_head) {
    if (!dropOldestNonSyncPoint()) {
      LidarPoint dropped;
      dequeuePoint(dropped);
      break;
    }
  }

  const size_t next_tail = (_queue_tail + 1) % POINT_QUEUE_SIZE;
  if (next_tail == _queue_head) {
    return;
  }
  _point_queue[_queue_tail] = point;
  _queue_tail = next_tail;
}

bool CoinD6::dequeuePoint(LidarPoint& point) {
  if (_queue_head == _queue_tail) {
    return false;
  }
  point = _point_queue[_queue_head];
  _queue_head = (_queue_head + 1) % POINT_QUEUE_SIZE;
  return true;
}

uint16_t CoinD6::read_u16_le(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}
