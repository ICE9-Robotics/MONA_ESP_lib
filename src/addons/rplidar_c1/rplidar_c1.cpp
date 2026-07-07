#include "rplidar_c1.h"

#include <string.h>

RPLidarC1::RPLidarC1(HardwareSerial& serial) : _serial(serial) {}

void RPLidarC1::flushInput() {
  while (_serial.available()) {
    _serial.read();
  }
}

bool RPLidarC1::reopen(uint32_t baud) {
  _serial.end();
  delay(50);
  _serial.begin(baud, SERIAL_8N1, _rx_pin, _tx_pin);
  _serial.setRxBufferSize(8192);
  _baud = baud;
  delay(100);
  return true;
}

bool RPLidarC1::begin(int8_t rx_pin, int8_t tx_pin, uint32_t baud) {
  _rx_pin = rx_pin;
  _tx_pin = tx_pin;
  reopen(baud);
  return true;
}

bool RPLidarC1::connect(int8_t rx_pin, int8_t tx_pin) {
  _rx_pin = rx_pin;
  _tx_pin = tx_pin;
  reopen(RPLIDAR_C1_BAUD);
  flushInput();
  delay(200);

  for (int attempt = 0; attempt < 2; ++attempt) {
    stop();
    delay(50);

    LidarDeviceInfo info;
    if (getDeviceInfo(info, 1500)) {
      return true;
    }
  }

  return false;
}

void RPLidarC1::dumpResponse(Stream& out, uint32_t timeout_ms) {
  size_t count = 0;
  const uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    while (_serial.available()) {
      const uint8_t byte = _serial.read();
      out.print("  RX: 0x");
      if (byte < 16) {
        out.print('0');
      }
      out.println(byte, HEX);
      ++count;
    }
    delay(1);
  }
  if (count == 0) {
    out.println("  (no response)");
  }
}

void RPLidarC1::sendCommand(uint8_t cmd, const void* payload, size_t payload_size) {
  uint8_t checksum = 0;
  uint8_t header[3];

  if (payload && payload_size > 0) {
    header[0] = CMD_SYNC_BYTE;
    header[1] = cmd | 0x80;
    header[2] = static_cast<uint8_t>(payload_size);
    for (size_t i = 0; i < 3; ++i) {
      checksum ^= header[i];
      _serial.write(header[i]);
    }
    const uint8_t* bytes = static_cast<const uint8_t*>(payload);
    for (size_t i = 0; i < payload_size; ++i) {
      checksum ^= bytes[i];
      _serial.write(bytes[i]);
    }
    _serial.write(checksum);
  } else {
    _serial.write(CMD_SYNC_BYTE);
    _serial.write(cmd);
  }
  _serial.flush();
}

uint16_t RPLidarC1::read_u16_le(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t RPLidarC1::read_u32_le(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}

bool RPLidarC1::waitResponse(uint8_t expected_type, uint8_t* payload, size_t payload_size,
                             uint32_t timeout_ms, uint8_t* actual_type) {
  const uint32_t start = millis();
  enum State { WAIT_SYNC1, WAIT_SYNC2, WAIT_SIZE, WAIT_TYPE, WAIT_PAYLOAD };
  State state = WAIT_SYNC1;
  uint8_t size_bytes[4] = {};
  uint8_t size_pos = 0;
  uint32_t payload_len = 0;
  uint8_t response_type = 0;
  size_t payload_pos = 0;

  auto reset_parser = [&]() {
    state = WAIT_SYNC1;
    size_pos = 0;
    payload_len = 0;
    response_type = 0;
    payload_pos = 0;
  };

  while (millis() - start < timeout_ms) {
    while (_serial.available()) {
      const uint8_t byte = _serial.read();

      switch (state) {
        case WAIT_SYNC1:
          if (byte == ANS_SYNC_BYTE1) {
            state = WAIT_SYNC2;
          }
          break;
        case WAIT_SYNC2:
          if (byte == ANS_SYNC_BYTE2) {
            state = WAIT_SIZE;
            size_pos = 0;
          } else {
            state = WAIT_SYNC1;
          }
          break;
        case WAIT_SIZE:
          size_bytes[size_pos++] = byte;
          if (size_pos == 4) {
            payload_len = read_u32_le(size_bytes) & 0x3FFFFFFF;
            state = WAIT_TYPE;
          }
          break;
        case WAIT_TYPE:
          response_type = byte;
          payload_pos = 0;
          state = payload_len > 0 ? WAIT_PAYLOAD : WAIT_SYNC1;
          if (payload_len == 0 && response_type == expected_type) {
            if (actual_type) {
              *actual_type = response_type;
            }
            return true;
          }
          break;
        case WAIT_PAYLOAD:
          if (payload && payload_pos < payload_size) {
            payload[payload_pos] = byte;
          }
          ++payload_pos;
          if (payload_pos >= payload_len) {
            if (response_type == expected_type) {
              if (actual_type) {
                *actual_type = response_type;
              }
              return true;
            }
            reset_parser();
          }
          break;
      }
    }
    delay(1);
  }

  return false;
}

bool RPLidarC1::getLidarConfU32(uint32_t conf_type, uint32_t& value, const void* extra,
                                size_t extra_size, uint32_t timeout_ms) {
  const bool resume = _scan_grabbing;
  if (resume) {
    setScanGrabbing(false);
  }

  uint8_t request[8] = {};
  memcpy(request, &conf_type, sizeof(conf_type));
  if (extra && extra_size > 0) {
    memcpy(request + sizeof(conf_type), extra, extra_size);
  }

  const size_t request_size = sizeof(conf_type) + extra_size;
  flushInput();
  sendCommand(CMD_GET_LIDAR_CONF, request, request_size);
  delay(10);

  uint8_t payload[16] = {};
  const bool ok = waitResponse(ANS_TYPE_GET_LIDAR_CONF, payload, sizeof(payload), timeout_ms);
  if (resume) {
    setScanGrabbing(true);
  }
  if (!ok) {
    return false;
  }

  const uint32_t replied_type = read_u32_le(payload);
  if (replied_type != conf_type) {
    return false;
  }

  value = read_u32_le(payload + 4);
  return true;
}

bool RPLidarC1::getTypicalScanMode(uint16_t& mode, uint32_t timeout_ms) {
  uint32_t raw = 0;
  if (!getLidarConfU32(CONF_SCAN_MODE_TYPICAL, raw, nullptr, 0, timeout_ms)) {
    return false;
  }
  mode = static_cast<uint16_t>(raw & 0xFFFF);
  return true;
}

bool RPLidarC1::getSampleDurationUs(uint16_t scan_mode, uint32_t timeout_ms) {
  uint32_t raw = 0;
  if (!getLidarConfU32(CONF_SCAN_MODE_US_PER_SAMPLE, raw, &scan_mode, sizeof(scan_mode),
                       timeout_ms)) {
    return false;
  }
  _us_per_sample = static_cast<uint16_t>(raw / 256);
  if (_us_per_sample == 0) {
    _us_per_sample = 200;
  }
  return true;
}

bool RPLidarC1::getDesiredMotorRpm(uint16_t& rpm, uint32_t timeout_ms) {
  uint32_t raw = 0;
  if (!getLidarConfU32(CONF_DESIRED_ROT_FREQ, raw, nullptr, 0, timeout_ms)) {
    return false;
  }
  rpm = static_cast<uint16_t>(raw & 0xFFFF);
  return true;
}

bool RPLidarC1::getScanModeAnsType(uint16_t scan_mode, uint8_t& ans_type, uint32_t timeout_ms) {
  uint32_t raw = 0;
  if (!getLidarConfU32(CONF_SCAN_MODE_ANS_TYPE, raw, &scan_mode, sizeof(scan_mode),
                       timeout_ms)) {
    return false;
  }
  ans_type = static_cast<uint8_t>(raw & 0xFF);
  return true;
}

bool RPLidarC1::reset() {
  sendCommand(CMD_RESET);
  delay(2000);
  flushInput();
  return true;
}

bool RPLidarC1::stop() {
  sendCommand(CMD_STOP);
  delay(200);
  flushInput();
  _capsule_pos = 0;
  _capsule_size = 0;
  _scan_type = 0;
  _scan_format = ScanFormat::None;
  _scan_grabbing = false;
  _prev_capsule_ready = false;
  _last_sync_bit = 0;
  _last_dist_q2 = 0;
  _queue_head = 0;
  _queue_tail = 0;
  return true;
}

void RPLidarC1::setScanGrabbing(bool enabled) {
  _scan_grabbing = enabled;
  if (!enabled) {
    flushInput();
    _capsule_pos = 0;
  }
}

bool RPLidarC1::getDeviceInfo(LidarDeviceInfo& info, uint32_t timeout_ms) {
  const bool resume = _scan_grabbing;
  if (resume) {
    setScanGrabbing(false);
  }
  flushInput();
  sendCommand(CMD_GET_DEVICE_INFO);
  delay(10);

  uint8_t payload[20] = {};
  const bool ok = waitResponse(ANS_TYPE_DEVINFO, payload, sizeof(payload), timeout_ms);
  if (resume) {
    setScanGrabbing(true);
  }
  if (!ok) {
    return false;
  }

  info.model = payload[0];
  info.firmware_version = read_u16_le(payload + 1);
  info.hardware_version = payload[3];
  memcpy(info.serialnum, payload + 4, 16);
  return true;
}

bool RPLidarC1::getHealth(LidarHealth& health, uint32_t timeout_ms) {
  const bool resume = _scan_grabbing;
  if (resume) {
    setScanGrabbing(false);
  }
  flushInput();
  sendCommand(CMD_GET_DEVICE_HEALTH);
  delay(10);

  uint8_t payload[3] = {};
  const bool ok = waitResponse(ANS_TYPE_DEVHEALTH, payload, sizeof(payload), timeout_ms);
  if (resume) {
    setScanGrabbing(true);
  }
  if (!ok) {
    return false;
  }

  health.status = payload[0];
  health.error_code = read_u16_le(payload + 1);
  return true;
}

bool RPLidarC1::startMotor(uint16_t rpm) {
  if (rpm == 0) {
    if (!getDesiredMotorRpm(rpm)) {
      rpm = 600;
    }
  }

  struct __attribute__((packed)) MotorPayload {
    uint16_t rpm;
  } payload = {rpm};

  sendCommand(CMD_HQ_MOTOR_SPEED_CTRL, &payload, sizeof(payload));
  delay(50);
  return true;
}

bool RPLidarC1::stopMotor() {
  struct __attribute__((packed)) MotorPayload {
    uint16_t rpm;
  } payload = {0};

  sendCommand(CMD_HQ_MOTOR_SPEED_CTRL, &payload, sizeof(payload));
  delay(50);
  return true;
}

bool RPLidarC1::waitForScanHeader(uint32_t timeout_ms) {
  const uint32_t start = millis();
  enum State { WAIT_SYNC1, WAIT_SYNC2, WAIT_SIZE, WAIT_TYPE, WAIT_PAYLOAD };
  State state = WAIT_SYNC1;
  uint8_t size_bytes[4] = {};
  uint8_t size_pos = 0;
  uint32_t payload_len = 0;

  while (millis() - start < timeout_ms) {
    while (_serial.available()) {
      const uint8_t byte = _serial.read();

      switch (state) {
        case WAIT_SYNC1:
          if (byte == ANS_SYNC_BYTE1) {
            state = WAIT_SYNC2;
          }
          break;
        case WAIT_SYNC2:
          if (byte == ANS_SYNC_BYTE2) {
            state = WAIT_SIZE;
            size_pos = 0;
          } else {
            state = WAIT_SYNC1;
          }
          break;
        case WAIT_SIZE:
          size_bytes[size_pos++] = byte;
          if (size_pos == 4) {
            payload_len = read_u32_le(size_bytes) & 0x3FFFFFFF;
            state = WAIT_TYPE;
          }
          break;
        case WAIT_TYPE:
          if (byte != _scan_type) {
            return false;
          }
          if (payload_len != 0 && payload_len != _capsule_size) {
            return false;
          }
          _capsule_pos = 0;

          state = payload_len > 0 ? WAIT_PAYLOAD : WAIT_SYNC1;
          if (payload_len == 0) {
            return true;
          }
          break;
        case WAIT_PAYLOAD:
          _capsule_buf[_capsule_pos++] = byte;
          if (_capsule_pos >= _capsule_size) {
            _capsule_pos = 0;
            decodeCapsule();
            return true;
          }
          break;
      }
    }
    delay(1);
  }

  return false;
}

bool RPLidarC1::startScan() {
  stop();

  uint16_t scan_mode = 0;
  if (!getTypicalScanMode(scan_mode)) {
    scan_mode = 0;
  }
  getSampleDurationUs(scan_mode);

  uint8_t ans_type = 0;
  if (!getScanModeAnsType(scan_mode, ans_type)) {
    return false;
  }
  if (ans_type == ANS_TYPE_MEASUREMENT_DENSE_CAPSULED) {
    _scan_format = ScanFormat::Dense;
    _scan_type = ans_type;
    _capsule_size = DENSE_CAPSULE_SIZE;
  } else if (ans_type == ANS_TYPE_MEASUREMENT_ULTRA_DENSE_CAPSULED) {
    _scan_format = ScanFormat::UltraDense;
    _scan_type = ans_type;
    _capsule_size = ULTRA_DENSE_CAPSULE_SIZE;
  } else {
    return false;
  }

  struct __attribute__((packed)) ExpressScanPayload {
    uint8_t working_mode;
    uint16_t working_flags;
    uint16_t param;
  } scan_req = {static_cast<uint8_t>(scan_mode), 0, 0};

  startMotor();
  delay(300);

  flushInput();
  sendCommand(CMD_EXPRESS_SCAN, &scan_req, sizeof(scan_req));

  if (!waitForScanHeader(3000)) {
    return false;
  }

  _scan_grabbing = true;
  return _scan_format != ScanFormat::None;
}

bool RPLidarC1::dropOldestNonSyncPoint() {
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

void RPLidarC1::enqueuePoint(const LidarPoint& point) {
  while (((_queue_tail + 1) % POINT_QUEUE_SIZE) == _queue_head) {
    if (!dropOldestNonSyncPoint()) {
      // Queue holds only sync markers; drop oldest to make room.
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

bool RPLidarC1::dequeuePoint(LidarPoint& point) {
  if (_queue_head == _queue_tail) {
    return false;
  }
  point = _point_queue[_queue_head];
  _queue_head = (_queue_head + 1) % POINT_QUEUE_SIZE;
  return true;
}

void RPLidarC1::decodeCapsule() {
  if (_raw_stream) {
    _raw_stream->onCapsule(_capsule_buf, _capsule_size);
  }

  if (_scan_format == ScanFormat::UltraDense) {
    UltraDenseCapsule capsule;
    memcpy(&capsule, _capsule_buf, _capsule_size);
    decodeUltraDenseCapsule(capsule);
  } else if (_scan_format == ScanFormat::Dense) {
    DenseCapsule capsule;
    memcpy(&capsule, _capsule_buf, _capsule_size);
    decodeDenseCapsule(capsule);
  }
}

void RPLidarC1::decodeDenseCapsule(const DenseCapsule& capsule) {
  uint8_t checksum = 0;
  const uint8_t* raw = reinterpret_cast<const uint8_t*>(&capsule);
  const uint8_t recv_checksum = (capsule.checksum_1 & 0x0F) | ((capsule.checksum_2 & 0x0F) << 4);

  for (size_t i = offsetof(DenseCapsule, start_angle_sync_q6); i < _capsule_size; ++i) {
    checksum ^= raw[i];
  }
  if (recv_checksum != checksum) {
    _prev_capsule_ready = false;
    return;
  }

  if (capsule.start_angle_sync_q6 & EXP_SYNCBIT) {
    _prev_capsule_ready = false;
    _last_sync_bit = 0;
  }

  if (_prev_capsule_ready) {
    int current_start_q8 = (capsule.start_angle_sync_q6 & 0x7FFF) << 2;
    int prev_start_q8 = (_prev_dense.start_angle_sync_q6 & 0x7FFF) << 2;
    int diff_q8 = current_start_q8 - prev_start_q8;
    if (prev_start_q8 > current_start_q8) {
      diff_q8 += (360 << 8);
    }

    const int max_diff_q8 =
        (360 * 100 * static_cast<int>(DENSE_CABIN_COUNT) / (1000000 / _us_per_sample)) << 8;
    if (diff_q8 <= max_diff_q8) {
      const int angle_inc_q16 = (diff_q8 << 8) / static_cast<int>(DENSE_CABIN_COUNT);
      int current_angle_q16 = prev_start_q8 << 8;

      for (size_t pos = 0; pos < DENSE_CABIN_COUNT; ++pos) {
        const uint16_t dist = _prev_dense.cabins[pos];
        int angle_q6 = current_angle_q16 >> 10;
        int sync_bit =
            (((current_angle_q16 + angle_inc_q16) % (360 << 16)) < (angle_inc_q16 << 1)) ? 1 : 0;
        sync_bit = (sync_bit ^ _last_sync_bit) & sync_bit;
        current_angle_q16 += angle_inc_q16;

        if (angle_q6 < 0) {
          angle_q6 += (360 << 6);
        }
        if (angle_q6 >= (360 << 6)) {
          angle_q6 -= (360 << 6);
        }

        LidarPoint point;
        point.angle_deg = angle_q6 / 64.0f;
        point.distance_mm = static_cast<float>(dist);
        point.quality = dist ? 47 : 0;
        point.sync = sync_bit != 0;
        _last_sync_bit = sync_bit;
        enqueuePoint(point);
      }
    }
  }

  _prev_dense = capsule;
  _prev_capsule_ready = true;
}

void RPLidarC1::decodeUltraDenseCapsule(const UltraDenseCapsule& capsule) {
  uint8_t checksum = 0;
  const uint8_t* raw = reinterpret_cast<const uint8_t*>(&capsule);
  const uint8_t recv_checksum = (capsule.checksum_1 & 0x0F) | ((capsule.checksum_2 & 0x0F) << 4);

  for (size_t i = offsetof(UltraDenseCapsule, time_stamp); i < _capsule_size; ++i) {
    checksum ^= raw[i];
  }
  if (recv_checksum != checksum) {
    _prev_capsule_ready = false;
    return;
  }

  if (capsule.start_angle_sync_q6 & EXP_SYNCBIT) {
    _prev_capsule_ready = false;
    _last_sync_bit = 0;
    _last_dist_q2 = 0;
  }

  if (_prev_capsule_ready) {
    int current_start_q8 = (capsule.start_angle_sync_q6 & 0x7FFF) << 2;
    int prev_start_q8 = (_prev_ultra_dense.start_angle_sync_q6 & 0x7FFF) << 2;
    int diff_q8 = current_start_q8 - prev_start_q8;
    if (prev_start_q8 > current_start_q8) {
      diff_q8 += (360 << 8);
    }

    const int max_diff_q8 =
        (360 * 100 * 64 / (1000000 / _us_per_sample)) << 8;
    if (diff_q8 <= max_diff_q8) {
      const int angle_inc_q16 = (diff_q8 << 8) / 64;
      int current_angle_q16 = prev_start_q8 << 8;

      for (int pos = 0; pos < 64; ++pos) {
        const size_t cabin_idx = static_cast<size_t>(pos >> 1);
        uint32_t quality_dist_scale;
        if ((pos & 0x1) == 0) {
          quality_dist_scale = _prev_ultra_dense.cabins[cabin_idx].qualityl_distance_scale[0] |
                               ((_prev_ultra_dense.cabins[cabin_idx].qualityh_array & 0x0F) << 16);
        } else {
          quality_dist_scale = _prev_ultra_dense.cabins[cabin_idx].qualityl_distance_scale[1] |
                               ((_prev_ultra_dense.cabins[cabin_idx].qualityh_array >> 4) << 16);
        }

        const uint8_t scale = quality_dist_scale & 0x3;
        uint8_t quality = 0;
        int dist_q2 = 0;

        switch (scale) {
          case 0:
            quality = quality_dist_scale >> 12;
            dist_q2 = (quality_dist_scale & 0xFFC) * 2;
            if (_last_dist_q2 && abs(dist_q2 - _last_dist_q2) <= 8) {
              dist_q2 = (dist_q2 + _last_dist_q2) >> 1;
            }
            break;
          case 1:
            quality = (quality_dist_scale >> 13) << 1;
            dist_q2 = (quality_dist_scale & 0x1FFC) * 3 + (2046 << 2);
            break;
          case 2:
            quality = (quality_dist_scale >> 14) << 2;
            dist_q2 = (quality_dist_scale & 0x3FFC) * 4 + (8187 << 2);
            break;
          case 3:
            quality = (quality_dist_scale >> 15) << 3;
            dist_q2 = (quality_dist_scale & 0x7FFC) * 5 + (24567 << 2);
            break;
        }
        _last_dist_q2 = dist_q2;

        int angle_q6 = current_angle_q16 >> 10;
        int sync_bit =
            (((current_angle_q16 + angle_inc_q16) % (360 << 16)) < (angle_inc_q16 << 1)) ? 1 : 0;
        sync_bit = (sync_bit ^ _last_sync_bit) & sync_bit;
        current_angle_q16 += angle_inc_q16;

        if (angle_q6 < 0) {
          angle_q6 += (360 << 6);
        }
        if (angle_q6 >= (360 << 6)) {
          angle_q6 -= (360 << 6);
        }

        LidarPoint point;
        point.angle_deg = angle_q6 / 64.0f;
        point.distance_mm = dist_q2 / 4.0f;
        point.quality = quality;
        point.sync = sync_bit != 0;
        _last_sync_bit = sync_bit;
        enqueuePoint(point);
      }
    }
  }

  _prev_ultra_dense = capsule;
  _prev_capsule_ready = true;
}

bool RPLidarC1::feedCapsuleByte(uint8_t byte) {
  if (_capsule_size == 0) {
    return false;
  }

  switch (_capsule_pos) {
    case 0:
      if ((byte >> 4) != 0x0A) {
        _prev_capsule_ready = false;
        return false;
      }
      break;
    case 1:
      if ((byte >> 4) != 0x05) {
        _capsule_pos = 0;
        _prev_capsule_ready = false;
        return false;
      }
      break;
    default:
      if (_capsule_pos == _capsule_size - 1) {
        _capsule_buf[_capsule_pos] = byte;
        _capsule_pos = 0;
        decodeCapsule();
        return true;
      }
      break;
  }

  _capsule_buf[_capsule_pos++] = byte;
  return false;
}

bool RPLidarC1::readCapsuleBlock(uint32_t timeout_ms, bool allow_partial) {
  if (!_scan_grabbing) {
    return false;
  }

  const uint32_t start = millis();

  while (true) {
    while (_serial.available()) {
      if (feedCapsuleByte(_serial.read())) {
        return true;
      }
    }
    if (allow_partial) {
      return false;
    }
    if (millis() - start >= timeout_ms) {
      return false;
    }
    delay(1);
  }
}

bool RPLidarC1::resyncScanStream() {
  if (!_scan_grabbing || _scan_format == ScanFormat::None || _capsule_size == 0) {
    return false;
  }

  _capsule_pos = 0;
  _prev_capsule_ready = false;
  _last_sync_bit = 0;
  _last_dist_q2 = 0;

  bool decoded = false;
  while (_serial.available()) {
    if (feedCapsuleByte(_serial.read())) {
      decoded = true;
    }
  }

  return decoded;
}

void RPLidarC1::pumpScan() {
  if (!_scan_grabbing || _scan_format == ScanFormat::None || _capsule_size == 0) {
    return;
  }

  while (_serial.available()) {
    feedCapsuleByte(_serial.read());
  }
}

LidarScan& RPLidarC1::getPoints(uint32_t point_timeout_ms, LidarPointStream* stream) {
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

bool RPLidarC1::readPoint(LidarPoint& point, uint32_t timeout_ms) {
  const uint32_t start = millis();

  while (millis() - start < timeout_ms) {
    pumpScan();

    if (dequeuePoint(point)) {
      return true;
    }

    if (_scan_format == ScanFormat::None || _capsule_size == 0) {
      delay(1);
      continue;
    }

    const uint32_t remaining = timeout_ms - (millis() - start);
    if (readCapsuleBlock(remaining > 0 ? remaining : 1)) {
      if (dequeuePoint(point)) {
        return true;
      }
    }
  }

  if (resyncScanStream()) {
    if (dequeuePoint(point)) {
      return true;
    }
  }

  return false;
}
