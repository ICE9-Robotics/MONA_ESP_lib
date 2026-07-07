#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

#include "lidar_points.h"

/** Default UART baud rate for the RPLIDAR C1. */
static const uint32_t RPLIDAR_C1_BAUD = 460800;

/** Device identification returned by the lidar GET_DEVICE_INFO command. */
struct LidarDeviceInfo {
  uint8_t model;
  uint16_t firmware_version;
  uint8_t hardware_version;
  uint8_t serialnum[16];
};

/** Device health status returned by the lidar GET_DEVICE_HEALTH command. */
struct LidarHealth {
  /** 0 = OK, 1 = warning, 2 = error. */
  uint8_t status;
  uint16_t error_code;
};

/**
 * Driver for the SLAMTEC RPLIDAR C1 on ESP32 over UART.
 *
 * Handles command/response exchange, express-scan startup, dense and
 * ultra-dense capsule decoding, and point queueing. Typical usage:
 *
 * @code
 * RPLidarC1 lidar(Serial1);
 * lidar.begin(rx_pin, tx_pin);
 * lidar.connect(rx_pin, tx_pin);
 * lidar.startScan();
 * LidarScan& scan = lidar.getPoints();
 * @endcode
 */
class RPLidarC1 {
 public:
  /**
   * @param serial Hardware UART used to talk to the lidar.
   */
  explicit RPLidarC1(HardwareSerial& serial);

  /**
   * Open the UART on the given pins.
   *
   * @param rx_pin ESP32 pin connected to the lidar TX line.
   * @param tx_pin ESP32 pin connected to the lidar RX line.
   * @param baud UART baud rate; defaults to @ref RPLIDAR_C1_BAUD.
   * @return Always true once the serial port has been configured.
   */
  bool begin(int8_t rx_pin, int8_t tx_pin, uint32_t baud = RPLIDAR_C1_BAUD);

  /**
   * Verify communication with the lidar.
   *
   * Reopens the UART, stops any active scan, and reads device info with
   * one retry. Does not start scanning.
   *
   * @param rx_pin ESP32 pin connected to the lidar TX line.
   * @param tx_pin ESP32 pin connected to the lidar RX line.
   * @return True if device info was received.
   */
  bool connect(int8_t rx_pin, int8_t tx_pin);

  /** Send RESET and wait for the lidar to reboot. */
  bool reset();

  /**
   * Stop scanning and clear internal scan/parser state.
   *
   * Sends STOP to the lidar, flushes pending UART input, and empties the
   * point queue.
   */
  bool stop();

  /**
   * Read lidar model, firmware, hardware, and serial number.
   *
   * @param info Populated on success.
   * @param timeout_ms Maximum wait for the response.
   * @return True if a valid device-info response was received.
   */
  bool getDeviceInfo(LidarDeviceInfo& info, uint32_t timeout_ms = 2000);

  /**
   * Read lidar health status and error code.
   *
   * @param health Populated on success.
   * @param timeout_ms Maximum wait for the response.
   * @return True if a valid health response was received.
   */
  bool getHealth(LidarHealth& health, uint32_t timeout_ms = 2000);

  /**
   * Start the lidar motor at the given RPM.
   *
   * @param rpm Target motor speed. Pass 0 to query the lidar for its desired
   *            rotation frequency, falling back to 600 RPM if unavailable.
   * @return True after the motor command has been sent.
   */
  bool startMotor(uint16_t rpm = 0);

  /** Stop the lidar motor (0 RPM). */
  bool stopMotor();

  /**
   * Start express scanning in the lidar's typical scan mode.
   *
   * Queries scan-mode configuration, starts the motor, sends EXPRESS_SCAN,
   * and waits for the first measurement capsule header. On success,
   * @ref pumpScan(), @ref readPoint(), and @ref getPoints() can consume data.
   *
   * @return True if a supported dense or ultra-dense scan stream started.
   */
  bool startScan();

  /**
   * Read the next decoded scan point.
   *
   * Drains the UART, decodes capsules as needed, and returns the oldest queued
   * point. Attempts stream resync once if the timeout expires with no data.
   *
   * @param point Populated on success.
   * @param timeout_ms Maximum wait for a point.
   * @return True if a point was dequeued.
   */
  bool readPoint(LidarPoint& point, uint32_t timeout_ms = 1000);

  /**
   * Collect one full rotation of points.
   *
   * Reads points until a sync marker arrives after at least one point has
   * been collected. The returned @ref LidarScan is reused on each call.
   * Optionally streams each point through @p stream as it is read.
   *
   * @param point_timeout_ms Per-point read timeout passed to @ref readPoint().
   * @param stream Optional sink for points in the current rotation.
   * @return Reference to the internal scan buffer (valid until the next call).
   */
  LidarScan& getPoints(uint32_t point_timeout_ms = 2000, LidarPointStream* stream = nullptr);

  /**
   * Register a callback for raw measurement capsules before point decoding.
   *
   * @param stream Receives each complete capsule buffer, or nullptr to disable.
   */
  void setRawCapsuleStream(LidarRawCapsuleStream* stream) { _raw_stream = stream; }

  /**
   * Decode any capsules currently available on the UART.
   *
   * Call regularly while scanning to keep the point queue filled, especially
   * when not blocked inside @ref readPoint() or @ref getPoints().
   */
  void pumpScan();

  /**
   * Attempt to recover capsule alignment from buffered UART bytes.
   *
   * Clears partial capsule state and tries to decode complete capsules from
   * bytes already in the serial buffer.
   *
   * @return True if at least one capsule was decoded.
   */
  bool resyncScanStream();

  /** Answer-type byte for the active scan stream (0 if not scanning). */
  uint8_t scanAnswerType() const { return _scan_type; }

  /** Byte length of one measurement capsule in the active scan format. */
  size_t capsuleSize() const { return _capsule_size; }

 private:
  static const uint8_t CMD_SYNC_BYTE = 0xA5;
  static const uint8_t ANS_SYNC_BYTE1 = 0xA5;
  static const uint8_t ANS_SYNC_BYTE2 = 0x5A;

  static const uint8_t CMD_STOP = 0x25;
  static const uint8_t CMD_RESET = 0x40;
  static const uint8_t CMD_GET_DEVICE_INFO = 0x50;
  static const uint8_t CMD_GET_DEVICE_HEALTH = 0x52;
  static const uint8_t CMD_GET_LIDAR_CONF = 0x84;
  static const uint8_t CMD_EXPRESS_SCAN = 0x82;
  static const uint8_t CMD_HQ_MOTOR_SPEED_CTRL = 0xA8;

  static const uint8_t ANS_TYPE_DEVINFO = 0x04;
  static const uint8_t ANS_TYPE_DEVHEALTH = 0x06;
  static const uint8_t ANS_TYPE_GET_LIDAR_CONF = 0x20;
  static const uint8_t ANS_TYPE_MEASUREMENT_DENSE_CAPSULED = 0x85;
  static const uint8_t ANS_TYPE_MEASUREMENT_ULTRA_DENSE_CAPSULED = 0x86;

  static const uint32_t CONF_DESIRED_ROT_FREQ = 0x00000001;
  static const uint32_t CONF_SCAN_MODE_TYPICAL = 0x0000007C;
  static const uint32_t CONF_SCAN_MODE_US_PER_SAMPLE = 0x00000071;
  static const uint32_t CONF_SCAN_MODE_ANS_TYPE = 0x00000075;

  static const uint16_t EXP_SYNCBIT = 0x8000;
  static const size_t DENSE_CAPSULE_SIZE = 84;
  static const size_t DENSE_CABIN_COUNT = 40;
  static const size_t ULTRA_DENSE_CAPSULE_SIZE = 170;
  static const size_t ULTRA_DENSE_CABIN_COUNT = 32;
  static const size_t MAX_CAPSULE_SIZE = ULTRA_DENSE_CAPSULE_SIZE;

  enum class ScanFormat { None, Dense, UltraDense };

  struct DenseCapsule {
    uint8_t checksum_1;
    uint8_t checksum_2;
    uint16_t start_angle_sync_q6;
    uint16_t cabins[DENSE_CABIN_COUNT];
  } __attribute__((packed));

  struct UltraDenseCabin {
    uint16_t qualityl_distance_scale[2];
    uint8_t qualityh_array;
  } __attribute__((packed));

  struct UltraDenseCapsule {
    uint8_t checksum_1;
    uint8_t checksum_2;
    uint32_t time_stamp;
    uint16_t dev_status;
    uint16_t start_angle_sync_q6;
    UltraDenseCabin cabins[ULTRA_DENSE_CABIN_COUNT];
  } __attribute__((packed));

  HardwareSerial& _serial;
  int8_t _rx_pin = -1;
  int8_t _tx_pin = -1;
  uint32_t _baud = RPLIDAR_C1_BAUD;

  uint8_t _capsule_buf[MAX_CAPSULE_SIZE];
  size_t _capsule_pos = 0;
  size_t _capsule_size = 0;
  uint8_t _scan_type = 0;
  ScanFormat _scan_format = ScanFormat::None;
  uint16_t _us_per_sample = 200;
  bool _scan_grabbing = false;

  DenseCapsule _prev_dense{};
  UltraDenseCapsule _prev_ultra_dense{};
  bool _prev_capsule_ready = false;
  int _last_sync_bit = 0;
  int _last_dist_q2 = 0;

  static const size_t POINT_QUEUE_SIZE = 512;
  LidarPoint _point_queue[POINT_QUEUE_SIZE];
  size_t _queue_head = 0;
  size_t _queue_tail = 0;

  LidarScan _scan;
  LidarRawCapsuleStream* _raw_stream = nullptr;

  void flushInput();
  bool reopen(uint32_t baud);
  void dumpResponse(Stream& out, uint32_t timeout_ms);
  void sendCommand(uint8_t cmd, const void* payload = nullptr, size_t payload_size = 0);
  bool waitResponse(uint8_t expected_type, uint8_t* payload, size_t payload_size,
                    uint32_t timeout_ms, uint8_t* actual_type = nullptr);
  bool getLidarConfU32(uint32_t conf_type, uint32_t& value, const void* extra = nullptr,
                       size_t extra_size = 0, uint32_t timeout_ms = 2000);
  bool getTypicalScanMode(uint16_t& mode, uint32_t timeout_ms = 2000);
  bool getSampleDurationUs(uint16_t scan_mode, uint32_t timeout_ms = 2000);
  bool getDesiredMotorRpm(uint16_t& rpm, uint32_t timeout_ms = 2000);
  bool getScanModeAnsType(uint16_t scan_mode, uint8_t& ans_type, uint32_t timeout_ms = 2000);
  void setScanGrabbing(bool enabled);
  bool waitForScanHeader(uint32_t timeout_ms);
  bool feedCapsuleByte(uint8_t byte);
  bool readCapsuleBlock(uint32_t timeout_ms, bool allow_partial = false);
  void decodeCapsule();
  void decodeDenseCapsule(const DenseCapsule& capsule);
  void decodeUltraDenseCapsule(const UltraDenseCapsule& capsule);
  bool dequeuePoint(LidarPoint& point);
  bool dropOldestNonSyncPoint();
  void enqueuePoint(const LidarPoint& point);
  static uint16_t read_u16_le(const uint8_t* data);
  static uint32_t read_u32_le(const uint8_t* data);
};
