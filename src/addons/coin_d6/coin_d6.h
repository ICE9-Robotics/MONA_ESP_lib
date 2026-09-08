#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

#include "addons/lidar_common/lidar_points.h"

/** Default UART baud rate for the COIN-D6. */
static const uint32_t COIN_D6_BAUD = 230400;

/** Identification parsed from the unsolicited A5 5A device-info frame. */
struct CoinD6DeviceInfo {
  char model[13];
  uint8_t software_version;
  bool valid;
};

/**
 * Driver for the CSPC COIN-D6 TOF lidar on ESP32 over UART.
 *
 * Handles start/stop, AA 55 scan-packet decoding, and point queueing.
 * Typical usage:
 *
 * @code
 * CoinD6 lidar(Serial1);
 * lidar.begin(rx_pin, tx_pin);
 * lidar.connect(rx_pin, tx_pin);
 * lidar.startScan();
 * LidarScan& scan = lidar.getPoints();
 * @endcode
 */
class CoinD6 {
 public:
  /**
   * @param serial Hardware UART used to talk to the lidar.
   */
  explicit CoinD6(HardwareSerial& serial);

  /**
   * Open the UART on the given pins.
   *
   * @param rx_pin ESP32 pin connected to the lidar TX line.
   * @param tx_pin ESP32 pin connected to the lidar RX line.
   * @param baud UART baud rate; defaults to @ref COIN_D6_BAUD.
   * @return Always true once the serial port has been configured.
   */
  bool begin(int8_t rx_pin, int8_t tx_pin, uint32_t baud = COIN_D6_BAUD);

  /**
   * Verify communication with the lidar.
   *
   * Reopens the UART, sends start, and waits for a device-info frame or a
   * valid scan packet. Stops scanning afterwards so @ref startScan() can
   * begin a clean stream. Does not require a GET_INFO command; COIN-D6
   * publishes info unsolicited at power-up.
   *
   * @param rx_pin ESP32 pin connected to the lidar TX line.
   * @param tx_pin ESP32 pin connected to the lidar RX line.
   * @return True if device info or a scan packet was received.
   */
  bool connect(int8_t rx_pin, int8_t tx_pin);

  /**
   * Stop scanning and clear internal scan/parser state.
   *
   * Sends STOP to the lidar, flushes pending UART input, and empties the
   * point queue.
   */
  bool stop();

  /**
   * Copy the last device-info frame, waiting if none has been seen yet.
   *
   * @param info Populated on success.
   * @param timeout_ms Maximum wait for an unsolicited info frame.
   * @return True if device info is available.
   */
  bool getDeviceInfo(CoinD6DeviceInfo& info, uint32_t timeout_ms = 2000);

  /**
   * Start scanning.
   *
   * Sends the COIN-D6 start command and waits for the first decoded point.
   * On success, @ref pumpScan(), @ref readPoint(), and @ref getPoints() can
   * consume data.
   *
   * @return True if at least one scan point was decoded.
   */
  bool startScan();

  /**
   * Read the next decoded scan point.
   *
   * Drains the UART, decodes packets as needed, and returns the oldest queued
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
   * Register a callback for raw scan packets before point decoding.
   *
   * @param stream Receives each complete AA 55 packet, or nullptr to disable.
   */
  void setRawCapsuleStream(LidarRawCapsuleStream* stream) { _raw_stream = stream; }

  /**
   * Decode any packets currently available on the UART.
   *
   * Call regularly while scanning to keep the point queue filled, especially
   * when not blocked inside @ref readPoint() or @ref getPoints().
   */
  void pumpScan();

  /**
   * Attempt to recover packet alignment from buffered UART bytes.
   *
   * Clears partial packet state and tries to decode complete packets from
   * bytes already in the serial buffer.
   *
   * @return True if at least one packet was decoded.
   */
  bool resyncScanStream();

  /** Scan-data type byte from the COIN-D6 manual (0x81), or 0 if not scanning. */
  uint8_t scanAnswerType() const { return _scanning ? 0x81 : 0; }

  /** Byte length of the last decoded scan packet (0 if none yet). */
  size_t capsuleSize() const { return _last_frame_len; }

  /** Rotation rate from the last start packet, in hertz. */
  float scanHz() const { return _scan_hz; }

 private:
  static const uint8_t kHeaderLen = 10;
  static const uint8_t kMaxLsn = 40;
  static const uint8_t kMaxCtrlData = 64;
  static const size_t kMaxScanBytes = kHeaderLen + kMaxLsn * 3;
  static const size_t POINT_QUEUE_SIZE = 512;

  enum class ParseState { Hunt, Scan, Ctrl };

  HardwareSerial& _serial;
  int8_t _rx_pin = -1;
  int8_t _tx_pin = -1;
  uint32_t _baud = COIN_D6_BAUD;

  CoinD6DeviceInfo _info{};
  bool _scanning = false;
  float _scan_hz = 0.0f;
  size_t _last_frame_len = 0;

  ParseState _parse = ParseState::Hunt;
  uint8_t _hdr0 = 0;
  uint8_t _buf[kMaxScanBytes];
  size_t _pos = 0;
  size_t _frame_len = 0;

  LidarPoint _point_queue[POINT_QUEUE_SIZE];
  size_t _queue_head = 0;
  size_t _queue_tail = 0;
  LidarScan _scan;
  LidarRawCapsuleStream* _raw_stream = nullptr;

  void flushInput();
  bool reopen(uint32_t baud);
  void sendCommand(const uint8_t* cmd, size_t len);
  void resetParser();
  bool feedByte(uint8_t byte);
  bool finishScanPacket();
  bool finishCtrlFrame();
  uint16_t checksum(const uint8_t* raw, uint8_t lsn) const;
  void decodePacket(const uint8_t* raw, uint8_t lsn);
  bool dequeuePoint(LidarPoint& point);
  bool dropOldestNonSyncPoint();
  void enqueuePoint(const LidarPoint& point);
  static uint16_t read_u16_le(const uint8_t* data);
};
