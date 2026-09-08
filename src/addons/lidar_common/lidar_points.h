#pragma once

#include <Arduino.h>
#include <math.h>

/** One decoded lidar sample in polar coordinates. */
struct LidarPoint {
  /** Bearing in degrees; 0 = forward, 90 = right (SLAMTEC convention). */
  float angle_deg;
  /** Range in millimeters; 0 means no valid return. */
  float distance_mm;
  /** Signal strength / confidence (0 when distance is invalid). */
  uint8_t quality;
  /** True at the first point of a new rotation. */
  bool sync;
};

/**
 * Convert polar lidar coordinates to Cartesian millimeters.
 *
 * SLAMTEC convention: 0 deg = forward (+Y), 90 deg = right (+X).
 *
 * @param angle_deg Bearing in degrees.
 * @param distance_mm Range in millimeters.
 * @param x_mm Output X coordinate in millimeters.
 * @param y_mm Output Y coordinate in millimeters.
 */
inline void lidarPolarToXY(float angle_deg, float distance_mm, float& x_mm, float& y_mm) {
  if (distance_mm <= 0.0f) {
    x_mm = 0.0f;
    y_mm = 0.0f;
    return;
  }
  const float rad = angle_deg * (PI / 180.0f);
  x_mm = sinf(rad) * distance_mm;
  y_mm = cosf(rad) * distance_mm;
}

/** @copydoc lidarPolarToXY(float, float, float&, float&) */
inline void lidarPolarToXY(const LidarPoint& point, float& x_mm, float& y_mm) {
  lidarPolarToXY(point.angle_deg, point.distance_mm, x_mm, y_mm);
}

/** Coordinate layout used by @ref LidarBinaryStream. */
enum class LidarPointFormat { Polar, Cartesian };

/**
 * Callback interface for streaming one rotation of decoded points.
 *
 * @ref beginRotation() is called once before the first point,
 * @ref onPoint() for each sample, and @ref endRotation() after the rotation
 * completes.
 */
class LidarPointStream {
 public:
  virtual ~LidarPointStream() = default;
  virtual void beginRotation() {}
  virtual void onPoint(const LidarPoint& point) = 0;
  virtual void endRotation(size_t count) {}
};

/**
 * Write raw measurement capsules to a stream.
 *
 * Frame format: `'LR'` + u16 length + raw capsule bytes.
 */
class LidarRawCapsuleStream {
 public:
  static const uint8_t FRAME_MAGIC[2];

  /**
   * @param out Destination stream (Serial, WiFiClient, etc.).
   */
  explicit LidarRawCapsuleStream(Stream& out);

  /**
   * Emit one capsule frame.
   *
   * @param data Raw capsule bytes from the lidar UART stream.
   * @param len Number of bytes in @p data.
   */
  void onCapsule(const uint8_t* data, size_t len);

 private:
  Stream& _out;
};

/**
 * Write decoded points for one rotation as a binary frame.
 *
 * Frame format: `'LD'` + int16 pairs per point + `'EE'` + u16 point count.
 * Each pair is either (x_mm, y_mm) or (angle_deg * 100, distance_mm),
 * depending on the configured @ref LidarPointFormat.
 */
class LidarBinaryStream : public LidarPointStream {
 public:
  static const uint8_t FRAME_MAGIC[2];
  static const uint8_t FRAME_END[2];

  /**
   * @param out Destination stream (Serial, WiFiClient, etc.).
   * @param format Coordinate layout for each point pair.
   */
  explicit LidarBinaryStream(Stream& out, LidarPointFormat format = LidarPointFormat::Cartesian);

  void beginRotation() override;
  void onPoint(const LidarPoint& point) override;
  void endRotation(size_t count) override;

 private:
  Stream& _out;
  LidarPointFormat _format;

  void writePointBytes(const LidarPoint& point);
};

/** Fixed-capacity buffer holding one rotation of @ref LidarPoint samples. */
class LidarScan {
 public:
  /** Default maximum points stored per rotation. */
  static const size_t DEFAULT_CAPACITY = 600;

  /** Number of points currently stored. */
  size_t count() const { return _count; }
  const LidarPoint& operator[](size_t i) const { return _points[i]; }
  const LidarPoint* data() const { return _points; }

  /** Remove all stored points. */
  void clear();

  /**
   * Append a point if capacity remains.
   *
   * @return False when @ref DEFAULT_CAPACITY would be exceeded.
   */
  bool add(const LidarPoint& point);

 private:
  size_t _count = 0;
  LidarPoint _points[DEFAULT_CAPACITY];
};
