#pragma once

#include <Arduino.h>
#include <math.h>

struct LidarPoint {
  float angle_deg;
  float distance_mm;
  uint8_t quality;
  bool sync;
};

// SLAMTEC convention: 0 deg = forward (+Y), 90 deg = right (+X), units mm.
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

inline void lidarPolarToXY(const LidarPoint& point, float& x_mm, float& y_mm) {
  lidarPolarToXY(point.angle_deg, point.distance_mm, x_mm, y_mm);
}

enum class LidarPointFormat { Polar, Cartesian };

// Stream one rotation of points as they are read (human text or binary frames).
class LidarPointStream {
 public:
  virtual ~LidarPointStream() = default;
  virtual void beginRotation() {}
  virtual void onPoint(const LidarPoint& point) = 0;
  virtual void endRotation(size_t count) {}
};

// 'LR' + u16 size + raw capsule bytes from the lidar UART stream.
class LidarRawCapsuleStream {
 public:
  static const uint8_t FRAME_MAGIC[2];

  explicit LidarRawCapsuleStream(Stream& out);

  void onCapsule(const uint8_t* data, size_t len);

 private:
  Stream& _out;
};

// 'LD' + int16 pairs per point + 'EE' + u16 count.
class LidarBinaryStream : public LidarPointStream {
 public:
  static const uint8_t FRAME_MAGIC[2];
  static const uint8_t FRAME_END[2];

  explicit LidarBinaryStream(Stream& out, LidarPointFormat format = LidarPointFormat::Cartesian);

  void beginRotation() override;
  void onPoint(const LidarPoint& point) override;
  void endRotation(size_t count) override;

 private:
  Stream& _out;
  LidarPointFormat _format;

  void writePointBytes(const LidarPoint& point);
};

// One lidar rotation worth of points.
class LidarScan {
 public:
  static const size_t DEFAULT_CAPACITY = 600;

  size_t count() const { return _count; }
  const LidarPoint& operator[](size_t i) const { return _points[i]; }
  const LidarPoint* data() const { return _points; }

  void clear();
  bool add(const LidarPoint& point);

 private:
  size_t _count = 0;
  LidarPoint _points[DEFAULT_CAPACITY];
};
