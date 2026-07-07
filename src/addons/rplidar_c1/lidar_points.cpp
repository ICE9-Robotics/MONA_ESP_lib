#include "lidar_points.h"

#include <math.h>

const uint8_t LidarBinaryStream::FRAME_MAGIC[2] = {'L', 'D'};
const uint8_t LidarBinaryStream::FRAME_END[2] = {'E', 'E'};

namespace {

void polarToXYmm(const LidarPoint& point, int16_t& x_mm, int16_t& y_mm) {
  float x = 0.0f;
  float y = 0.0f;
  lidarPolarToXY(point, x, y);
  x_mm = static_cast<int16_t>(lroundf(x));
  y_mm = static_cast<int16_t>(lroundf(y));
}

}  // namespace

LidarBinaryStream::LidarBinaryStream(Stream& out, LidarPointFormat format)
    : _out(out), _format(format) {}

void LidarBinaryStream::beginRotation() {
  _out.write(FRAME_MAGIC, sizeof(FRAME_MAGIC));
}

void LidarBinaryStream::writePointBytes(const LidarPoint& point) {
  int16_t xy[2];
  if (_format == LidarPointFormat::Cartesian) {
    polarToXYmm(point, xy[0], xy[1]);
  } else {
    xy[0] = static_cast<int16_t>(lroundf(point.angle_deg * 100.0f));
    xy[1] = static_cast<int16_t>(lroundf(point.distance_mm));
  }
  _out.write(reinterpret_cast<const uint8_t*>(xy), sizeof(xy));
}

void LidarBinaryStream::onPoint(const LidarPoint& point) {
  writePointBytes(point);
}

void LidarBinaryStream::endRotation(size_t count) {
  const uint16_t point_count = static_cast<uint16_t>(count);
  _out.write(FRAME_END, sizeof(FRAME_END));
  _out.write(reinterpret_cast<const uint8_t*>(&point_count), sizeof(point_count));
}

void LidarScan::clear() {
  _count = 0;
}

bool LidarScan::add(const LidarPoint& point) {
  if (_count >= DEFAULT_CAPACITY) {
    return false;
  }
  _points[_count++] = point;
  return true;
}
