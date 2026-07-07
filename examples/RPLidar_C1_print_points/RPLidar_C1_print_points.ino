#include "RPLidar_C1.h"

// Wiring (cross TX/RX):
//   LIDAR TX (yellow) -> ESP32 GPIO26 (RX)
//   LIDAR RX (green)  -> ESP32 GPIO27 (TX)
//   LIDAR GND (black) -> ESP32 GND
//   LIDAR VCC (red)   -> 5V (needs ~250mA; aim for 5.0V, not 4.7V)
static const int LIDAR_RX_PIN = 26;
static const int LIDAR_TX_PIN = 27;

#define LIDAR_SERIAL Serial1

static const uint32_t CONSOLE_BAUD = 921600;
static const LidarPointFormat POINT_OUTPUT = LidarPointFormat::Cartesian; // or LidarPointFormat::Polar

RPLidarC1 lidar(LIDAR_SERIAL);

void printPoint(Stream& out, const LidarPoint& point, LidarPointFormat format) {
  out.print("  ");
  if (format == LidarPointFormat::Cartesian) {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    lidarPolarToXY(point, x_mm, y_mm);
    out.print(x_mm, 1);
    out.print(" ");
    out.print(y_mm, 1);
    out.print(", ");
    return;
  }

  out.print(point.angle_deg, 2);
  out.print("deg ");
  out.print(point.distance_mm, 1);
  out.print("mm, ");
}

void printRotationSummary(Stream& out, const LidarScan& scan) {
  float min_dist = 99999.0f;
  float max_dist = 0.0f;
  float front_dist = 0.0f;
  float best_front_delta = 999.0f;

  for (size_t i = 0; i < scan.count(); ++i) {
    const LidarPoint& p = scan[i];
    if (p.distance_mm <= 0.0f) {
      continue;
    }
    if (p.distance_mm < min_dist) {
      min_dist = p.distance_mm;
    }
    if (p.distance_mm > max_dist) {
      max_dist = p.distance_mm;
    }

    float delta = p.angle_deg;
    if (delta > 180.0f) {
      delta = 360.0f - delta;
    }
    if (delta < best_front_delta) {
      best_front_delta = delta;
      front_dist = p.distance_mm;
    }
  }

  out.print("Rotation: ");
  out.print(scan.count());
  out.print(" points, nearest ");
  out.print(min_dist, 0);
  out.print(" mm, farthest ");
  out.print(max_dist, 0);
  out.print(" mm, front ");
  out.print(front_dist, 0);
  out.println(" mm");
}

void printScan(const LidarScan& scan, Stream& out = Serial,
               LidarPointFormat format = POINT_OUTPUT, bool include_summary = true) {
  if (include_summary) {
    printRotationSummary(out, scan);
  }

  for (size_t i = 0; i < scan.count(); ++i) {
    printPoint(out, scan[i], format);
  }
  out.println();

  if (include_summary) {
    out.println("---");
  }
}

void setup() {
  Serial.begin(CONSOLE_BAUD);
  Serial.setTxBufferSize(8192);
  delay(1000);
  lidar.begin(LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(500);

  if (!lidar.connect(LIDAR_RX_PIN, LIDAR_TX_PIN)) {
    Serial.println("Failed to connect to lidar.");
    while (true) {
      delay(1000);
    }
  }

  if (!lidar.startScan()) {
    Serial.println("Failed to start scan.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("Scan stream type 0x");
  Serial.print(lidar.scanAnswerType(), HEX);
  Serial.print(", capsule size ");
  Serial.println(lidar.capsuleSize());
}

void loop() {
  LidarScan& points = lidar.getPoints();
  printScan(points);
}
