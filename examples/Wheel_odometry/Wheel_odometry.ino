/*
  Wheel_odometry.ino
  Print quadrature hall encoder counts from the N20 motors.

  Counts are accumulated in ESP32 PCNT hardware. loop() only reads the
  counters; it does not sample the pins.
*/
#include <Wire.h>
#include "Mona_ESP_lib.h"

void setup() {
  Serial.begin(115200);
  Mona_ESP_init();
  Encoder_reset();
  Motors_forward(120);
}

void loop() {
  int32_t left = Encoder_left();
  int32_t right = Encoder_right();
  Serial.print("left=");
  Serial.print(left);
  Serial.print(" right=");
  Serial.println(right);
  delay(50);
}
