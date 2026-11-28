/*
  Encoder_print.ino
  Print left and right encoder counts.
*/
#include <Wire.h>
#include "Mona_ESP_lib.h"

void setup() {
  Serial.begin(115200);
  Mona_ESP_init();
  Motors_stop();
  Encoder_reset();
}

void loop() {
  Serial.print("left=");
  Serial.print(Encoder_left());
  Serial.print(" right=");
  Serial.println(Encoder_right());
  delay(50);
}
