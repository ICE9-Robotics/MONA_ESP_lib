#include "wifi_setup.h"

#include "config.h"
#include <WiFi.h>

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Connecting to WiFi ");
    Serial.print(WIFI_SSID);
    Serial.println("...");
  }

  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Lidar TCP port: ");
  Serial.println(LIDAR_TCP_PORT);
  Serial.print("Control TCP port: ");
  Serial.println(CONTROL_TCP_PORT);
}

void startHotspot() {
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("Hotspot started. IP address: ");
  Serial.println(myIP);
  Serial.print("Lidar TCP port: ");
  Serial.println(LIDAR_TCP_PORT);
  Serial.print("Control TCP port: ");
  Serial.println(CONTROL_TCP_PORT);
}

void wifiSetup() {
  if (WIFI_HOTSPOT_MODE) {
    startHotspot();
  } else {
    connectWifi();
  }
}