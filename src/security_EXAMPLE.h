#ifndef SECURITY_EXAMPLE_H
#define SECURITY_EXAMPLE_H

#include <WString.h>  // no idea why but for some reason it needs it here

// WiFi
const int KNOWN_WIFI_AMOUNT = 3;
char *KNOWN_WIFI_SSIDs[] = {"SSID", "SSID-1", "SSID-2"};
char *KNOWN_WIFI_PASSWORDs[] = {"PASSWORD", "PASSWORD-1", "PASSWORD-2"};
const char* WIFI_HOST_NAME = "ESP32-User";

  // MQTT
// Raspberry Pi Mosquitto MQTT Broker
// #define MQTT_HOST IPAddress(x, x, x, x)
// For a cloud MQTT broker, type the domain name
#define MQTT_HOST "raspberry.local"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt user"
#define MQTT_PASSWORD "mqtt password"

#endif