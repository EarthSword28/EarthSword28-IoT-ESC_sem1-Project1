#ifndef SECURITY_H
#define SECURITY_H

#include <WString.h>  // no idea why but for some reason it needs it here

// WiFi
const int KNOWN_WIFI_AMOUNT = 3;
char *KNOWN_WIFI_SSIDs[] = {"SSID-1", "SSID-2", "SSID-3"};
char *KNOWN_WIFI_PASSWORDs[] = {"PASSWORD-1", "PASSWORD-2", "PASSWORD-3"};

  // MQTT
// Raspberry Pi Mosquitto MQTT Broker
// #define MQTT_HOST IPAddress(x, x, x, x)
// For a cloud MQTT broker, type the domain name
#define MQTT_HOST "raspberry.local"
#define MQTT_PORT 183
#define MQTT_USER "mqtt user"
#define MQTT_PASSWORD "mqtt password"

#endif