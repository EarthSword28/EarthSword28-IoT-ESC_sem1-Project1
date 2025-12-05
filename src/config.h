#ifndef CONFIG_H
#define CONFIG_H

const float MIN_TEMPERATURE = 20.00;
const float MAX_TEMPERATURE = 25.00;

const int MEASURE_INTERVAL = 300000;
const int DISPLAY_INTERVAL = 60000;
const int BUTTON_DEBOUNCE = 1000;
const int DISCONNECTED_BLE_DELAY = 500;

// WiFi
const int WIFI_CONNECT_ATTEMPS = 5;  // Amount of attemps to connect to available WiFi network
const int WIFI_CONNECT_DELAY = 500;   // Time between connection Attemps
const char* WIFI_HOST_NAME = "ESP32-Jorden";

// MQTT
const int MQTT_RECONNECT_TIMER = 2000;

  // MQTT Topics
#define MQTT_PUB_TEMP "esp/output/temp"
#define MQTT_SUB_COOLER "esp/input/cooler"
#define MQTT_SUB_MEASURE "esp/input/measure"

// Deep Sleep
const boolean DEEP_SLEEP_DEFAULT_PERMISSION = LOW;

const byte DEEP_SLEEP_WAKE_UP_START = 1;
const byte DEEP_SLEEP_WAKE_UP_TIME = 2;
const byte DEEP_SLEEP_WAKE_UP_PANIC_BUTTON = 3;
const byte DEEP_SLEEP_WAKE_UP_UNDEFINED = 4;

/* DEBUG */
const boolean DEBUG_SCREEN = HIGH;
const boolean MOCK_SWITCH = LOW;
const float MOCK_TEMPERATURE = 20.00;

  // UUIDs
#define SERVICE_UUID        "743770a3-61e9-4edb-9291-dbd476f484d8"
#define SENSOR_CHARACTERISTIC_UUID "0f5bf109-7f09-4954-a0a3-5ec26d4da9a5"
#define COOLER_CHARACTERISTIC_UUID "689dffc1-9faa-4139-9004-e47b914b78ed"
#define DEEP_SLEEP_CHARACTERISTIC_UUID "1a2b1de3-a861-4415-9ce4-be5a5fbf4bc7"
#define MEASURE_CHARACTERISTIC_UUID "8f6e0141-d448-4bf1-8c04-f4ce063ba865"

#endif