/*
  BRONNEN:
  Project plantbewateringsysteem (29/09/2025): ik ben teruggegaan naar mijn project plantbewateringsysteem voor Embedded Systemen basis van academiejaar 24-25 om te kijken hoe ik gebruik gemaakt heb in het verleden van de DS18B20 temperatuur sensor en voor te kijken hoe ik deep sleep gebruikte
  pin layout ESP32-E (06/10/2025): https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654#6.%20Pinout
  aansluiten relays module (06/10/2025): https://randomnerdtutorials.com/esp32-relay-module-ac-web-server/
  web applicatie met Random Nerd Tutorial (09/10/2025): https://randomnerdtutorials.com/esp32-web-bluetooth/
  JavaScript voor de Web BLE Application (11/10/2025): https://www.w3schools.com/js/default.asp 
  information about classes in C++ (11/10/2025): https://www.w3schools.com/cpp/cpp_class_methods.asp

  randomnerdtutorials MQTT (05/12/2025): https://randomnerdtutorials.com/esp32-mqtt-publish-bme680-arduino/
  async MQTT client API reference (05/12/2025): https://github.com/marvinroger/async-mqtt-client/blob/develop/docs/2.-API-reference.md
  ChatGPT voor het oplossen van een compile error door config.h en security.h meerdere keren te includen en voor function declaraties (05/12/2025): https://chatgpt.com/share/6933024b-8cd8-800c-9ebf-42aab106238a
*/

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <config.h>
#include <security.h>
#include <functionDeclarations.h>

// WiFi
#include <WiFi.h>

// MQTT libraries
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

boolean dataRedSwitch;              // boolean for tracking if the sensor has been read out yet

// Define used Pins
#define ONE_WIRE_BUS 4  // temperatuur sensor

#define LED_RED 14
#define LED_BLUE 13
#define LED_GREEN 2

#define BUTTON 25

#define RELAY_MODULE 17 // de servo/ventilator

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

boolean ventilator;
boolean cooling;
boolean tempHigh;

byte manualOverrideCooler = 0;

float temperature;

unsigned long measureTimer = 0;
unsigned long displayTimer = 0;
unsigned long sleepTimer = 0;

unsigned long buttonDebounceTimer = 0;
boolean buttonSwitch;

// MQTT
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;

// send the data to the web app
void sendData(float valueTemp, float realTemp) {
  String tempString = String(valueTemp);
  // if (MOCK_SWITCH == HIGH) {
  //   tempString = "Mock temperature: " + tempString + "°C | Real Temperature: " + String(realTemp);
  // }

  mqttPublish(MQTT_PUB_TEMP, tempString);
}

void printData(float valueTemp, float realTemp) {
  String tempString = String(valueTemp);
  if (MOCK_SWITCH == HIGH) {
    tempString = "Mock temperature: " + tempString + "°C | Real Temperature: " + String(realTemp);
  }
  Serial.println(tempString);
}

float readTemperature() {
  displayTimer = millis() + DISPLAY_INTERVAL;
  measureTimer = millis() + MEASURE_INTERVAL;

  // Send the command to get temperatures
  sensors.requestTemperatures();

  // return the temperature in Celsius
  float temp = sensors.getTempCByIndex(0);

  if (MOCK_SWITCH == LOW) {
    sendData(temp);
    printData(temp);
    return temp;
  }
  else {
    sendData(MOCK_TEMPERATURE, temp);
    printData(MOCK_TEMPERATURE, temp);
    return MOCK_TEMPERATURE;
  }
}

/*
configure the color of the RGB LED
*/
void setLed(int valueRed, int valueBlue, int valueGreen) {
  analogWrite(LED_RED, (255 - valueRed));
  analogWrite(LED_BLUE, (255 - valueBlue));
  analogWrite(LED_GREEN, (255 - valueGreen));
}

void activateCooler() {
  if (manualOverrideCooler != 2) {
    digitalWrite(RELAY_MODULE, LOW);
    ventilator = HIGH;
    Serial.println("cooler on");
  }
}

void deactivateCooler() {
  if (manualOverrideCooler != 1) {
    digitalWrite(RELAY_MODULE, HIGH);
    cooling = LOW;
    ventilator = LOW;
    Serial.println("cooler off");
  }
}

void checkTemperatureStatus(float temp) {
  if (temp < MIN_TEMPERATURE) {
    // Color the RGB LED Blue
    setLed(0, 200, 0);
    tempHigh = LOW;
  }
  else if (temp < MAX_TEMPERATURE) {
    // Color the RGB LED Green
    setLed(0, 0, 200);
    tempHigh = LOW;
  }
  else {
    // Color the RGB LED Red
    setLed(200, 0, 0);
    tempHigh = HIGH;
    cooling = HIGH;
  }
}

void button() {
  buttonSwitch = HIGH;
  buttonDebounceTimer = millis() + BUTTON_DEBOUNCE;
  dataRedSwitch = LOW;
}

// MQTT functions
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  
  mqttClient.subscribe(MQTT_SUB_COOLER, 1);
  mqttClient.subscribe(MQTT_SUB_MEASURE, 1);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);

  char message[len + 1];
  memcpy(message, payload, len);
  message[len] = '\0';   // Now message is a proper string
  Serial.println(message);

  if (strcmp(topic, MQTT_SUB_COOLER) == 0) {
    mqttCoolerButton(message);
  }
  else if (strcmp(topic, MQTT_SUB_MEASURE) == 0) {
    dataRedSwitch = LOW;
  }
}

void mqttCoolerButton(char *payload) {
  if (strcmp(payload, "ON") == 0) {
    Serial.println("ON");
    manualOverrideCooler = 1;
    cooling = HIGH;
  }
  else if (strcmp(payload, "OFF") == 0) {
    Serial.println("OFF");
    manualOverrideCooler = 2;
  }
  else if (strcmp(payload, "AUTO") == 0) {
    Serial.println("AUTO");
    manualOverrideCooler = 3;
  }
}

void mqttPublish(const char* topic_PL, String payload_PL, uint8_t qos_PL, bool retain_PL) {
  // Publish an MQTT message on topic esp/output/button
  uint16_t packetIdPub = mqttClient.publish(topic_PL, qos_PL, retain_PL, String(payload_PL).c_str());
  Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", topic_PL, packetIdPub);
  Serial.printf("Message: %.2f \n", payload_PL);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void mqttSetup(int mqttReconectTimer_PL) {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(mqttReconectTimer_PL), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);

  connectToMqtt();
}

// Connect to a known WiFi network
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  int networks = WiFi.scanNetworks();
  if (networks > 0) {
    for (int n = 0; n < networks && WiFi.status() != WL_CONNECTED; n++) {
      String ssid = WiFi.SSID(n);
      for (int i = 0; i < KNOWN_WIFI_AMOUNT; i++) {
        if (ssid == KNOWN_WIFI_SSIDs[i]) {
          // if hostname, set hostname
          if (WIFI_HOST_NAME != "") {
            WiFi.setHostname(WIFI_HOST_NAME);
          }
          // connect to WiFi
          String password = KNOWN_WIFI_PASSWORDs[i];
          int connectAttemps = 0;
          WiFi.begin(ssid, password);
          Serial.print("connecting to: ");
          Serial.print(ssid);
          while (WiFi.status() != WL_CONNECTED && connectAttemps < WIFI_CONNECT_ATTEMPS) {
            connectAttemps++;
            delay(WIFI_CONNECT_DELAY);
            Serial.print(".");
          }
          if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nConnected to WiFi!");
            Serial.println(WiFi.localIP());
            break;
          }
          else {
            Serial.println("\nConnection Failed");
            continue;
          }
        }
      }
    }
  }
  else {
    Serial.println("No networks found");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ONE_WIRE_BUS, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(RELAY_MODULE, OUTPUT);
  digitalWrite(RELAY_MODULE, HIGH);

  temperature = -100.00;
  ventilator = LOW;
  cooling = LOW;
  tempHigh = LOW;

  buttonSwitch = LOW;
  dataRedSwitch = LOW;
  measureTimer = millis() + MEASURE_INTERVAL;
  displayTimer = millis() + DISPLAY_INTERVAL;
  
  setLed(0, 0, 0);
  
  delay(1000); // Take some time to open up the Serial Monitor

  // Start up the sensor library
  sensors.begin(); 

  // WiFi
  wifiConnect();

  // MQTT
  mqttSetup(MQTT_RECONNECT_TIMER);
}

void loop() {
  long currentMillis = millis();

  // logica for the reading of the data
  if (dataRedSwitch == LOW) {
    dataRedSwitch = HIGH;
    temperature = readTemperature();
    checkTemperatureStatus(temperature);
  }

  // logica for controling the ventilator
  if (cooling == HIGH) {
    displayTimer = millis() + DISPLAY_INTERVAL;
    if ((manualOverrideCooler == 1 || tempHigh == HIGH) && ventilator == LOW) {
      activateCooler();
    }
    else if (tempHigh == LOW || manualOverrideCooler == 2) {
      deactivateCooler();
    }
    if (currentMillis >= measureTimer) {
      dataRedSwitch = LOW;
    }
  }

  // logica for the Button input
  if (buttonSwitch == HIGH && currentMillis >= buttonDebounceTimer) {
    buttonSwitch = LOW;
  }
  else if (digitalRead(BUTTON) == HIGH) {
    button();
  }
}