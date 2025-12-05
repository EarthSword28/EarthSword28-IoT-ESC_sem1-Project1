#ifndef FUNCTION_DECLARATIONS_H
#define FUNCTION_DECLARATIONS_H

#include <WiFi.h>
// MQTT libraries
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
// Web applicatie libraries
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// general
void printData(float valueTemp, float realTemp = 0.0);
void sendData(float valueTemp, float realTemp = 0.0);
float readTemperature();
void setLed(int valueRed, int valueBlue, int valueGreen);
void activateCooler();
void deactivateCooler();
void checkTemperatureStatus(float temp);
void button();
// deep sleep
void get_wakeup_reason();
void activate_deep_sleep();
void configureDeepSleep();
// BlueTooth
void setupBLE();
void disconnectBLE(boolean delayFunction);
void connectBLE();
// MQTT
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttPublish(uint16_t packetId);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void mqttCoolerButton(char *payload);
void mqttPublish(const char* topic_PL, String payload_PL, uint8_t qos_PL = 1, bool retain_PL = true);
void connectToMqtt();
void mqttSetup(int mqttReconectTimer_PL);
// WiFi
void wifiConnect();

#endif