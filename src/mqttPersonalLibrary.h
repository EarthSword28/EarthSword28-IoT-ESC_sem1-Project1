// MQTT libraries
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#include <config.h>
#include <security.h>

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;

int mqttEventCode = 0;
char* mqttEventMessage;

void handleReceivedMQTT(char* topic_PL, char* payload_PL, size_t len_PL) {
  char message[len_PL + 1];
  memcpy(message, payload_PL, len_PL);
  message[len_PL] = '\0';   // Now message is a proper string

  if (strcmp(topic_PL, MQTT_SUB_COOLER) == 0) {
    mqttEventCode = 1;
    mqttEventMessage = message;
  }
  else if (strcmp(topic_PL, MQTT_SUB_MEASURE) == 0) {
    mqttEventCode = 2;
    mqttEventMessage = message;
  }
}

void mqttPublish(const char* topic_PL, String payload_PL, uint8_t qos_PL = 1, bool retain_PL = true){
  // Publish an MQTT message on topic esp/output/button
  uint16_t packetIdPub = mqttClient.publish(topic_PL, qos_PL, retain_PL, String(payload_PL).c_str());
  Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", topic_PL, packetIdPub);
  Serial.printf("Message: %.2f \n", payload_PL);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

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

  handleReceivedMQTT(topic, payload, len);
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
