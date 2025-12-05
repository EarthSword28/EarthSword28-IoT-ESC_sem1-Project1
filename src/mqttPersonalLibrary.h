// MQTT libraries
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;

int mqttListenCount_PL = 0;                                               // the amount of MQTT topics to listen to
char *mqttListenTopics_PL[10] = {"", "", "", "", "", "", "", "", "", ""};      // the MQTT topocs to listen to

int mqttEventCode = 0;
char* mqttEventMessage = "";

void mqttInit(int mqttListenAmounts_i_PL, char *Listen_i_PL_1 = "", char *Listen_i_PL_2 = "", char *Listen_i_PL_3 = "", char *Listen_i_PL_4 = "", char *Listen_i_PL_5 = "", char *Listen_i_PL_6 = "", char *Listen_i_PL_7 = "", char *Listen_i_PL_8 = "", char *Listen_i_PL_9 = "", char *Listen_i_PL_10 = "") {
  mqttListenCount_PL = mqttListenAmounts_i_PL;
  char *mqttListenTopics_i_PL[10] = {Listen_i_PL_1, Listen_i_PL_2, Listen_i_PL_3, Listen_i_PL_4, Listen_i_PL_5, Listen_i_PL_6, Listen_i_PL_7, Listen_i_PL_8, Listen_i_PL_9, Listen_i_PL_10};
  char *mqttListenTopics_PL[10] = mqttListenTopics_i_PL[10];
}

void handleReceivedMQTT(char* topic_PL, char* payload_PL, size_t len_PL) {
  char message[len_PL + 1];
  memcpy(message, payload_PL, len_PL);
  message[len] = '\0';   // Now message is a proper string

  for (int i = 0; i < mqttListenCount_PL && i < 10; i++) {
    if (strcmp(topic_PL, mqttListenTopics_PL[i]) == 0) {
      mqttEventCode = i+1;
      mqttEventMessage = message;
      break;
    }
  }
}

void mqttPublish(const char* topic_PL, String payload_PL, uint8_t qos_PL = 1, bool retain_PL = true){
  // Publish an MQTT message on topic esp/output/button
  uint16_t packetIdPub2 = mqttClient.publish(topic_PL, qos_PL, retain_PL, String(payload_PL).c_str());
  Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", topic_PL, packetIdPub2);
  Serial.printf("Message: %.2f \n", payload_PL);
}

void mqttConnect() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  
  if (mqttListenCount_PL > 0) {
    for (int i = 0; i < mqttListenCount_PL && i < 10; i++) {
      mqttClient.subscribe(mqttListenTopics_PL[i], 1);
    }
  }
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
