#ifndef FUNCTION_DECLARATIONS_H
#define FUNCTION_DECLARATIONS_H

#include <Arduino.h>

// deep sleep
void get_wakeup_reason();
void activate_deep_sleep();
void configureDeepSleep();

// data processing
void printData(float valueTemp, float realTemp = 0.0);
void sendData(float valueTemp, float realTemp = 0.0);
void checkTemperatureStatus(float temp);

// hardware control
float readTemperature();
void setLed(int valueRed, int valueBlue, int valueGreen);
void activateCooler();
void deactivateCooler();
void button();

// BlueTooth
void setupBLE();
void disconnectBLE(boolean delayFunction);
void connectBLE();

#endif