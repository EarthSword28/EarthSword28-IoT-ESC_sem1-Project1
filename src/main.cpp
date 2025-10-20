/*
  BRONNEN:
  Project plantbewateringsysteem (29/09/2025): ik ben teruggegaan naar mijn project plantbewateringsysteem voor Embedded Systemen basis van academiejaar 24-25 om te kijken hoe ik gebruik gemaakt heb in het verleden van de DS18B20 temperatuur sensor en voor te kijken hoe ik deep sleep gebruikte
  pin layout ESP32-E (06/10/2025): https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654#6.%20Pinout
  aansluiten relays module (06/10/2025): https://randomnerdtutorials.com/esp32-relay-module-ac-web-server/
  web applicatie met Random Nerd Tutorial (09/10/2025): https://randomnerdtutorials.com/esp32-web-bluetooth/
  JavaScript voor de Web BLE Application (11/10/2025): https://www.w3schools.com/js/default.asp 
  information about classes in C++ (11/10/2025): https://www.w3schools.com/cpp/cpp_class_methods.asp
*/

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <config.h>
#include <security.h>

// Web applicatie libraries
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// deep sleep problemen oplossen
#include <esp_system.h>
#include <esp_sleep.h>

#include <driver/rtc_io.h>

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO) // 2 ^ GPIO_NUMBER in hex
#define WAKEUP_GPIO GPIO_NUM_25                 // de knop kan de ESP uit deep sleep halen

#define uS_TO_mS_FACTOR 1000              /* Conversion factor for micro seconds to milli seconds */
#define TIME_TO_SLEEP MEASURE_INTERVAL    /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

boolean deepSleepPermission;        // will deep sleep be activated after a while
boolean dataRedSwitch;              // boolean for tracking if the sensor has been read out yet
String deepSleepWakeUpReason = "";  // the reason the ESP woke up

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

// BLE Applicatie
  // Global Variables
BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pCoolerCharacteristic = NULL;
BLECharacteristic* pDeepSleepCharacteristic = NULL;
BLECharacteristic* pMeasureCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

unsigned long disconnectedDelayTimerBLE = 0;
boolean disconnectedSwitchBLE;

  // UUIDs
#define SERVICE_UUID        "743770a3-61e9-4edb-9291-dbd476f484d8"
#define SENSOR_CHARACTERISTIC_UUID "0f5bf109-7f09-4954-a0a3-5ec26d4da9a5"
#define COOLER_CHARACTERISTIC_UUID "689dffc1-9faa-4139-9004-e47b914b78ed"
#define DEEP_SLEEP_CHARACTERISTIC_UUID "1a2b1de3-a861-4415-9ce4-be5a5fbf4bc7"
#define MEASURE_CHARACTERISTIC_UUID "8f6e0141-d448-4bf1-8c04-f4ce063ba865"

// CLASSES
  // Device connection and disconnection
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

  // lees de characteristics van de cooler op de web app wanneer deze doorgestuurd worden
class CoolerCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCoolerCharacteristic) {
    // de volgende 2 lijnen code zijn aangepast naar de code op canvas om het probleem met de originel code op te lossen
    std::string coolerValue  = pCoolerCharacteristic->getValue(); 
    String value = String(coolerValue.c_str());

    if (value.length() > 0) {
      Serial.print("Characteristic event, written: ");
      Serial.println(static_cast<int>(value[0])); // Print the integer value

      int receivedValue = static_cast<int>(value[0]);
      if (receivedValue == 1) {
        manualOverrideCooler = 1;
        cooling = HIGH;
      } 
      else if (receivedValue == 2) {
        manualOverrideCooler = 2;
      }
      else {
        manualOverrideCooler = 0;
      }
    }
  }
};

  // lees de characteristics van de Deep Sleep Status op de web app wanneer deze doorgestuurd worden
class DeepSleepCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pDeepSleepCharacteristic) {
    // de volgende 2 lijnen code zijn aangepast naar de code op canvas om het probleem met de originel code op te lossen
    std::string deepSleepValue  = pDeepSleepCharacteristic->getValue(); 
    String value = String(deepSleepValue.c_str());

    if (value.length() > 0) {
      Serial.print("Characteristic event, written: ");
      Serial.println(static_cast<int>(value[0])); // Print the integer value

      int receivedValue = static_cast<int>(value[0]);
      if (receivedValue == 1) {
        deepSleepPermission = HIGH;
      } 
      else {
        deepSleepPermission = LOW;
      }
    }
  }
};

  // voer een meting uit wanneer dit word gevraagd via de web app
class MeasureCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pMeasureCharacteristic) {
    dataRedSwitch = LOW;
  }
};

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void get_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Wakeup caused by external signal using RTC_IO");
    deepSleepWakeUpReason = DEEP_SLEEP_WAKE_UP_PANIC_BUTTON;
  }
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    deepSleepWakeUpReason = DEEP_SLEEP_WAKE_UP_UNDEFINED;
  }
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("Wakeup caused by timer");
    deepSleepWakeUpReason = DEEP_SLEEP_WAKE_UP_TIME;
  }
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_TOUCHPAD) {
    Serial.println("Wakeup caused by touchpad");
    deepSleepWakeUpReason = DEEP_SLEEP_WAKE_UP_UNDEFINED;
  }
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_ULP) {
    Serial.println("Wakeup caused by ULP program");
    deepSleepWakeUpReason = DEEP_SLEEP_WAKE_UP_UNDEFINED;
  }
  else if (bootCount == 1) {
    Serial.println("Startup");
    deepSleepWakeUpReason = DEEP_SLEEP_WAKE_UP_START;
  }
  else {
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    deepSleepWakeUpReason = DEEP_SLEEP_WAKE_UP_UNDEFINED;
  }
}

void activate_deep_sleep() {
  Serial.println("Preparing to sleep");

  sleepTimer = MEASURE_INTERVAL;
  Serial.print("Going to sleep now for: ");
  Serial.println(sleepTimer);
  esp_sleep_enable_timer_wakeup(sleepTimer * uS_TO_mS_FACTOR);
  Serial.flush();
  delay(200);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void configureDeepSleep() {
  Serial.println("Booting...");

  esp_reset_reason_t reset_reason = esp_reset_reason();
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  //Print the wakeup reason for ESP32
  get_wakeup_reason();

  Serial.print("Reset reason: ");
  Serial.println((int)reset_reason);

  Serial.print("Wakeup reason: ");
  Serial.println((int)wakeup_reason);

  Serial.print("Boot count (before): ");
  Serial.println(bootCount);

  ++bootCount;
  Serial.print("Boot count (after increment): ");
  Serial.println(bootCount);
  
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKEUP_GPIO, 1); // 1 = High, 0 = Low
  // Configure pullup/downs via RTCIO to tie wakeup pins to inactive level during deepsleep.
  // EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
  // No need to keep that power domain explicitly, unlike EXT1.
  rtc_gpio_pullup_dis(WAKEUP_GPIO);
  rtc_gpio_pulldown_en(WAKEUP_GPIO);
}

// prinde de data op het debug scherm
void printData(float valueTemp, float realTemp = 0.0) {
  if (DEBUG_SCREEN == HIGH) {
    Serial.print("bootCount: ");
    Serial.print(bootCount);
    Serial.print(" | Temperature: ");
    Serial.print(valueTemp);

    if (MOCK_SWITCH == HIGH) {
      Serial.print("°C | Real Temperature: ");
      Serial.print(realTemp);
    }

    Serial.println("°C");
    Serial.println("--------");
  }
}

// stuur de data door naar de web app
void sendData(float valueTemp, float realTemp = 0.0) {
  String tempString = String(valueTemp);
  if (MOCK_SWITCH == HIGH) {
    tempString = "Mock temperature: " + tempString + "°C | Real Temperature: " + String(realTemp);
  }

  if (deviceConnected) {
    pSensorCharacteristic->setValue(tempString.c_str());
    pSensorCharacteristic->notify();
  }
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
configureer de kleur van de RGB LED
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
  }
}

void deactivateCooler() {
  if (manualOverrideCooler != 1) {
    digitalWrite(RELAY_MODULE, HIGH);
    cooling = LOW;
    ventilator = LOW;
  }
}

void checkTemperatureStatus(float temp) {
  if (temp < MIN_TEMPERATURE) {
    // kleur de RGB LED Blauw
    setLed(0, 200, 0);
    tempHigh = LOW;
  }
  else if (temp < MAX_TEMPERATURE) {
    // kleur de RGB LED Groen
    setLed(0, 0, 200);
    tempHigh = LOW;
  }
  else {
    // Kleur de RGB LED ROOD
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

// setup voor de BLE funcionaliteit (code vrijwel direct afkomstig van Random Nerd Tutorial over het onderwerp, zie bronnen)
void setupBLE() {
  // Create the BLE Device
  BLEDevice::init("ESP32-E-Jorden");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
                      SENSOR_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create the Cooler button Characteristic
  pCoolerCharacteristic = pService->createCharacteristic(
                      COOLER_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Register the callback for the Cooler button characteristic
  pCoolerCharacteristic->setCallbacks(new CoolerCharacteristicCallbacks());

  // Create the Deep Sleep button Characteristic
  pDeepSleepCharacteristic = pService->createCharacteristic(
                      DEEP_SLEEP_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Register the callback for the Cooler button characteristic
  pDeepSleepCharacteristic->setCallbacks(new DeepSleepCharacteristicCallbacks());

  // Create the Measurement button Characteristic
  pMeasureCharacteristic = pService->createCharacteristic(
                      MEASURE_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Register the callback for the measure button characteristic
  pMeasureCharacteristic->setCallbacks(new MeasureCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pCoolerCharacteristic->addDescriptor(new BLE2902());
  pDeepSleepCharacteristic->addDescriptor(new BLE2902());
  pMeasureCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void disconnectBLE(boolean delayFunction) {
  // if function to give the bluetooth stack the chance to get things ready
  if (delayFunction == LOW) {
    Serial.println("Device disconnected.");
    disconnectedDelayTimerBLE = millis() + DISCONNECTED_BLE_DELAY;
  }
  else {
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
    disconnectedSwitchBLE = LOW;
  }
}

void connectBLE() {
  // do stuff here on connecting
  oldDeviceConnected = deviceConnected;
  Serial.println("Device Connected");
  dataRedSwitch = LOW;
}

void setup() {
  pinMode(ONE_WIRE_BUS, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(RELAY_MODULE, OUTPUT);

  digitalWrite(RELAY_MODULE, HIGH);
  Serial.begin(115200);
  temperature = -100.00;
  deepSleepPermission = LOW;
  ventilator = LOW;
  cooling = LOW;
  tempHigh = LOW;

  buttonSwitch = LOW;
  dataRedSwitch = LOW;
  measureTimer = millis() + MEASURE_INTERVAL;
  displayTimer = millis() + DISPLAY_INTERVAL;
  
  setLed(0, 0, 0);
  
  delay(1000); // Take some time to open up the Serial Monitor
  configureDeepSleep();

  // Start up the sensor library
  sensors.begin(); 

  // activeer de BlueTooth functionaliteit
  setupBLE();

  disconnectedSwitchBLE = LOW;
}

void loop() {
  long currentMillis = millis();

  // logica voor het uitlezen van de data
  if (dataRedSwitch == LOW) {
    dataRedSwitch = HIGH;
    temperature = readTemperature();
    checkTemperatureStatus(temperature);
  }

  // logica voor het besturen van de ventilator
  if (cooling == HIGH) {
    displayTimer = millis() + DISPLAY_INTERVAL;
    if (manualOverrideCooler == 1 || (tempHigh == HIGH && ventilator == LOW)) {
      activateCooler();
    }
    else if (tempHigh == LOW || manualOverrideCooler == 2) {
      deactivateCooler();
    }
    if (currentMillis >= measureTimer) {
      dataRedSwitch = LOW;
    }
  }
  else if (deepSleepPermission == HIGH && currentMillis >= displayTimer) {
    activate_deep_sleep();
  }

  // logica voor het afhandellen van de knop
  if (buttonSwitch == HIGH && currentMillis >= buttonDebounceTimer) {
    buttonSwitch = LOW;
  }
  else if (digitalRead(BUTTON) == HIGH) {
    button();
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    if (disconnectedSwitchBLE == LOW) {
      disconnectedSwitchBLE = HIGH;
      disconnectBLE(LOW);
    }
    else if (currentMillis >= disconnectedDelayTimerBLE) {
      disconnectBLE(HIGH);
    }
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    connectBLE();
  }
}