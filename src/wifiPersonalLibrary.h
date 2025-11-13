/*
  BRONNEN:
  ArduinoDocs, refresher listed strings (23/10/2025): https://docs.arduino.cc/language-reference/en/variables/data-types/string/
*/

#include <WiFi.h>

int wifiAmounts_PL = 0;   // the amount of known WiFi networks
char *knownWifiSsid_PL[10] = {"", "", "", "", "", "", "", "", "", ""};      // the known WiFi networks
char *knownWifiPassword_PL[10] = {"", "", "", "", "", "", "", "", "", ""};  // the passwords for the known WiFi networks
int wifiAttemps_PL = 5;   // amount of allowed attemps for device to connect to WiFi
int wifiDelay_PL = 500;   // how much time between attemps

// Initiate the WiFi Personal Library
void wifiInit(int wifiAmounts_i_PL, char *knownWifiSsid_i_PL[], char *knownWifiPassword_i_PL[], int wifiAttemps_i_PL = 5, int wifiDelay_i_PL = 500) {
  wifiAmounts_PL = wifiAmounts_i_PL;
  if (wifiAmounts_PL > 0) {
    if (wifiAmounts_PL > 10) {
      Serial.println("WARNING: too many networks, only the first 10 will be saved");
    }
    for (int i = 0; i < wifiAmounts_PL && i < 10; i++) {
      knownWifiSsid_PL[i] = knownWifiSsid_i_PL[i];
      knownWifiPassword_PL[i] = knownWifiPassword_i_PL[i];
    }
    wifiAttemps_PL = wifiAttemps_i_PL;
    wifiDelay_PL = wifiDelay_i_PL;
  }
  else {
    Serial.println("ERROR: no networks given.");
  }
}

// Connect to a known WiFi network
void wifiConnect(const char* wifiHostName_PL = "", wifi_mode_t wifiMode_PL = WIFI_STA) {
  WiFi.mode(wifiMode_PL);
  int networks = WiFi.scanNetworks();
  if (networks > 0) {
    for (int n = 0; n < networks && WiFi.status() != WL_CONNECTED; n++) {
      String ssid = WiFi.SSID(n);
      for (int i = 0; i < wifiAmounts_PL; i++) {
        if (ssid == knownWifiSsid_PL[i]) {
          // if hostname, set hostname
          if (wifiHostName_PL != "") {
            WiFi.setHostname(wifiHostName_PL);
          }
          // connect to WiFi
          String password = knownWifiPassword_PL[i];
          int connectAttemps = 0;
          WiFi.begin(ssid, password);
          Serial.print("connecting to: ");
          Serial.print(ssid);
          while (WiFi.status() != WL_CONNECTED && connectAttemps < wifiAttemps_PL) {
            connectAttemps++;
            delay(wifiDelay_PL);
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

// Get the status of the WiFi connection
void getWifiStatus() {
  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    String ssid = WiFi.SSID();
    int rssi = WiFi.RSSI();
    Serial.print("Connected to: ");
    Serial.print(ssid);
    Serial.print(" - ");
    Serial.println(rssi);
  }
  else {
    Serial.println("Status: Not Connected");
  }
}

// Event-handler functies

void onGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void onDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WiFi Disconnected, Attempting to Reconnect...");
  WiFi.reconnect();
  delay(wifiDelay_PL);
  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    Serial.println("Reconnected");
  }
  else {
    Serial.println("Connection Failed");
  }
}

/* void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);

  switch (event) {
    case SYSTEM_EVENT_STA_START:
      Serial.println("WiFi adapter gestart");
      break;

    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("Verbonden met WiFi netwerk");
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Verbinding verbroken - probeer opnieuw te verbinden...");
      WiFi.reconnect();
      break;

    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("IP-adres gekregen: ");
      Serial.println(WiFi.localIP());
      break;

    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.println("IP-adres verloren!");
      break;

    default:
      Serial.println("Onbekend WiFi-event");
      break;
  }
} */
