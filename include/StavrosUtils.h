#include <DNSServer.h>
#include <EEPROM.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClientSecureBearSSL.h>
#include <WiFiUdp.h>
#include "WiFiManager.h"

#define CONFIG_VERSION 3926590

typedef struct {
    unsigned long int version;
    char someString[51];
} StateType;


class StavrosUtils {
private:
    WiFiManager wifiManager;
    IPAddress broadcastIP;


    void _connectToWiFi(int timeout) {
        if (WiFi.status() == WL_CONNECTED) {
            return;
        }
        debug("Connecting to WiFi...");
        WiFi.forceSleepWake();
        if (timeout > 0) {
            wifiManager.setConfigPortalTimeout(timeout);
        }
        wifiManager.setHostname(PROJECT_NAME);
        wifiManager.autoConnect(PROJECT_NAME);
        this->broadcastIP = WiFi.localIP();
        this->broadcastIP[3] = 255;
        debug("Connected to WiFi.");
    }

public:
    StateType state;

    void loadState() {
        EEPROM.begin(sizeof(this->state));
        EEPROM.get(0, this->state);
        if (this->state.version != CONFIG_VERSION) {
            state = {CONFIG_VERSION, '\0'};
        }
    }

    void saveState() {
        EEPROM.begin(sizeof(this->state));
        EEPROM.put(0, this->state);
        EEPROM.commit();
    }

    void logUDP(String message) {
        if (WiFi.status() != WL_CONNECTED) {
            return;
        }

        WiFiUDP Udp;

        // Listen with `nc -kul 37243`.
        Udp.beginPacket(this->broadcastIP, 37243);
        Udp.write(("(" PROJECT_NAME  ": " + String(millis()) + " - " + WiFi.localIP().toString() + ") " + ": " + message + "\n").c_str());
        Udp.endPacket();
    }

    void debug(String text) {
        Serial.println(text);
        logUDP(text);
    }

    void doHTTPUpdate() {
        if (WiFi.status() != WL_CONNECTED) {
            return;
        }
        debug("[update] Looking for an update from v" VERSION ".");

        std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);
        client->setFingerprint(OTA_FINGERPRINT);
        t_httpUpdate_return ret = ESPhttpUpdate.update(*client, "https://" OTA_HOSTNAME "/" PROJECT_NAME "/", VERSION);
        switch (ret) {
        case HTTP_UPDATE_FAILED:
            debug("[update] Update failed.");
            break;
        case HTTP_UPDATE_NO_UPDATES:
            debug("[update] No update from v" VERSION ".");
            break;
        case HTTP_UPDATE_OK:
            debug("[update] Update ok.");
            break;
        }
        debug("Ready.");
    }


    void resetWiFiSettings() {
        wifiManager.resetSettings();
    }

    void connectToWiFi() {
        _connectToWiFi(0);
    }


    void connectToWiFi(int timeout) {
        _connectToWiFi(timeout);
    }
};
