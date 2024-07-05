#include <StavrosUtils.h>
#include "PubSubClient.h"
#include <Servo.h>

#define MQTT_PORT 1883
#define OPEN_TIMEOUT 30 * 60

Servo myservo;
StavrosUtils utils;
WiFiClient wclient;
PubSubClient client(wclient);
bool lidOpen = false;
unsigned int lastOpen = 0;
int pos = 1; // Just so the closing routine runs at startup.

// Publish a message to MQTT if connected.
void mqttPublish(String topic, String payload) {
    if (!client.connected()) {
        return;
    }
    client.publish(topic.c_str(), payload.c_str());
}

// Receive a message from MQTT and act on it.
void mqttCallback(char *chTopic, byte *chPayload, unsigned int length) {
    chPayload[length] = '\0';
    String payload = String((char *)chPayload);

    if (payload == "open") {
        lidOpen = true;
        lastOpen = millis() / 1000;
        utils.debug("Got command to open lid.");
    } else if (payload == "close") {
        lidOpen = false;
        utils.debug("Got command to close lid.");
    } else if (payload == "toggle") {
        if (lidOpen) {
            lidOpen = false;
        } else {
            lidOpen = true;
            lastOpen = millis() / 1000;
        }
        utils.debug("Got command to toggle the lid.");
    } else if (payload == "reboot") {
        utils.debug("Got a reboot command, rebooting...");
        delay(500);
        ESP.restart();
    }
}

// Check the MQTT connection and reboot if we can't connect.
void connectMQTT() {
    client.loop();

    if (client.connected()) {
        return;
    }

    client.setBufferSize(2 * 1024);
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqttCallback);

    int retries = 4;
    utils.debug("Connecting to MQTT...");
    while (!client.connect(PROJECT_NAME) && retries--) {
        delay(500);
        utils.debug("Retry...");
    }

    if (!client.connected()) {
        utils.debug("\nfatal: MQTT server connection failed. Rebooting.");
        delay(500);
        ESP.restart();
    }

    utils.debug("Connected to MQTT.");
    client.subscribe("catfeeder/command");
}

void setup() {
    Serial.begin(115200);
    utils.connectToWiFi();
    utils.doHTTPUpdate();
    connectMQTT();
    myservo.attach(5);
    myservo.write(0);
    pinMode(LED_BUILTIN, OUTPUT);
}

void updateServo() {
    int newPos = pos;

    if (lidOpen && (((millis() / 1000) - lastOpen) > OPEN_TIMEOUT)) {
        utils.debug("The lid has been open too long, closing...");
        lidOpen = false;
    }

    if (newPos >= 0 && newPos <= 180) {
        if (lidOpen) {
            newPos += 3;
        } else {
            newPos -= 1;
        }
        int outPos = constrain(newPos, 0, 180);
        if (outPos != pos) {
            myservo.write(outPos);
            analogWrite(LED_BUILTIN, outPos == 0 ? HIGH : LOW);
            pos = outPos;
        }
    }
}

void loop() {
    utils.connectToWiFi(5 * 60);
    connectMQTT();
    updateServo();

    if (WiFi.status() != WL_CONNECTED) {
        utils.debug("Not connected to WiFi. Rebooting...");
        delay(500);
        ESP.restart();
    }
    delay(2);
}
