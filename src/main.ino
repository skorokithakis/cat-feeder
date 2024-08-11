#include <StavrosUtils.h>
#include "PubSubClient.h"
#include <Servo.h>

#define MQTT_PORT 1883
#define OPEN_TIMEOUT 30 * 60

#define TRIGGER_PIN 12
#define ECHO_PIN 14

#define CLOSING_GRACE_PERIOD_SECS 15
#define MAX_DISTANCE 50

#define SOUND_VELOCITY 0.034 // Centimeters per microsecond.

Servo myservo;
StavrosUtils utils;
WiFiClient wclient;
PubSubClient client(wclient);
int closeDistances = 0;
bool lidOpen = false;

// Whether the cat has come near the food after opening. This is so we can start the
// countdown for closing.
bool catThere = false;

unsigned int lastOpen = 0;
unsigned int lastNearby = 0;
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

    int retries = 4;
    String mac = WiFi.macAddress();
    utils.debug("Connecting to MQTT, MAC is " + mac + "...");
    while (!client.connect(mac.c_str()) && retries--) {
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

int readDistance() {
    long duration;
    long distance;

    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH, 20 * 1000);
    distance = duration * SOUND_VELOCITY / 2;

    return distance;
}

void setup() {
    Serial.begin(115200);
    utils.connectToWiFi();
    utils.doHTTPUpdate();

    client.setBufferSize(2 * 1024);
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqttCallback);

    connectMQTT();

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    myservo.attach(5);
    myservo.write(0);
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
            digitalWrite(LED_BUILTIN, outPos < 100 ? HIGH : LOW);
            pos = outPos;
        }
    }
}

void checkDistance() {
    int distance;

    if (!lidOpen) {
        // Set lastNearby to now so the lid doesn't immediately close when we try to
        // open it.
        lastNearby = millis() / 1000;
        return;
    }

    if (millis() % 50 > 0) {
        return;
    }
    distance = readDistance();
    utils.debug(String("Distance: ") + String(distance));

    if (distance < MAX_DISTANCE) {
        closeDistances++;
    }

    // Sometimes we get spurious readings, so require multiple distances to be close.
    if (closeDistances > 1) {
        if (!catThere) {
            utils.debug("The cat is here, will start the countdown.");
            catThere = true;
        }
        lastNearby = millis() / 1000;
        closeDistances = 0;
    }

    if (catThere && ((millis() / 1000) - lastNearby) > CLOSING_GRACE_PERIOD_SECS) {
        lidOpen = false;
        catThere = false;
        closeDistances = 0;
    }
}

void loop() {
    utils.connectToWiFi(5 * 60);
    connectMQTT();
    updateServo();

    checkDistance();

    if (WiFi.status() != WL_CONNECTED) {
        utils.debug("Not connected to WiFi. Rebooting...");
        delay(500);
        ESP.restart();
    }
    delay(2);
}
