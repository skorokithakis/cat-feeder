#include <StavrosUtils.h>
#include "PubSubClient.h"
#include <Servo.h>
#include <VL53L0X.h>

#define SERVO_PIN 5

#define MQTT_PORT 1883
#define OPEN_TIMEOUT 10 * 60

#define CLOSING_GRACE_PERIOD_SECS 15
#define MAX_DISTANCE 30

#define SOUND_VELOCITY 0.034 // Centimeters per microsecond.

enum LidState {
    LID_CLOSED,
    LID_OPEN,
    LID_PERSISTENT_OPEN
};

Servo myservo;
VL53L0X rangefinder;
StavrosUtils utils;
WiFiClient wclient;
PubSubClient client(wclient);
unsigned int lastCheck;
int closeDistances = 0;
LidState lidState = LID_CLOSED;
bool rebootAfterClose = false;

// Add near other global variables
bool isCatNear = false;
bool closeWhenCatLeaves = false;

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

void openLid(bool persistent = false) {
    closeWhenCatLeaves = false; // Cancel any pending close
    lidState = persistent ? LID_PERSISTENT_OPEN : LID_OPEN;
    lastOpen = millis() / 1000;

    // Set lastCheck in the future so we don't start reading while the lid is opening.
    lastCheck = millis() + 2000;
}

void closeLid() {
    if (isCatNear) {
        utils.debug("Cat is nearby; will close when cat leaves.");
        closeWhenCatLeaves = true;
        return;
    }
    lidState = LID_CLOSED;
    closeWhenCatLeaves = false;
}

// Receive a message from MQTT and act on it.
void mqttCallback(char *chTopic, byte *chPayload, unsigned int length) {
    chPayload[length] = '\0';
    String payload = String((char *)chPayload);

    if (payload == "open") {
        openLid(false);
        utils.debug("Got command to open lid.");
    } else if (payload == "persistent_open") {
        openLid(true);
        utils.debug("Got command to open lid persistently.");
    } else if (payload == "close") {
        closeLid();
        utils.debug("Got command to close lid.");
    } else if (payload == "toggle") {
        if (lidState != LID_CLOSED) {
            closeLid();
        } else {
            openLid(false);
        }
        utils.debug("Got command to toggle the lid.");
    } else if (payload == "reboot") {
        utils.debug("Got a reboot command, rebooting...");
        closeLid();
        rebootAfterClose = true;
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
        if (lidState != LID_CLOSED) {
            utils.debug("\nfatal: MQTT server connection failed with the lid open, closing lid to reboot.");
            closeLid();
            rebootAfterClose = true;
        } else {
            utils.debug("\nfatal: MQTT server connection failed. Rebooting.");
            delay(500);
            ESP.restart();
        }
    }

    utils.debug("Connected to MQTT.");
    client.subscribe("catfeeder/command");
}

long readDistance() {
    long distance = rangefinder.readRangeSingleMillimeters();

    if (rangefinder.timeoutOccurred()) {
        distance = 1000;
    }

    return distance / 10;
}

void updateServo() {
    int newPos = pos;

    if (newPos >= 0 && newPos <= 180) {
        if (lidState != LID_CLOSED) {
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

    if (pos == 0 && rebootAfterClose) {
        utils.debug("Lid closed, rebooting...");
        delay(500);
        ESP.restart();
    }
}

void checkDistance() {
    if (millis() < lastCheck + 100) {
        return;
    }

    // Only check distance if lid is open
    if (lidState == LID_CLOSED) {
        return;
    }

    long distance = readDistance();
    utils.debug(String("Distance: ") + String(distance));

    if (distance < MAX_DISTANCE) {
        closeDistances++;
    } else {
        closeDistances = 0;
    }

    // Sometimes we get spurious readings, so require multiple distances to be close
    bool wasNear = isCatNear;
    isCatNear = (closeDistances > 1);

    if (isCatNear) {
        lastNearby = millis() / 1000;
        if (!wasNear) {
            utils.debug("Cat detected nearby.");
        }
    } else if (wasNear) {
        utils.debug("Cat has left");
        // Check if we were waiting to close
        if (closeWhenCatLeaves) {
            utils.debug("Executing delayed close command...");
            closeWhenCatLeaves = false;
            lidState = LID_CLOSED;
        }
    }

    lastCheck = millis();
}

// Add new function to handle lid state changes
void updateLidState() {
    // Handle auto-closing for normal open state
    if (lidState == LID_OPEN && !isCatNear &&
            ((millis() / 1000) - lastOpen) > OPEN_TIMEOUT) {
        utils.debug("Auto-closing lid after timeout...");
        lidState = LID_CLOSED;
    }
}

void setup() {
    Serial.begin(115200);

    utils.connectToWiFi();
    utils.doHTTPUpdate();

    int count;

    for (byte i = 8; i < 120; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX);
            Serial.println(")");
            count++;
            delay(1);
        }
    }
    Serial.println("Done searching for I2C.");

    client.setBufferSize(2 * 1024);
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqttCallback);

    connectMQTT();

    pinMode(LED_BUILTIN, OUTPUT);

    myservo.attach(SERVO_PIN);
    myservo.write(0);

    Wire.begin(12, 14); // SDA, SCL
    rangefinder.setTimeout(500);
    if (!rangefinder.init()) {
        utils.debug("Failed to detect and initialize sensor!");
        delay(1000);
        ESP.restart();
    }
    rangefinder.setSignalRateLimit(0.1);
    rangefinder.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    rangefinder.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    rangefinder.setMeasurementTimingBudget(20 * 1000);
}

void loop() {
    utils.connectToWiFi(5 * 60);
    connectMQTT();

    checkDistance();  // Just updates isCatNear
    updateLidState(); // Handles automatic lid state changes
    updateServo();    // Handles physical lid movement

    if (WiFi.status() != WL_CONNECTED) {
        utils.debug("Not connected to WiFi. Rebooting...");
        delay(500);
        ESP.restart();
    }
    delay(10);
}
