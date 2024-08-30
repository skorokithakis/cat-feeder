#include <StavrosUtils.h>
#include "PubSubClient.h"
#include <Servo.h>
#include <VL53L0X.h>


#define SERVO_PIN 5

#define MQTT_PORT 1883
#define OPEN_TIMEOUT 30 * 60

#define CLOSING_GRACE_PERIOD_SECS 15
#define MAX_DISTANCE 30

#define SOUND_VELOCITY 0.034 // Centimeters per microsecond.

Servo myservo;
VL53L0X rangefinder;
StavrosUtils utils;
WiFiClient wclient;
PubSubClient client(wclient);
unsigned int lastCheck;
int closeDistances = 0;
bool lidOpen = false;
bool rebootAfterClose = false;

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

void openLid() {
    lidOpen = true;
    lastOpen = millis() / 1000;

    // Set lastCheck in the future so we don't start reading while the lid is opening.
    lastCheck = millis() + 2000;
}

void closeLid() {
    lidOpen = false;
}

// Receive a message from MQTT and act on it.
void mqttCallback(char *chTopic, byte *chPayload, unsigned int length) {
    chPayload[length] = '\0';
    String payload = String((char *)chPayload);

    if (payload == "open") {
        openLid();
        utils.debug("Got command to open lid.");
    } else if (payload == "close") {
        closeLid();
        utils.debug("Got command to close lid.");
    } else if (payload == "toggle") {
        if (lidOpen) {
            closeLid();
        } else {
            openLid();
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
        if (lidOpen) {
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

    if (pos == 0 && rebootAfterClose) {
        utils.debug("Lid closed, rebooting...");
        delay(500);
        ESP.restart();
    }
}

void checkDistance() {
    if (!lidOpen) {
        // Set lastNearby to now so the lid doesn't immediately close when we try to
        // open it.
        lastNearby = millis() / 1000;
        return;
    }

    if (millis() < lastCheck + 100) {
        return;
    }

    long distance = readDistance();
    utils.debug(String("Distance: ") + String(distance));

    if (distance < MAX_DISTANCE) {
        closeDistances++;
    } else {
        closeDistances = 0;
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
    lastCheck = millis();
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

    Wire.begin(12, 14);  // SDA, SCL
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
    updateServo();

    checkDistance();

    if (WiFi.status() != WL_CONNECTED) {
        utils.debug("Not connected to WiFi. Rebooting...");
        delay(500);
        ESP.restart();
    }
    delay(10);
}
