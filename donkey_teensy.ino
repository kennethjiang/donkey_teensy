#include <Servo.h>

#include "Support.h"
#include "PinPulseIn.h"

#define LED_PIN 13

#define RC_STR_PIN 10
#define RC_TRTL_PIN 9
#define CAR_STR_PIN 23
#define CAR_TRTL_PIN 22

uint32_t COMM_TIMEOUT = 300;

PinPulseIn<RC_STR_PIN> rcSteer;
PinPulseIn<RC_TRTL_PIN> rcThrottle;

Servo carSteer;
Servo carThrottle;

uint32_t steeringOut = 1500;
uint32_t throttleOut = 1500;

bool hasRC = false;
uint32_t lastRCTime = 0;

bool hasSerial = false;
uint32_t lastSerialTime = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  
  rcSteer.init();
  rcThrottle.init();
  onCrash(detachServos);

  pinMode(LED_PIN, OUTPUT);
}

void detachServos() {
  if (carSteer.attached()) {
    carSteer.detach();
  }
  if (carThrottle.attached()) {
    carThrottle.detach();
  }
}

void attachServos() {
  if (!carSteer.attached()) {
    carSteer.attach(CAR_STR_PIN);
  }
  if (!carThrottle.attached()) {
    carThrottle.attach(RC_TRTL_PIN);
  }
}

int readSerialIfAvailable()
{  
  while (Serial.available() >= 3) {
    char b = Serial.read();
    
    if (b == 'S') {
      // Steering value in us
      // S <steer>\n
      steeringOut = Serial.parseInt();
    }
    else if (b == 'T') {
      // Throttle value in us
      // T <throttle>\n
      throttleOut = Serial.parseInt();
    }
    
    // Read the newline
    do {
      b = Serial.read();
    }
    while (b != '\n');
    
    lastSerialTime = millis();
    hasSerial = true;
  }
}

void loop() {
  uint32_t now = millis();

  //  I only use PinPulseIn to detect that there is a carrier
  if (rcSteer.hasValue() || rcThrottle.hasValue()) {
    lastRCTime = now;
    hasRC = true;
  }

  if (now - lastRCTime > COMM_TIMEOUT) {
    hasRC = false;
  }

  if (now - lastSerialTime > COMM_TIMEOUT) {
    hasSerial = false;
  }

  if (!hasRC || !hasSerial) {
    /* turn off the servos */
    detachServos();
    blink();
  } else {
    attachServos();
    carSteer.writeMicroseconds(steeringOut);
    carThrottle.writeMicroseconds(throttleOut);
    ledOn();
  }

  if (rcSteer.hasValue()) {
    Serial.print("S");
    Serial.println(rcSteer.getValue());
  }
  if (rcThrottle.hasValue()) {
    Serial.print("T");
    Serial.println(rcThrottle.getValue());
  }

  readSerialIfAvailable();
}


// LED
enum LED_STATUS {
  ON,
  OFF,
  BLINK,
} ledStatus;

bool blinkState = LOW;
long lastSwitchingTime = 0;

void blink() {
    ledStatus = BLINK;
    led();
}

void ledOn() {
    ledStatus = ON;
    led();
}

void led() {
    switch (ledStatus) {
        case BLINK:
            if ((millis() - lastSwitchingTime) > 200) {
                blinkState = !blinkState;
                digitalWrite(LED_PIN, blinkState);
                lastSwitchingTime = millis();
            }
          break;
        case ON:
            digitalWrite(LED_PIN, HIGH);
            break;
        case OFF:
            digitalWrite(LED_PIN, LOW);
            break;
    }
}

