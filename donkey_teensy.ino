#include <Servo.h>

#include "Support.h"
#include "PinPulseIn.h"

#define LED_PIN 13

#define RC_STR_PIN 10
#define RC_TRTL_PIN 9
#define CAR_STR_PIN 23
#define CAR_TRTL_PIN 22

uint32_t COMM_TIMEOUT = 300;

uint32_t KILL_ZONE_HI = 1700;
uint32_t KILL_ZONE_LO = 1450;

uint32_t CYCLE = 10; // 10ms cycle

PinPulseIn<RC_STR_PIN> rcSteer;
PinPulseIn<RC_TRTL_PIN> rcThrottle;

Servo carSteer;
Servo carThrottle;

uint32_t steeringIn = 1500;
uint32_t throttleIn = 1500;
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
    carThrottle.attach(CAR_TRTL_PIN);
  }
}

int readSerialIfAvailable()
{  
  if (Serial.available() < 4) {
    return;
  }

  char b = Serial.read();
  
  if (b == 'S') {
    // Steering value in us
    // S<steer>\n
    steeringOut = Serial.parseInt();
  }
  else if (b == 'T') {
    // Throttle value in us
    // T<throttle>\n
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
    uint32_t finalThrottle = maybeKilled();
    carThrottle.writeMicroseconds(finalThrottle);
    ledOn();
  }

  if (rcSteer.hasValue()) {
    steeringIn = rcSteer.getValue();
    Serial.print("S");
    Serial.println(steeringIn);
  }
  if (rcThrottle.hasValue()) {
    throttleIn = rcThrottle.getValue();
    Serial.print("T");
    Serial.println(throttleIn);
  }

  readSerialIfAvailable();
}

// "kill switch" is when throttle goes above KILL_ZONE_HI (high speed reverse)
// Kill switch will be deactivated when throttle falls below KILL_ZONE_LOW (low speed forward)
// When robot is "killed", it simply takes RC transmitter throttle input
// Also remember that ESC needs "double reverses" - the 1st reverse serves as brake. When throttle
//   goes back to neutral and goes into reverse zone again, it starts to reverse

bool killed = false;

uint32_t maybeKilled() {
  if (throttleIn > KILL_ZONE_HI) {
    killed = true;
    return throttleIn;
  }

  if (throttleIn < KILL_ZONE_LO) {
    killed = false;
    return throttleOut;
  }

  // fill the cycle
  if ((lastCycleTime + CYCLE) > millis()) {
    delay(lastCycleTime + CYCLE - millis());
  }
  lastCycleTime = millis();
  
  // When throttleIn is between KILL_ZONE_LO and KILL_ZONE_HI
  if (killed) {
    return throttleIn;
  } else {
    return throttleOut;
  }
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

