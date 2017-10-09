#include <Servo.h>

#include "Support.h"
#include "PinPulseIn.h"

#define RC_STR_PIN 10
#define RC_TRTL_PIN 9
#define CAR_STR_PIN 23
#define CAR_TRTL_PIN 22

PinPulseIn<RC_STR_PIN> rcSteer;
PinPulseIn<RC_TRTL_PIN> rcThrottle;

Servo carSteer;
Servo carThrottle;

bool hasRC = false;
uint32_t lastModeTime = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  
  rcSteer.init();
  rcThrottle.init();
  onCrash(detachServos);

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

void loop() {
  uint32_t now = millis();

  //  I only use PinPulseIn to detect that there is a carrier
  if (rcSteer.hasValue() || rcThrottle.hasValue()) {
    lastModeTime = now;
    hasRC = true;
  }

  if (now - lastModeTime > 300) {
    lastModeTime = now - 300;
    hasRC = false;
  }

  if (rcSteer.hasValue()) {
    uint32_t steer = rcSteer.getValue();
    Serial.print("steering: ");
    Serial.println(steer);
    carSteer.writeMicroseconds(steer);
  }
  if (rcThrottle.hasValue()) {
    Serial.print("throttle: ");
    Serial.println(rcThrottle.getValue());
  }
      delay(10);
}
