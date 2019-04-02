#include <Arduino.h>
#include <TeensyThreads.h>
#include "Logger.h"
#include "Motor.h"
#include "Positioning.h"

int ledPin = 13;

Motor rightMotor(0);
Motor leftMotor(1);

Positioning positioning(0);
Logger logger(0);

volatile int blinkcode = 3;

void blinkthread() {
  while(1) {
      for (int i=0; i<blinkcode; i++) {
        digitalWrite(ledPin, HIGH);
        threads.delay(150);
        digitalWrite(ledPin, LOW);
        threads.delay(150);
      }
      threads.delay(2000);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("START");
  threads.setDefaultTimeSlice(1);
  pinMode(ledPin, OUTPUT);
  logger.init();
  positioning.init();
  threads.addThread(blinkthread);
}

void loop() {
  positioning.update();
  logger.update();
  threads.delay(10);      // wait for a second
  rightMotor.set(0.3);
  leftMotor.set(0.3);
}
