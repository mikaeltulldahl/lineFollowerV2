#include <Arduino.h>
#include "Motor.h"

int led = 13;

Motor rightMotor(0);
Motor leftMotor(1);

void setup()
{
	pinMode(led, OUTPUT);
}

void loop()
{
	digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(1000);               // wait for a second
	digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
	delay(1000);               // wait for a second
	rightMotor.set(0.3);
	leftMotor.set(0.3);
}
