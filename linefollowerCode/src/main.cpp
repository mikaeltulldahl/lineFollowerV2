#include <Arduino.h>
#include "Motor.h"
#include <SPI.h>
#include <MPU9250.h>

int led = 13;

Motor rightMotor(0);
Motor leftMotor(1);

MPU9250 IMU(SPI,10);

float heading = 0;
int status;

void setup()
{
	pinMode(led, OUTPUT);

	SPI.setMOSI(11);
	SPI.setMISO(12);
	SPI.setSCK(27);

	  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}
    	digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(500);               // wait for a second
	digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
	delay(500);               // wait for a second

  // start communication with IMU 
  status = IMU.begin();
  IMU.setGyroRange(IMU.GYRO_RANGE_2000DPS);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  	digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(500);               // wait for a second
	digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
	delay(500);               // wait for a second
}

void loop()
{
	digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(1000);               // wait for a second
	digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
	delay(1000);               // wait for a second
	rightMotor.set(0.3);
	leftMotor.set(0.3);
	IMU.readSensor();
	  Serial.print(180/3.14159*IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  heading += 0.1*180/3.14159*IMU.getGyroZ_rads();
  Serial.println(heading,6);

}
