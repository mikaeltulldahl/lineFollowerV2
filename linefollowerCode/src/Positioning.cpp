#include "Positioning.h"
#include <Arduino.h>
#include <MPU9250.h>
#include <SPI.h>

MPU9250 IMU(SPI, 10);

int status;
int lastTime;
float gyroZbias;

Positioning::Positioning(int i) {}

void Positioning::init(void) {
  headingRad = 0;
  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(27);

  // start communication with IMU
  status = IMU.begin();
  IMU.setGyroRange(IMU.GYRO_RANGE_2000DPS);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.print("Status: ");
    Serial.println(status);
  }
  lastTime = millis();
  gyroZbias = IMU.getGyroZ_rads();
}

void Positioning::update(void) {
  int newTime = millis();
  float dt = 0.001 * (float)(newTime - lastTime);
  lastTime = newTime;
  IMU.readSensor();
  //Serial.print(180 / 3.14159 * IMU.getGyroZ_rads(), 6);
  //Serial.print("\t");
  angVelRad = IMU.getGyroZ_rads() - gyroZbias;
  headingRad += dt * angVelRad;
  //Serial.println(180 / 3.14159 * headingRad, 6);
}
