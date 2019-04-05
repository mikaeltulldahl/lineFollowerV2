#include "Positioning.h"
#include <Arduino.h>
#include <MPU9250.h>
#include <SPI.h>
#include <TeensyThreads.h>

MPU9250 IMU(SPI, 10);

int status;
int lastTime;
float gyroZbias;

Positioning::Positioning(int i) {}

void Positioning::init(void) {
  heading = 0;
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
  gyroCalibrated = false;
  calibrateGyroBias();
}

void Positioning::calibrateGyroBias() {
#define CALIBRATION_SAMPLES_NUM 100
#define CALIBRATION_STABILITY_CRITERIA 0.5f
  float samples[CALIBRATION_SAMPLES_NUM];
  float sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES_NUM; i++) {
    IMU.readSensor();
    samples[i] = 180.0f / 3.14159f * IMU.getGyroZ_rads();
    sum += samples[i];
    threads.delay(10);
  }
  float diffSqr = 0;
  float meanSample = sum / (float)CALIBRATION_SAMPLES_NUM;
  for (int i = 0; i < CALIBRATION_SAMPLES_NUM; i++) {
    float diff = samples[i] - meanSample;
    diffSqr += diff * diff;
  }

  float stdSample = sqrtf(diffSqr / (float(CALIBRATION_SAMPLES_NUM)));
  Serial.print("std: ");
  Serial.print(stdSample, 3);
  Serial.print(", mean: ");
  Serial.println(meanSample, 3);
  boolean stable = stdSample <= CALIBRATION_STABILITY_CRITERIA;
  if (stable) {
    gyroZbias = meanSample;
  }
  gyroCalibrated = stable;
}

void Positioning::update(void) {
  int newTime = millis();
  float dt = 0.001 * (float)(newTime - lastTime);
  lastTime = newTime;
  IMU.readSensor();
  // Serial.print(180 / 3.14159 * IMU.getGyroZ_rads(), 6);
  // Serial.print("\t");
  angVel = (180.0f / 3.14159f) * IMU.getGyroZ_rads() - gyroZbias;
  heading += dt * angVel;
  // Serial.println(heading, 6);
}
