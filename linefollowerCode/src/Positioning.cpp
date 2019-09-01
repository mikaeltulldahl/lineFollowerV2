#include "Positioning.h"
#include <Arduino.h>
#include <MPU9250.h>
#include <SPI.h>
#include <TeensyThreads.h>
#include "EncoderHelper.h"

MPU9250 IMU(SPI, 10);

EncoderHelper rightEncoder(2, 3);
EncoderHelper leftEncoder(30, 29);

#define micros2sec(micros) (0.000001f * ((float)micros))
#define CALIBRATION_SAMPLES_NUM 100
#define CALIBRATION_STABILITY_CRITERIA 0.5f

int status;
float gyroZbias;

Positioning::Positioning(int i) {}

void Positioning::init(void) {
  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(27);
  // SPI.beginTransaction()

  // start communication with IMU
  status = IMU.begin();
  IMU.setGyroRange(IMU.GYRO_RANGE_2000DPS);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.print("Status: ");
    Serial.println(status);
  }
  gyroCalibrated = false;
  // calibrateGyroBias();
  previousTime = (int32_t)micros();
  reset();
}

void Positioning::reset() {
  rightEncoder.reset();
  leftEncoder.reset();
  heading = 0;
  velocity = 0;
  posX = 0;
  posY = 0;
}

void Positioning::calibrateGyroBias() {
  float samples[CALIBRATION_SAMPLES_NUM];
  float sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES_NUM; i++) {
    // IMU.readSensor();
    samples[i] = 180.0f / M_PI * IMU.getGyroZ_rads();
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
  int32_t newTime = (int32_t)micros();
  int32_t timeDiff = newTime - previousTime;
  previousTime = newTime;
  IMU.readSensor();
  angVel = -((180.0f / M_PI) * IMU.getGyroZ_rads() - gyroZbias);
  heading += micros2sec(timeDiff) * angVel;
  float dPosRight = rightEncoder.update();
  float dPosLeft = leftEncoder.update();
  float dPos = 0.5f * (dPosRight + dPosLeft);
  velocity = 0.5f * (rightEncoder.velocity + leftEncoder.velocity);
  posX += dPos * cosf(M_PI / 180.0f * heading);
  posY += dPos * sinf(M_PI / 180.0f * heading);
}

int32_t Positioning::getDistRight() {
  return rightEncoder.dist;
}

int32_t Positioning::getDistLeft() {
  return leftEncoder.dist;
}

float Positioning::getPosX() {
  return posX;
}

float Positioning::getPosY() {
  return posY;
}