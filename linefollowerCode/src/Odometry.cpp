#include "Odometry.h"

#include <Arduino.h>
#include <MPU9250.h>
#include <SPI.h>
#include <TeensyThreads.h>

#include "EncoderHelper.h"
#include "Utils.h"

MPU9250 IMU(SPI, 10);

EncoderHelper rightEncoder(2, 3);
EncoderHelper leftEncoder(30, 29);

#define micros2sec(micros) (0.000001f * ((float)micros))
#define CALIBRATION_SAMPLES_NUM 100
#define CALIBRATION_STABILITY_CRITERIA 0.5f

int status;
float gyroZbias;

Odometry::Odometry() {}

void Odometry::init(void) {
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

void Odometry::reset() {
  rightEncoder.reset();
  leftEncoder.reset();
  heading = 0;
  velocity = 0;
  x = 0;
  y = 0;
}

void Odometry::calibrateGyroBias() {
  float samples[CALIBRATION_SAMPLES_NUM];
  for (int i = 0; i < CALIBRATION_SAMPLES_NUM; i++) {
    samples[i] = 180.0f / M_PI * IMU.getGyroZ_rads();
    threads.delay(10);
  }
  float meanSample, stdSample;
  getbufferMeanStd(samples, CALIBRATION_SAMPLES_NUM, meanSample, stdSample);

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

void Odometry::update(void) {
  int32_t newTime = (int32_t)micros();
  int32_t timeDiff = newTime - previousTime;
  previousTime = newTime;
  IMU.readSensor();
  angVel = -((180.0f / M_PI) * IMU.getGyroZ_rads() - gyroZbias);
  heading += micros2sec(timeDiff) * angVel;
  float dDistRight = rightEncoder.update();
  float dDistLeft = leftEncoder.update();
  float dDist = 0.5f * (dDistRight + dDistLeft);
  velocity = 0.5f * (rightEncoder.velocity + leftEncoder.velocity);
  x += dDist * cosf(M_PI / 180.0f * heading);
  y += dDist * sinf(M_PI / 180.0f * heading);
}

int32_t Odometry::getDistRight() {
  return rightEncoder.dist;
}

int32_t Odometry::getDistLeft() {
  return leftEncoder.dist;
}

float Odometry::getX() {
  return x;
}

float Odometry::getY() {
  return y;
}

float Odometry::getHeading() {
  return heading;
}

void Odometry::resetHeading() {
  heading = 0;
}

float Odometry::getAngVel() {
  return angVel;
}

float Odometry::getVelocity() {
  return velocity;
}