#include "Positioning.h"
#include <Arduino.h>
#include <Encoder.h>
#include <MPU9250.h>
#include <SPI.h>
#include <TeensyThreads.h>

MPU9250 IMU(SPI, 10);

Encoder rightEncoder(2, 3);
Encoder leftEncoder(30, 29);

#define ENCODER_SCALING (0.030f * M_PI / (10.0f * 12.0f))

int status;
float gyroZbias;

Positioning::Positioning(int i) {}

void Positioning::init(void) {
  reset();
  velocity = 0;
  velocityRight = 0;
  velocityLeft = 0;
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
  microsSinceEncoderMovedRight = 0;
  microsSinceEncoderMovedLeft = 0;
  oldPosRight = rightEncoder.read();
  oldPosLeft = leftEncoder.read();
  distRight = ENCODER_SCALING * (float)oldPosRight;
  distLeft = ENCODER_SCALING * (float)oldPosLeft;
}

void Positioning::reset() {
  heading = 0;
  distRight = 0;
  distLeft = 0;
  posX = 0;
  posY = 0;
}

void Positioning::calibrateGyroBias() {
#define CALIBRATION_SAMPLES_NUM 100
#define CALIBRATION_STABILITY_CRITERIA 0.5f
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

#define micros2sec(micros) (0.000001f * ((float)micros))

void Positioning::update(void) {
  int32_t newTime = (int32_t)micros();
  int32_t timeDiff = newTime - previousTime;
  previousTime = newTime;
  IMU.readSensor();
  angVel = (180.0f / M_PI) * IMU.getGyroZ_rads() - gyroZbias;
  heading += micros2sec(timeDiff) * angVel;
  int32_t newPosRight = rightEncoder.read();
  int32_t newPosLeft = leftEncoder.read();
  distRight = ENCODER_SCALING * (float)newPosRight;
  distLeft = ENCODER_SCALING * (float)newPosLeft;

  microsSinceEncoderMovedRight += timeDiff;
  if (newPosRight != oldPosRight) {
    float dPosRight = ENCODER_SCALING * ((float)(newPosRight - oldPosRight));
    oldPosRight = newPosRight;
    velocityRight = (float)dPosRight / micros2sec(microsSinceEncoderMovedRight);
    microsSinceEncoderMovedRight = 0;
  }
  if (micros2sec(microsSinceEncoderMovedRight) > 0.1f) {
    velocityRight = 0;
  }

  microsSinceEncoderMovedLeft += timeDiff;
  if (newPosLeft != oldPosLeft) {
    float dPosLeft = ENCODER_SCALING * ((float)(newPosLeft - oldPosLeft));
    oldPosLeft = newPosLeft;
    velocityLeft = (float)dPosLeft / micros2sec(microsSinceEncoderMovedLeft);
    microsSinceEncoderMovedLeft = 0;
  }
  if (micros2sec(microsSinceEncoderMovedLeft) > 0.1f) {
    velocityLeft = 0;
  }

  float velInstant = 0.5f * (velocityRight + velocityLeft);
  float rc = 0.002;
  float alpha = micros2sec(timeDiff) / (rc + micros2sec(timeDiff));
  velocity = (1.0f - alpha) * velocity + alpha * velInstant;
  posX += micros2sec(timeDiff) * velocity * cosf(M_PI / 180.0f * heading);
  posY += micros2sec(timeDiff) * velocity * sinf(M_PI / 180.0f * heading);
}
