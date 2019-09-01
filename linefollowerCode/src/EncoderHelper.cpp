#include "EncoderHelper.h"
#include <Arduino.h>
#include <Encoder.h>

#define ENCODER_SCALING (0.030f * M_PI / (10.0f * 12.0f))
#define micros2sec(micros) (0.000001f * ((float)micros))

EncoderHelper::EncoderHelper(int pin1, int pin2) {
  Encoder encoder(pin1, pin2);
  encPtr = &encoder;
}

void EncoderHelper::init(void) {
  reset();
  velocity = 0;
  previousTime = (int32_t)micros();
  microsSinceEncoderMoved = 0;
  dist = encPtr->read();
  oldDist = dist;
}

void EncoderHelper::reset() {
  dist = 0;
  oldDist = 0;
  encPtr->write(0);
}

float EncoderHelper::update(void) {
  int32_t newTime = (int32_t)micros();
  int32_t timeDiff = newTime - previousTime;
  previousTime = newTime;
  dist = encPtr->read();
  distMeters = ENCODER_SCALING * (float)dist;
  microsSinceEncoderMoved += timeDiff;
  float dPos;
  if (dist != oldDist) {
    dPos = ENCODER_SCALING * ((float)(dist - oldDist));
    oldDist = dist;
    float velocityInstant = (float)dPos / micros2sec(microsSinceEncoderMoved);
    float rc = 0.005;
    float alpha = micros2sec(microsSinceEncoderMoved) / (rc + micros2sec(microsSinceEncoderMoved));
    velocity = (1.0f - alpha) * velocity + alpha * velocityInstant;
    microsSinceEncoderMoved = 0;
  }else{
    dPos = 0;
  }
  if (micros2sec(microsSinceEncoderMoved) > 0.1f) {
    velocity = 0;
  }
  return dPos;
}
