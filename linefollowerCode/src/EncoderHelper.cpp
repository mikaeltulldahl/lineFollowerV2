#include "EncoderHelper.h"

#include <Arduino.h>
#include <Encoder.h>

#include "Utils.h"

#define GEAR_RATIO 10.0f
#define WHEEL_DIAMETER 0.030f
#define ENCODER_STEPS_PER_TURN 12.0f
#define ENCODER_SCALING \
  (WHEEL_DIAMETER * M_PI / (GEAR_RATIO * ENCODER_STEPS_PER_TURN))
#define micros2sec(micros) (0.000001f * ((float)micros))

const float VELOCITY_LOW_PASS_RC = 0.005;

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
  float dDist;
  if (dist != oldDist) {
    dDist = ENCODER_SCALING * ((float)(dist - oldDist));
    oldDist = dist;
    float velocityInstant = (float)dDist / micros2sec(microsSinceEncoderMoved);
    velocity =
        lowPassIIR(velocityInstant, velocity,
                   micros2sec(microsSinceEncoderMoved), VELOCITY_LOW_PASS_RC);
    microsSinceEncoderMoved = 0;
  } else {
    dDist = 0;
  }
  if (micros2sec(microsSinceEncoderMoved) > 0.1f) {
    velocity = 0;
  }
  return dDist;
}
