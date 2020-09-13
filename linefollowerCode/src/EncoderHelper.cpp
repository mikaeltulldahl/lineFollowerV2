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

const float VELOCITY_LOW_PASS_RC = 0.05;

EncoderHelper::EncoderHelper(int pin1, int pin2) : encoder(pin1, pin2) {}

void EncoderHelper::init(void) {
  reset();
  velocity = 0;
  previousTime = (int32_t)micros();
  microsSinceEncoderMoved = 0;
  steps = encoder.read();
  previousSteps = steps;
}

void EncoderHelper::reset() {
  steps = 0;
  previousSteps = 0;
  encoder.write(0);
}

float EncoderHelper::update(void) {
  int32_t newTime = (int32_t)micros();
  int32_t timeDiff = newTime - previousTime;
  float dDist = 0;
  if (timeDiff > 0) {
    steps = encoder.read();
    dist = ENCODER_SCALING * (float)steps;
    microsSinceEncoderMoved += timeDiff;
    if (steps != previousSteps) {
      dDist = ENCODER_SCALING * ((float)(steps - previousSteps));
      float velocityInstant = dDist / micros2sec(microsSinceEncoderMoved);
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
    previousTime = newTime;
    previousSteps = steps;
  }
  return dDist;
}
