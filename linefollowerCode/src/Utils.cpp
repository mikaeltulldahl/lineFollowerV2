
#include "Utils.h"

#include <Arduino.h>

/*
 * Calculates the mean and the standard deviation on a buffer of values
 */
void getbufferMeanStd(const float* buffer,
                      const int length,
                      float& mean,
                      float& std) {
  float sum = 0;
  for (int i = 0; i < length; i++) {
    sum += buffer[i];
  }
  float diffSqr = 0;
  mean = sum / (float)length;
  for (int i = 0; i < length; i++) {
    float diff = buffer[i] - mean;
    diffSqr += diff * diff;
  }
  std = sqrtf(diffSqr / (float(length)));
}

/*
 * First order infinite impulse response filter according to
 * https://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
 */
float lowPassIIR(float in, float outPrevious, float dt, float rc) {
  float alpha = dt / (rc + dt);
  return (1.0f - alpha) * outPrevious + alpha * in;
}

float pow2(float in) {
  return in * in;
}

float signum(float in) {
  if (in >= 0.0f) {
    return 1.0f;
  } else {
    return -1.0f;
  }
}
