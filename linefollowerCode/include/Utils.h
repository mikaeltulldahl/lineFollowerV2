#ifndef UTILS_H_
#define UTILS_H_
#include <Arduino.h>

void getbufferMeanStd(const float* buffer,
                      const int length,
                      float& mean,
                      float& std);

float lowPassIIR(float in, float outPrevious, float dt, float rc);

#endif /* UTILS_H_ */