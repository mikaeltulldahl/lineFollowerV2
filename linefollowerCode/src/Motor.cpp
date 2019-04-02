
#include "Motor.h"
#include <Arduino.h>

// https://www.pjrc.com/teensy/td_pulse.html

#define NO_MOTORS 2

int dirPins[NO_MOTORS] = {5, 6};
int pwmPins[NO_MOTORS] = {4, 7};

#define MOTOR_FREQ 20000
#define MOTOR_PWM_RESOLUTION 2047

Motor::Motor(int side) {
  _dirPin = dirPins[side];
  _pwmPin = pwmPins[side];
  pinMode(_dirPin, OUTPUT);
  pinMode(_pwmPin, OUTPUT);
  analogWriteResolution(11);
  analogWriteFrequency(_pwmPin, MOTOR_FREQ);
  _forwardDir = false;
  stop();
}

void Motor::set(float pwm) {
  bool newForwardDir = pwm >= 0;
  if (newForwardDir != _forwardDir) {
    digitalWrite(_dirPin, (int)newForwardDir);
    _forwardDir = newForwardDir;
  }
  analogWrite(_pwmPin, (int)(saturate(abs(pwm)) * MOTOR_PWM_RESOLUTION));
}

float Motor::saturate(float in) {
  if (in > 1) {
    return 1;
  } else {
    return in;
  }
}

void Motor::stop() {
  set(0);
}
