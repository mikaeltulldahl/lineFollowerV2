#ifndef MOTOR_H_
#define MOTOR_H_

class Motor {
 public:
  Motor(int side);
  void set(float pwm);
  void stop();

 private:
  float saturate(float in);
  int _dirPin, _pwmPin;
  bool _forwardDir;
};
#endif /* MOTOR_H_ */
