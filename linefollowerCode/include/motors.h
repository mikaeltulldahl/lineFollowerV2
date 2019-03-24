#ifndef MOTORS_H_
#define MOTORS_H_

void initMotors(void);
void stopMotors(void);

void setRightMotor(float pwm);
void setLeftMotor(float pwm);

#endif /* MOTORS_H_ */
