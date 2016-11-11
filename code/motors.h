#ifndef MOTORS_H_
#define MOTORS_H_

#include "master_include.h"

extern volatile float motor_right;
extern volatile float motor_left;

extern volatile uint32_t motor_right_ticks;
extern volatile uint32_t motor_left_ticks;

void initMotors(void);
void stopMotors(void);
//void setMotorPwm(int motorIndex, int32_t motorPwm);
//void setMotorPwms(float* motorPwms);
//void stopMotors(void);

void setRightMotor(float pwm);
void setLeftMotor(float pwm);

//void pwm_out_sendSignalsToMotors();
#endif /* MOTORS_H_ */
