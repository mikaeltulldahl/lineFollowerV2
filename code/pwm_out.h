#ifndef PWM_OUT_H
#define PWM_OUT_H

#include "master_include.h"

#define PWM_OUT_FREQ 15000 //Hz
#define PWM_OUT_PERIOD_MS (1000/(float)PWM_OUT_FREQ) //2 ms
#define PWM_OUT_PERIOD_US (1000000/(float)PWM_OUT_FREQ) //2000 us

#define PWM_OUT1_INITDUTY 0
#define PWM_OUT2_INITDUTY 0
#define PWM_OUT3_INITDUTY 0
#define PWM_OUT4_INITDUTY 0


#define PWM_OUT_PERIOD_TICKS 11200

#define PWM_OUT_NORMALIZED_TO_TICKS(pwm) ((pwm)*(float)PWM_OUT_PERIOD_TICKS)

#define PWM_OUT1_DUTY		PWM_OUT_TIMER->CCR1
#define PWM_OUT2_DUTY		PWM_OUT_TIMER->CCR2
#define PWM_OUT3_DUTY		PWM_OUT_TIMER->CCR3
#define PWM_OUT4_DUTY		PWM_OUT_TIMER->CCR4

void initPwm_out(void);

#endif //PWM_OUT_H
