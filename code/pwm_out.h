#ifndef PWM_OUT_H
#define PWM_OUT_H

#include "master_include.h"
#include <stdio.h>
#include <math.h>


#define PWM_OUT_FREQ 15000//Hz
#define PWM_OUT_PERIOD_MS (1000/(float)PWM_OUT_FREQ) //2 ms
#define PWM_OUT_PERIOD_US (1000000/(float)PWM_OUT_FREQ) //2000 us

#define PWM_OUT1_INITDUTY 0
#define PWM_OUT2_INITDUTY 0
#define PWM_OUT3_INITDUTY 0
#define PWM_OUT4_INITDUTY 0


#define PWM_OUT_TIMER_MAX_PERIOD 65535

#if (PWM_OUT_TIMER_FREQ/PWM_OUT_TIMER_MAX_PERIOD) >= PWM_OUT_FREQ
#define PWM_OUT_PRESCALER (ceil(PWM_OUT_TIMER_FREQ/(float)PWM_OUT_TIMER_MAX_PERIOD/(float)PWM_OUT_FREQ) - 1)
#else
#define PWM_OUT_PRESCALER 0
#endif



#define PWM_OUT_PERIOD_TICKS (PWM_OUT_TIMER_FREQ/(PWM_OUT_PRESCALER+1)/PWM_OUT_FREQ)

#define PWM_OUT_NORMALIZED_TO_TICKS(pwm) ((pwm)*(float)PWM_OUT_PERIOD_TICKS)

#define PWM_OUT1_DUTY		PWM_OUT_TIMER->CCR1
#define PWM_OUT2_DUTY		PWM_OUT_TIMER->CCR2
#define PWM_OUT3_DUTY		PWM_OUT_TIMER->CCR3
#define PWM_OUT4_DUTY		PWM_OUT_TIMER->CCR4

void initPwm_out(void);

#endif //PWM_OUT_H
