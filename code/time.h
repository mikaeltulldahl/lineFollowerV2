#ifndef TIME_H
#define TIME_H

#include "master_include.h"

#define CONTROL_LOOP_FREQ 500

#define CONTROL_LOOP_PERIOD (1/(float)CONTROL_LOOP_FREQ)
#define get_time_tics() TIM2->CNT
#define sleep_until_ticks(finish_tick) while(finish_tick > TIM2->CNT){}
#define ms_to_tics(ms) (ms*(float)100)

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void initTime(void);




#endif //TIME_H
