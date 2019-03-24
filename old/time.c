#include "master_include.h"

//TODO led indicate when in a delay
void delay_ms(uint32_t ms){
	volatile uint32_t stop_time= (TIM2->CNT)+ms*100;
	while(stop_time > TIM2->CNT);
}

void delay_us(uint32_t us){
	volatile uint32_t stop_time= (TIM2->CNT)+(us/10);
	while(stop_time > TIM2->CNT);
}

//uint32_t get_time_tics(){
//	return TIM2->CNT;
//}

//void sleep_until_ticks(uint32_t stop_tick){
//	while(stop_tick > TIM2->CNT);
//}

void initTime(){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	//84mhz, 10us per tick
	TIM_TimeBaseStructure.TIM_Prescaler=840;
	TIM_TimeBaseStructure.TIM_Period=0xFFFFFFFF;// 4294967295;//2^32-1 gets overflow after about 11.9 hours
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

	TIM_Cmd(TIM2,ENABLE);
}
