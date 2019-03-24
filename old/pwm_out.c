#include "master_include.h"

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;

void initPwm_out(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(PWM_OUT_GPIO_CLK,ENABLE);

	/* GPIOC Configuration */
	GPIO_InitStructure.GPIO_Pin=PWM_OUT_PIN1 | PWM_OUT_PIN2 | PWM_OUT_PIN3 | PWM_OUT_PIN4;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(PWM_OUT_GPIO,&GPIO_InitStructure);

	GPIO_PinAFConfig(PWM_OUT_GPIO,PWM_OUT_PINSOURCE1,PWM_OUT_AF);
	GPIO_PinAFConfig(PWM_OUT_GPIO,PWM_OUT_PINSOURCE2,PWM_OUT_AF);
	GPIO_PinAFConfig(PWM_OUT_GPIO,PWM_OUT_PINSOURCE3,PWM_OUT_AF);
	GPIO_PinAFConfig(PWM_OUT_GPIO,PWM_OUT_PINSOURCE4,PWM_OUT_AF);

	RCC_APB2PeriphClockCmd(PWM_OUT_TIM_CLK,ENABLE);

	TIM_TimeBaseStructure.TIM_Period=PWM_OUT_PERIOD_TICKS;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision=0;

	TIM_TimeBaseStructure.TIM_Prescaler=PWM_OUT_PRESCALER;
	TIM_TimeBaseInit(PWM_OUT_TIMER,&TIM_TimeBaseStructure);

	//PWM out 1
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;

	TIM_OCInitStructure.TIM_Pulse =PWM_OUT1_INITDUTY;
	TIM_OC1Init(PWM_OUT_TIMER, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(PWM_OUT_TIMER, TIM_OCPreload_Enable);

	//PWM out 2
	TIM_OCInitStructure.TIM_Pulse=PWM_OUT2_INITDUTY;
	TIM_OC2Init(PWM_OUT_TIMER,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(PWM_OUT_TIMER,TIM_OCPreload_Enable);

	//PWM out 3
	TIM_OCInitStructure.TIM_Pulse=PWM_OUT3_INITDUTY;
	TIM_OC3Init(PWM_OUT_TIMER,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(PWM_OUT_TIMER,TIM_OCPreload_Enable);

	//PWM out 4
	TIM_OCInitStructure.TIM_Pulse=PWM_OUT4_INITDUTY;
	TIM_OC4Init(PWM_OUT_TIMER,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(PWM_OUT_TIMER,TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(PWM_OUT_TIMER,ENABLE);
	 TIM_CtrlPWMOutputs(TIM8, ENABLE); // because timer 1 & 8 is special and doesn't just work like they ought to (ARRGH)
	TIM_Cmd(PWM_OUT_TIMER,ENABLE);
}
