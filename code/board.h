#ifndef VILSECOPTERBOARD_H
#define VILSECOPTERBOARD_H

#include "master_include.h"
// LED macros
#define SET_LED() GPIO_ResetBits(GPIOA, GPIO_Pin_12)
#define CLEAR_LED() GPIO_SetBits(GPIOA, GPIO_Pin_12)

//pwm out
#define PWM_OUT_GPIO	GPIOC
#define	PWM_OUT_TIMER	TIM8 //16 bit, 168MHz clk
#define	PWM_OUT_TIMER_FREQ	168000000
#define PWM_OUT_AF 	GPIO_AF_TIM8
#define PWM_OUT_GPIO_CLK RCC_AHB1Periph_GPIOC
#define PWM_OUT_TIM_CLK RCC_APB2Periph_TIM8

#define PWM_OUT_PIN1		GPIO_Pin_6
#define PWM_OUT_PIN2		GPIO_Pin_7
#define PWM_OUT_PIN3		GPIO_Pin_8
#define PWM_OUT_PIN4		GPIO_Pin_9

#define PWM_OUT_PINSOURCE1		GPIO_PinSource6
#define PWM_OUT_PINSOURCE2		GPIO_PinSource7
#define PWM_OUT_PINSOURCE3		GPIO_PinSource8
#define PWM_OUT_PINSOURCE4		GPIO_PinSource9

#endif //VILSECOPTERBOARD_H
