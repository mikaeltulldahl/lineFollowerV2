/*
 * adc.c
 *
 *  Created on: 28 feb 2014
 *  edited 2015-08-20
 *      Author: vilse
 */

#include "master_include.h"

// Settings

// Global variables
volatile uint16_t ADC_Value[ADC_CHANNEL_NUM] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
volatile float ADC_ValueAveraged[ADC_CHANNEL_NUM] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

//Private variables

// Private functions

void init_adc(void) {

	/*
	 * using: PC0,PC1,PC2,PC3, PA0,PA1,PA2,PA3,PA4
	 * ADC1, ADC2
	 * ch10- ch13, ch0-ch4
	 *
	 * TIM4, cc4
	 * old: TIM3, CC1
	 */

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

#define ADC_CDR_ADDRESS			((uint32_t)0x40012308)

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	/* DMA2 Stream0 channel0 configuration */
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_CDR_ADDRESS;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC_CHANNEL_NUM; // 12 fungerar
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	// DMA2_Stream0 enable
	DMA_Cmd(DMA2_Stream0, ENABLE);

	// Enable transfer complete interrupt
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);
	ADC_DeInit();

	// ADC Common Init
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Channel-specific settings
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 8;

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_Init(ADC2, &ADC_InitStructure);

#define SAMPLETIME ADC_SampleTime_3Cycles

	// ADC1 regular channels 0, 5, 10, 13
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, SAMPLETIME);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, SAMPLETIME);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, SAMPLETIME);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 4, SAMPLETIME);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, SAMPLETIME);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 6, SAMPLETIME);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 7, SAMPLETIME);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 8, SAMPLETIME);

	// ADC2 regular channels 1, 6, 11, 15
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, SAMPLETIME);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 2, SAMPLETIME);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 3, SAMPLETIME);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 4, SAMPLETIME);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 5, SAMPLETIME);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_7, 6, SAMPLETIME);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 7, SAMPLETIME);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 8, SAMPLETIME);

	// Enable DMA request after last transfer (Multi-ADC mode)
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);
	//	ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Enable ADC2
	ADC_Cmd(ADC2, ENABLE);

	// ------------- Timer4 for ADC sampling ------------- //
	// Time Base configuration
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 84000000 / ADC_SAMPLING_FREQUENCY;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 100;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); //why is cc2 initiated?? it doesn't work without
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);

	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(TIM4, ENABLE);

	TIM_Cmd(TIM4, ENABLE);


	/* Enable ADC1 DMA since ADC1 is the Master*/
	ADC_DMACmd(ADC1, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the DMA2 Stream0 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// Called every time all ADC channels are converted
void DMA2_Stream0_IRQHandler() {

	/* Test on DMA Stream Transfer Complete interrupt */
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0)){
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
	}
		int i;
		for(i=0; i < ADC_CHANNEL_NUM; i++ ){
			ADC_ValueAveraged[i]= ADC_AVERAGE_FILTERCONSTANT*ADC_ValueAveraged[i] + (1.0f - ADC_AVERAGE_FILTERCONSTANT)*ADC_Value[i];
		}
}
