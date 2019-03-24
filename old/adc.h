/*
 * vilseAdc.h
 *
 *  Created on: 28 feb 2014
 *      Author: vilse
 */

#ifndef ADC_H_
#define ADC_H_

#include "master_include.h"

// Settings
#define ADC_CHANNEL_NUM				16
#define ADC_SAMPLING_FREQUENCY		10000
#define ADC_AVERAGE_FILTERCONSTANT  0.8

// Global variables
extern volatile uint16_t ADC_Value[ADC_CHANNEL_NUM];
extern volatile float ADC_ValueAveraged[ADC_CHANNEL_NUM];

// Functions
void init_adc(void);
void DMA2_Stream0_IRQHandler(void);

#endif /* ADC_H_ */
