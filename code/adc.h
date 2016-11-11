/*
 * vilseAdc.h
 *
 *  Created on: 28 feb 2014
 *      Author: vilse
 */

#ifndef ADC_H_
#define ADC_H_

#include "master_include.h"


//
///* Total number of channels to be sampled by a single ADC operation.*/
//#define ADC_GRP1_NUM_CHANNELS   2
//
///* Depth of the conversion buffer, channels are sampled four times each.*/
//#define ADC_GRP1_BUF_DEPTH      4
//
//extern void vilseAdcInit(void);
//extern void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
//
//
//
///*
// * ADC samples buffer.
// */
//
//static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
//static adcsample_t avg_ch1, avg_ch2;
//
///*
// * ADC conversion group.
// * Mode:        Linear buffer, 4 samples of 2 channels, SW triggered.
// * Channels:    IN11   (48 cycles sample time)
// *              Sensor (192 cycles sample time)
// */
//static const ADCConversionGroup adcgrpcfg = {
//  FALSE,
//  ADC_GRP1_NUM_CHANNELS,
//  adccb,
//  NULL,
//  /* HW dependent part.*/
//  0,
//  ADC_CR2_SWSTART,
//  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_56) | ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_144),
//  0,
//  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
//  0,
//  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) | ADC_SQR3_SQ1_N(ADC_CHANNEL_SENSOR)
//};


//#include <stdint.h>

// Settings
#define ADC_CHANNEL_NUM				16
#define ADC_SAMPLING_FREQUENCY		10000
//#define ADC_AVERAGE_COUNT 50

// Global variables
extern volatile uint16_t ADC_Value[ADC_CHANNEL_NUM];
//extern volatile uint32_t ADC_ValuesSummed[ADC_CHANNEL_NUM];
extern volatile float ADC_ValueAveraged[ADC_CHANNEL_NUM];

// Functions
void init_adc(void);
void DMA2_Stream0_IRQHandler(void);







#endif /* ADC_H_ */
