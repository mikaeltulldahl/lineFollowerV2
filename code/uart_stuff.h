#ifndef UART_STUFF_H
#define UART_STUFF_H

#include "master_include.h"

void initUart(void);
void sendByte(int b);
void sendFloat(float f);

void uart_send_data(void);
void uart_bufferFloat(volatile uint8_t* bufptr, float f);
void uart_bufferInt32(volatile uint8_t* bufptr, uint32_t integer);
void DMA2_Stream7_IRQHandler(void);

#endif

//#ifndef UART_STUFF_H
//#define UART_STUFF_H
//
//#include "master_include.h"
//
//void initUart(void);
//void sendByte(int b);
//void sendIntsToProcessing(int a, int b, int c);
//int reverseInt(int num);
//void sendInt(int r);
//void sendFloat(float f);
//
//void uart_send_data(void);
//void uart_bufferFloat(volatile uint8_t* bufptr, float f);
//void DMA2_Stream7_IRQHandler(void);
//
//#endif
