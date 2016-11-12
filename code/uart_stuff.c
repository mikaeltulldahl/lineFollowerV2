
#include "master_include.h"

DMA_InitTypeDef DMA_InitStructure; //this declaration is here because the initstructure is used every time a new DMA transfer is done to set everything up

#define UART_TX_MAX_DATA_ITEMS 11
#define UART_TX_MAX_BUFFERSIZE (6+4+4*UART_TX_MAX_DATA_ITEMS) // 4 bytes of sync data and then each data item is 4 uart transfers long
volatile uint8_t uart_tx_buf[UART_TX_MAX_BUFFERSIZE];

volatile bool uart_tx_complete = true;
volatile bool uart_send_data_1 = true;
volatile bool uart_send_data_2 = false;
volatile bool uart_send_data_3 = false;
volatile bool uart_send_data_4 = false;
volatile bool uart_send_time = false;

volatile float dummy = 0;

//don't use this, it is slow
void sendFloat(float f){
	int i;
	char *str = (char *) &f;

	for(i=3; i>=0;i--){
		sendByte(str[i]); // reverse order
	}
}

/*
 * fills the specified buffer with 4 bytes containing the float
 */
void uart_bufferFloat(volatile uint8_t* bufptr, float f){
	int i;
	char *str = (char *) &f;

	int j = 0;
	for(i=3; i>=0;i--){
		bufptr[j] = str[i];
		j++;
		// reverse order
	}
}

/*
 * fills the specified buffer with 4 bytes containing the integer
 */
void uart_bufferInt32(volatile uint8_t* bufptr, uint32_t integer){
	int i;
	char *str = (char *) &integer;
	for(i=0; i<4;i++){
		bufptr[i] = str[i];
	}
}

void initUart(void) {
	/*
	 * PA9 tx
	 * PA10 rx
	 *
	 * USART1
	 */

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Configure USART Rx & Tx as alternate function */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Mux out USART1 Rx & Tx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	/* Enable USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_InitTypeDef USART_InitStructure;

	USART_OverSampling8Cmd(USART1, DISABLE);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);



	//interrupt setup
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;// this sets the priority group of the USART1 interrupts, this is the next highest priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt



	//DMA TX setup
	/*
	 * DMA2, channel 4, stream7: USART1_TX
	 *
	 */


	/* Enable DMA clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);


	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){
	}

	if(DMA_GetCmdStatus(DMA2_Stream7) == ENABLE){
		while(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) == RESET);
	}

	/* Configure DMA Stream */
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_BASE+0x04;//(uint32_t) USART1->DR; //??
	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)uart_tx_buf; // tog bort & framför
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = (uint16_t)UART_TX_MAX_BUFFERSIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //??
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	// Init of dma everytime i use it

	USART_DMACmd(USART1,USART_DMAReq_Tx, ENABLE);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);

}

void DMA2_Stream7_IRQHandler(void){
	//TODO kolla om TC enabled, dock onödigt nu pga inga andra interrupt aktiverade
	//clear pending bit
	DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);

	//disable dma
	DMA_Cmd(DMA2_Stream7, DISABLE); //ev onödigt

	uart_tx_complete = true;
}


// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){

		char temp = USART1->DR;
		//check which command the


		float recieved_parameter; // dummy variable, not used
		(void) recieved_parameter;

		switch(temp){
		case 'P' :
			while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); /* Wait for next rx */
			recieved_parameter = 0.01f*0.00392156862f* USART1->DR; // the following value is the parameter
			//		    	Kp_p = 0.00392156862f* USART1->DR; // the following value is the parameter
			//if 255 is sent, kp will end up as around 1
			break;
		case 'I' :
			while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); /* Wait for next rx */
			recieved_parameter = 0.01f*0.00392156862f* USART1->DR; // the following value is the parameter
			//		    	Ki_p = 0.00392156862f* USART1->DR; // the following value is the parameter
			break;
		case 'D' :
			while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); /* Wait for next rx */
			recieved_parameter = 0.01f*0.00392156862f* USART1->DR; // the following value is the parameter
			//		    	Kd_p = 0.00392156862f* USART1->DR; // the following value is the parameter
			break;
		case 'C' :
			while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); /* Wait for next rx */
			int temp1 = USART1->DR;
			while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); /* Wait for next rx */
			int temp2 = USART1->DR;
			bool temp3=false;
			if(temp2 == 'E') {
				temp3 = true;
			}
			switch (temp1) {
				case 0:
					uart_send_data_1=temp3;
					break;
				case 1:
					uart_send_data_2=temp3;
					break;
				case 2:
					uart_send_data_3=temp3;
					break;
				case 3:
					uart_send_data_4=temp3;
					break;
				case 4:
					uart_send_time=temp3;
					break;
				default:
					break;
			}
			break;
		}
	}
}

/*
 * call this function to begin uart transfer of data
 * transfer is taken care of by DMA and nothing will happen if the DMA is busy
 */
void uart_send_data(void){
	if(!uart_tx_complete){
		if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7) == RESET){
			return; // if last data transfer isn't complete yet, we can't wait here to let it finish, just throw away the data
		}
	}
	//check if there is something to send
	if(!uart_send_data_1 && !uart_send_data_2 && !uart_send_data_3 && !uart_send_data_4 && !uart_send_time){
		return;

	}

	// fill buffer with data
	// start with the 4 sync-bytes ,TODO this doesn't need to be done every time
	uart_tx_buf[0] = 0xFF;
	uart_tx_buf[1] = 0xFF;
	uart_tx_buf[2] = 0x00;
	uart_tx_buf[3] = 0xFF;

	int j = 4;

	if(uart_send_data_1){
		uart_tx_buf[j] = '1';
		j++;
		uart_bufferFloat(uart_tx_buf + j, 300*getSensorReadingNormalized(0));
		j=j+4;
		uart_bufferFloat(uart_tx_buf + j, 300*getSensorReadingNormalized(1));
		j=j+4;
		uart_bufferFloat(uart_tx_buf + j, 300*getSensorReadingNormalized(2));
		j=j+4;
	}
	if(uart_send_data_2){
		uart_tx_buf[j] = '2';
		j++;
		uart_bufferFloat(uart_tx_buf + j, 300*getSensorReadingNormalized(3));
		j=j+4;
		uart_bufferFloat(uart_tx_buf + j, 300*getSensorReadingNormalized(4));
		j=j+4;
		uart_bufferFloat(uart_tx_buf + j, 300*getSensorReadingNormalized(5));
		j=j+4;
	}
	if(uart_send_data_3){
		uart_tx_buf[j] = '3';
		j++;//2700
		uart_bufferFloat(uart_tx_buf + j, 300*getSensorReadingNormalized(6));
		j=j+4;
		uart_bufferFloat(uart_tx_buf + j, 300*getSensorReadingNormalized(7));
		j=j+4;
		uart_bufferFloat(uart_tx_buf + j, dummy);
		j=j+4;
	}
	if(uart_send_data_4){
		uart_tx_buf[j] = '4';
		j++;
		uart_bufferFloat(uart_tx_buf + j, (300/(float)35)*lineSensorValue);
		j=j+4;
	}
	if(uart_send_time){
		uart_tx_buf[j] = 't';
		j++;
		uint32_t curr_time = get_time_tics();
		uart_bufferInt32(uart_tx_buf + j, curr_time);
		j=j+4;
	}

	uart_tx_buf[j] = 4; // indicating the end with "end of transmission in ascii
	j++;

	//TODO clean up rest of the buffer

	//set up DMA to transmit buffer
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC, DISABLE);
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);

	uart_tx_complete = false;

	DMA_Cmd(DMA2_Stream7, DISABLE);
	DMA_SetCurrDataCounter(DMA2_Stream7, j);

	/* Reset DMA Stream registers (for debug purpose) */
	DMA_DeInit(DMA2_Stream7);
	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)uart_tx_buf; // tog bort & framför
	DMA_InitStructure.DMA_BufferSize = (uint16_t)j;
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);

//	/* Enable DMA Stream Transfer Complete interrupt */
//	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);



	/* DMA Stream enable */
	DMA_Cmd(DMA2_Stream7, ENABLE);

	/* Check if the DMA Stream has been effectively enabled.
			     The DMA Stream Enable bit is cleared immediately by hardware if there is an
			     error in the configuration parameters and the transfer is no started (ie. when
			     wrong FIFO threshold is configured ...) */
	while ((DMA_GetCmdStatus(DMA2_Stream7) != ENABLE)){
	}
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC, ENABLE); // enable the transfer complete interrupt

}

void sendByte(int b) {
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); /* Waitwhile TX full */
	USART_SendData(USART1, b);
}
