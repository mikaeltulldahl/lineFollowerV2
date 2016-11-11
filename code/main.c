#include "master_include.h"

void initLEDS(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	CLEAR_LED();
}

#define NEWLINE() sendByte(13);sendByte(10);

//void printInteger(int32_t in){
//	uint8_t output[8];
//	int i;
//	if (in < 0){
//		sendByte('-');
//		in = -in;
//	}
//
//	for(i = 0; i < 8; i++){
//
//	}
//
//	sendByte(88);
//
//}

int main(void) {
	int i;
//	SystemCoreClockUpdate(); //makes sure SystemCoreClock is set to the correct value

	initLEDS(); //GPIOA
	initTime(); // TIM2

	initPwm_out(); //TIM8
	init_adc(); //ADC1, ADC2, TIM4

	initMotors();
//	initIR_receiver(); //TIM 3
	initMPU9250(); //SPI2
	initUart(); //USART1
	initLineSensor();


	uint32_t start_tick = get_time_tics();
	uint32_t control_period_ticks = 100000*CONTROL_LOOP_PERIOD; //500Hz control loop
//

	while(1);
//	SET_LED();
//	delay_ms(200);
//	CLEAR_LED();
//	delay_ms(200);
	while (1) {
		SET_LED();
		sleep_until_ticks(start_tick);
		CLEAR_LED();
		start_tick=get_time_tics()+ control_period_ticks;

		mpu9250_read_all();

		float derp = gyroscope_data[2];
		float temp = ADC_ValueAveraged[0];


		motor_right= 0.5 + gyroscope_data[2]*(float)0.01;
		motor_left= 0.5 + gyroscope_data[2]*(float)0.01;


		updateSensorReading();
		if(i > 30){
			lineSensorCalibrate();
			i = 0;
		}else{
			i++;
		}


//		for(i = 0; i < ADC_ValueAveraged[1]/100; i ++){
//			sendByte(88);
//		}
//		NEWLINE();
//		sendByte(lineSensorValue*10);



//		if (derp > 0){
//			SET_LED();
//		}else{
//			CLEAR_LED();
//		}

//		pwm_out_sendSignalsToMotors();
		uart_send_data();

	}
}


