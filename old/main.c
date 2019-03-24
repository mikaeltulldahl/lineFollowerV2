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

int main(void) {
	int i=0;
//	SystemCoreClockUpdate(); //makes sure SystemCoreClock is set to the correct value

	initLEDS(); //GPIOA
	initTime(); // TIM2

	initPwm_out(); //TIM8
	init_adc(); //ADC1, ADC2, TIM4

	initMotors();
	initMPU9250(); //SPI2
//	initUart(); //USART1
	initLineSensor();


	uint32_t start_tick = get_time_tics();
	uint32_t control_period_ticks = 100000*CONTROL_LOOP_PERIOD; //500Hz control loop


//	setLeftMotor(0);
//	setRightMotor(0.3);
//	while(1);
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
		if(i > 30){
			lineSensorCalibrate();
			i = 0;
		}else{
			i++;
		}
		updateSensorReading();

//		float derp = gyroscope_data[2];
//		float temp = ADC_ValueAveraged[0];

		float leftPwm = 0;
		float rightPwm = 0;
		float error = lineSensorValue/35.0f;
		if (error >1) error = 1;
		float forwardPwm = 0.9 - 0.7*fabs(error);

//		leftPwm = forwardPwm -lineSensorValue/30.0f + gyroscope_data[2]/500.0f;
//		rightPwm = forwardPwm + lineSensorValue/30.0f - gyroscope_data[2]/200.0f;

		float referenceAngRate = lineSensorValue*18;
		leftPwm = forwardPwm + (gyroscope_data[2]- referenceAngRate)/800.0f;
		rightPwm = forwardPwm -(gyroscope_data[2]- referenceAngRate)/800.0f;
//		if (lineSensorState==lostLineLeft){
//			leftPwm = 0;
//			rightPwm = -0.5;
//		}
//		if (lineSensorState==lostLineRight){
//			leftPwm = -0.5;
//			rightPwm = 0;
//		}



		setLeftMotor(leftPwm);
		setRightMotor(rightPwm);
//		uart_send_data();

	}
}
