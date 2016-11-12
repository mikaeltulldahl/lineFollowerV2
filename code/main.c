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
	initUart(); //USART1
	initLineSensor();


	uint32_t start_tick = get_time_tics();
	uint32_t control_period_ticks = 100000*CONTROL_LOOP_PERIOD; //500Hz control loop

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

		float leftPwm = -lineSensorValue/100.0f;
		float rightPwm = lineSensorValue/100.0f;

		setLeftMotor(leftPwm);
		setRightMotor(rightPwm);
		uart_send_data();

	}
}
