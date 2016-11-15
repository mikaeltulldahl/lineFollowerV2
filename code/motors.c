
#include "master_include.h"

volatile float motor_right;
volatile float motor_left;

volatile uint32_t motor_right_ticks;
volatile uint32_t motor_left_ticks;

#define PWM_IN1_A2 PWM_OUT1_DUTY//PC6
#define PWM_IN2_A2 PWM_OUT2_DUTY//PC7
#define PWM_IN2_B2 PWM_OUT3_DUTY//PC8
#define PWM_IN1_B2 PWM_OUT4_DUTY//PC9



//uint8_t pin_in1[] = {pin_in1_A1, pin_in1_A2, pin_in1_B1, pin_in1_B2};
//uint8_t pin_in2[] = {pin_in2_A1, pin_in2_A2, pin_in2_B1, pin_in2_B2};
//
//float lastPwm[] = {0,0};
//int32_t motorPwmFreq = 15000;
//volatile float timeStepcurrentmilliAmpsControll = 1/(float)motorPwmFreq;
//
//// U=R*I, R = 270, I =  0.0024*currentmilliAmps
//// U = 270*0.0024*currentmilliAmps
//// U = 0,648*currentmilliAmps
//float lsbVolt = 0.0012;
//volatile int32_t multiplier = round(1000*(1/(float)0.648)*lsbVolt);
//
//volatile int32_t currentmilliAmpsRef = 0;
//
//volatile int32_t currentmilliAmpsIntergral = 0;
//volatile int32_t val = 0;
//
//volatile int32_t pidP = (float)0.2*(float)motorPwmFreq;
//volatile int32_t pidI = 250;
//volatile int32_t currentmilliAmps = 0;

#define ENABLE_MOTORS() GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define DISABLE_MOTORS() GPIO_SetBits(GPIOB, GPIO_Pin_5)

void initMotors(){
	//disable motors PB5

	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

//	stopMotors();
	ENABLE_MOTORS();
}

float saturate(float in){
	if (in > 1){
		 return 1;
	}else if(in < -1){
		return -1;
	}else {
		return in;
	}
}


void setRightMotor(float pwm){
	pwm = saturate(pwm);

	pwm *= 0.5;
	if(pwm >= 0){
		PWM_IN1_A2 = PWM_OUT_NORMALIZED_TO_TICKS(pwm);
		PWM_IN2_A2 = PWM_OUT_NORMALIZED_TO_TICKS(0);
	}else{
		PWM_IN1_A2 = PWM_OUT_NORMALIZED_TO_TICKS(0);
		PWM_IN2_A2 = PWM_OUT_NORMALIZED_TO_TICKS(-pwm);
	}
}

void setLeftMotor(float pwm){
	pwm = saturate(pwm);

	if(pwm >= 0){
		PWM_IN1_B2 = PWM_OUT_NORMALIZED_TO_TICKS(pwm);
		PWM_IN2_B2 = PWM_OUT_NORMALIZED_TO_TICKS(0);
	}else{
		PWM_IN1_B2 = PWM_OUT_NORMALIZED_TO_TICKS(0);
		PWM_IN2_B2 = PWM_OUT_NORMALIZED_TO_TICKS(-pwm);
	}
}

///*
// * motorPwm is in range -1000 to 1000
// */
//void setMotorPwm(int motorIndex, int32_t motorPwm){
//	int32_t maxPwm = 1000;
//	if (motorPwm > maxPwm){
//		motorPwm = maxPwm;
//	}else if(motorPwm < -maxPwm){
//		motorPwm = -maxPwm;
//	}
//
//	int32_t absPwm = motorPwm;
//	if (absPwm < 0) absPwm = - absPwm;
//
////	if (motorIndex == 0){
////		temptemp = (maxAnalogWrite*(maxPwm-absPwm))/maxPwm;
////	}
//
////	if (motorPwm == 0){
////		analogWrite(pin_in1[motorIndex], 0);
////		analogWrite(pin_in2[motorIndex], 0);
////	}else if (motorPwm < 0){
////		analogWrite(pin_in1[motorIndex], maxAnalogWrite);
////		analogWrite(pin_in2[motorIndex], (maxAnalogWrite*(maxPwm-absPwm))/maxPwm);
////	}else{
////		analogWrite(pin_in1[motorIndex], (maxAnalogWrite*(maxPwm-absPwm))/maxPwm);
////		analogWrite(pin_in2[motorIndex], maxAnalogWrite);
////	}
//	//	if (motorPwm*lastPwm[motorIndex] > 0){
//	//		//keep going in same direction, so nothing to do
//	//	}else if (motorPwm==0) {
//	//		digitalWrite(directionPin1[motorIndex],HIGH);
//	//		digitalWrite (directionPin2[motorIndex],HIGH);
//	//	}else if(motorPwm>0){
//	//		digitalWrite(directionPin1[motorIndex],HIGH);
//	//		digitalWrite (directionPin2[motorIndex],LOW);
//	//	}
//	//	else{
//	//		digitalWrite(directionPin1[motorIndex],LOW);
//	//		digitalWrite (directionPin2[motorIndex],HIGH);
//	//	}
//	//
//	//	analogWrite(pwmPin[motorIndex], abs((int)round(maxAnalogWrite*motorPwm)));
//	//	lastPwm[motorIndex]=motorPwm;
//}

//void setMotorPwms(float* motorPwms){
//	int i;
//	for(i = 0; i < 3; ++i){
//		setMotorPwm(i, motorPwms[i]);
//	}
//}

void stopMotors(void){
	setLeftMotor(0);
	setRightMotor(0);
}

//void initMotors(void){
//	pinMode(pin_disable_motors,OUTPUT);
//	digitalWrite(pin_disable_motors, LOW);
//
//	pinMode(pin_in1_A1,OUTPUT);
//	pinMode(pin_in2_A1,OUTPUT);
//	pinMode(pin_statusFlag_A1,INPUT);
//
//	pinMode(pin_in1_A2,OUTPUT);
//	pinMode(pin_in2_A2,OUTPUT);
//	pinMode(pin_statusFlag_A2,INPUT);
//
//	pinMode(pin_in1_B1,OUTPUT);
//	pinMode(pin_in2_B1,OUTPUT);
//	pinMode(pin_statusFlag_B1,INPUT);
//
//	pinMode(pin_in1_B2,OUTPUT);
//	pinMode(pin_in2_B2,OUTPUT);
//	pinMode(pin_statusFlag_B2,INPUT);
//
//	analogWriteFrequency(pin_in1_A1, motorPwmFreq);//affects all pwm pins on the same timer
//
//
//	stopMotors();
//}

//motor value is between 0 and 1 for vortex, -1 to 1 for drive motors
//void pwm_out_sendSignalsToMotors(){
//	if(motor_right > 1) motor_right = 1;
//	if(motor_right < 0) motor_right=0;
//
//	motor_right_ticks = PWM_OUT_NORMALIZED_TO_TICKS(motor_right);
//	PWM_OUT2_DUTY = motor_right_ticks;
//
//	if(motor_left > 1) motor_left = 1;
//	if(motor_left < 0) motor_left=0;
//
//	motor_left_ticks = PWM_OUT_NORMALIZED_TO_TICKS(motor_left);
//	PWM_OUT1_DUTY = motor_left_ticks;
//}

//INTERUPT:
//digitalWrite(tempPin,1);
////	  FTM0_SC &=  ~FTMx_CnSC_CHF; // clear channel 0 interrupt
//  FTM0_SC &=  ~FTM_SC_TOF;
////	  for(volatile int j= 0; j < 3; j++);//mini delay
//
//  	currentmilliAmps = multiplier*analogRead(pin_feedbackA1);
//  	if (val < 0) currentmilliAmps = -currentmilliAmps;
//  	/*
//  	 * 0.068V = 60
//  	 * 0.734V  = 620
//  	 * lsb = 0.0012 V
//  	 * offset = 0V
//  	 *
//  	 */
//  	//	float currentmilliAmps = (4.53/(float)1023) * analogRead(pin_feedbackA1);
//  	//	currentmilliAmps = 0.648*currentmilliAmps;
//  	//	float p = 0.04;
//  	//	float currentmilliAmpsError = 0.1 - currentmilliAmps;
//  	//	float val = p*currentmilliAmpsError;
//
//
//  	//	val = 0.5;
//  	//	val = ref;
//
//  	int32_t currentmilliAmpsError = currentmilliAmpsRef - currentmilliAmps;
////	  	currentmilliAmpsIntergral += timeStepcurrentmilliAmpsControll*currentmilliAmpsError;
////	  	currentmilliAmpsIntergral += currentmilliAmpsError/motorPwmFreq;
//  	currentmilliAmpsIntergral += currentmilliAmpsError*pidI;
//  	int32_t integralMax = 1000*motorPwmFreq;
//  	if(currentmilliAmpsIntergral > integralMax) currentmilliAmpsIntergral = integralMax;
//  	if(currentmilliAmpsIntergral < -integralMax) currentmilliAmpsIntergral = -integralMax;
//
//
//  	val = (pidP*currentmilliAmpsError + currentmilliAmpsIntergral)/motorPwmFreq;
//
//
//  	setMotorPwm(0, val);
////	  	setMotorPwm(1, val);
////	  	setMotorPwm(2, val);
////	  	setMotorPwm(3, val);
//
//  	digitalWrite(tempPin,0);
