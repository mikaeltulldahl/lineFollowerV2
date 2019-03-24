
#include <Arduino.h>
#include "motors.h"

volatile float motor_right;
volatile float motor_left;

#define MOTOR_1_PWM_PIN 4
#define MOTOR_1_DIR_PIN 5
#define MOTOR_2_PWM_PIN 7
#define MOTOR_2_DIR_PIN 6

#define ENABLE_MOTORS() GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define DISABLE_MOTORS() GPIO_SetBits(GPIOB, GPIO_Pin_5)

void initMotors(){
	pinMode(MOTOR_1_DIR_PIN, OUTPUT);
	pinMode(MOTOR_1_PWM_PIN, OUTPUT);	
	pinMode(MOTOR_2_DIR_PIN, OUTPUT);
	pinMode(MOTOR_2_PWM_PIN, OUTPUT);	
	analogWriteResolution(11);
	analogWriteFrequency(MOTOR_1_PWM_PIN, 20000);
	analogWriteFrequency(MOTOR_2_PWM_PIN, 20000);
	
	stopMotors();
//	ENABLE_MOTORS();
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
	float scaling = 2047;
	if(pwm >= 0){
		digitalWrite(MOTOR_1_DIR_PIN, HIGH);
		//analogWrite(MOTOR_1_PWM_PIN, (int) pwm*scaling);
		analogWrite(MOTOR_1_PWM_PIN, (int) ((float)pwm*scaling));
	}else{
		digitalWrite(MOTOR_1_DIR_PIN, LOW);
		analogWrite(MOTOR_1_PWM_PIN, (int) -pwm*scaling);
	}
}

void setLeftMotor(float pwm){
	pwm = saturate(pwm);
	float scaling = 2047;
	if(pwm >= 0){
		digitalWrite(MOTOR_2_DIR_PIN, HIGH);
		analogWrite(MOTOR_2_PWM_PIN, (int) pwm*scaling);
	}else{
		digitalWrite(MOTOR_2_DIR_PIN, LOW);
		analogWrite(MOTOR_2_PWM_PIN, (int) -pwm*scaling);
	}
}

void stopMotors(void){
	setLeftMotor(0);
	setRightMotor(0);
}