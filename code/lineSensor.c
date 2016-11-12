/*
 * vilseSensor.c
 *
 *  Created on: 9 mar 2014
 *      Author: vilse
 */

#include "master_include.h"

sensorState lineSensorState =  init;
float lineSensorValue = 0;

// the coordinate for each sensor in x-axis
int32_t sensorCoordinate[SENSOR_COUNT]= {-35, -25, -15, -5, 5, 15, 25, 35};
const int32_t orderInADC[SENSOR_COUNT]= {2, 3, 4, 5, 6, 7, 10, 11};


int32_t sensorMins[SENSOR_COUNT]= {4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000};
int32_t sensorMaxs[SENSOR_COUNT]= {0, 0, 0, 0, 0, 0, 0, 0,};

void initLineSensor(void){
	CLEAR_LED();
	delay_ms(100);
	SET_LED();
	delay_ms(100);
	CLEAR_LED();
	delay_ms(100);
	SET_LED();
	volatile int32_t finishedTick = get_time_tics() + ms_to_tics(3000);
	while(get_time_tics()<finishedTick){
		lineSensorCalibrate();
	}
	CLEAR_LED();
	delay_ms(100);
}



void lineSensorCalibrate(void){
	int i;

	for(i=0; i<SENSOR_COUNT; i++){
		if(lineSensorState== onLine){
			sensorMaxs[i]-=1;
			sensorMins[i]+=1;
		}
		float reading = getSensorReadingRaw(i);
		if (reading >sensorMaxs[i]) sensorMaxs[i] = reading;
		else if(reading < sensorMins[i]) sensorMins[i] = reading;
	}
}

//uint16_t sortArrayIndexes(int n, float indexes[], float x[]){
//	uint16_t temp;
//    int i, j;
//    // the following two loops sort the array x in ascending order
//    for(i=0; i<n-1; i++) {
//        for(j=i+1; j<n; j++) {
//            if(x[j] < x[i]) {
//                // swap elements
//                temp = x[i];
//                x[i] = x[j];
//                x[j] = temp;
//            }
//        }
//    }


// https://en.wikiversity.org/wiki/C_Source_Code/Find_the_median_and_mean
uint16_t median(int n, uint16_t x[]) {
	uint16_t temp;
    int i, j;
    // the following two loops sort the array x in ascending order
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }

    //return the average of the elements in the middle
#define AVERAGE_SAMPLES 20
    int start = n/2 -  AVERAGE_SAMPLES/2;
    int end = n/2 +  AVERAGE_SAMPLES/2;
    uint32_t average = 0;

    for(i = start; i < end; i++) average += x[i];

    average /= AVERAGE_SAMPLES;
    return average;
}

void updateSensorReading(void){
	int32_t y = 0;
	float sensorSum = 0;
	int i;
	float sensorVal=0;
//	float indexesSorted[SENSOR_COUNT];

	float sensorMax = 0;
	int maxIndex = 0;
	float sensorSecondMax = 0;
	int secondMaxIndex = 0;

	for(i=0; i < SENSOR_COUNT; ++i){
		sensorVal = getSensorReadingNormalized(i);//ADC_ValueAveraged[i] - sensorMins[i];
//		sensorVal-= 0.1;
		if (sensorVal < 0) sensorVal =0;

		y += sensorCoordinate[i]*sensorVal;
		sensorSum += sensorVal;

		if(sensorVal>sensorMax){
			sensorMax = sensorVal;
			maxIndex = i;
		}else if (sensorVal>sensorSecondMax){
			sensorSecondMax = sensorVal;
			secondMaxIndex = i;
		}
	}

	if(sensorMax < BLACK_TRESHOLD){ // if no sensor sees black, it must be seeing only white= lost line
		if(lineSensorValue < 0){
			lineSensorState=lostLineLeft;
		}else{
			lineSensorState=lostLineRight;
		}
	}else if(sensorSum > SENSOR_COUNT * WHITE_TRESHOLD){//if the average is more than white
		lineSensorState=inAir;
	}else{
		lineSensorState = onLine;
		lineSensorValue = y/sensorSum;
//		lineSensorValue = (sensorCoordinate[maxIndex]*sensorMax + sensorCoordinate[secondMaxIndex]*sensorSecondMax)/(sensorMax + sensorSecondMax);
	}
}
