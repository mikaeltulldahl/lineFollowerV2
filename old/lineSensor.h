/*
 * vilseSensor.h
 *
 *  Created on: 9 mar 2014
 *      Author: vilse
 */

#ifndef VILSESENSOR_H_
#define VILSESENSOR_H_

#include "master_include.h"

#define SENSOR_COUNT 8
#define BLACK_TRESHOLD 0.5 //1000 // is all readings lower than this, something happens
#define WHITE_TRESHOLD 0.5 //1000 // is the average of readings higher than this, something happens


typedef enum {init, onLine, lostLineRight, lostLineLeft, inAir}sensorState;

extern sensorState lineSensorState;
extern float lineSensorValue;


extern int32_t sensorCoordinate[SENSOR_COUNT];
extern int32_t sensorMins[SENSOR_COUNT];
extern int32_t sensorMaxs[SENSOR_COUNT];
extern const int32_t orderInADC[SENSOR_COUNT];

#define getSensorReadingRaw(i) (ADC_ValueAveraged[orderInADC[i]])
#define getSensorReadingNormalized(i) ((getSensorReadingRaw(i) - sensorMins[i])/(float)(sensorMaxs[i] - sensorMins[i]))

void initLineSensor(void);

void lineSensorCalibrate(void);
void updateSensorReading(void);

#endif /* VILSESENSOR_H_ */
