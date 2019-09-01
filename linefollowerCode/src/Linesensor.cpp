#include "Linesensor.h"
#include <ADC.h>
#include <Arduino.h>

#define VREF 3.3f
#define SENSOR_COUNT 24

// right to left
const uint8_t adc_pins[] = {A26, A25, A10, A11, A9,  A8,  A7,  A6,
                            A3,  A2,  A1,  A0,  A22, A21, A20, A19,
                            A24, A23, A18, A17, A16, A15, A14, A12};

int32_t sensorMins[SENSOR_COUNT];
int32_t sensorMaxs[SENSOR_COUNT];
int32_t sensorRange[SENSOR_COUNT];
float inverseRange[SENSOR_COUNT];

int adc_value[SENSOR_COUNT];

#define getSensorPos(idx) (-0.10925f + (float)(idx)*0.0095f)
#define getVolt(rawValue) ((rawValue)*VREF / (float)adc->getMaxValue(ADC_0))
#define getRawValue(volt) \
  ((int)((volt) * (float)adc->getMaxValue(ADC_0) / VREF))
#define getSensorReadingNormalized(i) \
  ((adc_value[i] - sensorMins[i]) * inverseRange[i])

ADC* adc = new ADC();  // adc object

Linesensor::Linesensor(int i) {
  lineSensorState = SensorState::uninitiated;
}

void Linesensor::init(void) {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(adc_pins[i], INPUT);
  }

  int adcs[2] = {ADC_0, ADC_1};
  for (int i = 0; i < 2; i++) {
    adc->setReference(ADC_REFERENCE::REF_3V3, adcs[i]);
    adc->setAveraging(0, adcs[i]);  // no averaging
    adc->setResolution(16, adcs[i]);
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS, adcs[i]);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED, adcs[i]);
    // If you enable interrupts, notice that the isr will read the result, so
    // that isComplete() will return false (most of the time)
    // adc->enableInterrupts(adcs[i] );
  }

  measureAll();

  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMins[i] = adc_value[i];
    sensorMaxs[i] = adc_value[i];
    sensorRange[i] = 1;
    inverseRange[i] = 1.0f;
  }
}

void Linesensor::measureAll() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    adc_value[i] = adc->analogRead(adc_pins[i]);
  }
}

int calibCounter = 0;
void Linesensor::updateCalibration() {
  bool performCalibration = (lineSensorState == onLine) && (calibCounter == 0);
  calibCounter++;
  calibCounter %= 10;
  if (performCalibration) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      if (adc_value[i] > sensorMaxs[i]) {
        sensorMaxs[i] = adc_value[i];
      } else if (adc_value[i] < sensorMins[i]) {
        sensorMins[i] = adc_value[i];
      }  // else if (sensorRange[i] > 2) {
         // sensorMaxs[i] -= 1;
         // sensorMins[i] += 1;
      //}

      int diff = sensorMaxs[i] - sensorMins[i];
      if (diff <= 0) {
        sensorRange[i] = 1;
      } else {
        sensorRange[i] = diff;
      }
      inverseRange[i] = 1.0f / (float)sensorRange[i];
    }
  }
}

/*
 * Typical voltage:
 * inAir: 2.7 - 3.1
 * White: 0.2 - 0.6
 * Black: 1.5 - 2.7
 * uninitiated, onLine, lostLineRight, lostLineLeft, inAir
 */

#define WHITE_MAX_VAL getRawValue(0.6f)
#define BLACK_MIN_VAL getRawValue(1.5f)

void Linesensor::updateLine() {
  int whiteCnt = 0;
  int blackCnt = 0;
  int darkestIdx = 0;
  int darkestVal = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (adc_value[i] < WHITE_MAX_VAL) {
      whiteCnt++;
    } else if (adc_value[i] > BLACK_MIN_VAL) {
      blackCnt++;
      if (adc_value[i] > darkestVal) {
        darkestIdx = i;
        darkestVal = adc_value[i];
      }
    }
  }

  if (blackCnt == 0) {  // only seeing white
    switch (lineSensorState) {
      case uninitiated:
        break;
      case onLine:
        if (lineSensorValue > 0) {
          lineSensorState = lostLineLeft;
          lineSensorValue = getSensorPos(SENSOR_COUNT);
        } else {
          lineSensorState = lostLineRight;
          lineSensorValue = getSensorPos(0);
        }
        break;
      case lostLineRight:
        break;
      case lostLineLeft:
        break;
      case inAir:
        lineSensorState = uninitiated;
        break;
    }
  } else if (blackCnt > (int)(0.7 * SENSOR_COUNT)) {  // seeing a lot of black
    lineSensorState = inAir;
  } else {
    lineSensorState = onLine;
    float y = 0;
    float sensorSum = 0;
    float normalized;
    for (int i = max(0, darkestIdx - 2); i < min(SENSOR_COUNT, darkestIdx + 2);
         i++) {
      normalized = getSensorReadingNormalized(i);
      y += getSensorPos(i) * normalized;
      sensorSum += normalized;
    }
    lineSensorValue = y / sensorSum;
  }
}

void Linesensor::update(volatile float posX,
                        volatile float posY,
                        volatile float heading) {
  measureAll();
  updateLine();
  updateCalibration();

  if (lineSensorState != inAir) {
    float length = 0.085f;  // mm
    float cosHeading = cosf(M_PI / 180.0f * heading);
    float sinHeading = sinf(M_PI / 180.0f * heading);
    lineSensorPosX = posX + length * cosHeading - lineSensorValue * sinHeading;
    lineSensorPosY = posY + length * sinHeading + lineSensorValue * cosHeading;
  }

    // printAsciiLineValue();
    // printAsciiMeasurments();
    // printVolts();
  
}

String Linesensor::stateToString(SensorState i) {
  switch (i) {
    case uninitiated:
      return "Init";
      break;
    case onLine:
      return "On Line";
      break;
    case lostLineRight:
      return "Lost Line Right";
      break;
    case lostLineLeft:
      return "Lost Line Left";
      break;
    case inAir:
      return "In Air";
      break;
    default:
      return "";
  }
}

void Linesensor::printAsciiLineValue() {
  Serial.print("<");
  for (int i = -100; i < 100; i++) {
    if ((int)round(lineSensorValue) == i) {
      Serial.print("#");
    } else {
      Serial.print("_");
    }
  }
  Serial.println(">");
}

void Linesensor::printAsciiMeasurments(){
  Serial.print("<");
  for(int i = 0; i < SENSOR_COUNT; i++) {
    if (getSensorReadingNormalized(i) > 0.5f) {
      Serial.print("#");
    } else {
      Serial.print("_");
    }
  }
  Serial.println(">");
}

void Linesensor::printVolts(){
  for (int i=0;i<SENSOR_COUNT;i++) {
        Serial.print(i);
        Serial.print(": ");
        Serial.print(getVolt(adc_value[i]),1);
        Serial.print(" ");
    }
    Serial.println();
}