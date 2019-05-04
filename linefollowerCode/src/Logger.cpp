#include "Logger.h"
#include <SD.h>
#include <SPI.h>
#include "Arduino.h"
#include "Linesensor.h"
#include "Positioning.h"

File logFile;
bool sdInitialized = false;
const int chipSelect = BUILTIN_SDCARD;

Logger::Logger(Positioning* posObj, Linesensor* lineObj) {
  positioning = posObj;
  linesensor = lineObj;
}

void Logger::init() {
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  } else {
    Serial.println("initialization done.");
    sdInitialized = true;
  }

  logFile = SD.open("lineLog.txt", FILE_WRITE);

  if (logFile) {
    Serial.print("Writing to lineLog.txt...");
    logFile.println("------------- new log ---------------");
    logFile.close();
    Serial.println("done.");
  } else {
    Serial.println("error opening lineLog.txt");
  }
}

void Logger::update(int controllerState) {
  switch (controllerState) {
    case 1:  // init
    case 2:  // reset line calibration, wait to stand still
    case 3:  // turn 360 deg
    case 4:  // center on line
      if (sdInitialized && logFile) {
        logFile.close();
        Serial.println("logfile closed");
      }
      break;
    case 5:  // running
      if ((*linesensor).lineSensorState == Linesensor::onLine) {
        if (sdInitialized && !logFile) {
          logFile = SD.open("lineLog.txt", FILE_WRITE);
          Serial.println("logfile opened");
        }
        if (sdInitialized && logFile) {
          String logString =
              (String)millis() + (String)(*positioning).velocity +
              (String)(*positioning).heading + (String)(*positioning).posX +
              (String)(*positioning).posY + (String)(*positioning).distRight +
              (String)(*positioning).distLeft +
              (String)(*linesensor).lineSensorValue;
          logFile.println(logString);
        }
      }
      break;
  }
}