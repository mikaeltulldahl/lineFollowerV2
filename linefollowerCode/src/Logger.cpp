#include "Logger.h"

#include <SD.h>
#include <SPI.h>

#include "Arduino.h"
#include "Linesensor.h"
#include "Positioning.h"
#include "StateMachine.h"

Logger::Logger(Positioning* posObj, Linesensor* lineObj) {
  positioning = posObj;
  linesensor = lineObj;
  sdInitialized = false;
}

void Logger::init() {
  Serial.print("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    return;
  } else {
    Serial.println("initialization done.");
    sdInitialized = true;
  }

  // Find unique log name
  int logIdx = 0;
  do {
    sprintf(logFileName, "log%03d.txt", logIdx);
    logIdx++;
  } while (SD.exists(logFileName) && logIdx < 999);
  Serial.print("logfile name: ");
  Serial.println(logFileName);

  logFile = SD.open(logFileName, FILE_WRITE);
  if (logFile) {
    logFile.println("------------- new log ---------------");
    logFile.close();
    Serial.println("done.");
  } else {
    Serial.print("error opening ");
    Serial.println(logFileName);
  }
}

void Logger::update(int controllerState) {
  switch (controllerState) {
    case INIT:
    case LINE_CALIB_RESET:
    case TURN_360:
    case CENTER_ON_LINE:
      if (sdInitialized && logFile) {
        logFile.println("stop");
        logFile.close();
        Serial.println("logfile closed");
      }
      break;
    case RUNNING:
      if (linesensor->lineSensorState == Linesensor::onLine) {
        if (sdInitialized && !logFile) {
          logFile = SD.open(logFileName, FILE_WRITE);
          if (logFile) {
            logFile.println("start");
            Serial.println("logfile opened");
          }
        }
        if (sdInitialized && logFile) {
          logFile.printf(
              "%6lu,%5.2f,%6.1f,%6.3f,%6.3f,%6ld,%6ld,%d,%6.3f,%6.3f,%6.3f",
              millis(), positioning->velocity, positioning->heading,
              positioning->getPosX(), positioning->getPosY(),
              positioning->getDistRight(), positioning->getDistLeft(),
              linesensor->lineSensorState, linesensor->lineSensorValue,
              linesensor->lineSensorPosX, linesensor->lineSensorPosY);
        }
      }
      break;
  }
}