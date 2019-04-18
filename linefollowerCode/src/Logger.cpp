#include "Logger.h"
#include <SD.h>
#include <SPI.h>
#include "Arduino.h"
#include "Linesensor.h"
#include "Positioning.h"

File logFile;
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
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  logFile = SD.open("lineLog.txt", FILE_WRITE);
  logFileOpen = true;

  // if the file opened okay, write to it:
  if (logFile) {
    Serial.print("Writing to test.txt...");
    logFile.println("Line log v1");
    // close the file:
    logFile.close();
    logFileOpen = false;
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  logFile = SD.open("test.txt");
  logFileOpen = true;
  if (logFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (logFile.available()) {
      Serial.write(logFile.read());
    }
    // close the file:
    logFile.close();
    logFileOpen = false;
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void Logger::update(int controllerState) {
  switch (controllerState) {
    case 1:  // init
    case 2:  // reset line calibration, wait to stand still
    case 3:  // turn 360 deg
    case 4:  // center on line
      if (logFileOpen) {
        logFile.close();
        logFileOpen = false;
      }
      break;
    case 5:  // running
      if ((*linesensor).lineSensorState == Linesensor::onLine) {
        if (logFile && !logFileOpen) {
          logFile = SD.open("lineLog.txt", FILE_WRITE);
          logFileOpen = true;
        }
        if (logFileOpen) {
          String logString = "t " + (String)millis() + " v " +
                             (String)(*positioning).velocity + " h " +
                             (String)(*positioning).heading + " px " +
                             (String)(*positioning).posX + " py " +
                             (String)(*positioning).posY + " dr " +
                             (String)(*positioning).distRight + " dl " +
                             (String)(*positioning).distLeft + " l " +
                             (String)(*linesensor).lineSensorValue + "\n";
          logFile.println(logString);
        }
      }
      break;
  }
}