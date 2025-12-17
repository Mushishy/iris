// IRIS Flight Controller - Sensor Telemetry System

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>  // Built-in IMU (accel, gyro, mag)
#include <Arduino_LPS22HB.h>        // Built-in barometric pressure sensor
#include <TinyGPSPlus.h>            // External GPS module

#include "settings.h"
#include "structs.h"
#include "sensors.h"
#include "telemetry.h"
#include "sdCards.h"
#include "states.h"
#include "unifiedCollector.h"

FlightState currentState;
UnifiedCollectorBuffer collectorBuffer;
FlightData flightData;
uint32_t startTime;

void setup() {
  // Begin Serial
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Nano 33 BLE Sense Rev2.");
  changeState(SENSORS_CALIBRATING);

  // Initialize Sensors
  if (!initializeSensors()) {
    Serial.println("Sensor initialization failure!");
    while(1);
  }

  //testSDCard(SD_CS_1);
  //testSDCard(SD_CS_2);
  deleteFlightDataFile(SD_CS_2);
  //changeState(GROUND_IDLE);
  changeState(ASCENDING);
  startTime = millis();
}

void loop() {
  switch (currentState) {
    case SENSORS_CALIBRATING:
      break;
    case GROUND_IDLE:
      groundIdleLoop();
      break;
    case ASCENDING:
    case DESCENDING:
      collectTelemetryLoop();
      // Check if 1 minute has passed since start
      if (millis() - startTime >= 60000) {  // 60000ms = 1 minute
        Serial.println("1 minute telemetry collection complete");
        changeState(LANDED);
      }
      break;
    case LANDED:
      landedLoop();
      break;
  }
}