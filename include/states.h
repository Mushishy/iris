#ifndef STATES_H
#define STATES_H

#include "sensors.h"
#include "settings.h"
#include "unifiedCollector.h"

extern uint32_t startTime;

enum FlightState {
  SENSORS_CALIBRATING,
  GROUND_IDLE,                      
  ASCENDING,
  DESCENDING,
  LANDED
};

extern FlightState currentState;

// Function to change state and print status
void changeState(FlightState newState) {
  currentState = newState;
  Serial.print("STATE: ");
  
  switch (newState) {
    case SENSORS_CALIBRATING:
      Serial.println("Sensors Calibrating");
      break;
    case GROUND_IDLE:
      Serial.println("Ground Idle");
      break;
    case ASCENDING:
      Serial.println("Ascending");
      break;
    case DESCENDING:
      Serial.println("Descending");
      break;
    case LANDED:
      Serial.println("Landed");
      break;
  }
}

unsigned long launchDetectionStart = 0;

void groundIdleLoop() {
  AccelerometerData data;
  // Wait for launch detection (acceleration > threshold)
  if (readAccelerometer(data) && abs(data.z) > LAUNCH_THRESHOLD) { 
    if (launchDetectionStart == 0) {
      // Start the 0.1 second timer
      launchDetectionStart = millis();
    } else {
      // Check if 0.1 seconds have passed
      if (millis() - launchDetectionStart >= 100) {
        Serial.println("Launch detected!");
        changeState(ASCENDING);
      }
    }  
  }
}

void collectTelemetryLoop() { 
  updateSensors();
  if (collectorBuffer.bufferIndex > UNIFIED_BUFFER_FLUSH_THRESHOLD) {
    flushBuffer();
  }
}

void landedLoop() {
  readBufferFromSD();
  while(1){

  }
}
/*
  float maxAltitude = 0;
  float launchAltitude = 0;
  unsigned int packetCounter = 0;
  unsigned long lastAltitudeTime = 0;
  float previousAltitude = 0;     
  // Update altitude comparison every 1 second
  if (millis() - lastAltitudeTime >= 1000) { // 1000ms = 1 second
    float currentAltitude = collectorBuffer.env.altitude;
    
    // Check altitude trend for state transitions
    if (currentState == ASCENDING) {
      if (currentAltitude < previousAltitude) {
        Serial.println("STATE: Transitioning to DESCENDING");
        currentState = DESCENDING;
      }
    } else if (currentState == DESCENDING) {
      if (currentAltitude > previousAltitude) {
        Serial.println("STATE: Transitioning to ASCENDING");
        currentState = ASCENDING;
      }
    }
    
    // Update stored values
    previousAltitude = currentAltitude;
    lastAltitudeTime = millis();
  }
  
  // Update max altitude tracker
  if (collectorBuffer.env.altitude > maxAltitude) {
    maxAltitude = collectorBuffer.env.altitude;
  }
  */

#endif