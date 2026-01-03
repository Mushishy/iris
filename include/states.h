#ifndef STATES_H
#define STATES_H

#include <Arduino.h>
#include "settings.h"
#include "telemetry.h"

// Forward declarations to avoid circular dependencies
struct UnifiedCollectorBuffer;
struct FlightData;

// ========== FLIGHT STATE MANAGEMENT ====================
enum FlightState {
  SENSORS_CALIBRATING,
  GROUND_IDLE,                      
  ASCENDING_NO_ENGINE,
  ASCENDING_ENGINE,
  DESCENDING,
  LANDED,
  DEBUG
};

struct FlightStateData {
  FlightState currentState;
  uint32_t startTime = 0;
  uint32_t launchDetectionStart = 0;
  uint32_t engineCutoffStart = 0;
  float previousAltitude = 0;
  uint32_t lastAltitudeTime = 0;
  
  // Improved detection tracking
  uint8_t descendingCount = 0;     // Count consecutive descending readings
  uint32_t landingConfirmStart = 0; // Start time for landing confirmation
  float landingAltitude = 0;       // Altitude when landing detection started
  
#ifdef DEBUG_STATE_ENABLED
  uint32_t lastEchoTime = 0;
#endif

  static constexpr uint8_t SIZE = sizeof(FlightState) + 6 * sizeof(uint32_t) + sizeof(uint8_t) + 2 * sizeof(float)
#ifdef DEBUG_STATE_ENABLED
    + sizeof(uint32_t)
#endif
    ;  // FlightState(4) + 6*uint32_t(24) + uint8_t(1) + 2*float(8) + padding = ~37-41 bytes
};

extern UnifiedCollectorBuffer collectorBuffer;
extern FlightData flightData;
extern FlightStateData stateData;

// Function declarations
void changeState(FlightState newState);

extern FlightStateData stateData;

void changeState(FlightState newState);
void collectTelemetry();
void debugLoop(TelemetryType type);
void groundIdleLoop();
void ascendingNoEngineLoop();
void ascendingEngineLoop();
void descendingLoop();
void landedLoop();

#endif