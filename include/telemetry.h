#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "settings.h"
#include "structs.h"

// ========== TELEMETRY MANAGEMENT ====================
enum TelemetryType {
  ACCEL,
  GYRO, 
  MAG,
  ENV,
  GPS,
  ALL
};

String formatTestTelemetry(const FlightData& data, TelemetryType type = ALL);

#endif