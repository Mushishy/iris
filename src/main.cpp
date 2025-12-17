// IRIS Flight Controller - Sensor Telemetry System

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>  // Built-in IMU (accel, gyro, mag)
#include <Arduino_LPS22HB.h>        // Built-in barometric pressure sensor
#include <TinyGPSPlus.h>

#include "settings.h"
#include "structs.h"
#include "sensors.h"
#include "telemetry.h"

FlightData telemetryData;

// Hardware instances
UART gpsSerial(digitalPinToPinName(GPS_RX_PIN), digitalPinToPinName(GPS_TX_PIN));
TinyGPSPlus gps;

void setup() {
  setupSerial();

  // Initialize sensors with error handling
  if (!initializeSensors()) {
    Serial.println("Sensor initialization failure!");
    while(1);
  }
}

void loop() {
  static uint16_t packetCounter = 0;
  
  // Process incoming GPS data
  processGPSData();
  
  // Collect sensor readings and send telemetry
  telemetryData = collectTelemetryData();
  telemetryData.packetId = packetCounter++;
  Serial.println(formatTelemetry(telemetryData));
  
  delay(TELEMETRY_LOOP_DELAY_MS);
}