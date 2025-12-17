#ifndef STRUCTS_H
#define STRUCTS_H

#include <Arduino.h>

// ========== DATA STRUCTURES ==========

// Accelerometer data structure (g) - BMI270 returns float
struct AccelerometerData {
  float x, y, z;
  static constexpr uint8_t SIZE = 12;  // 3 * sizeof(float) = 12 bytes
};

// Gyroscope data structure (°/s) - BMI270 returns float  
struct GyroscopeData {
  float x, y, z;
  static constexpr uint8_t SIZE = 12;  // 3 * sizeof(float) = 12 bytes
};

// Magnetometer data structure (µT) - BMM150 returns float
struct MagnetometerData {
  float x, y, z;
  float heading; // 0° = North, 90° = East, 180° = South, 270° = West
  static constexpr uint8_t SIZE = 16;  // 4 * sizeof(float) = 16 bytes
};

// Environmental sensor data structure - LPS22HB returns float
struct EnvironmentalData {
  float temperature;    // °C
  float pressure;       // kPa  
  float altitude;       // m
  static constexpr uint8_t SIZE = 12;  // 3 * sizeof(float) = 12 bytes
};

// GPS data structure - TinyGPS++ returns double for lat/lng, others float
struct GPSData {
  double latitude;      // degrees - TinyGPS++ uses double for precision
  double longitude;     // degrees - TinyGPS++ uses double for precision  
  float hdop;           // horizontal dilution of precision
  uint16_t satellites;  // number of satellites (0-255 would fit in uint8_t but uint16_t for safety)
  float speed;          // km/h
  float course;         // degrees (0-360°)
  static constexpr uint8_t SIZE = 26;  // 2*8 + 3*4 + 2 = 26 bytes
};

// Complete flight data telemetry packet
struct FlightData {
  AccelerometerData accel;         // 12 bytes
  GyroscopeData gyro;              // 12 bytes  
  MagnetometerData mag;            // 16 bytes
  EnvironmentalData env;           // 12 bytes
  GPSData gps;                     // 26 bytes
  
  // Timing variables for collection intervals
  uint32_t lastAccelGyroTime = 0;  // 4 bytes
  uint32_t lastMagTime = 0;        // 4 bytes
  uint32_t lastBaroTime = 0;       // 4 bytes
  uint32_t lastGPSTime = 0;        // 4 bytes
  // Total FlightData: ~94 bytes
};

struct UnifiedCollectorBuffer {
  uint8_t buffer[UNIFIED_BUFFER_SIZE];  // 8192 bytes
  uint16_t bufferIndex = 0;             // 2 bytes
  // Total buffer struct: ~8194 bytes
};

#endif