#ifndef STRUCTS_H
#define STRUCTS_H

#include <Arduino.h>

#include "settings.h"

// vx = ax * t; (you can find out position sx += vx * t)
// if I know velocity I can calculate max velocity


// ========== SENSOR DATA STRUCTURES ===================
// Accelerometer data structure (g) - BMI270 returns float
struct AccelerometerData {
  float x, y, z;
  static constexpr uint8_t SIZE = 12;
  // float totalAcceleration = sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z); // g
};

// Attitude data structure - calculated from accelerometer
struct AttitudeData {
  float pitch;    // degrees
  float roll;     // degrees
  float yaw;      // degrees (from magnetometer heading)
  float offVert;  // degrees (off vertical)
  static constexpr uint8_t SIZE = 16;
};

// Gyroscope data structure (°/s) - BMI270 returns float  
struct GyroscopeData {
  float x, y, z;
  static constexpr uint8_t SIZE = 12;  
  // float angularRate = sqrt(gyro.x*gyro.x + gyro.y*gyro.y + gyro.z*gyro.z); °/s
};

// Magnetometer data structure (µT) - BMM150 returns float
struct MagnetometerData {
  float x, y, z;
  static constexpr uint8_t SIZE = 12; 
  // float magneticFieldStrength = sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z); // µT
};

// Environmental sensor data structure - LPS22HB returns float
struct EnvironmentalData {
  float temperature;            // °C
  float pressure;               // kPa  
  float rawAlt;                 // m
  static constexpr uint8_t SIZE = 12; 
  // float airDensity = (env.pressure * 100.0f) / (DRY_AIR_GAS_CONSTANT * (env.temperature + CELSIUS_TO_KELVIN)); // in kg/m³
  // float altitudeAboveLaunchPad = env.rawAlt - launchPadAltitude; 
};

// GPS data structure - TinyGPS++ returns double for lat/lng, others float
struct GPSData {
  double latitude;      // degrees
  double longitude;     // degrees  
  float hdop;           // horizontal dilution of precision
  float speed;          // km/h
  float course;         // degrees 
  uint16_t satellites;  // number of satellites
  static constexpr uint8_t SIZE = 26;
};

// ========== GLOBALS ====================================  
struct FlightData {
  AccelerometerData accel;         
  GyroscopeData gyro;                
  MagnetometerData mag;            
  EnvironmentalData env;           
  GPSData gps;                     
  
  // Altitude tracking
  float maxAltitude = 0;           
  float launchAltitude = 0;        
  // float maxTotalAcceleration = 0;           
  
  // Timing variables for collection intervals
  uint32_t lastAccelGyroTime = 0;  
  uint32_t lastMagTime = 0;         
  uint32_t lastBaroTime = 0;       
  uint32_t lastGPSTime = 0;        
  
  static constexpr uint8_t SIZE = 98;
};

struct UnifiedCollectorBuffer {
  uint8_t buffer[UNIFIED_BUFFER_SIZE];  
  uint16_t bufferIndex = 0;             
  static constexpr uint16_t SIZE = UNIFIED_BUFFER_SIZE + 2;
};

#endif