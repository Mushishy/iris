#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "settings.h"
#include "structs.h"

struct AccelerometerCalibration {
  float biasX, biasY, biasZ;              
  float scaleX, scaleY, scaleZ;           
  bool isCalibrated = false;
  static constexpr uint8_t SIZE = 25;     
};

struct GyroscopeCalibration {
  float biasX, biasY, biasZ;              
  bool isCalibrated = false;
  static constexpr uint8_t SIZE = 13; 
};

#ifdef MAGNETOMETER_CALIBRATION_ENABLED
struct MagnetometerCalibration {
  float hardIronX, hardIronY, hardIronZ;  
  float softIronXX, softIronXY, softIronXZ; 
  float softIronYX, softIronYY, softIronYZ;
  float softIronZX, softIronZY, softIronZZ;
  bool isCalibrated = false;
  static constexpr uint8_t SIZE = 49;     
};
#else
struct MagnetometerCalibration {
  bool isCalibrated = false;
  static constexpr uint8_t SIZE = 1;        
};
#endif

struct SensorCalibration {
  AccelerometerCalibration accel;
  GyroscopeCalibration gyro;
  MagnetometerCalibration mag;
#ifdef MAGNETOMETER_CALIBRATION_ENABLED
  static constexpr uint8_t SIZE = 87;     // 25 + 13 + 49 = 87 bytes
#else
  static constexpr uint8_t SIZE = 39;     // 25 + 13 + 1 = 39 bytes
#endif
};

extern SensorCalibration sensorCalibration;

void applyCalibratedAccelerometer(AccelerometerData& data);
void applyCalibratedGyroscope(GyroscopeData& data);

#ifdef MAGNETOMETER_CALIBRATION_ENABLED
void applyCalibratedMagnetometer(MagnetometerData& data);
bool calibrateMagnetometer();
#endif

bool calibrateAccelerometer();
bool calibrateGyroscope();

bool calibrateAllSensors();

#endif 