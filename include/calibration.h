#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "settings.h"
#include "structs.h"

struct AccelerometerCalibration {
  float biasX, biasY, biasZ;              
  float scaleX, scaleY, scaleZ;           
  bool isCalibrated;
};

struct GyroscopeCalibration {
  float biasX, biasY, biasZ;              
  bool isCalibrated;
};

#ifdef MAGNETOMETER_CALIBRATION_ENABLED
struct MagnetometerCalibration {
  float hardIronX, hardIronY, hardIronZ;  
  float softIronXX, softIronXY, softIronXZ; 
  float softIronYX, softIronYY, softIronYZ;
  float softIronZX, softIronZY, softIronZZ;
  bool isCalibrated;
};
#else
struct MagnetometerCalibration {
  bool isCalibrated;
};
#endif

struct SensorCalibration {
  AccelerometerCalibration accel;
  GyroscopeCalibration gyro;
  MagnetometerCalibration mag;
};

extern SensorCalibration sensorCalibration;

AccelerometerData applyCalibratedAccelerometer(AccelerometerData data);
GyroscopeData applyCalibratedGyroscope(GyroscopeData data);
MagnetometerData applyCalibratedMagnetometer(MagnetometerData data);

#ifdef MAGNETOMETER_CALIBRATION_ENABLED
bool calibrateMagnetometer();
#endif

bool calibrateAccelerometer();
bool calibrateGyroscope();
bool calibrateBarometer();

bool calibrateAllSensors();

#endif 