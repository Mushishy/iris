#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "config.h"
#include "structs.h"

struct AccelerometerCalibration
{
  float biasX, biasY, biasZ;
  float scaleX, scaleY, scaleZ;
};

struct GyroscopeCalibration
{
  float biasX, biasY, biasZ;
};

struct MagnetometerCalibration
{
  float hardIronX, hardIronY, hardIronZ;
  float softIronXX, softIronXY, softIronXZ;
  float softIronYX, softIronYY, softIronYZ;
  float softIronZX, softIronZY, softIronZZ;
};

AccelerometerData applyCalibratedAccelerometer(AccelerometerData data);
GyroscopeData applyCalibratedGyroscope(GyroscopeData data);
MagnetometerData applyCalibratedMagnetometer(MagnetometerData data);

void calibrateMagnetometer();
void calibrateAccelerometer();
void calibrateGyroscope();
void calibrateBarometer();

void calibrateAllSensors();
#endif