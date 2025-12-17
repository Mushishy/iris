#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>  // Built-in IMU (accel, gyro, mag)
#include <Arduino_LPS22HB.h>        // Built-in barometric pressure sensor
#include <TinyGPSPlus.h>
#include "structs.h"
#include "settings.h"

// Built-in sensor references for Arduino Nano 33 BLE
// IMU and BARO are global objects provided by the libraries
extern UART gpsSerial;
extern TinyGPSPlus gps;

// ========== SENSOR READING FUNCTIONS ==========

AccelerometerData readAccelerometer() {
  AccelerometerData data;
  
  // Read from built-in BMI270 accelerometer
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(data.x, data.y, data.z);
    // BMI270 already returns values in g, convert to m/s²
    data.x *= EARTH_GRAVITY_MS2;
    data.y *= EARTH_GRAVITY_MS2;
    data.z *= EARTH_GRAVITY_MS2;
  } else {
    data.x = data.y = data.z = 0.0;
  }
  
  return data;
}

GyroscopeData readGyroscope() {
  GyroscopeData data;
  
  // Read from built-in BMI270 gyroscope
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(data.x, data.y, data.z);
    // BMI270 returns values in °/s (degrees per second)
  } else {
    data.x = data.y = data.z = 0.0;
  }
  
  return data;
}

MagnetometerData readMagnetometer() {
  MagnetometerData data;
  
  // Read from built-in BMM150 magnetometer
  if (IMU.magneticFieldAvailable() && IMU.accelerationAvailable()) {
    IMU.readMagneticField(data.x, data.y, data.z);
    // BMM150 returns values in µT (microtesla), convert to mG
    data.x *= 10.0;  // 1 µT = 10 mG
    data.y *= 10.0;
    data.z *= 10.0;
    
    // Get accelerometer data for tilt compensation
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    
    // Calculate pitch and roll from accelerometer (in radians)
    float pitch = atan2(ax, sqrt(ay * ay + az * az));
    float roll = atan2(ay, az);
    
    // Tilt-compensated magnetometer readings
    float mag_x_comp = data.x * cos(pitch) + data.z * sin(pitch);
    float mag_y_comp = data.x * sin(roll) * sin(pitch) + 
                       data.y * cos(roll) - 
                       data.z * sin(roll) * cos(pitch);
    
    // Calculate tilt-compensated heading (0-360°)
    data.heading = atan2(mag_y_comp, mag_x_comp) * 180.0 / PI;
    if (data.heading < 0) {
      data.heading += 360.0; // Normalize to 0-360°
    }
    
    // Debug output (remove after testing)
    // Serial.print("Pitch: "); Serial.print(pitch * 180.0 / PI);
    // Serial.print(" Roll: "); Serial.print(roll * 180.0 / PI);
    // Serial.print(" Heading: "); Serial.println(data.heading);
    
    // Apply calibration offset and magnetic declination correction
    // TODO: Calibrate by pointing device north and noting the offset needed
    // data.heading += COMPASS_CALIBRATION_OFFSET;  // Add to settings.h
    // data.heading += MAGNETIC_DECLINATION_DEGREES; // Add to settings.h  
    // if (data.heading >= 360.0) data.heading -= 360.0;
  } else {
    data.x = data.y = data.z = 0.0;
    data.heading = 0.0;
  }
  
  return data;
}

EnvironmentalData readEnvironmental() {
  EnvironmentalData data;
  
  // Read from built-in LPS22HB barometric pressure sensor
  data.temperature = BARO.readTemperature();
  data.pressure = BARO.readPressure() * 1000.0;  // Convert kPa to Pa
  
  // Calculate altitude using barometric formula
  float pressure_kPa = data.pressure / 1000.0;
  data.altitude = ALTITUDE_CALCULATION_CONSTANT * (1.0 - pow(pressure_kPa / STANDARD_SEA_LEVEL_PRESSURE_KPA, BAROMETRIC_EXPONENT));
  
  if (isnan(data.temperature)) data.temperature = 0.0;
  if (isnan(data.pressure)) data.pressure = 0.0;
  if (isnan(data.altitude)) data.altitude = 0.0;
  
  return data;
}

GPSData readGPS() {
  GPSData data;
  data.latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
  data.longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
  data.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;
  data.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
  data.speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  data.course = gps.course.isValid() ? gps.course.deg() : 0.0;
  return data;
}

// ========== SENSOR INITIALIZATION ==========

bool initializeSensors() {
  Serial.println("Initializing Arduino Nano 33 BLE built-in sensors...");
  
  // Initialize GPS
  gpsSerial.begin(GPS_BAUD_RATE);
#ifdef GPS_PPS_PIN
  pinMode(GPS_PPS_PIN, INPUT);
#endif
  Serial.println("  -> GPS initialized waiting for satelite fix");

  // Initialize built-in IMU (BMI270 + BMM150)
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    return false;
  }
  Serial.println("  -> IMU initialized");
  
  // Initialize built-in barometric pressure sensor (LPS22HB)
  if (!BARO.begin()) {
    Serial.println("Failed to initialize BARO!");
  } else {
    Serial.println("  -> Baro initialized");
  }
  
  return true;
}

void setupSerial() {
  // Begin Serial
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Nano 33 BLE Sense Rev2.");
}

// =============== SENSOR LOOP ===============

void processGPSData() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

FlightData collectTelemetryData() {
  FlightData data;
  data.timestamp = millis();
  data.packetId = 0;
  data.accel = readAccelerometer();
  data.gyro = readGyroscope();
  data.mag = readMagnetometer();
  data.env = readEnvironmental();
  data.gps = readGPS();
  return data;
}

#endif