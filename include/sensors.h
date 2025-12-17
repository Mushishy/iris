#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>  // Built-in IMU (accel, gyro, mag)
#include <Arduino_LPS22HB.h>        // Built-in barometric pressure sensor
#include <TinyGPSPlus.h>
#include "structs.h"
#include "settings.h"

UART gpsSerial(digitalPinToPinName(GPS_RX_PIN), digitalPinToPinName(GPS_TX_PIN));
TinyGPSPlus gps;

// ========== SENSOR READING FUNCTIONS ==========

bool readAccelerometer(AccelerometerData& data) {
  // Read from built-in BMI270 accelerometer - returns raw values in g
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(data.x, data.y, data.z);
    return true;
  }
  return false;
}

bool readGyroscope(GyroscopeData& data) {
  // Read from built-in BMI270 gyroscope
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(data.x, data.y, data.z);
    // BMI270 returns values in °/s (degrees per second)
    return true;
  }
  return false;
}

bool readMagnetometer(MagnetometerData& data) {
  // Read from built-in BMM150 magnetometer - returns raw values in µT
  if (IMU.magneticFieldAvailable() && IMU.accelerationAvailable()) {
    IMU.readMagneticField(data.x, data.y, data.z);
    
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
    return true;
  }
  return false;
}

bool readEnvironmental(EnvironmentalData& data) {
  // Read from built-in LPS22HB barometric pressure sensor - returns raw values
  data.temperature = BARO.readTemperature();  // °C
  data.pressure = BARO.readPressure();        // kPa (raw)
  
  // Calculate altitude using barometric formula  
  data.altitude = ALTITUDE_CALCULATION_CONSTANT * (1.0 - pow(data.pressure / STANDARD_SEA_LEVEL_PRESSURE_KPA, BAROMETRIC_EXPONENT));
  
  // Check for valid readings
  if (isnan(data.temperature) || isnan(data.pressure) || isnan(data.altitude)) {
    return false;
  }
  
  return true;
}

bool readGPS(GPSData& data) {
  // GPS is valid if we have at least location data
  if (gps.location.isValid()) {
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : -1.0;
    data.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
    data.speed = gps.speed.isValid() ? gps.speed.kmph() : -1.0;
    data.course = gps.course.isValid() ? gps.course.deg() : -1.0;
    return true;
  }
  return false;
}

void processGPSData() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
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

#ifdef GPS_WAIT_FOR_SATELLITES
  Serial.println("Waiting for GPS lock...");
  while (gps.satellites.value() < 3) {
    processGPSData();
  }
  Serial.println("GPS lock acquired.");
#endif

  // Initialize built-in IMU (BMI270 + BMM150)
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    return false;
  }
  Serial.println("  -> IMU initialized");

  Serial.println(String("Accel Sample Rate: ") + IMU.accelerationSampleRate());
  Serial.println(String("Gyro Sample Rate: ") + IMU.gyroscopeSampleRate());
  Serial.println(String("Mag Sample Rate: ") + IMU.magneticFieldSampleRate());  
  
  // Initialize built-in barometric pressure sensor (LPS22HB)
  if (!BARO.begin()) {
    Serial.println("Failed to initialize BARO!");
  } else {
    Serial.println("  -> Baro initialized");
  }
  
  return true;
}

#endif