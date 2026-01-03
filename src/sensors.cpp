#include "sensors.h"

// ========== GLOBAL VARIABLES (DEFINITIONS) ==========
UART gpsSerial(digitalPinToPinName(GPS_RX_PIN), digitalPinToPinName(GPS_TX_PIN));
TinyGPSPlus gps;
extern FlightData flightData;

// ========== SENSOR READING FUNCTIONS ==========

bool readAccelerometer(AccelerometerData& data) {
  // Read from built-in BMI270 accelerometer
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(data.x, data.y, data.z);
    // BMI270 returns values in g (earth gravity)
    
#ifdef CALIBRATION_ENABLED
    extern SensorCalibration sensorCalibration;
    if (sensorCalibration.accel.isCalibrated) {
      applyCalibratedAccelerometer(data);
    }
#endif
    return true;
  }
  return false;
}

bool readGyroscope(GyroscopeData& data) {
  // Read from built-in BMI270 gyroscope
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(data.x, data.y, data.z);
    // BMI270 returns values in °/s (degrees per second)
    
#ifdef CALIBRATION_ENABLED
    extern SensorCalibration sensorCalibration;
    if (sensorCalibration.gyro.isCalibrated) {
      applyCalibratedGyroscope(data);
    }
#endif
    return true;
  }
  return false;
}

bool readMagnetometer(MagnetometerData& data) {
  // Read from built-in BMM150 magnetometer
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(data.x, data.y, data.z);  
    // BMM150 returns values in µT (micro Tesla)
#ifdef CALIBRATION_ENABLED
#ifdef MAGNETOMETER_CALIBRATION_ENABLED
    extern SensorCalibration sensorCalibration;
    if (sensorCalibration.mag.isCalibrated) {
      applyCalibratedMagnetometer(data);
    }
#endif
#endif
    return true;
  }
  return false;
}

bool readEnvironmental(EnvironmentalData& data) {
  // Read from built-in LPS22HB barometric pressure sensor
  data.temperature = BARO.readTemperature();  // °C
  data.pressure = BARO.readPressure();        // kPa (raw)
  
  // Calculate altitude using barometric formula  
  data.rawAlt = SEA_LEVEL_ALTITUDE * (1.0 - pow(data.pressure / STANDARD_SEA_LEVEL_PRESSURE_KPA, BAROMETRIC_EXPONENT));
  
  // Check for valid readings
  if (isnan(data.temperature) || isnan(data.pressure) || isnan(data.rawAlt)) {
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

/*
// Unused function for attitude calculation (roll, pitch, yaw) from accel and mag that could be calculated on Hermes
AttitudeData calculateAttitude(const AccelerometerData& accel, const MagnetometerData& mag) {
  AttitudeData attitude;
  
  // Roll and Pitch from accelerometer
  attitude.roll = atan2(accel.y, sqrt(pow(accel.x, 2) + pow(accel.z, 2))) * 180.0/PI;
  attitude.pitch = atan2(-accel.x, sqrt(pow(accel.y, 2) + pow(accel.z, 2))) * 180.0 / PI;
  
  // Convert to radians for tilt compensation
  float pitch_rad = attitude.pitch * PI / 180.0;
  float roll_rad = attitude.roll * PI / 180.0;
  
  // Tilt-compensated magnetometer readings
  float mag_x_comp = mag.x * cos(pitch_rad) + mag.z * sin(pitch_rad);
  float mag_y_comp = mag.x * sin(roll_rad) * sin(pitch_rad) + 
                     mag.y * cos(roll_rad) - 
                     mag.z * sin(roll_rad) * cos(pitch_rad);
  
  // Yaw (heading) from tilt-compensated magnetometer
  attitude.yaw = atan2(mag_y_comp, mag_x_comp) * 180.0 / PI;
  attitude.yaw = fmod(attitude.yaw + 360.0, 360.0); 

  // Calculate off-vertical angle
  attitude.offVert = sqrt(pow(attitude.roll, 2) + pow(attitude.pitch, 2));
  
  return attitude;
}
*/

// ========== SENSOR INITIALIZATION ==========

bool initializeSensors() {
  // Initialize GPS
  gpsSerial.begin(GPS_BAUD_RATE);
#ifdef GPS_PPS_PIN
  pinMode(GPS_PPS_PIN, INPUT);
#endif
  Serial.println("GPS init");

#ifdef GPS_WAIT_FOR_SATELLITES
  Serial.println("Waiting for GPS...");
  while (gps.satellites.value() < 3) {
    processGPSData();
  }
  Serial.println("GPS ready");
#endif

  // Initialize built-in IMU (BMI270 + BMM150)
  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    return false;
  }
  Serial.println("IMU init");
  
  // Initialize built-in barometric pressure sensor (LPS22HB)
  if (!BARO.begin()) {
    Serial.println("Baro init failed!");
  } else {
    Serial.println("Baro init");
  }
  
#ifdef CALIBRATION_ENABLED
  Serial.println();
  if (calibrateAllSensors()) {
    Serial.println("Sensors ready");
  } else {
    Serial.println("Calibration failed - using raw data");
  }
#else
  Serial.println("Sensors ready (no calibration)");
#endif
  
  return true;
}