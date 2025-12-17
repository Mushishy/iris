#ifndef SETTINGS_H
#define SETTINGS_H

// ========== PHYSICAL CONSTANTS ==========
#define EARTH_GRAVITY_MS2 9.8          // Earth's gravity acceleration (m/s²)
#define STANDARD_SEA_LEVEL_PRESSURE_KPA 101.325  // Standard atmospheric pressure at sea level (kPa)
#define ALTITUDE_CALCULATION_CONSTANT 44330.0    // Constant for barometric altitude calculation (m)
#define BAROMETRIC_EXPONENT 0.1903               // Exponent for barometric altitude formula

// ========== COMMUNICATION SETTINGS ==========
#define SERIAL_BAUD_RATE 115200        // Serial monitor baud rate
#define GPS_BAUD_RATE 9600             // GPS module baud rate

// ========== HARDWARE ADDRESSES ==========
// Arduino Nano 33 BLE built-in sensors use internal I2C addresses
// No manual addressing required for BMI270, BMM150, or LPS22HB

// ========== PIN ASSIGNMENTS ==========
#define GPS_RX_PIN 9                  
#define GPS_TX_PIN 8                  
//#define GPS_PPS_PIN -                

// ========== TIMING PARAMETERS ==========
#define TELEMETRY_LOOP_DELAY_MS 100    // Main loop delay for telemetry output (ms)

// ========== CONDITIONAL COMPILATION ==========
// #define CALIBRATE_MAGNETOMETER       // Uncomment to enable magnetometer calibration
#define ENABLE_SENSOR_CALIBRATION    // Comment this line to disable all calibration

#endif