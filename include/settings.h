#ifndef SETTINGS_H
#define SETTINGS_H

// Phyiscal Constants
#define EARTH_GRAVITY_MS2 9.8                    // Earth's gravity acceleration (m/s²)
#define STANDARD_SEA_LEVEL_PRESSURE_KPA 101.325  // Standard atmospheric pressure at sea level (kPa)
#define ALTITUDE_CALCULATION_CONSTANT 44330.0    // Constant for barometric altitude calculation (m)
#define BAROMETRIC_EXPONENT 0.1903               // Exponent for barometric altitude formula

// Communication Settings
#define SERIAL_BAUD_RATE 115200                 
#define GPS_BAUD_RATE 9600             

// GPS
//#define GPS_WAIT_FOR_SATELLITES               // Uncomment to enable waiting for GPS lock before proceeding
#define GPS_RX_PIN 9                  
#define GPS_TX_PIN 8                  
//#define GPS_PPS_PIN -                

// SD 
#define SD_CS_1 10                      
#define SD_CS_2 2    
#define SD_FILENAME "test.txt"                   
#define O_READ 0x01
#define O_WRITE 0x02
//#define DEBUG_SD_VERBOSE                     // Uncomment to enable verbose SD card write debugging

// Buffer Settings
#define UNIFIED_BUFFER_SIZE 8192  // 8KB buffer - adjust based on available memory
#define UNIFIED_BUFFER_FLUSH_THRESHOLD (UNIFIED_BUFFER_SIZE * 0.8)  // Flush when 80% full

#define SENSOR_ACCEL 0x01
#define SENSOR_GYRO  0x02
#define SENSOR_MAG   0x03
#define SENSOR_BARO  0x04
#define SENSOR_GPS   0x05

#define ACCEL_GYRO_INTERVAL 10     // 10ms = 100Hz (actual measured rate: 99.84Hz)
#define MAG_INTERVAL   100         // 100ms = 10Hz (actual measured rate: 10.00Hz)
#define BARO_INTERVAL  20          // 20ms = 50Hz
#define GPS_INTERVAL   333         // 333ms ≈ 3Hz

// States
#define LAUNCH_THRESHOLD 15.0 / EARTH_GRAVITY_MS2  // Launch detection threshold in g 

#endif