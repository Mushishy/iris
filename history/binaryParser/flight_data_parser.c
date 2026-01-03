#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// Sensor type constants (match your Arduino code)
#define SENSOR_ACCEL 1
#define SENSOR_GYRO  2
#define SENSOR_MAG   3
#define SENSOR_BARO  4
#define SENSOR_GPS   5

int main() {
    FILE *input_file = fopen("test.txt", "rb");
    if (!input_file) {
        printf("Error: Cannot open test.txt for reading\n");
        return true;
    }
    
    FILE *output_file = fopen("test.csv", "w");
    if (!output_file) {
        printf("Error: Cannot create test.csv for writing\n");
        fclose(input_file);
        return true;
    }
    
    // Write CSV header
    fprintf(output_file, "Timestamp,SensorType,X,Y,Z,Extra1,Extra2,Extra3\n");
    
    printf("Converting flight data from test.txt to test.csv...\n");
    
    // Get file size
    fseek(input_file, 0, SEEK_END);
    long file_size = ftell(input_file);
    fseek(input_file, 0, SEEK_SET);
    
    printf("File size: %ld bytes\n", file_size);
    
    long bytes_processed = 0;
    int chunk_number = 1;
    
    while (bytes_processed < file_size) {
        // Read buffer size header (2 bytes)
        uint16_t buffer_size;
        if (fread(&buffer_size, sizeof(uint16_t), 1, input_file) != 1) {
            printf("End of file or error reading buffer size\n");
            break;
        }
        bytes_processed += sizeof(uint16_t);
        
        printf("Processing chunk #%d: %d bytes\n", chunk_number++, buffer_size);
        
        // Validate buffer size
        if (buffer_size == 0 || buffer_size > 10000) {
            printf("Invalid buffer size: %d\n", buffer_size);
            break;
        }
        
        // Process sensors in this buffer
        uint16_t chunk_bytes_read = 0;
        while (chunk_bytes_read < buffer_size) {
            // Read sensor type (1 byte)
            uint8_t sensor_type;
            if (fread(&sensor_type, 1, 1, input_file) != 1) {
                printf("Error reading sensor type\n");
                break;
            }
            chunk_bytes_read++;
            bytes_processed++;
            
            // Read timestamp (4 bytes)
            uint32_t timestamp;
            if (fread(&timestamp, sizeof(uint32_t), 1, input_file) != 1) {
                printf("Error reading timestamp\n");
                break;
            }
            chunk_bytes_read += 4;
            bytes_processed += 4;
            
            // Process sensor data based on type
            switch (sensor_type) {
                case SENSOR_ACCEL: {
                    float values[3];
                    if (fread(values, sizeof(float), 3, input_file) == 3) {
                        fprintf(output_file, "%u,ACCEL,%.3f,%.3f,%.3f,,,\n", 
                               timestamp, values[0], values[1], values[2]);
                        chunk_bytes_read += 12;
                        bytes_processed += 12;
                    } else {
                        printf("Error reading accelerometer data\n");
                    }
                    break;
                }
                
                case SENSOR_GYRO: {
                    float values[3];
                    if (fread(values, sizeof(float), 3, input_file) == 3) {
                        fprintf(output_file, "%u,GYRO,%.3f,%.3f,%.3f,,,\n", 
                               timestamp, values[0], values[1], values[2]);
                        chunk_bytes_read += 12;
                        bytes_processed += 12;
                    } else {
                        printf("Error reading gyroscope data\n");
                    }
                    break;
                }
                
                case SENSOR_MAG: {
                    float values[4];
                    if (fread(values, sizeof(float), 4, input_file) == 4) {
                        fprintf(output_file, "%u,MAG,%.3f,%.3f,%.3f,%.3f,,\n", 
                               timestamp, values[0], values[1], values[2], values[3]);
                        chunk_bytes_read += 16;
                        bytes_processed += 16;
                    } else {
                        printf("Error reading magnetometer data\n");
                    }
                    break;
                }
                
                case SENSOR_BARO: {
                    float values[3];
                    if (fread(values, sizeof(float), 3, input_file) == 3) {
                        fprintf(output_file, "%u,BARO,%.3f,%.3f,%.3f,,,\n", 
                               timestamp, values[0], values[1], values[2]); // temp, pressure, altitude
                        chunk_bytes_read += 12;
                        bytes_processed += 12;
                    } else {
                        printf("Error reading barometer data\n");
                    }
                    break;
                }
                
                case SENSOR_GPS: {
                    struct {
                        double lat, lng;
                        float hdop, speed, course;
                        uint16_t sats;
                    } gps_data;
                    
                    if (fread(&gps_data, 26, 1, input_file) == 1) {
                        fprintf(output_file, "%u,GPS,%.6f,%.6f,%.2f,%.2f,%.2f,%u\n", 
                               timestamp, gps_data.lat, gps_data.lng, gps_data.hdop, 
                               gps_data.speed, gps_data.course, gps_data.sats);
                        chunk_bytes_read += 26;
                        bytes_processed += 26;
                    } else {
                        printf("Error reading GPS data\n");
                    }
                    break;
                }
                
                default:
                    printf("Unknown sensor type: %d\n", sensor_type);
                    // Skip to avoid infinite loop - this is dangerous without knowing size
                    goto end_processing;
            }
            
            // Safety check
            if (bytes_processed >= file_size) {
                break;
            }
        }
        
        // Show progress for large files
        if (file_size > 10000) {
            printf("Progress: %ld%% (%ld/%ld bytes)\n", 
                   (bytes_processed * 100) / file_size, bytes_processed, file_size);
        }
    }
    
end_processing:
    fclose(input_file);
    fclose(output_file);
    
    printf("Conversion complete!\n");
    printf("Processed %ld bytes total\n", bytes_processed);
    printf("Output saved to test.csv\n");
    
    return false;
}