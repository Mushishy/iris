#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"

#define UNIFIED_BUFFER_SIZE 4096

BMI270 imu;

struct UnifiedCollectorBuffer {
  uint8_t buffer[UNIFIED_BUFFER_SIZE];  // 4096 bytes
  uint16_t bufferIndex = 0;             // 2 bytes
  // Total buffer struct: ~4098 bytes
};

void connectBMI();

void setup()
{
    // Start serial
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    Serial.println("BMI270 Example 1 - Basic Readings I2C");

    connectBMI();
    Serial.println("BMI270 connected!");
}

void loop()
{
    imu.getSensorData();

    // Print acceleration data
    Serial.print("Acceleration in g's");
    Serial.print("\t");
    Serial.print("X: ");
    Serial.print(imu.data.accelX, 3);
    Serial.print("\t");
    Serial.print("Y: ");
    Serial.print(imu.data.accelY, 3);
    Serial.print("\t");
    Serial.print("Z: ");
    Serial.print(imu.data.accelZ, 3);

    Serial.print("\t");

    // Print rotation data
    Serial.print("Rotation in deg/sec");
    Serial.print("\t");
    Serial.print("X: ");
    Serial.print(imu.data.gyroX, 3);
    Serial.print("\t");
    Serial.print("Y: ");
    Serial.print(imu.data.gyroY, 3);
    Serial.print("\t");
    Serial.print("Z: ");
    Serial.println(imu.data.gyroZ, 3);
}

void connectBMI() {
    Wire1.begin();
    Wire1.setClock(400000);

    while(imu.beginI2C(BMI2_I2C_PRIM_ADDR, Wire1) != BMI2_OK)
    {
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        delay(1000);
    }

    imu.setAccelODR(BMI2_ACC_ODR_1600HZ);
    imu.setGyroODR(BMI2_GYR_ODR_1600HZ);
}