#include <Arduino.h>
#include <TinyGPSPlus.h>
#define E220_22
#define FREQUENCY_868
#include <LoRa_E220.h>
#include <SD.h>

// Built in Arduino sensors
#include <Arduino_LPS22HB.h> //Baro
#include <Arduino_BMI270_BMM150.h> //IMU

// GPS - hardware serial (existing, d0, d1)
// VCC 3.3V
// GND GND 
#define GPS_TX 8 // sensor - 10kohm - arduino - 20kohm - ground with voltage divider
#define GPS_RX 9
UART gpsSerial(digitalPinToPinName(GPS_RX), digitalPinToPinName(GPS_TX));
TinyGPSPlus gps;


// LoRa - hardware serial (created, 3, 4)
#define LORA_M0 5
#define LORA_M1 6
#define LORA_RX 3
#define LORA_TX 4
#define LORA_AUX 7
// VCC 3.3V
// GND GND 
UART loraSerial(digitalPinToPinName(LORA_RX), digitalPinToPinName(LORA_TX));
LoRa_E220 e220ttl(&loraSerial, LORA_AUX, LORA_M0, LORA_M1);



// VIN 3.3V 
// GND GND
// SCK  = 13 (automatic)
// MISO = 12 (automatic) 
// MOSI = 11 (automatic)
#define SD_CS 10
const char *testfilename = "test.txt";

#define SD_CS_2 2
// 3V3 3.3
// CS 2
// MOSI 11
// CLK Pin 13 
// MISO Pin 12
// GND GND

// Helper functions to switch between SD cards
void useCard1() {
  SD.end();
  if (!SD.begin(SD_CS)) {
    Serial.println("Failed to switch to SD Card 1");
  }
}

void useCard2() {
  SD.end();
  if (!SD.begin(SD_CS_2)) {
    Serial.println("Failed to switch to SD Card 2");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Nano 33 BLE Sense Rev2.");

  // 1. GPS setup
  gpsSerial.begin(9600);
  Serial.println("  -> GPS initialized.");

  // 2. LoRa setup
  pinMode(LORA_AUX, INPUT);
  pinMode(LORA_M0, OUTPUT);
  digitalWrite(21, LOW);
  pinMode(LORA_M1, OUTPUT);
  digitalWrite(19, LOW);
  e220ttl.begin();
  delay(100);
  ResponseStatus rs = e220ttl.sendMessage("Setup Msg Test.");
  Serial.println(rs.getResponseDescription());
  Serial.println("  -> LoRa initialized.");
 
  // 3. SD Card 1 setup
  useCard1();
  Serial.println("  -> SD Card 1 initialized.");

  if (SD.exists(testfilename)) {
    Serial.print("Card 1 - File exists. Reading ");
    Serial.println(testfilename);

    File file = SD.open(testfilename, FILE_READ);
    if (file) {
      Serial.println("Card 1 - File contents:");
      while (file.available()) {
        Serial.write(file.read());
      }
      file.close();
      Serial.println("\n--- End of Card 1 file ---");
    } else {
      Serial.println("Card 1 - Error opening file for reading.");
    }
  } else {
    Serial.print("Card 1 - File not found. Creating ");
    Serial.println(testfilename);

    File file = SD.open(testfilename, FILE_WRITE);
    if (file) {
      file.println("Hello from Card 1!");
      file.close();
      Serial.println("Card 1 - File created and written successfully!");
    } else {
      Serial.println("Card 1 - Error creating file.");
    }
  }

  // 4. SD Card 2 setup
  useCard2();
  Serial.println("  -> SD Card 2 initialized.");

  if (SD.exists(testfilename)) {
    Serial.print("Card 2 - File exists. Reading ");
    Serial.println(testfilename);

    File file = SD.open(testfilename, FILE_READ);
    if (file) {
      Serial.println("Card 2 - File contents:");
      while (file.available()) {
        Serial.write(file.read());
      }
      file.close();
      Serial.println("\n--- End of Card 2 file ---");
    } else {
      Serial.println("Card 2 - Error opening file for reading.");
    }
  } else {
    Serial.print("Card 2 - File not found. Creating ");
    Serial.println(testfilename);

    File file = SD.open(testfilename, FILE_WRITE);
    if (file) {
      file.println("Hello from Card 2!");
      file.close();
      Serial.println("Card 2 - File created and written successfully!");
    } else {
      Serial.println("Card 2 - Error creating file.");
    }
  }
  Serial.println("");


  // 4. IMU setup
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("  -> IMU initialized.");
  Serial.print("Accelerometer sample rate = ");
  Serial.println(IMU.accelerationSampleRate());

  // 5. Baro setup
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }
  Serial.println("  -> Baro initialized.");
  float pressure = BARO.readPressure();
  float altitude = 44330 * ( 1 - pow(pressure/101.325, 1/5.255) );
  Serial.print("Altitude according to kPa is = ");
  Serial.print(altitude);
  Serial.println(" m");
}

void loop() {
  //Serial.println(gpsSerial.read());
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    Serial.println("GPS FIX acquired ✅");
    if (gps.location.isUpdated()) {
      Serial.print("Lat: "); Serial.println(gps.location.lat(), 6);
      Serial.print("Lng: "); Serial.println(gps.location.lng(), 6);

      ResponseStatus rs = e220ttl.sendMessage("LOC PLACEHOLDER");
      Serial.print("Sent location update message - ");
      Serial.println(rs.getResponseDescription());
    }
  }
  else {
    //Serial.println("No fix yet ❌...");  
  }
}