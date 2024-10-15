#include <ModbusMaster.h>

// Instantiate ModbusMaster object
ModbusMaster sensor;

// Define the pins for RS485 communication
const int RX_PIN = 16; // Set to the RX pin you are using for Serial2
const int TX_PIN = 17; // Set to the TX pin you are using for Serial2
const int RE = 19;   // Control the transmit/receive mode of RS485 module
const int DE = 18;   // Control the transmit/receive mode of RS485 module

// milis()
unsigned long previousMillis1 = 0; // Store the last time sensor 1 was read
unsigned long previousMillis2 = 0; // Store the last time sensor 2 was read
const long interval1 = 500; // Interval for sensor 1 (500 ms = 0.5 second)
const long interval2 = 1000; // Interval for sensor 1 (1000 ms = 1 second) 

// Variables to store sensor data
uint16_t sensor1Data[7]; // Array to store data from Sensor 1
uint16_t sensor2Data[7]; // Array to store data from Sensor 2

void preTransmission() {
  digitalWrite(RE, HIGH);
  digitalWrite(DE, HIGH);
}

void postTransmission() {
  digitalWrite(RE, LOW);
  digitalWrite(DE, LOW);
}


void setup() {
  Serial.begin(9600);  // Serial communication for debugging
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Configure Serial2 for RS485

  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  digitalWrite(RE, LOW);
  digitalWrite(DE, LOW);

  // Modbus communication setup for sensors
  sensor.begin(1, Serial2); // Address of the first sensor
  sensor.preTransmission(preTransmission);
  sensor.postTransmission(postTransmission);

  sensor.begin(2, Serial2); // Address of the first sensor
  sensor.preTransmission(preTransmission);
  sensor.postTransmission(postTransmission);

  delay(1000); // Allow sensor to stabilize
  Serial.println("Setup complete, starting communication...");
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time

  // Check if it's time to read sensor 1
  if (currentMillis - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis; // Save the last time you read the sensor
    sensor.begin(1, Serial2); // Begin communication with sensor 1
    readNPK("Sensor 1", sensor1Data); // Read data from sensor 1
  }

  // Check if it's time to read sensor 2
  if (currentMillis - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis; // Save the last time you read the sensor
    sensor.begin(2, Serial2); // Begin communication with sensor 2
    readNPK("Sensor 2", sensor2Data); // Read data from sensor 2
  }

  delay(5000);
}


void readNPK(const char* sensorName, uint16_t* data) {
  uint8_t result;

  Serial.print("Requesting data from ");
  Serial.println(sensorName);

  result = sensor.readHoldingRegisters(0x0000, 7); // Start address 0x0000, reading 7 registers

  if (result == sensor.ku8MBSuccess) {
    data[0] = sensor.getResponseBuffer(0x00); // Humidity
    data[1] = sensor.getResponseBuffer(0x01); // Temperature
    data[2] = sensor.getResponseBuffer(0x02); // Conductivity
    data[3] = sensor.getResponseBuffer(0x03); // PH
    data[4] = sensor.getResponseBuffer(0x04); // Nitrogen (N)
    data[5] = sensor.getResponseBuffer(0x05); // Phosphorus (P)
    data[6] = sensor.getResponseBuffer(0x06); // Potassium (K)

    Serial.print(sensorName);
    Serial.print(" - Humidity: ");
    Serial.print(data[0] * 0.1);
    Serial.println(" %RH");

    Serial.print(sensorName);
    Serial.print(" - Temperature: ");
    Serial.print(data[1] * 0.1);
    Serial.println(" °C");

    Serial.print(sensorName);
    Serial.print(" - Conductivity: ");
    Serial.print(data[2]);
    Serial.println(" us/cm");

    Serial.print(sensorName);
    Serial.print(" - PH: ");
    Serial.print(data[3] * 0.1);
    Serial.println();

    Serial.print(sensorName);
    Serial.print(" - Nitrogen (N): ");
    Serial.print(data[4]);
    Serial.println(" mg/kg");

    Serial.print(sensorName);
    Serial.print(" - Phosphorus (P): ");
    Serial.print(data[5]);
    Serial.println(" mg/kg");

    Serial.print(sensorName);
    Serial.print(" - Potassium (K): ");
    Serial.print(data[6]);
    Serial.println(" mg/kg");
  } else {
    Serial.print(sensorName);
    Serial.print(" - Failed to read sensor data, Error code: ");
    Serial.println(result);
  }
}

void verifyAddresses() {
  // Scan Modbus addresses from 1 to 10
  // check the available address in modbus
  for (uint8_t address = 1; address <= 10; address++) {
    sensor.begin(address, Serial2); // Set the Modbus address for the current scan
    Serial.print("Checking address: ");
    Serial.println(address);

    // address register from datasheet (0x0000..0xFFFF)
    // address register remotely (1..125, enforced by remote device)
    uint8_t result = sensor.readHoldingRegisters(0x0000, 1); // Try to read from a known register/address in modbus

    // ku8MBSuccess adalah kode hasil (return code) yang menandakan kesuksesan ketika mengirim atau menerima data dalam protokol Modbus.
    if (result == sensor.ku8MBSuccess) {
      Serial.print("Device found at address: ");
      Serial.println(address);
    } else {
      Serial.print("No response from address: ");
      Serial.println(address);
    }

    delay(200); // Small delay to avoid bus contention during the scan
  }

  Serial.println("Modbus address scan complete.");
  delay(10000); // Wait for 10 seconds before scanning again
}


void changeDeviceAddress() {
  // 0x07D0 adalah alamat register yang mengatur slave ID atau alamat Modbus dari perangkat
  // 1 pada code ini adalah alamat yang akan digunakan
  // jika ingin mengubah alamat menjadi 3. 
  // maka ubah angka 1 menjadi 3
  // saat menjalankan ini pastikan hanya ada satu sensor yang terhubung
  uint8_t result = sensor.writeSingleRegister(0x07D0, 1); // Register 0x07D0 is for the slave ID
  if (result == sensor.ku8MBSuccess) {
    Serial.println("Successfully changed sensor address from 2 to 1.");
  } else {
    Serial.print("Failed to change sensor address, Error code: ");
    Serial.println(result);
  }
}