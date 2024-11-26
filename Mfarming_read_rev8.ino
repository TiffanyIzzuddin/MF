#include <BlockNot.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <WiFi.h>
#include <ModbusMaster.h>

// DHT Sensor
#define DHTPIN 23
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// BH1750
BH1750 lightMeter;

// ModbusMaster object for NPK sensors
ModbusMaster sensor;

// LED Pins
#define RED_LED 4
#define GREEN_LED 2
#define BLUE_LED 15

// RS485 Pins
#define DE_PIN 18
#define RE_PIN 19
#define DI_PIN 17
#define RO_PIN 16

// Relay Pins
#define RELAY1 32
#define RELAY2 33
#define RELAY3 25
#define RELAY4 26
#define RELAY5 27
#define RELAY6 14

// MQTT settings
const char* wifiName = "SMARTFARMING MODEM";
const char* wifiPass = "smfamodem";
const char* brokerUser = "smartfarming";
const char* brokerPass = "smartfarming";
const char* brokerHost = "54.196.58.97";

// MQTT Topics
const char* topicBHT = "farm/bht";
const char* topicNpk1 = "farm/npk1";
const char* topicNpk2 = "farm/npk2";

const char* topicRelay1 = "relay1"; //valve air
const char* topicRelay2 = "relay2"; //valve nutrisi
const char* topicRelay3 = "relay3";
const char* topicRelay4 = "relay4";
const char* topicRelay5 = "relay5";
const char* topicRelay6 = "relay6";

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

ModbusMaster sensor1;
ModbusMaster sensor2;

// Sensor data
float lux = 0;
float temperature = 0;
float humidity = 0;

uint16_t sensor1Data[7];
uint16_t sensor2Data[7];

// Timer
BlockNot timer30Detik(30000);
BlockNot timer5Detik(5000);
BlockNot timer10Detik(10000);
BlockNot timer3Detik(3000);

bool statusGantian = true;

// Pre and post transmission for Modbus communication
void preTransmission() {
  digitalWrite(RE_PIN, HIGH);
  digitalWrite(DE_PIN, HIGH);
}

void postTransmission() {
  digitalWrite(RE_PIN, LOW);
  digitalWrite(DE_PIN, LOW);
}

void setup() {
  Serial.begin(9600);
  KoneksiWIFI();
  client.setServer(brokerHost, 1883);
  client.setCallback(callback);

  // RS485 and Modbus setup
  pinMode(DE_PIN, OUTPUT);
  pinMode(RE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);

  Serial2.begin(9600, SERIAL_8N1, RO_PIN, DI_PIN);

  // Initialize the first Modbus sensor at address 1
  sensor1.begin(1, Serial2);
  sensor1.preTransmission(preTransmission);
  sensor1.postTransmission(postTransmission);

  // Initialize the second Modbus sensor at address 2
  sensor2.begin(2, Serial2);
  sensor2.preTransmission(preTransmission);
  sensor2.postTransmission(postTransmission);

  // DHT and BH1750 sensor setup
  dht.begin();
  Serial.println("Setup DHT complete, starting communication...");
  Wire.begin();
  lightMeter.begin();
  Serial.println("Setup BH1750 complete, starting communication...");

  // LED setup
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  // LCD setup
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Farming Init");

  // Relay setup
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY5, OUTPUT);
  pinMode(RELAY6, OUTPUT);
  turnOffRelays();  // Ensure all relays are OFF initially
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Read sensor data
  readDHT();
  readLight();
  if (timer3Detik.TRIGGERED) {
    readNPK(sensor1, "Sensor 1", sensor1Data);
    readNPK(sensor2, "Sensor 2", sensor2Data);
  }

  // Publish sensor data every 30 seconds
  if (timer30Detik.TRIGGERED) {
    publishData();
    publishDataNPK("Sensor 1", topicNpk1, sensor1Data); // publish data npk1
    publishDataNPK("Sensor 2", topicNpk2, sensor2Data); // publish data npk1
  }

  if (timer5Detik.TRIGGERED) {
    lcd.clear();
    updateLCD();
    npkLCD(1, sensor1Data);
  }

  if (timer10Detik.TRIGGERED) {
    lcd.clear();
    updateLCD();
    npkLCD(2, sensor2Data);
  }
}

// Read data from DHT sensor
void readDHT() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Temperature: ");
  Serial.println(temperature);
}

// Read data from BH1750 sensor
void readLight() {
  lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
}

// Display sensor readings on LCD
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print((int)temperature);
  lcd.print("C ");
  lcd.print("H:");
  lcd.print((int)humidity);
  lcd.print("%");

  lcd.setCursor(12, 0);
  lcd.print("Lux:");
  lcd.print((int)lux);
}

void npkLCD(int sensorNumber, uint16_t* data) {
  lcd.setCursor(19, 3);
  lcd.print(sensorNumber);

  lcd.setCursor(0, 1);
  lcd.print("SH:");
  lcd.print(data[0]);
  lcd.setCursor(8, 1);
  lcd.print("% ST:");
  lcd.print(data[1]);
  lcd.print("C ");

  lcd.setCursor(0, 2);
  lcd.print("SC:");
  lcd.print(data[2]);
  lcd.setCursor(8, 2);
  lcd.print("uS SpH:");
  lcd.print(data[3]);

  lcd.setCursor(0, 3);
  lcd.print("N:");
  lcd.print(data[4]);
  lcd.setCursor(5, 3);
  lcd.print(" P:");
  lcd.print(data[12]);
  lcd.print(" K:");
  lcd.print(data[6]);
}
// Publish sensor data to MQTT broker
void publishData() {
  DynamicJsonDocument json(1024);
  json["viciTemperature"] = temperature;
  json["viciHumidity"] = humidity;
  json["viciLuminosity"] = lux;

  char buffer[256];
  size_t n = serializeJson(json, buffer);
  client.publish(topicBHT, buffer, n);
}

void publishDataNPK(const char* sensorName, const char* topic, uint16_t* data) {
  StaticJsonDocument<200> dataNPK;

  dataNPK["npkName"] = sensorName;
  dataNPK["soilHumidity"] = data[0];
  dataNPK["soilTemperature"] = data[1];
  dataNPK["soilConductivity"] = data[2];
  dataNPK["soilPh"] = data[3];
  dataNPK["soilNitrogen"] = data[4];
  dataNPK["soilPhosphorus"] = data[5];
  dataNPK["soilPotassium"] = data[6];

  String jsonNpk;
  serializeJson(dataNPK, jsonNpk);
  client.publish(topic, jsonNpk.c_str());
  Serial.print("Published data to topic: ");
  Serial.println(topic);
  Serial.println(jsonNpk);
}

// Function to handle relay control via MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (strcmp(topic, topicRelay1) == 0) {
    if (message == "0") {
      digitalWrite(RELAY1, HIGH);
    } else if (message == "1") {
      digitalWrite(RELAY1, LOW);
    }
  } else if (strcmp(topic, topicRelay2) == 0) {
    if (message == "0") {
      digitalWrite(RELAY2, HIGH);
    } else if (message == "1") {
      digitalWrite(RELAY2, LOW);
    }
  } else if (strcmp(topic, topicRelay3) == 0) {
    if (message == "0") {
      digitalWrite(RELAY3, HIGH);
    } else if (message == "1") {
      digitalWrite(RELAY3, LOW);
    }
  } else if (strcmp(topic, topicRelay4) == 0) {
    if (message == "0") {
      digitalWrite(RELAY4, HIGH);
    } else if (message == "1") {
      digitalWrite(RELAY4, LOW);
    }
  } else if (strcmp(topic, topicRelay5) == 0) {
    if (message == "0") {
      digitalWrite(RELAY5, HIGH);
    } else if (message == "1") {
      digitalWrite(RELAY5, LOW);
    }
  } else if (strcmp(topic, topicRelay6) == 0) {
    if (message == "0") {
      digitalWrite(RELAY6, HIGH);
    } else if (message == "1") {
      digitalWrite(RELAY6, LOW);
    }
  }
}

// Connect to WiFi
void KoneksiWIFI() {
  Serial.print("Konek ke: ");
  Serial.println(wifiName);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiName, wifiPass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Reconnect to MQTT broker
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "SMART FARMING-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), brokerUser, brokerPass)) {
      Serial.println("connected");
      client.subscribe(topicRelay1, 0);
      client.subscribe(topicRelay2, 0);
      client.subscribe(topicRelay3, 0);
      client.subscribe(topicRelay4, 0);
      client.subscribe(topicRelay5, 0);
      client.subscribe(topicRelay6, 0);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Turn off all relays
void turnOffRelays() {
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);
  digitalWrite(RELAY3, HIGH);
  digitalWrite(RELAY4, HIGH);
  digitalWrite(RELAY5, HIGH);
  digitalWrite(RELAY6, HIGH);
}

// Read NPK sensor data
void readNPK(ModbusMaster &sensor, const char* sensorName, uint16_t* data) {
  uint8_t result = sensor.readHoldingRegisters(0x0000, 7);

  if (result == sensor.ku8MBSuccess) {
    data[0] = sensor.getResponseBuffer(0x00);  // Humidity
    data[1] = sensor.getResponseBuffer(0x01);  // Temperature
    data[2] = sensor.getResponseBuffer(0x02);  // Conductivity
    data[3] = sensor.getResponseBuffer(0x03);  // PH
    data[4] = sensor.getResponseBuffer(0x04);  // Nitrogen (N)
    data[5] = sensor.getResponseBuffer(0x05);  // Phosphorus (P)
    data[6] = sensor.getResponseBuffer(0x06);  // Potassium (K)

    Serial.print(sensorName);
    Serial.print(" - Soil Humidity: ");
    Serial.print(data[0] * 0.1);
    Serial.println(" %RH");

    Serial.print(sensorName);
    Serial.print(" - Soil Temperature: ");
    Serial.print(data[1] * 0.1);
    Serial.println(" °C");

    Serial.print(sensorName);
    Serial.print(" - Soil Conductivity: ");
    Serial.print(data[2]);
    Serial.println(" us/cm");

    Serial.print(sensorName);
    Serial.print(" - Soil PH: ");
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