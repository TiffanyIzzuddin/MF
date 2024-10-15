#include <LiquidCrystal_I2C.h> 

// Inisialisasi LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

float lux = 0;
float temperature = 0;
float humidity = 0; 

void setup() {
  Serial.begin(9600);

  // Inisialisasi LCD
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Farming Init");
}

void loop() {
  // put your main code here, to run repeatedly:

}

void LCD() {
  // Baris pertama: Sensor DHT (Temperature dan Humidity)
  if (isnan(temperature) || isnan(humidity)) {
    lcd.setCursor(0, 0);
    lcd.print("Error DHT");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print((int)temperature);
    lcd.print("C ");
    lcd.print("H:");
    lcd.print((int)humidity);
    lcd.print("%");
  }

  // Baris kedua: Sensor BH1750 (Lux)
  if (isnan(lux)) {
    lcd.setCursor(0, 0);
    lcd.print("Error BH1750");
  } else {
    lcd.setCursor(12, 0);
    lcd.print("Lux:");
    lcd.print((int)lux);
  }

  lcd.clear();
  delay(3000);
}
