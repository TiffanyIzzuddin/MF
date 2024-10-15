
#include <BH1750.h>
#include <Wire.h>
//#include <PubSubClient.h>

BH1750 lightMeter;


void setup() {
  Serial.begin(9600);
  // Initialize
  Wire.begin();
  // On esp8266 you can select SDA and SCL pins using Wire.begin(D4, D3);
  // For Wemos / Lolin D1 Mini Pro and the Ambient Light shield use Wire.begin(D2, D1);

  lightMeter.begin();
  Serial.println(F("BH1750 Test begin"));
}

void loop() {
  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  delay(1000);

}
