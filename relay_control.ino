#include <WiFi.h>
#include <PubSubClient.h>

const char* wifiName = "Pasca LAB TI";
const char* wifiPass = "nakamsenjem";

const char* brokerUser = "obyskxhx:obyskxhx";
const char* brokerPass = "Fe_3_tBuwmc8vMMqT2hYiboTsBlBmPz1";
const char* brokerHost = "armadillo.rmq.cloudamqp.com";  // broker

const char* TopicAll = "farm/all";  // TOPIC

const char* topicRelay1 = "relay1";
const char* topicRelay2 = "relay2";
const char* topicRelay3 = "relay3";
const char* topicRelay4 = "relay4";
const char* topicRelay5 = "relay5";
const char* topicRelay6 = "relay6";

// Deklarasi client wifi
WiFiClient espClient;
// Deklarasi MQTT Client
PubSubClient client(espClient);

// GPIO pins for relays
#define RELAY1 32
#define RELAY2 33
#define RELAY3 25
#define RELAY4 26
#define RELAY5 27
#define RELAY6 14

void setup() {
  Serial.begin(9600); //115200

  // Setup relay pins as OUTPUT
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY5, OUTPUT);
  pinMode(RELAY6, OUTPUT);

  // Ensure all relays are OFF initially
  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  digitalWrite(RELAY3, LOW);
  digitalWrite(RELAY4, LOW);
  digitalWrite(RELAY5, LOW);
  digitalWrite(RELAY6, LOW);

  KoneksiWIFI();
  client.setServer(brokerHost, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void KoneksiWIFI() {
  Serial.print("Konek ke: ");
  Serial.println(wifiName);

  // Memposisikan MCU sebagai station
  // MCU dihubungkan ke Router Access Point
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiName, wifiPass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // NodeMCU telah terhubung ke Access Point
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Membentuk koneksi dengan Message Broker
  // dalam hal ini ke server RabbitMQ

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Membangkitkan ID client acak ke Message Broker
    String clientId = "DSPTECH-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), brokerUser, brokerPass)) {
      Serial.println("connected");
      client.subscribe(topicRelay1);
      client.subscribe(topicRelay2);
      client.subscribe(topicRelay3);
      client.subscribe(topicRelay4);
      client.subscribe(topicRelay5);
      client.subscribe(topicRelay6);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message received: ");
  Serial.println(message);

  if (strcmp(topic, topicRelay1) == 0) {
    if (message == "ON") {
      digitalWrite(RELAY1, HIGH);
    } else if (message == "OFF") {
      digitalWrite(RELAY1, LOW);
    }
  } else if (strcmp(topic, topicRelay2) == 0) {
    if (message == "ON") {
      digitalWrite(RELAY2, HIGH);
    } else if (message == "OFF") {
      digitalWrite(RELAY2, LOW);
    }
  } else if (strcmp(topic, topicRelay3) == 0) {
    if (message == "ON") {
      digitalWrite(RELAY3, HIGH);
    } else if (message == "OFF") {
      digitalWrite(RELAY3, LOW);
    }
  } else if (strcmp(topic, topicRelay4) == 0) {
    if (message == "ON") {
      digitalWrite(RELAY4, HIGH);
    } else if (message == "OFF") {
      digitalWrite(RELAY4, LOW);
    }
  } else if (strcmp(topic, topicRelay5) == 0) {
    if (message == "ON") {
      digitalWrite(RELAY5, HIGH);
    } else if (message == "OFF") {
      digitalWrite(RELAY5, LOW);
    }
  } else if (strcmp(topic, topicRelay6) == 0) {
    if (message == "ON") {
      digitalWrite(RELAY6, HIGH);
    } else if (message == "OFF") {
      digitalWrite(RELAY6, LOW);
    }
  }
}