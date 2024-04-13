
#include <WiFi.h>
#include <PubSubClient.h>

#include "Adafruit_SHT31.h"

const char* ssid = "isaknarin";
const char* password = "12345678";

#define mqttServer "broker.emqx.io"
#define mqttPort 1883
#define mqttUser ""
#define mqttPassword ""

#define R1 32
#define R2 14

String msg;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
WiFiClient wificlient;
PubSubClient mqttClient(wificlient);
int myTime = 0;
void setup() {
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  digitalWrite(R1, HIGH);
  digitalWrite(R2, LOW);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("SHT31 test");
  if (!sht31.begin(0x44)) {  // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  setupMQTT();
}

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  if ((millis() - myTime) > 10000) {  // 10S = 10000. 22000-11000 > 10000 = true
    //mqttClient.publish("CPE345IoT/65051443/msg/spu", String(millis() / 1000).c_str());
    read();
    myTime = millis();
  }
  mqttClient.loop();
  delay(1000);
}

void read() {
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  String data = "Temp: " + String(t) + " Humidity: " + String(h);
  if (millis() - myTime > 10000) {
    mqttClient.publish("CPE345IoT/65051443/msg/spu", String(data).c_str());
    myTime = millis();  //11000
  }
}

void setupMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
    Serial.println("Reconnecting to MQTT Broker..");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("Connected.");
      Serial.println(clientId);
      // subscribe to topic
      mqttClient.subscribe("CPE345IoT/65051443/msg/relay");
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String sTopic = String(topic);
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    msg += (char)payload[i];
  }
  if (sTopic == "CPE345IoT/65051443/msg/relay") {
    if (msg == "ON1") {
      digitalWrite(R1, LOW);
    } else if (msg == "OFF1") {
      digitalWrite(R1, HIGH);
    }
    if (msg == "ON2") {
      digitalWrite(R2, LOW);
    } else if (msg == "OFF2") {
      digitalWrite(R2, HIGH);
    }
  }

  msg = "";
  Serial.println("\n======================================");
}