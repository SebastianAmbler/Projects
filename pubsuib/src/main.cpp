#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <LoRa.h>

// ---------------- WiFi ----------------
const char* ssid = "EMB5326";
const char* password = "cdti12345";

// ---------------- MQTT ----------------
const char* mqtt_broker = "192.168.1.112";
const int   mqtt_port   = 1883;
const char* mqtt_client_id = "LoRaGateway01";
const char* publish_topic = "esp32/lora/g1";   

WiFiClient espClient;
PubSubClient client(espClient);

// ---------------- LoRa Pins (ESP32) ----------------
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS   5
#define LORA_RST  4
#define LORA_DIO0 26

#define LED_PIN 16

// ---------------- LoRa Config ----------------
const long LORA_FREQ = 433E6;
const uint8_t SYNC_WORD = 0x12;

// ---------------- ID ----------------
const char* GROUP_ID = "G1";

// ---------- WiFi connect ----------
void setup_wifi() {
  Serial.print("Connecting WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ---------- MQTT reconnect ----------
void mqtt_reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_client_id)) {   // ถ้ามี user/pass: client.connect(id,user,pass)
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 3s");
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // LoRa init
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (1) delay(1000);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();
  LoRa.setSyncWord(SYNC_WORD);

  Serial.println("LoRa RX Ready");

  // WiFi + MQTT init
  setup_wifi();
  client.setServer(mqtt_broker, mqtt_port);
}

void loop() {
  // MQTT connection
  if (!client.connected()) mqtt_reconnect();
  client.loop();

  // รับ LoRa
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  String msg = LoRa.readString();
  msg.trim();

  Serial.print("RECV = ");
  Serial.println(msg);

  // 1) เช็ก group
  if (!msg.startsWith(String(GROUP_ID) + ",")) {
    Serial.println("Not my group → ignore");
    return;
  }

  // 2) ตัด "G1," ออก เหลือ JSON ล้วน
  int comma = msg.indexOf(',');
  String jsonPart = msg.substring(comma + 1);
  jsonPart.trim();

  // เช็กว่าเป็น JSON จริงไหม
  if (!jsonPart.startsWith("{") || !jsonPart.endsWith("}")) {
    Serial.println("Invalid JSON payload");
    return;
  }

  // 3) ส่ง JSON ขึ้น MQTT ตรง ๆ
  Serial.print("MQTT PUB -> ");
  Serial.println(jsonPart);

  client.publish(publish_topic, jsonPart.c_str());

  // blink LED
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}