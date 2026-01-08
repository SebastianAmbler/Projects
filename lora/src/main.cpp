#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// ---------------- LoRa Pins (ESP32) ----------------
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 4
#define LORA_DIO0 26
#define LED_PIN 15

// ---------------- LoRa Config ----------------
const long LORA_FREQ = 433E6;
const uint8_t SYNC_WORD = 0x12;

// ---------------- ID ----------------
const char* GROUP_ID = "G7";  // ต้องตรงกับฝั่งส่ง
const char* deviceId = "Sudhlor";
unsigned long startTime = 0;
const unsigned long SEND_INTERVAL = 1000;  // ส่งทุก 2 วินาที
uint32_t counter = 0;

Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // เริ่ม SPI & LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.begin(LORA_FREQ);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (1) delay(1000);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();
  LoRa.setSyncWord(SYNC_WORD);

  Serial.println("LoRa TX Ready");

 // LoRa.onReceive(onReceive);
 // LoRa.receive();
 unsigned status;
    
    // default settings
    status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
}

void loop() {
  unsigned long now = millis();
  if (now - startTime > SEND_INTERVAL) {
    startTime = now;
    
    String payload = String(GROUP_ID) + "," + deviceId + "," + bme.readTemperature() + " °C," +
    bme.readHumidity() + " %," + bme.readPressure() + " Pa," + String(counter++);

    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();

    Serial.print("Sent: ");
    Serial.println(payload);

    // กระพริบ LED ให้รู้ว่ามีการส่ง
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

// void onReceive(int packetSize) {
//   if (packetSize == 0) return;
//   String msg = "";

//   while (LoRa.available()) {
//     msg += (char)LoRa.read();
//   }

//   msg.trim();

//   if (!msg.startsWith("G1")) {
//     Serial.println("Ignore other group");
//     return;
//   }

//   Serial.print("My Group Packet: ");
//   Serial.println(msg);
//   Serial.print("RSSI=");
//   Serial.println(LoRa.packetRssi());
//   Serial.print("Snr=");
//   Serial.println(LoRa.packetSnr());
//   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
// }
