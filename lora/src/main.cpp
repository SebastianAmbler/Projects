#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- OLED Config ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
const char* GROUP_ID = "G7";
const char* deviceId = "Sudhlor";

unsigned long startTime = 0;
const unsigned long SEND_INTERVAL = 1000;
uint32_t counter = 0;

Adafruit_BME280 bme;

bool initSensor() {
  for (int attempt = 1; attempt <= 100; ++attempt) {
    if (bme.begin(0x76)) {
      bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,
        Adafruit_BME280::SAMPLING_X2,
        Adafruit_BME280::SAMPLING_X16,
        Adafruit_BME280::SAMPLING_X1
      );
      Serial.println("BME280 initialized");
      return true;
    }
    Serial.print("BME280 init attempt ");
    Serial.print(attempt);
    Serial.println(" failed, retrying...");
    delay(200);
  }
  Serial.println("BME280 init failed after all attempts");
  return false;
}

void updateDisplay(float temp, float hum, float press, uint32_t cnt) {
  display.clearDisplay();
  
  // Header
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(GROUP_ID);
  display.print(" - ");
  display.println(deviceId);
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // Temperature
  display.setCursor(0, 14);
  display.print("Temp: ");
  display.print(temp, 1);
  display.println(" C");
  
  // Humidity
  display.setCursor(0, 24);
  display.print("Hum:  ");
  display.print(hum, 1);
  display.println(" %");
  
  // Pressure
  display.setCursor(0, 34);
  display.print("Pres: ");
  display.print(press / 100.0, 1);
  display.println(" hPa");
  
  // Counter
  display.setCursor(0, 44);
  display.print("Packets: ");
  display.println(cnt);
  
  // Status indicator
  display.setCursor(0, 54);
  display.print("TX: ");
  display.println(millis() / 500 % 2 ? ">>>>" : "    ");
  
  display.display();
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();
  
  // Initialize SPI & LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("LoRa FAILED!");
    display.display();
    while (1) delay(1000);
  }
  
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();
  LoRa.setSyncWord(SYNC_WORD);
  Serial.println("LoRa TX Ready");
  
  // Initialize BME280
  bool sensorPresent = initSensor();
  
  if (!sensorPresent) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("BME280 FAILED!");
    display.display();
    while (1) delay(1000);
  }
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ready!");
  display.display();
  delay(1000);
}

void loop() {
  unsigned long now = millis();
  
  if (now - startTime > SEND_INTERVAL) {
    startTime = now;
    
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float press = bme.readPressure();
    
    String payload = String(GROUP_ID) + "," + deviceId + "," + 
                     String(temp) + " °C," +
                     String(hum) + " %," + 
                     String(press) + " Pa," + 
                     String(counter);
    
    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();
    
    Serial.print("Sent: ");
    Serial.println(payload);
    
    // Update OLED display
    updateDisplay(temp, hum, press, counter);
    
    counter++;
    
    // Toggle LED
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}