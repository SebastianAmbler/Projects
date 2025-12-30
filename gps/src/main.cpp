#include <Arduino.h> // Required for PlatformIO
#include <TinyGPS++.h>

TinyGPSPlus gps;

HardwareSerial GPS_Serial(1);

const int GPS_RX = 44;  // ESP32 Pin 44 (U0RXD) connected to GPS TX
const int GPS_TX = 43;  // ESP32 Pin 43 (U0TXD) connected to GPS RX
const uint32_t GPS_BAUD = 9600; 

void setup() {
  Serial.begin(115200);
  Serial.println("System Started. Initializing GPS on Pins 43/44...");
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
}

void loop() {
  while (GPS_Serial.available()) {
    char c = GPS_Serial.read();
    
    gps.encode(c);        
  }

  // 2) When location is updated, print parsed info
  if (gps.location.isUpdated()) {
    Serial.println();
    Serial.println(F("==== PARSED DATA ===="));
    Serial.print(F("LAT: "));
    Serial.println(gps.location.lat(), 6);

    Serial.print(F("LON: "));
    Serial.println(gps.location.lng(), 6);

    Serial.print(F("SPEED (km/h): "));
    Serial.println(gps.speed.kmph());

    Serial.print(F("ALT (m): "));
    Serial.println(gps.altitude.meters());

    Serial.print(F("Sats: "));
    Serial.println(gps.satellites.value());

    Serial.println();
  }
}