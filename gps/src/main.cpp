#include <TinyGPS++.h>

TinyGPSPlus gps;

// UART2 on ESP32
HardwareSerial GPS_Serial(2);

const int GPS_RX = 16;  // ESP32 RX  <- GPS TX
const int GPS_TX = 17;  // ESP32 TX  -> GPS RX
const uint32_t GPS_BAUD = 9600; // try 9600 first

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting GPS...");
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
}

void loop() {
  // 1) Mirror raw data from GPS to USB serial so you can SEE NMEA
  while (GPS_Serial.available()) {
    char c = GPS_Serial.read();
    Serial.write(c);      // show raw GPS sentences like $GNGGA,...
    gps.encode(c);        // feed TinyGPS++
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
