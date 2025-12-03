// Stable BME280 reader with moving average + auto re-init
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

// ===== Configurable parameters =====
const float SEALEVEL_PRESSURE = 1013.25f;      // hPa — change to your local mean sea-level pressure
const unsigned long SAMPLE_INTERVAL_MS = 1000; // time between output updates
const int AVG_SIZE = 8;                        // smoothing window
const int MAX_INIT_ATTEMPTS = 5;              // tries per init
const int REINIT_AFTER_FAILURES = 5;          // consecutive bad reads before re-init

// ===== Rolling buffers for moving average =====
float tempBuf[AVG_SIZE];
float presBuf[AVG_SIZE];
float humBuf[AVG_SIZE];
float altBuf[AVG_SIZE];
int bufIndex = 0;
int bufCount = 0;

unsigned long lastSample = 0;
int consecutiveBadReads = 0;
bool sensorPresent = false;

// ===== Functions =====
bool initSensor() {
  for (int attempt = 1; attempt <= MAX_INIT_ATTEMPTS; ++attempt) {
    if (bme.begin(0x76)) {
      // Configure BME280 sampling (stable, not too noisy)
      bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,
        Adafruit_BME280::SAMPLING_X2,   // temperature oversampling
        Adafruit_BME280::SAMPLING_X16,  // pressure oversampling
        Adafruit_BME280::SAMPLING_X1    // humidity oversampling
        // Filter & standby left at defaults
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

void pushSample(float t, float p, float h, float a) {
  tempBuf[bufIndex] = t;
  presBuf[bufIndex] = p;
  humBuf[bufIndex]  = h;
  altBuf[bufIndex]  = a;

  bufIndex = (bufIndex + 1) % AVG_SIZE;
  if (bufCount < AVG_SIZE) {
    ++bufCount;
  }
}

float average(const float *arr, int count) {
  if (count <= 0) return NAN;
  float s = 0.0f;
  for (int i = 0; i < count; ++i) {
    s += arr[i];
  }
  return s / count;
}

// ===== Arduino setup/loop =====
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Try to initialize the sensor; if it fails, loop() will retry
  sensorPresent = initSensor();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSample < SAMPLE_INTERVAL_MS) {
    return; // non-blocking timing
  }
  lastSample = now;

  // If sensor was not found or lost, try to re-init
  if (!sensorPresent) {
    sensorPresent = initSensor();
    if (!sensorPresent) {
      // Avoid spamming the serial output
      delay(500);
      return;
    }
  }

  // Read sensor
  float temp     = bme.readTemperature();              // °C
  float pres     = bme.readPressure() / 100.0f;        // hPa
  float hum      = bme.readHumidity();                 // %
  float altitude = bme.readAltitude(SEALEVEL_PRESSURE); // meters

  // Validate readings
  if (isnan(temp) || isnan(pres) || isnan(hum) || isnan(altitude)) {
    Serial.println("BME280 read returned NaN");
    ++consecutiveBadReads;

    if (consecutiveBadReads >= REINIT_AFTER_FAILURES) {
      Serial.println("Multiple bad reads — attempting sensor re-init");
      sensorPresent = false; // force re-init next loop
      consecutiveBadReads = 0;
    }
    return;
  }

  // Good read
  consecutiveBadReads = 0;
  pushSample(temp, pres, hum, altitude);

  float tAvg = average(tempBuf, bufCount);
  float pAvg = average(presBuf, bufCount);
  float hAvg = average(humBuf, bufCount);
  float aAvg = average(altBuf, bufCount);

  // Output smoothed values
  Serial.print("Temp: ");
  Serial.print(tAvg, 2);
  Serial.print(" °C   Pressure: ");
  Serial.print(pAvg, 2);
  Serial.print(" hPa   Humidity: ");
  Serial.print(hAvg, 2);
  Serial.print(" %   Altitude: ");
  Serial.print(aAvg, 2);
  Serial.println(" m");
}
