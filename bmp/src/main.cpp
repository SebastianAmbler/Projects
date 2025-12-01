// Stability-improved BME280 reader
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

// Configurable parameters
const float SEALEVEL_PRESSURE = 1013.25f; // hPa — change to your local mean sea-level pressure if known
const unsigned long SAMPLE_INTERVAL_MS = 1000;
const int AVG_SIZE = 8; // smoothing window (power-of-two-ish is nice but not required)
const int MAX_INIT_ATTEMPTS = 5;
const int REINIT_AFTER_FAILURES = 5; // consecutive bad reads before trying re-init

// Rolling buffers for simple moving average
float tempBuf[AVG_SIZE];
float presBuf[AVG_SIZE];
float humBuf[AVG_SIZE];
float altBuf[AVG_SIZE];
int bufIndex = 0;
int bufCount = 0;

unsigned long lastSample = 0;
int consecutiveBadReads = 0;
bool sensorPresent = false;

bool initSensor() {
  for (int attempt = 1; attempt <= MAX_INIT_ATTEMPTS; ++attempt) {
    if (bme.begin(0x76)) {
      // Configure sampling to reduce noise and improve stability
      // Use a conservative sampling configuration; avoid referencing
      // standby enum variants that may differ between library versions.
      bme.setSampling(Adafruit_BME280::MODE_NORMAL,
              Adafruit_BME280::SAMPLING_X2,  // temp
              Adafruit_BME280::SAMPLING_X16, // press
              Adafruit_BME280::SAMPLING_X1); // hum
      Serial.println("BME280 initialized");
      return true;
    }
    Serial.print("BME280 init attempt ");
    Serial.print(attempt);
    Serial.println(" failed, retrying...");
    delay(200);
  }
  Serial.println("BME280 init failed after attempts");
  return false;
}

void pushSample(float t, float p, float h, float a) {
  tempBuf[bufIndex] = t;
  presBuf[bufIndex] = p;
  humBuf[bufIndex] = h;
  altBuf[bufIndex] = a;

  bufIndex = (bufIndex + 1) % AVG_SIZE;
  if (bufCount < AVG_SIZE) ++bufCount;
}

float average(const float *arr, int count) {
  if (count <= 0) return NAN;
  float s = 0.0f;
  for (int i = 0; i < count; ++i) s += arr[i];
  return s / count;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Try to initialize the sensor; if it fails we'll keep trying from loop
  sensorPresent = initSensor();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSample < SAMPLE_INTERVAL_MS) return; // non-blocking
  lastSample = now;

  if (!sensorPresent) {
    // Try to re-initialize periodically if sensor isn't present
    sensorPresent = initSensor();
    if (!sensorPresent) {
      // Avoid spamming Serial too fast
      delay(500);
      return;
    }
  }

  float temp = bme.readTemperature();      // °C
  float pres = bme.readPressure() / 100.0f; // hPa
  float hum  = bme.readHumidity();         // %
  float altitude = bme.readAltitude(SEALEVEL_PRESSURE);

  // Validate readings
  if (isnan(temp) || isnan(pres) || isnan(hum) || isnan(altitude)) {
    Serial.println("BME280 read returned NaN");
    ++consecutiveBadReads;
    if (consecutiveBadReads >= REINIT_AFTER_FAILURES) {
      Serial.println("Multiple bad reads — attempting sensor re-init");
      sensorPresent = false; // will trigger re-init on next loop
      consecutiveBadReads = 0;
    }
    return;
  }

  // Good read — reset failure counter and record
  consecutiveBadReads = 0;
  pushSample(temp, pres, hum, altitude);

  float tAvg = average(tempBuf, bufCount);
  float pAvg = average(presBuf, bufCount);
  float hAvg = average(humBuf, bufCount);
  float aAvg = average(altBuf, bufCount);

  Serial.print("Temp: ");
  Serial.print(tAvg);
  Serial.print(" C   Pressure: ");
  Serial.print(pAvg);
  Serial.print(" hPa   Humidity: ");
  Serial.print(hAvg);
  Serial.print(" %   Altitude: ");
  Serial.print(aAvg);
  Serial.println(" m");
}
