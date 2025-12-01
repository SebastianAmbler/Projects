#include <Arduino.h>
#include "MPU9250_SPI.h"
#include "MadgwickAHRS.h"

// ===========================
// Customizable SPI Pins
// ===========================
#define SPI_MOSI_PIN 11   // GPIO 35 (default MOSI for ESP32-S3) - Change this
#define SPI_MISO_PIN 13   // GPIO 37 (default MISO for ESP32-S3) - Change this
#define SPI_SCK_PIN  12   // GPIO 36 (default SCK for ESP32-S3)  - Change this
#define SPI_CS_PIN   10   // GPIO 34 (Chip Select)              - Change this

// ===========================
// MPU9250 Configuration
// ===========================
MPU9250_SPI imu(SPI_CS_PIN);
Madgwick ahrs(100.0f, 0.08f); // lower beta => smoother but slower convergence

// Magnetic declination in degrees (adjust for your location)
// Find yours at: https://www.ngdc.noaa.gov/geomag/declination.shtml
#define MAGNETIC_DECLINATION 0.0f // degrees (0 = magnetic north = true north)

// Helper: convert heading to compass direction
const char* getCompassDirection(float heading) {
  if (heading >= 337.5f || heading < 22.5f) return "N";
  if (heading >= 22.5f && heading < 67.5f) return "NE";
  if (heading >= 67.5f && heading < 112.5f) return "E";
  if (heading >= 112.5f && heading < 157.5f) return "SE";
  if (heading >= 157.5f && heading < 202.5f) return "S";
  if (heading >= 202.5f && heading < 247.5f) return "SW";
  if (heading >= 247.5f && heading < 292.5f) return "W";
  if (heading >= 292.5f && heading < 337.5f) return "NW";
  return "?";
}

// Magnetometer sensitivity adjustments (from AK8963 Fuse ROM)
float magAdjust[3] = {1.0f, 1.0f, 1.0f};

// MPU9250 register constants (used to configure I2C master)
const uint8_t REG_USER_CTRL = 0x6A;
const uint8_t REG_I2C_MST_CTRL = 0x24;
const uint8_t REG_I2C_SLV0_ADDR = 0x25;
const uint8_t REG_I2C_SLV0_REG = 0x26;
const uint8_t REG_I2C_SLV0_DO = 0x63;
const uint8_t REG_I2C_SLV0_CTRL = 0x27;
const uint8_t REG_EXT_SENS_DATA_00 = 0x49;

const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D; // 400 kHz approx in many libs
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;

// AK8963 I2C address and registers
const uint8_t AK8963_ADDR = 0x0C;
const uint8_t AK8963_REG_WIA = 0x00;
const uint8_t AK8963_REG_ST1 = 0x02;
const uint8_t AK8963_REG_HXL = 0x03;
const uint8_t AK8963_REG_ST2 = 0x09;
const uint8_t AK8963_REG_CNTL1 = 0x0A;
const uint8_t AK8963_REG_ASAX = 0x10;

// Helpers that use the MPU's I2C master to access AK8963 (no Wire)
bool akWriteViaMPU(uint8_t reg, uint8_t val) {
  // Set slave0 to write 1 byte to AK8963 register
  if (!imu.writeMPURegister(REG_I2C_SLV0_ADDR, AK8963_ADDR)) return false;
  if (!imu.writeMPURegister(REG_I2C_SLV0_REG, reg)) return false;
  if (!imu.writeMPURegister(REG_I2C_SLV0_DO, val)) return false;
  // enable transfer of 1 byte
  if (!imu.writeMPURegister(REG_I2C_SLV0_CTRL, I2C_SLV0_EN | 1)) return false;
  delay(10);
  return true;
}

bool akReadViaMPU(uint8_t reg, uint8_t* buf, uint8_t len) {
  // Configure slave0 to read `len` bytes from AK8963 starting at `reg`
  if (!imu.writeMPURegister(REG_I2C_SLV0_ADDR, AK8963_ADDR | I2C_READ_FLAG)) return false;
  if (!imu.writeMPURegister(REG_I2C_SLV0_REG, reg)) return false;
  if (!imu.writeMPURegister(REG_I2C_SLV0_CTRL, I2C_SLV0_EN | len)) return false;
  // Give MPU time to perform the I2C read and populate EXT_SENS_DATA
  delay(10);
  // Read from EXT_SENS_DATA_00
  return imu.readMPURegister(REG_EXT_SENS_DATA_00, buf, len);
}

// Function to initialize SPI and MPU9250
void setupMPU9250() {
  // Configure SPI pins
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
  
  // Initialize MPU9250
  if (!imu.begin()) {
    Serial.println("MPU9250 initialization failed!");
    while (1) {
      delay(1000);
    }
  } else {
    Serial.println("MPU9250 initialized successfully!");
    uint8_t whoAmI = imu.whoAmI();
    Serial.print("WHO_AM_I: 0x");
    Serial.println(whoAmI, HEX);
    // Configure MPU to act as I2C master so AK8963 can be accessed via SPI only
    // Enable I2C master
    imu.writeMPURegister(REG_USER_CTRL, I2C_MST_EN);
    // Set I2C master clock speed
    imu.writeMPURegister(REG_I2C_MST_CTRL, I2C_MST_CLK);

    // AK8963 init via MPU I2C master (no MCU Wire use):
    // 1) Power down
    akWriteViaMPU(AK8963_REG_CNTL1, 0x00);
    delay(10);
    // 2) Enter Fuse ROM access mode
    akWriteViaMPU(AK8963_REG_CNTL1, 0x0F);
    delay(10);
    // 3) Read ASA (ASAX..ASAZ)
    uint8_t asa[3] = {0};
    if (akReadViaMPU(AK8963_REG_ASAX, asa, 3)) {
      for (int i = 0; i < 3; ++i) {
        magAdjust[i] = (((float)asa[i] - 128.0f) / 256.0f) + 1.0f;
      }
      Serial.printf("AK8963 ASA: %u %u %u -> adj: %.3f %.3f %.3f\n",
                    asa[0], asa[1], asa[2], magAdjust[0], magAdjust[1], magAdjust[2]);
    } else {
      Serial.println("Failed to read AK8963 ASA (via MPU)");
    }
    // 4) Power down
    akWriteViaMPU(AK8963_REG_CNTL1, 0x00);
    delay(10);
    // 5) Set continuous measurement mode 2, 16-bit
    akWriteViaMPU(AK8963_REG_CNTL1, 0x16);
    delay(10);
    // 6) Configure slave0 to continuously read 7 bytes from AK8963 HXL
    imu.writeMPURegister(REG_I2C_SLV0_ADDR, AK8963_ADDR | I2C_READ_FLAG);
    imu.writeMPURegister(REG_I2C_SLV0_REG, AK8963_REG_HXL);
    imu.writeMPURegister(REG_I2C_SLV0_CTRL, I2C_SLV0_EN | 7);
  }
}

// Function to read and print IMU data
void readMPU9250() {
  // Raw accelerometer and gyroscope data
  int16_t accel[3], gyro[3];

  // Read accel/gyro
  if (imu.readAccelGyro(accel, gyro)) {
    Serial.printf("Accel: [%6d, %6d, %6d] ", accel[0], accel[1], accel[2]);
    Serial.printf("| Gyro: [%6d, %6d, %6d] ", gyro[0], gyro[1], gyro[2]);

    // Convert accel to g, gyro to rad/s for Madgwick
    const float ACCEL_SCALE = 16384.0f; // LSB/g for ±2g
    const float GYRO_SCALE = 131.0f;    // LSB/(deg/s) for ±250 dps
    float ax_g = (float)accel[0] / ACCEL_SCALE;
    float ay_g = (float)accel[1] / ACCEL_SCALE;
    float az_g = (float)accel[2] / ACCEL_SCALE;
    float gx_rads = ((float)gyro[0] / GYRO_SCALE) * (3.14159265358979323846f/180.0f);
    float gy_rads = ((float)gyro[1] / GYRO_SCALE) * (3.14159265358979323846f/180.0f);
    float gz_rads = ((float)gyro[2] / GYRO_SCALE) * (3.14159265358979323846f/180.0f);

    // Read magnetometer (AK8963) via MPU's EXT_SENS_DATA_00 (populated by I2C master)
    uint8_t raw[7];
    if (imu.readMPURegister(REG_EXT_SENS_DATA_00, raw, 7)) {
      // ST2 is raw[6], check overflow bit
      if (!(raw[6] & 0x08)) {
        int16_t mx = (int16_t)((raw[1] << 8) | raw[0]);
        int16_t my = (int16_t)((raw[3] << 8) | raw[2]);
        int16_t mz = (int16_t)((raw[5] << 8) | raw[4]);
        // Convert to microtesla using sensitivity and ASA adjustments
        const float MAG_SCALE = 4912.0f / 32760.0f; // µT per LSB for 16-bit
        float magx = mx * MAG_SCALE * magAdjust[0];
        float magy = my * MAG_SCALE * magAdjust[1];
        float magz = mz * MAG_SCALE * magAdjust[2];

        // Update Madgwick filter
        ahrs.update(gx_rads, gy_rads, gz_rads, ax_g, ay_g, az_g, magx, magy, magz);

        // Get magnetic heading from Madgwick (yaw)
        float magHeading = ahrs.getHeading();

        // Apply magnetic declination to get true heading
        float trueHeading = magHeading + MAGNETIC_DECLINATION;
        if (trueHeading >= 360.0f) trueHeading -= 360.0f;
        if (trueHeading < 0.0f) trueHeading += 360.0f;

        // Get compass bearing
        const char* bearing = getCompassDirection(trueHeading);

        // Calculate total magnetic field strength
        float magStrength = sqrtf(magx*magx + magy*magy + magz*magz);

        Serial.printf("| Heading: %.1f° [%s] (Mag: %.1f µT)\n", trueHeading, bearing, magStrength);
      } else {
        Serial.println("| Mag: overflow (ST2)");
      }
    } else {
      Serial.println("| Mag: read failed (EXT_SENS_DATA)");
    }
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== MPU9250 SPI Test ===");
  Serial.printf("SPI Config: MOSI=%d, MISO=%d, SCK=%d, CS=%d\n", 
                SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN, SPI_CS_PIN);
  
  // Initialize MPU9250
  setupMPU9250();
  
  Serial.println("Setup complete!\n");
}

void loop() {
  // Read and print MPU9250 data every 100ms
  readMPU9250();
  delay(100);
}
