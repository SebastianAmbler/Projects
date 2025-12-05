#include <Wire.h>
#include "MPU9250.h"

// AHRS filters
#include <MadgwickAHRS.h>
#include <MahonyAHRS.h>


MPU9250 IMU(SPI, 10);

// ===================== CALIBRATION CONSTANTS =====================
// Accel (g)
const float AX_BIAS_G = 0.21f;
const float AY_BIAS_G = 0.16f;
const float AZ_BIAS_G = -0.17f;

const float AX_SCALE = 1.00f;
const float AY_SCALE = 1.00f;
const float AZ_SCALE = 0.99f;

// Gyro (rad/s)
const float GXB_RADPS = 0.01f;
const float GYB_RADPS = 0.04f;
const float GZB_RADPS = -0.01f;

// Mag (uT)
const float MX_BIAS_UT = 33.17f;
const float MY_BIAS_UT = 20.35f;
const float MZ_BIAS_UT = -25.80f;

const float MX_SCALE = 1.03f;
const float MY_SCALE = 0.97f;
const float MZ_SCALE = 1.00f;

const float G_TO_MPS2 = 9.80665f;
const float RADTODEG = 180.0f / 3.1415926535f;

const float SAMPLE_FREQ_HZ = 50.0f;

// Filters
Madgwick madgwick;
Mahony mahony;

// Timing
unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL_MS = 100; // 10 Hz

// Offsets (roll + pitch only)
bool offsetsSet = false;
float rollOffset = 0.0f;
float pitchOffset = 0.0f;

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Initializing MPU9250...");

  int status = IMU.begin();
  if (status < 0) {
    Serial.print("IMU init failed: ");
    Serial.println(status);
    while (1) {}
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  IMU.setSrd(19); // 50 Hz

  madgwick.begin(SAMPLE_FREQ_HZ);
  mahony.begin(SAMPLE_FREQ_HZ);

  Serial.println("Ready. Yaw NOT zeroed. Roll/Pitch will zero after 2 sec.");
  Serial.println();
}

// ===================== LOOP =====================
void loop() {
  IMU.readSensor();

  float raw_ax = IMU.getAccelX_mss();
  float raw_ay = IMU.getAccelY_mss();
  float raw_az = IMU.getAccelZ_mss();

  float raw_gx = IMU.getGyroX_rads();
  float raw_gy = IMU.getGyroY_rads();
  float raw_gz = IMU.getGyroZ_rads();

  float raw_mx = IMU.getMagX_uT();
  float raw_my = IMU.getMagY_uT();
  float raw_mz = IMU.getMagZ_uT();

  // === Apply calibration ===
  float ax = (raw_ax - AX_BIAS_G * G_TO_MPS2) * AX_SCALE;
  float ay = (raw_ay - AY_BIAS_G * G_TO_MPS2) * AY_SCALE;
  float az = (raw_az - AZ_BIAS_G * G_TO_MPS2) * AZ_SCALE;

  float gx = raw_gx - GXB_RADPS;
  float gy = raw_gy - GYB_RADPS;
  float gz = raw_gz - GZB_RADPS;

  float mx = (raw_mx - MX_BIAS_UT) * MX_SCALE;
  float my = (raw_my - MY_BIAS_UT) * MY_SCALE;
  float mz = (raw_mz - MZ_BIAS_UT) * MZ_SCALE;

  // === Axis remap (IMU rotated 90° so +Y is forward) ===
  // Also flip Z so gravity is +Z for AHRS
  float ax_f =  ay;
  float ay_f = -ax;
  float az_f = -az;

  float gx_f =  gy;
  float gy_f = -gx;
  float gz_f = -gz;

  float mx_f =  my;
  float my_f = -mx;
  float mz_f = -mz;

  // === Convert units for filter ===
  float ax_g = ax_f / G_TO_MPS2;
  float ay_g = ay_f / G_TO_MPS2;
  float az_g = az_f / G_TO_MPS2;

  float gx_dps = gx_f * RADTODEG;
  float gy_dps = gy_f * RADTODEG;
  float gz_dps = gz_f * RADTODEG;

  // === Update AHRS ===
  madgwick.update(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, mx_f, my_f, mz_f);
  mahony.update(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, mx_f, my_f, mz_f);

  float rollMad  = madgwick.getRoll();
  float pitchMad = madgwick.getPitch();
  float yawMad   = madgwick.getYaw(); // NOT zeroed

  float rollMah  = mahony.getRoll();
  float pitchMah = mahony.getPitch();
  float yawMah   = mahony.getYaw();   // NOT zeroed

  // === Zero roll + pitch ONLY ===
  static unsigned long startTime = millis();
  if (!offsetsSet && millis() - startTime > 2000) {
    rollOffset  = rollMad;
    pitchOffset = pitchMad;
    offsetsSet = true;
  }

  float rollMadZero  = rollMad  - rollOffset;
  float pitchMadZero = pitchMad - pitchOffset;

  float rollMahZero  = rollMah  - rollOffset;
  float pitchMahZero = pitchMah - pitchOffset;

  // === Printing at 10 Hz ===
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;

    Serial.println("------------- IMU (Robot Frame) -------------");

    Serial.print("Accel[m/s^2]: ");
    Serial.print(ax_f,3); Serial.print("\t");
    Serial.print(ay_f,3); Serial.print("\t");
    Serial.println(az_f,3);

    Serial.print("Gyro [rad/s]: ");
    Serial.print(gx_f,3); Serial.print("\t");
    Serial.print(gy_f,3); Serial.print("\t");
    Serial.println(gz_f,3);

    Serial.println("\n----- Madgwick (deg) -----");
    Serial.print("Roll  : "); Serial.print(rollMadZero,2);
    Serial.print("\tPitch : "); Serial.print(pitchMadZero,2);
    Serial.print("\tYaw   : "); Serial.println(yawMad,2);   // ABSOLUTE yaw
    Serial.println();

    Serial.println("----- Mahony (deg) -----");
    Serial.print("Roll  : "); Serial.print(rollMahZero,2);
    Serial.print("\tPitch : "); Serial.print(pitchMahZero,2);
    Serial.print("\tYaw   : "); Serial.println(yawMah,2);   // ABSOLUTE yaw
    Serial.println("---------------------------------------------\n");
  }

  delay(1);
}
