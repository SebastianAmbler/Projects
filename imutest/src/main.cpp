#include <Wire.h>
#include "MPU9250.h"
#include <MadgwickAHRS.h>

MPU9250 IMU(SPI, 10);

// ===================== CALIBRATION CONSTANTS =====================
// Accel biases (g) and scale factors
constexpr float ACCEL_BIAS[3] = {0.22f, 0.20f, -0.13f};
constexpr float ACCEL_SCALE[3] = {1.00f, 1.00f, 0.99f};

// Gyro biases (rad/s)
constexpr float GYRO_BIAS[3] = {0.01f, 0.04f, -0.01f};

// Magnetometer biases (uT) and scale factors
constexpr float MAG_BIAS[3] = {33.17f, 20.35f, -25.80f};
constexpr float MAG_SCALE[3] = {1.03f, 0.97f, 1.00f};

// ===================== CONSTANTS =====================
constexpr float G_TO_MPS2 = 9.80665f;
constexpr float SAMPLE_FREQ_HZ = 50.0f;
constexpr float MADGWICK_BETA_INIT = 0.8f;  // High beta for fast initial convergence
constexpr float MADGWICK_BETA_STEADY = 0.041f;  // Lower beta for steady state
constexpr unsigned long CONVERGENCE_TIME_MS = 3000;  // Time for filter to converge
constexpr unsigned long ZERO_DELAY_MS = 3500;  // Zero after convergence
constexpr unsigned long PRINT_INTERVAL_MS = 200;

// ===================== STATE VARIABLES =====================
Madgwick madgwick;

struct IMUData {
  float accel[3];
  float gyro[3];
  float mag[3];
};

struct Orientation {
  float roll;
  float pitch;
  float yaw;
};

enum SystemState {
  CONVERGING,      // Filter is converging with high beta
  STABILIZING,     // Switched to low beta, waiting to zero
  READY            // Zeroed and ready
};

unsigned long lastPrintMs = 0;
unsigned long startTimeMs = 0;
SystemState state = CONVERGING;
Orientation zeroOffset = {0.0f, 0.0f, 0.0f};

// ===================== FUNCTION PROTOTYPES =====================
void calibrateIMU(float raw[3], const float bias[3], const float scale[3], float output[3], bool isMPS2 = false);
void remapAxes(const float in[3], float out[3], bool invertZ = false);
void printIMUData(const IMUData &data, const Orientation &orient);
void initializeFilterOrientation();
float normalizeAngle(float angle);

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("Initializing MPU9250..."));

  int status = IMU.begin();
  if (status < 0) {
    Serial.print(F("IMU initialization failed with code: "));
    Serial.println(status);
    while (1) { delay(100); }
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  IMU.setSrd(19);

  // Initialize filter with high beta for fast convergence
  madgwick.begin(SAMPLE_FREQ_HZ);
  madgwick.setBeta(MADGWICK_BETA_INIT);

  Serial.println(F("IMU Ready!"));
  Serial.println(F("Initializing orientation filter..."));
  
  // Pre-initialize filter with several readings
  initializeFilterOrientation();
  
  startTimeMs = millis();
  Serial.println(F("Filter converging (3 sec)..."));
  Serial.println();
}

// ===================== MAIN LOOP =====================
void loop() {
  static unsigned long lastReadMs = 0;
  unsigned long now = millis();
  unsigned long elapsedMs = now - startTimeMs;
  
  if (now - lastReadMs < (1000 / SAMPLE_FREQ_HZ)) {
    return;
  }
  lastReadMs = now;

  // Read and process sensor data
  IMU.readSensor();

  float rawAccel[3] = {IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss()};
  float rawGyro[3] = {IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads()};
  float rawMag[3] = {IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT()};

  float calAccel[3], calGyro[3], calMag[3];
  calibrateIMU(rawAccel, ACCEL_BIAS, ACCEL_SCALE, calAccel, true);
  calibrateIMU(rawGyro, GYRO_BIAS, nullptr, calGyro);
  calibrateIMU(rawMag, MAG_BIAS, MAG_SCALE, calMag);

  IMUData data;
  remapAxes(calAccel, data.accel, true);
  remapAxes(calGyro, data.gyro, true);
  remapAxes(calMag, data.mag, true);

  float accelG[3] = {data.accel[0] / G_TO_MPS2, data.accel[1] / G_TO_MPS2, data.accel[2] / G_TO_MPS2};
  float gyroDPS[3] = {data.gyro[0] * RAD_TO_DEG, data.gyro[1] * RAD_TO_DEG, data.gyro[2] * RAD_TO_DEG};

  // Update filter
  madgwick.update(gyroDPS[0], gyroDPS[1], gyroDPS[2], 
                  accelG[0], accelG[1], accelG[2], 
                  data.mag[0], data.mag[1], data.mag[2]);

  Orientation orient = {madgwick.getRoll(), madgwick.getPitch(), madgwick.getYaw()};

  // State machine for initialization
  if (state == CONVERGING && elapsedMs >= CONVERGENCE_TIME_MS) {
    madgwick.setBeta(MADGWICK_BETA_STEADY);
    state = STABILIZING;
    Serial.println(F(">>> Filter converged, switching to steady-state mode <<<\n"));
  }
  
  if (state == STABILIZING && elapsedMs >= ZERO_DELAY_MS) {
    zeroOffset.roll = orient.roll;
    zeroOffset.pitch = orient.pitch;
    state = READY;
    Serial.println(F(">>> Roll/Pitch zeroed - System ready <<<\n"));
  }

  // Apply offsets
  if (state == READY) {
    orient.roll -= zeroOffset.roll;
    orient.pitch -= zeroOffset.pitch;
  }

  // Print output
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    
    if (state == CONVERGING) {
      Serial.print(F("Converging... "));
      Serial.print((CONVERGENCE_TIME_MS - elapsedMs) / 1000.0f, 1);
      Serial.println(F("s remaining"));
    }
    
    printIMUData(data, orient);
  }
}

// ===================== HELPER FUNCTIONS =====================

/**
 * Pre-initialize filter with static readings for better starting point
 */
void initializeFilterOrientation() {
  const int numSamples = 50;
  
  for (int i = 0; i < numSamples; i++) {
    IMU.readSensor();
    
    float rawAccel[3] = {IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss()};
    float rawMag[3] = {IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT()};
    
    float calAccel[3], calMag[3];
    calibrateIMU(rawAccel, ACCEL_BIAS, ACCEL_SCALE, calAccel, true);
    calibrateIMU(rawMag, MAG_BIAS, MAG_SCALE, calMag);
    
    IMUData data;
    remapAxes(calAccel, data.accel, true);
    remapAxes(calMag, data.mag, true);
    
    float accelG[3] = {data.accel[0] / G_TO_MPS2, data.accel[1] / G_TO_MPS2, data.accel[2] / G_TO_MPS2};
    
    // Update with zero gyro (stationary assumption)
    madgwick.update(0, 0, 0, accelG[0], accelG[1], accelG[2], 
                    data.mag[0], data.mag[1], data.mag[2]);
    
    delay(20);
  }
}

void calibrateIMU(float raw[3], const float bias[3], const float scale[3], float output[3], bool isMPS2) {
  for (int i = 0; i < 3; i++) {
    float biasValue = isMPS2 ? (bias[i] * G_TO_MPS2) : bias[i];
    output[i] = (raw[i] - biasValue) * (scale ? scale[i] : 1.0f);
  }
}

void remapAxes(const float in[3], float out[3], bool invertZ) {
  out[0] = in[1];
  out[1] = -in[0];
  out[2] = invertZ ? -in[2] : in[2];
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void printIMUData(const IMUData &data, const Orientation &orient) {
  Serial.println(F("========== IMU Data (Robot Frame) =========="));
  
  Serial.print(F("Accel [m/s²]: "));
  Serial.print(data.accel[0], 3); Serial.print(F("\t"));
  Serial.print(data.accel[1], 3); Serial.print(F("\t"));
  Serial.println(data.accel[2], 3);

  Serial.print(F("Gyro [rad/s]: "));
  Serial.print(data.gyro[0], 3); Serial.print(F("\t"));
  Serial.print(data.gyro[1], 3); Serial.print(F("\t"));
  Serial.println(data.gyro[2], 3);

  Serial.print(F("Mag [uT]:     "));
  Serial.print(data.mag[0], 2); Serial.print(F("\t"));
  Serial.print(data.mag[1], 2); Serial.print(F("\t"));
  Serial.println(data.mag[2], 2);

  Serial.println(F("\n--- Orientation (Madgwick) ---"));
  Serial.print(F("Roll:  ")); Serial.print(orient.roll, 2); Serial.print(F("°"));
  if (state != READY) Serial.print(F(" (not zeroed)"));
  Serial.println();
  
  Serial.print(F("Pitch: ")); Serial.print(orient.pitch, 2); Serial.print(F("°"));
  if (state != READY) Serial.print(F(" (not zeroed)"));
  Serial.println();
  
  Serial.print(F("Yaw:   ")); Serial.print(orient.yaw, 2); Serial.println(F("° (absolute)"));
  Serial.println(F("==========================================\n"));
}