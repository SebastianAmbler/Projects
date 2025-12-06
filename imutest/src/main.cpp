#include <Wire.h>
#include "MPU9250.h"
#include <MadgwickAHRS.h>

MPU9250 IMU(SPI, 10);

// ===================== CALIBRATION CONSTANTS =====================
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

// Adaptive beta parameters
constexpr float BETA_INIT = 0.8f;           // Fast initial convergence
constexpr float BETA_STATIC = 0.041f;       // When stationary (lower noise)
constexpr float BETA_MOVING = 0.15f;        // During motion (faster mag correction)
constexpr float BETA_TURNING = 0.20f;       // During yaw rotation (even faster)

// Motion detection thresholds
constexpr float GYRO_MOTION_THRESHOLD = 0.15f;  // rad/s (~8.6 deg/s) - any axis
constexpr float YAW_RATE_THRESHOLD = 0.20f;     // rad/s (~11.5 deg/s) - Z axis rotation
constexpr float ACCEL_MOTION_THRESHOLD = 1.5f;  // m/s^2 deviation from gravity

constexpr unsigned long CONVERGENCE_TIME_MS = 3000;
constexpr unsigned long ZERO_DELAY_MS = 3500;
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
  CONVERGING,
  STABILIZING,
  READY
};

enum MotionState {
  STATIC,
  MOVING,
  TURNING
};

unsigned long lastPrintMs = 0;
unsigned long startTimeMs = 0;
SystemState state = CONVERGING;
MotionState motionState = STATIC;
Orientation zeroOffset = {0.0f, 0.0f, 0.0f};

// Low-pass filter for smooth beta transitions
float currentBeta = BETA_INIT;
constexpr float BETA_FILTER_ALPHA = 0.1f;  // Smooth beta changes

// ===================== FUNCTION PROTOTYPES =====================
void calibrateIMU(float raw[3], const float bias[3], const float scale[3], float output[3], bool isMPS2 = false);
void remapAxes(const float in[3], float out[3], bool invertZ = false);
void printIMUData(const IMUData &data, const Orientation &orient);
void initializeFilterOrientation();
MotionState detectMotion(const IMUData &data);
float updateAdaptiveBeta(MotionState motion);
const char* getMotionStateName(MotionState motion);

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

  madgwick.begin(SAMPLE_FREQ_HZ);
  madgwick.setBeta(BETA_INIT);

  Serial.println(F("IMU Ready!"));
  Serial.println(F("Initializing orientation filter..."));
  
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

  // Detect motion and adapt beta accordingly
  if (state == READY) {
    motionState = detectMotion(data);
    float targetBeta = updateAdaptiveBeta(motionState);
    
    // Smooth beta transitions with low-pass filter
    currentBeta = currentBeta * (1.0f - BETA_FILTER_ALPHA) + targetBeta * BETA_FILTER_ALPHA;
    madgwick.setBeta(currentBeta);
  }

  float accelG[3] = {data.accel[0] / G_TO_MPS2, data.accel[1] / G_TO_MPS2, data.accel[2] / G_TO_MPS2};
  float gyroDPS[3] = {data.gyro[0] * RAD_TO_DEG, data.gyro[1] * RAD_TO_DEG, data.gyro[2] * RAD_TO_DEG};

  madgwick.update(gyroDPS[0], gyroDPS[1], gyroDPS[2], 
                  accelG[0], accelG[1], accelG[2], 
                  data.mag[0], data.mag[1], data.mag[2]);

  Orientation orient = {madgwick.getRoll(), madgwick.getPitch(), madgwick.getYaw()};

  // State machine for initialization
  if (state == CONVERGING && elapsedMs >= CONVERGENCE_TIME_MS) {
    madgwick.setBeta(BETA_STATIC);
    currentBeta = BETA_STATIC;
    state = STABILIZING;
    Serial.println(F(">>> Filter converged <<<\n"));
  }
  
  if (state == STABILIZING && elapsedMs >= ZERO_DELAY_MS) {
    zeroOffset.roll = orient.roll;
    zeroOffset.pitch = orient.pitch;
    state = READY;
    Serial.println(F(">>> System ready - Adaptive beta enabled <<<\n"));
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
 * Detect motion state based on gyro and accel data
 */
MotionState detectMotion(const IMUData &data) {
  // Calculate magnitudes
  float gyroMag = sqrt(data.gyro[0]*data.gyro[0] + 
                       data.gyro[1]*data.gyro[1] + 
                       data.gyro[2]*data.gyro[2]);
  
  float yawRate = abs(data.gyro[2]);  // Z-axis rotation rate
  
  // Calculate total acceleration magnitude
  float accelMag = sqrt(data.accel[0]*data.accel[0] + 
                        data.accel[1]*data.accel[1] + 
                        data.accel[2]*data.accel[2]);
  
  // Deviation from gravity (stationary = ~9.81 m/s^2)
  float accelDeviation = abs(accelMag - G_TO_MPS2);
  
  // Priority: TURNING > MOVING > STATIC
  if (yawRate > YAW_RATE_THRESHOLD) {
    return TURNING;
  } else if (gyroMag > GYRO_MOTION_THRESHOLD || accelDeviation > ACCEL_MOTION_THRESHOLD) {
    return MOVING;
  } else {
    return STATIC;
  }
}

/**
 * Select appropriate beta based on motion state
 */
float updateAdaptiveBeta(MotionState motion) {
  switch (motion) {
    case TURNING:
      return BETA_TURNING;  // Highest beta for fast mag correction during yaw
    case MOVING:
      return BETA_MOVING;   // Medium beta during translation/rotation
    case STATIC:
    default:
      return BETA_STATIC;   // Lowest beta when stationary for minimal noise
  }
}

/**
 * Get human-readable motion state name
 */
const char* getMotionStateName(MotionState motion) {
  switch (motion) {
    case TURNING: return "TURNING";
    case MOVING: return "MOVING";
    case STATIC: return "STATIC";
    default: return "UNKNOWN";
  }
}

/**
 * Pre-initialize filter with static readings
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
  
  // Show motion state and current beta
  if (state == READY) {
    Serial.print(F("Motion: ")); Serial.print(getMotionStateName(motionState));
    Serial.print(F(" | Beta: ")); Serial.println(currentBeta, 3);
  }
  
  Serial.println(F("==========================================\n"));
}