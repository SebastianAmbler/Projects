#include <Wire.h>
#include "MPU9250.h"

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
constexpr float DT = 1.0f / SAMPLE_FREQ_HZ; // 0.02 seconds

// Magnetic declination for your location: -0° 41'
constexpr float MAG_DECLINATION_RAD = -0.01193f; // -0.683° in radians

// Kalman filter parameters
constexpr float Q_ANGLE = 0.001f;  // Process noise variance for accelerometer
constexpr float Q_BIAS = 0.003f;   // Process noise variance for gyroscope bias
constexpr float R_MEASURE = 0.03f; // Measurement noise variance

constexpr unsigned long CALIBRATION_TIME_MS = 2000;

// FreeRTOS Configuration
constexpr uint32_t SENSOR_TASK_STACK = 4096;
constexpr uint32_t PROCESS_TASK_STACK = 4096;
constexpr uint32_t PRINT_TASK_STACK = 3072;

// Core assignments (ESP32-S3 has dual cores: 0 and 1)
constexpr BaseType_t SENSOR_TASK_CORE = 1;   // Time-critical on Core 1
constexpr BaseType_t PROCESS_TASK_CORE = 1;  // Processing on Core 1
constexpr BaseType_t PRINT_TASK_CORE = 0;    // I/O on Core 0 (same as Arduino loop)

// ===================== DATA STRUCTURES =====================
struct IMUData {
  float accel[3];
  float gyro[3];
  float mag[3];
};

struct KalmanState {
  float angle;
  float bias;
  float rate;
  float P[2][2];
};

struct Orientation {
  float roll;
  float pitch;
  float yaw;
};

// ===================== STATE VARIABLES =====================
KalmanState pitchKalman = {0, 0, 0, {{0, 0}, {0, 0}}};
KalmanState rollKalman = {0, 0, 0, {{0, 0}, {0, 0}}};
Orientation orient = {0, 0, 0};
Orientation zeroOffset = {0, 0, 0};

IMUData currentData;
bool isCalibrated = false;
unsigned long calibrationStartMs = 0;

// ===================== FREERTOS HANDLES =====================
SemaphoreHandle_t xDataMutex = NULL;
TaskHandle_t xSensorTaskHandle = NULL;
TaskHandle_t xProcessTaskHandle = NULL;
TaskHandle_t xPrintTaskHandle = NULL;

// ===================== FUNCTION PROTOTYPES =====================
void calibrateIMU(float raw[3], const float bias[3], const float scale[3], float output[3], bool isMPS2 = false);
void remapAxes(const float in[3], float out[3], bool invertZ = false);
float kalmanFilter(KalmanState &state, float gyroRate, float accelAngle, float dt);
float calculateYaw(float mx, float my, float roll, float pitch);
void printIMUData(const IMUData &data, const Orientation &orient);
void initializeKalmanFilter();

// ===================== FREERTOS TASKS =====================

/**
 * Task: Read sensor data at 50 Hz (Core 1)
 */
void vSensorReadTask(void *pvParameters) {
  // Wait for signal that all tasks are ready
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz = 20ms period

  Serial.println("Sensor Read Task started on Core " + String(xPortGetCoreID()));

  for (;;) {
    // Read sensor data
    IMU.readSensor();

    float rawAccel[3] = {IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss()};
    float rawGyro[3] = {IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads()};
    float rawMag[3] = {IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT()};

    // Apply calibration
    float calAccel[3], calGyro[3], calMag[3];
    calibrateIMU(rawAccel, ACCEL_BIAS, ACCEL_SCALE, calAccel, true);
    calibrateIMU(rawGyro, GYRO_BIAS, nullptr, calGyro);
    calibrateIMU(rawMag, MAG_BIAS, MAG_SCALE, calMag);

    // Remap axes to robot frame
    IMUData tempData;
    remapAxes(calAccel, tempData.accel, true);
    remapAxes(calGyro, tempData.gyro, true);
    remapAxes(calMag, tempData.mag, true);

    // Copy to shared data with mutex protection
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(&currentData, &tempData, sizeof(IMUData));
      xSemaphoreGive(xDataMutex);
      
      // Notify processing task that new data is available
      xTaskNotifyGive(xProcessTaskHandle);
    }

    // Wait for next cycle (maintains precise 50 Hz timing)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * Task: Process sensor data (Kalman filtering) (Core 1)
 */
void vDataProcessTask(void *pvParameters) {
  // Wait for signal that all tasks are ready
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  
  Serial.println("Data Process Task started on Core " + String(xPortGetCoreID()));

  for (;;) {
    // Wait for notification from sensor read task (blocks indefinitely)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    IMUData localData;
    
    // Copy current data with mutex protection
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(&localData, &currentData, sizeof(IMUData));
      xSemaphoreGive(xDataMutex);
    } else {
      continue; // Skip this cycle if we can't get the mutex
    }

    // Calculate accelerometer angles
    float accelPitch = atan2(-localData.accel[0], 
                            sqrt(localData.accel[1] * localData.accel[1] + 
                                 localData.accel[2] * localData.accel[2])) * RAD_TO_DEG;
    float accelRoll = atan2(localData.accel[1], localData.accel[2]) * RAD_TO_DEG;

    // Apply Kalman filter for pitch and roll
    orient.pitch = kalmanFilter(pitchKalman, localData.gyro[0] * RAD_TO_DEG, accelPitch, DT);
    orient.roll = kalmanFilter(rollKalman, localData.gyro[1] * RAD_TO_DEG, accelRoll, DT);

    // Calculate yaw from magnetometer with tilt compensation
    orient.yaw = calculateYaw(localData.mag[0], localData.mag[1], orient.roll, orient.pitch);

    // Apply zero offset after calibration period
    if (!isCalibrated && (millis() - calibrationStartMs) >= CALIBRATION_TIME_MS) {
      zeroOffset.roll = orient.roll;
      zeroOffset.pitch = orient.pitch;
      isCalibrated = true;
      Serial.println(F(">>> System ready - Zero offset applied <<<\n"));
    }

    if (isCalibrated) {
      orient.roll -= zeroOffset.roll;
      orient.pitch -= zeroOffset.pitch;
    }
  }
}

/**
 * Task: Print data at 5 Hz (200ms interval) (Core 0)
 */
void vPrintTask(void *pvParameters) {
  // Wait for signal that all tasks are ready
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200); // 5 Hz = 200ms period

  Serial.println("Print Task started on Core " + String(xPortGetCoreID()));

  for (;;) {
    IMUData localData;
    Orientation localOrient;
    bool localCalibrated;
    unsigned long elapsedMs = millis() - calibrationStartMs;

    // Copy data with mutex protection
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(&localData, &currentData, sizeof(IMUData));
      memcpy(&localOrient, &orient, sizeof(Orientation));
      localCalibrated = isCalibrated;
      xSemaphoreGive(xDataMutex);
    } else {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      continue;
    }

    // Print calibration status
    if (!localCalibrated) {
      Serial.print(F("Stabilizing... "));
      Serial.print((CALIBRATION_TIME_MS - elapsedMs) / 1000.0f, 1);
      Serial.println(F("s remaining"));
    }

    // Print IMU data
    printIMUData(localData, localOrient);

    // Wait for next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(1000); // Give time for serial to initialize
  
  Serial.println(F("\n=== ESP32-S3 FreeRTOS IMU System ==="));
  Serial.printf("Running on Core: %d\n", xPortGetCoreID());
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());

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
  IMU.setSrd(19); // Sample rate divider for 50 Hz

  Serial.println(F("IMU Ready!"));
  Serial.println(F("Calibrating Kalman filter..."));
  
  initializeKalmanFilter();
  
  calibrationStartMs = millis();
  Serial.println(F("Calibration complete. Stabilizing for 2 seconds...\n"));

  // Create mutex
  xDataMutex = xSemaphoreCreateMutex();
  
  if (xDataMutex == NULL) {
    Serial.println(F("Failed to create mutex!"));
    while (1) { delay(100); }
  }

  Serial.println(F("Creating FreeRTOS tasks..."));

  // Create sensor read task on Core 1 (time-critical)
  BaseType_t result = xTaskCreatePinnedToCore(
    vSensorReadTask,
    "SensorRead",
    SENSOR_TASK_STACK,
    NULL,
    3,    // Priority (highest)
    &xSensorTaskHandle,
    SENSOR_TASK_CORE
  );
  
  if (result != pdPASS) {
    Serial.println(F("Failed to create Sensor Read Task!"));
    while (1) { delay(100); }
  }

  // Create data processing task on Core 1
  result = xTaskCreatePinnedToCore(
    vDataProcessTask,
    "DataProcess",
    PROCESS_TASK_STACK,
    NULL,
    2,    // Priority (medium)
    &xProcessTaskHandle,
    PROCESS_TASK_CORE
  );
  
  if (result != pdPASS) {
    Serial.println(F("Failed to create Data Process Task!"));
    while (1) { delay(100); }
  }

  // Create print task on Core 0 (I/O operations)
  result = xTaskCreatePinnedToCore(
    vPrintTask,
    "Print",
    PRINT_TASK_STACK,
    NULL,
    1,    // Priority (lowest)
    &xPrintTaskHandle,
    PRINT_TASK_CORE
  );
  
  if (result != pdPASS) {
    Serial.println(F("Failed to create Print Task!"));
    while (1) { delay(100); }
  }

  Serial.println(F("All tasks created successfully!"));
  Serial.printf("Free Heap after task creation: %d bytes\n\n", ESP.getFreeHeap());
  
  // Small delay to ensure all tasks are waiting
  delay(100);
  
  // Signal all tasks to start
  Serial.println(F("Starting all tasks..."));
  xTaskNotifyGive(xSensorTaskHandle);
  xTaskNotifyGive(xProcessTaskHandle);
  xTaskNotifyGive(xPrintTaskHandle);
}

// ===================== MAIN LOOP =====================
void loop() {
  // Empty - FreeRTOS tasks handle everything
  // This runs on Core 0 but we're not using it
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ===================== HELPER FUNCTIONS =====================

/**
 * Initialize Kalman filter with static readings
 */
void initializeKalmanFilter() {
  const int numSamples = 50;
  
  for (int i = 0; i < numSamples; i++) {
    IMU.readSensor();
    
    float rawAccel[3] = {IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss()};
    float calAccel[3];
    calibrateIMU(rawAccel, ACCEL_BIAS, ACCEL_SCALE, calAccel, true);
    
    IMUData data;
    remapAxes(calAccel, data.accel, true);
    
    // Initialize with accelerometer angles
    float accelPitch = atan2(-data.accel[0], sqrt(data.accel[1] * data.accel[1] + 
                                                   data.accel[2] * data.accel[2])) * RAD_TO_DEG;
    float accelRoll = atan2(data.accel[1], data.accel[2]) * RAD_TO_DEG;
    
    pitchKalman.angle = accelPitch;
    rollKalman.angle = accelRoll;
    
    delay(20);
  }
}

/**
 * Kalman filter implementation for sensor fusion
 */
float kalmanFilter(KalmanState &state, float gyroRate, float accelAngle, float dt) {
  // Prediction step
  state.rate = gyroRate - state.bias;
  state.angle += dt * state.rate;

  // Update error covariance matrix
  state.P[0][0] += dt * (dt * state.P[1][1] - state.P[0][1] - state.P[1][0] + Q_ANGLE);
  state.P[0][1] -= dt * state.P[1][1];
  state.P[1][0] -= dt * state.P[1][1];
  state.P[1][1] += Q_BIAS * dt;

  // Update step
  float S = state.P[0][0] + R_MEASURE; // Innovation covariance
  float K[2];                           // Kalman gain
  K[0] = state.P[0][0] / S;
  K[1] = state.P[1][0] / S;

  float y = accelAngle - state.angle;   // Innovation (measurement residual)
  state.angle += K[0] * y;
  state.bias += K[1] * y;

  // Update error covariance matrix
  float P00_temp = state.P[0][0];
  float P01_temp = state.P[0][1];

  state.P[0][0] -= K[0] * P00_temp;
  state.P[0][1] -= K[0] * P01_temp;
  state.P[1][0] -= K[1] * P00_temp;
  state.P[1][1] -= K[1] * P01_temp;

  return state.angle;
}

/**
 * Calculate yaw from magnetometer with tilt compensation
 */
float calculateYaw(float mx, float my, float roll, float pitch) {
  // Convert angles to radians for tilt compensation
  float rollRad = roll * DEG_TO_RAD;
  float pitchRad = pitch * DEG_TO_RAD;
  
  // Tilt compensation with inverted X-axis
  float magX = mx * cos(pitchRad) + my * sin(rollRad) * sin(pitchRad);
  float magY = my * cos(rollRad);
  
  // Calculate yaw with inverted X
  float yaw = atan2(magY, -magX);
  
  // Apply magnetic declination
  yaw += MAG_DECLINATION_RAD;
  
  // Normalize to 0-360 degrees
  if (yaw < 0) yaw += 2 * PI;
  if (yaw > 2 * PI) yaw -= 2 * PI;
  
  return yaw * RAD_TO_DEG;
}

/**
 * Apply calibration to raw sensor data
 */
void calibrateIMU(float raw[3], const float bias[3], const float scale[3], float output[3], bool isMPS2) {
  for (int i = 0; i < 3; i++) {
    float biasValue = isMPS2 ? (bias[i] * G_TO_MPS2) : bias[i];
    output[i] = (raw[i] - biasValue) * (scale ? scale[i] : 1.0f);
  }
}

/**
 * Remap sensor axes to robot frame
 */
void remapAxes(const float in[3], float out[3], bool invertZ) {
  out[0] = in[1];
  out[1] = -in[0];
  out[2] = invertZ ? -in[2] : in[2];
}

/**
 * Print formatted IMU data and orientation
 */
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

  Serial.println(F("\n--- Orientation (Kalman Filter) ---"));
  Serial.print(F("Roll:  ")); Serial.print(orient.roll, 2); Serial.print(F("°"));
  if (!isCalibrated) Serial.print(F(" (not zeroed)"));
  Serial.println();
  
  Serial.print(F("Pitch: ")); Serial.print(orient.pitch, 2); Serial.print(F("°"));
  if (!isCalibrated) Serial.print(F(" (not zeroed)"));
  Serial.println();
  
  Serial.print(F("Yaw:   ")); Serial.print(orient.yaw, 2); Serial.println(F("° (absolute)"));
  
  Serial.println(F("==========================================\n"));
}