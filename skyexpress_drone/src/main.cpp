/*
====================================================================================
🛩️  ESP32 Quadcopter Flight Controller with MPU9250 (FreeRTOS) - FIXED VERSION
====================================================================================
Integration of tuned MPU9250 sensor with flight controller
Based on Carbon Aeronautics architecture, adapted for ESP32
====================================================================================
*/

#include <Wire.h>
#include <Arduino.h>
#include <PPMReader.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include "MPU9250.h"

// ===================== MPU9250 SETUP =====================
// FIXED: Using I2C interface (address 0x68 or 0x69 depending on AD0 pin)
MPU9250 IMU(Wire, 0x68);

// Calibration constants from your tuned code
constexpr float ACCEL_BIAS[3] = {0.22f, 0.20f, -0.13f};
constexpr float ACCEL_SCALE[3] = {1.00f, 1.00f, 0.99f};
constexpr float GYRO_BIAS[3] = {0.01f, 0.04f, -0.01f};
constexpr float MAG_BIAS[3] = {33.17f, 20.35f, -25.80f};
constexpr float MAG_SCALE[3] = {1.03f, 0.97f, 1.00f};

constexpr float G_TO_MPS2 = 9.80665f;

// Kalman filter parameters
constexpr float Q_ANGLE = 0.001f;
constexpr float Q_BIAS = 0.003f;
constexpr float R_MEASURE = 0.03f;

struct KalmanState {
  float angle;
  float bias;
  float rate;
  float P[2][2];
};

KalmanState pitchKalman = {0, 0, 0, {{0, 0}, {0, 0}}};
KalmanState rollKalman = {0, 0, 0, {{0, 0}, {0, 0}}};

// ===================== MUTEXES =====================
SemaphoreHandle_t i2cMutex;

// ===================== CONFIGURATION CONSTANTS =====================
const float MOTOR_SCALE_FACTOR = 1.024f;
const float ANGLE_SCALE_FACTOR = 0.10f;
const float YAW_RATE_SCALE_FACTOR = 0.15f;
const int THROTTLE_IDLE = 1180;
const int THROTTLE_CUTOFF = 1000;
const int THROTTLE_MAX = 1800;
const int MOTOR_MAX = 1989;
const float LOOP_TIME_SEC = 0.004f; // 4ms = 250Hz

// ===================== PID TUNING =====================
float PRate = 0.08, IRate = 0.04, DRate = 0.001;
float PAngle = 4, IAngle = 0.04, DAngle = 0.0;

// ===================== BATTERY MONITORING =====================
const int analogPin = 25;
const float referenceVoltage = 3.3;
const float r1Value = 77600.0;
const float r2Value = 29400.0;
const float BATTERY_WARNING_VOLTAGE = 10.5; // 3.5V per cell for 3S
const float BATTERY_CRITICAL_VOLTAGE = 9.9; // 3.3V per cell for 3S
float battery_voltage = 10;

// ===================== PWM MOTOR CONTROL =====================
const int PWM_PIN_M1 = 5;
const int PWM_PIN_M2 = 18;
const int PWM_PIN_M3 = 19;
const int PWM_PIN_M4 = 15;
const int PWM_FREQUENCY = 250;
const int PWM_RESOLUTION = 12;

// ===================== PPM RECEIVER =====================
byte interruptPin = 4;
byte channelAmount = 6;
PPMReader ppm(interruptPin, channelAmount);
int ppmData[6] = {0, 0, 0, 0, 0, 0};

// PPM timeout
const unsigned long ppmTimeout = 5000000; // 5 seconds
unsigned long lastPPMUpdateTime = 0;
bool ppmSignalLost = true;
int lastPPMData[6] = {0};
const int changeThreshold = 10;

// ===================== ARMING SYSTEM =====================
bool isArmed = false;
unsigned long armingStartTime = 0;
const unsigned long ARMING_TIME_MS = 2000; // 2 seconds to arm
bool armingInProgress = false;

// ===================== DEBUG FLAGS =====================
bool enableSerialDebug = false; // Set to true only for ground testing

// ===================== FLIGHT CONTROL VARIABLES =====================
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};

// PID Rate
float PRateRoll = PRate;
float PRatePitch = PRateRoll;
float PRateYaw = 0.100;
float IRateRoll = IRate;
float IRatePitch = IRateRoll;
float IRateYaw = 0.030;
float DRateRoll = DRate;
float DRatePitch = DRateRoll;
float DRateYaw = 0.000;

float MotorInput1 = 0, MotorInput2 = 0, MotorInput3 = 0, MotorInput4 = 0;

float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;

// PID Angle
float PAngleRoll = PAngle;
float PAnglePitch = PAngleRoll;
float IAngleRoll = IAngle;
float IAnglePitch = IAngleRoll;
float DAngleRoll = DAngle;
float DAnglePitch = DAngleRoll;

// ===================== HELPER FUNCTIONS =====================

void calibrateIMU(float raw[3], const float bias[3], const float scale[3], float output[3], bool isMPS2 = false) {
  for (int i = 0; i < 3; i++) {
    float biasValue = isMPS2 ? (bias[i] * G_TO_MPS2) : bias[i];
    output[i] = (raw[i] - biasValue) * (scale ? scale[i] : 1.0f);
  }
}

void remapAxes(const float in[3], float out[3], bool invertZ = false) {
  out[0] = in[1];
  out[1] = -in[0];
  out[2] = invertZ ? -in[2] : in[2];
}

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
  float S = state.P[0][0] + R_MEASURE;
  float K[2];
  K[0] = state.P[0][0] / S;
  K[1] = state.P[1][0] / S;

  float y = accelAngle - state.angle;
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

void reset_motors() {
  MotorInput1 = THROTTLE_CUTOFF;
  MotorInput2 = THROTTLE_CUTOFF;
  MotorInput3 = THROTTLE_CUTOFF;
  MotorInput4 = THROTTLE_CUTOFF;
}

void checkArmingSequence() {
  // Arming: Throttle LOW + Yaw RIGHT for 2 seconds
  // Disarming: Throttle LOW + Yaw LEFT for 2 seconds
  
  bool throttleLow = ppmData[2] < 1050;
  bool yawRight = ppmData[3] > 1800;
  bool yawLeft = ppmData[3] < 1200;
  
  if (throttleLow && yawRight && !isArmed) {
    if (!armingInProgress) {
      armingInProgress = true;
      armingStartTime = millis();
    } else if (millis() - armingStartTime > ARMING_TIME_MS) {
      isArmed = true;
      armingInProgress = false;
      Serial.println("*** ARMED ***");
    }
  } else if (throttleLow && yawLeft && isArmed) {
    if (!armingInProgress) {
      armingInProgress = true;
      armingStartTime = millis();
    } else if (millis() - armingStartTime > ARMING_TIME_MS) {
      isArmed = false;
      armingInProgress = false;
      reset_motors();
      Serial.println("*** DISARMED ***");
    }
  } else {
    armingInProgress = false;
  }
}

void ppmloop() {
  bool valuesChanged = false;

  for (byte channel = 0; channel < channelAmount; ++channel) {
    int value = ppm.latestValidChannelValue(channel + 1, -1);
    if (value != -1) {
      ppmData[channel] = value;
      if (abs(ppmData[channel] - lastPPMData[channel]) > changeThreshold) {
        valuesChanged = true;
      }
    }
    
    if (enableSerialDebug) {
      Serial.print(ppmData[channel]);
      Serial.print("\t");
    }
  }

  unsigned long currentTime = micros();
  unsigned long elapsedTime = currentTime - lastPPMUpdateTime;

  if (valuesChanged) {
    lastPPMUpdateTime = currentTime;
    ppmSignalLost = false;
  } else if (elapsedTime > ppmTimeout) {
    ppmSignalLost = true;
  }

  if (enableSerialDebug) {
    Serial.print(" | Elapsed: ");
    Serial.print(elapsedTime / 1000000.0);
    Serial.print(" s | Lost: ");
    Serial.println(ppmSignalLost ? "Y" : "N");
  }

  for (byte channel = 0; channel < channelAmount; ++channel) {
    lastPPMData[channel] = ppmData[channel];
  }

  if (ppmSignalLost) {
    if (enableSerialDebug) {
      Serial.println("PPM Signal Lost! Emergency stop!");
    }
    ppmData[2] = THROTTLE_CUTOFF;
    ppmData[0] = 1500;
    ppmData[1] = 1500;
    ppmData[3] = 1500;
    isArmed = false;
    reset_pid();
    reset_motors();
  }
  
  // Check arming/disarming
  checkArmingSequence();
}

void pwmsetup() {
  pinMode(PWM_PIN_M1, OUTPUT);
  pinMode(PWM_PIN_M2, OUTPUT);
  pinMode(PWM_PIN_M3, OUTPUT);
  pinMode(PWM_PIN_M4, OUTPUT);

  ledcAttach(PWM_PIN_M1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWM_PIN_M2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWM_PIN_M3, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWM_PIN_M4, PWM_FREQUENCY, PWM_RESOLUTION);
}

void pwmloop(int motor_input, int PWM_PIN) {
  ledcWrite(PWM_PIN, motor_input);
}

float batteryvoltage() {
  // Average multiple readings for stability
  const int samples = 5;
  int sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(analogPin);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  int rawValue = sum / samples;
  
  float voltage = (rawValue / 4095.0) * referenceVoltage;
  float inputVoltage = voltage * ((r1Value + r2Value) / r2Value);
  return inputVoltage;
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * LOOP_TIME_SEC / 2;

  if (Iterm > 400)
    Iterm = 400;
  else if (Iterm < -400)
    Iterm = -400;

  float Dterm = D * (Error - PrevError) / LOOP_TIME_SEC;
  float PIDOutput = Pterm + Iterm + Dterm;

  if (PIDOutput > 400)
    PIDOutput = 400;
  else if (PIDOutput < -400)
    PIDOutput = -400;

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void) {
  PrevErrorRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;

  PrevErrorAngleRoll = 0;
  PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0;
  PrevItermAnglePitch = 0;
}

void gyro_signals(void) {
  if (i2cMutex != NULL) {
    if (xSemaphoreTake(i2cMutex, (TickType_t)10) == pdTRUE) {
      // Read MPU9250 sensor
      IMU.readSensor();

      // Get raw sensor data
      float rawAccel[3] = {IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss()};
      float rawGyro[3] = {IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads()};

      // Apply calibration
      float calAccel[3], calGyro[3];
      calibrateIMU(rawAccel, ACCEL_BIAS, ACCEL_SCALE, calAccel, true);
      calibrateIMU(rawGyro, GYRO_BIAS, nullptr, calGyro);

      // Remap axes to robot frame
      float tempAccel[3], tempGyro[3];
      remapAxes(calAccel, tempAccel, true);
      remapAxes(calGyro, tempGyro, true);

      // Convert gyro from rad/s to deg/s
      RateRoll = tempGyro[0] * RAD_TO_DEG;
      RatePitch = tempGyro[1] * RAD_TO_DEG;
      RateYaw = tempGyro[2] * RAD_TO_DEG;

      // Accelerometer data (in m/s²)
      AccX = tempAccel[0];
      AccY = tempAccel[1];
      AccZ = tempAccel[2];

      // Calculate angles from accelerometer
      AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEG;
      AnglePitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG;

      xSemaphoreGive(i2cMutex);
    } else {
      Serial.println("I2C mutex busy");
    }
  }
}

// ===================== FREERTOS TASKS =====================

void batteryMonitorTask(void *pvParameters) {
  while (1) {
    battery_voltage = batteryvoltage();
    
    if (battery_voltage < BATTERY_CRITICAL_VOLTAGE) {
      Serial.println("!!! CRITICAL BATTERY - EMERGENCY LANDING !!!");
      digitalWrite(2, HIGH);
      vTaskDelay(pdMS_TO_TICKS(200));
      digitalWrite(2, LOW);
      vTaskDelay(pdMS_TO_TICKS(200));
      
      // Force disarm after critical voltage
      isArmed = false;
      reset_motors();
      
    } else if (battery_voltage < BATTERY_WARNING_VOLTAGE) {
      Serial.println("! Battery Low - Land Soon !");
      digitalWrite(2, HIGH);
      vTaskDelay(pdMS_TO_TICKS(500));
      digitalWrite(2, LOW);
      vTaskDelay(pdMS_TO_TICKS(500));
      
    } else {
      digitalWrite(2, HIGH);
      vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds when OK
    }
    
    if (enableSerialDebug) {
      Serial.print("Battery: ");
      Serial.print(battery_voltage);
      Serial.println("V");
    }
  }
}

void flightControlTask(void *pvParameters) {
  while (1) {
    gyro_signals();
    
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    // Apply Kalman filter
    KalmanAngleRoll = kalmanFilter(rollKalman, RateRoll, AngleRoll, LOOP_TIME_SEC);
    KalmanAnglePitch = kalmanFilter(pitchKalman, RatePitch, AnglePitch, LOOP_TIME_SEC);

    ppmloop();
    
    DesiredAngleRoll = ANGLE_SCALE_FACTOR * (ppmData[0] - 1500);
    DesiredAnglePitch = ANGLE_SCALE_FACTOR * (ppmData[1] - 1500);

    if (enableSerialDebug) {
      Serial.print(" Roll: ");
      Serial.print(KalmanAngleRoll);
      Serial.print(" Pitch: ");
      Serial.println(KalmanAnglePitch);
    }

    InputThrottle = ppmData[2];
    DesiredRateYaw = YAW_RATE_SCALE_FACTOR * (ppmData[3] - 1500);
    
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
    DesiredRateRoll = PIDReturn[0];
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];

    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];

    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;

    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch = PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];

    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[1];
    PrevItermRateYaw = PIDReturn[2];

    if (InputThrottle > THROTTLE_MAX) InputThrottle = THROTTLE_MAX;

    // Motor mixing for X-configuration quadcopter
    // M1: Front-Right, M2: Front-Left, M3: Rear-Left, M4: Rear-Right
    MotorInput1 = MOTOR_SCALE_FACTOR * (InputThrottle - InputRoll - InputPitch - InputYaw);
    MotorInput2 = MOTOR_SCALE_FACTOR * (InputThrottle - InputRoll + InputPitch + InputYaw);
    MotorInput3 = MOTOR_SCALE_FACTOR * (InputThrottle + InputRoll + InputPitch - InputYaw);
    MotorInput4 = MOTOR_SCALE_FACTOR * (InputThrottle + InputRoll - InputPitch + InputYaw);

    // Limit maximum motor output
    if (MotorInput1 > 2000) MotorInput1 = MOTOR_MAX;
    if (MotorInput2 > 2000) MotorInput2 = MOTOR_MAX;
    if (MotorInput3 > 2000) MotorInput3 = MOTOR_MAX;
    if (MotorInput4 > 2000) MotorInput4 = MOTOR_MAX;

    // Apply minimum throttle when armed
    if (MotorInput1 < THROTTLE_IDLE) MotorInput1 = THROTTLE_IDLE;
    if (MotorInput2 < THROTTLE_IDLE) MotorInput2 = THROTTLE_IDLE;
    if (MotorInput3 < THROTTLE_IDLE) MotorInput3 = THROTTLE_IDLE;
    if (MotorInput4 < THROTTLE_IDLE) MotorInput4 = THROTTLE_IDLE;

    // Safety: Cut throttle if not armed or throttle stick is low
    if (!isArmed || ppmData[2] < 1050) {
      MotorInput1 = THROTTLE_CUTOFF;
      MotorInput2 = THROTTLE_CUTOFF;
      MotorInput3 = THROTTLE_CUTOFF;
      MotorInput4 = THROTTLE_CUTOFF;
      reset_pid();
    }

    pwmloop(MotorInput1, PWM_PIN_M1);
    pwmloop(MotorInput2, PWM_PIN_M2);
    pwmloop(MotorInput3, PWM_PIN_M3);
    pwmloop(MotorInput4, PWM_PIN_M4);

    // Maintain 250Hz loop rate (4ms)
    while ((micros() - LoopTimer) < 4000) {
      taskYIELD(); // Allow other tasks to run
    }
    LoopTimer = micros();
  }
}

void gyroscope_calibration() {
  Serial.println("=== GYROSCOPE CALIBRATION ===");
  Serial.println("Keep drone LEVEL and STATIONARY!");
  delay(2000);
  
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    
    if (RateCalibrationNumber % 200 == 0) {
      Serial.print(".");
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  
  Serial.println("\n=== CALIBRATION COMPLETE ===");
  Serial.print("Roll offset: "); Serial.println(RateCalibrationRoll);
  Serial.print("Pitch offset: "); Serial.println(RateCalibrationPitch);
  Serial.print("Yaw offset: "); Serial.println(RateCalibrationYaw);
}

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait up to 3s for serial
  
  Serial.println(F("\n\n================================="));
  Serial.println(F("ESP32 Quadcopter Flight Controller"));
  Serial.println(F("=================================\n"));
  
  // Create I2C mutex
  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL) {
    Serial.println("ERROR: Failed to create I2C mutex!");
    while (1) {
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
      delay(100);
    }
  }

  pinMode(2, OUTPUT);
  analogReadResolution(12);
  pinMode(analogPin, INPUT);

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C

  Serial.println(F("Initializing MPU9250..."));
  
  int status = IMU.begin();
  if (status < 0) {
    Serial.print(F("ERROR: IMU initialization failed! Code: "));
    Serial.println(status);
    Serial.println(F("Check I2C connections and address (0x68 or 0x69)"));
    while (1) {
      digitalWrite(2, HIGH);
      delay(250);
      digitalWrite(2, LOW);
      delay(250);
    }
  }

  // Configure IMU
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  IMU.setSrd(19); // Sample rate divider for 50 Hz (1000Hz / (1 + 19) = 50Hz)

  Serial.println(F("IMU initialized successfully!"));
  
  delay(500);
  
  // Calibrate gyroscope
  gyroscope_calibration();
  
  // Setup PWM for motors
  pwmsetup();
  reset_motors();
  
  LoopTimer = micros();

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
    batteryMonitorTask,
    "BatteryMonitor",
    4096,
    NULL,
    1,
    NULL,
    0  // Core 0
  );
  
  xTaskCreatePinnedToCore(
    flightControlTask,
    "FlightControl",
    10240, // Increased stack size
    NULL,
    2,     // Higher priority
    NULL,
    1      // Core 1
  );

  Serial.println(F("\n=== FLIGHT CONTROLLER READY ==="));
  Serial.println(F("ARM: Throttle LOW + Yaw RIGHT for 2s"));
  Serial.println(F("DISARM: Throttle LOW + Yaw LEFT for 2s"));
  Serial.println(F("================================\n"));
}

void loop() {
  // Empty - FreeRTOS tasks handle everything
  vTaskDelay(pdMS_TO_TICKS(1000));
}