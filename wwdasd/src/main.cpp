/*
====================================================================================
🛩️  ESP32 Quadcopter Flight Controller with MPU9250 (FreeRTOS)
====================================================================================
Integration of tuned MPU9250 sensor with flight controller
Based on Carbon Aeronautics architecture, adapted for ESP32
====================================================================================
*/

#include <Wire.h>
#include <Arduino.h>
#include <IBusBM.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include "MPU9250.h"

// ===================== MPU9250 SETUP =====================
MPU9250 IMU(SPI, 10);

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

// ===================== PID TUNING =====================
float PRate = 0.08, IRate = 0.04, DRate = 0.001;
float PAngle = 4, IAngle = 0.04, DAngle = 0.0;

// ===================== BATTERY MONITORING =====================
const int analogPin = 25;
float referenceVoltage = 3.3;
float r1Value = 77600.0;
float r2Value = 29400.0;
float battery_voltage = 10;

// ===================== PWM MOTOR CONTROL =====================
const int PWM_PIN_M1 = 5;   // Motor 1 - Counter-clockwise
const int PWM_PIN_M2 = 18;  // Motor 2 - Clockwise
const int PWM_PIN_M3 = 19;  // Motor 3 - Counter-clockwise
const int PWM_PIN_M4 = 15;  // Motor 4 - Clockwise
const int PWM_FREQUENCY = 250;
const int PWM_RESOLUTION = 12;

// ===================== iBUS RECEIVER (FS-iA6B) =====================
IBusBM ibus;
const int IBUS_RX_PIN = 16; // RX pin for iBUS (Serial2)
const int IBUS_TX_PIN = 17; // TX pin for iBUS telemetry (optional)
int ibusData[10] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000}; // Default safe values

// iBUS timeout
const unsigned long ibusTimeout = 1000000; // 1 second timeout
unsigned long lastIBusUpdateTime = 0;
bool ibusSignalLost = true;
int lastIBusData[10] = {0};
const int changeThreshold = 10;

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
  MotorInput1 = 1000;
  MotorInput2 = 1000;
  MotorInput3 = 1000;
  MotorInput4 = 1000;
}

void ibusloop() {
  bool valuesChanged = false;

  // Read all 10 channels (FS-iA6B has 6 physical + 4 virtual on iBUS)
  for (int channel = 0; channel < 10; channel++) {
    int value = ibus.readChannel(channel);
    
    // iBUS returns 1000-2000, or 0 if invalid
    if (value >= 1000 && value <= 2000) {
      ibusData[channel] = value;
      
      // Check if values have changed significantly
      if (abs(ibusData[channel] - lastIBusData[channel]) > changeThreshold) {
        valuesChanged = true;
      }
    }
  }

  // Print first 6 channels for monitoring
  Serial.print("iBUS Ch: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(ibusData[i]);
    Serial.print("\t");
  }
  Serial.print(" | ");

  unsigned long currentTime = micros();
  unsigned long elapsedTime = currentTime - lastIBusUpdateTime;

  // If values change, update timestamp and reset signal lost flag
  if (valuesChanged) {
    lastIBusUpdateTime = currentTime;
    ibusSignalLost = false;
  }
  // If no changes for timeout period, consider signal lost
  else if (elapsedTime > ibusTimeout) {
    ibusSignalLost = true;
  }

  // Print debug information
  Serial.print("Elapsed: ");
  Serial.print(elapsedTime / 1000.0); // Convert to milliseconds
  Serial.print(" ms | Signal Lost: ");
  Serial.println(ibusSignalLost ? "YES" : "NO");

  // Save current values for next loop
  for (int channel = 0; channel < 10; channel++) {
    lastIBusData[channel] = ibusData[channel];
  }

  // Handle iBUS signal loss
  if (ibusSignalLost) {
    Serial.println("⚠️ iBUS Signal Lost! Resetting Controls...");
    
    // Set to safe values
    ibusData[0] = 1500; // Roll center
    ibusData[1] = 1500; // Pitch center
    ibusData[2] = 1000; // Throttle minimum (CRITICAL)
    ibusData[3] = 1500; // Yaw center
    ibusData[4] = 1000; // Aux1
    ibusData[5] = 1000; // Aux2

    reset_pid();
    reset_motors();
  }
}

void pwmsetup() {
  pinMode(PWM_PIN_M1, OUTPUT);
  pinMode(PWM_PIN_M2, OUTPUT);
  pinMode(PWM_PIN_M3, OUTPUT);
  pinMode(PWM_PIN_M4, OUTPUT);

  // Setup PWM channels (compatible with ESP32 core 2.x)
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION); // Channel 0 for M1
  ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION); // Channel 1 for M2
  ledcSetup(2, PWM_FREQUENCY, PWM_RESOLUTION); // Channel 2 for M3
  ledcSetup(3, PWM_FREQUENCY, PWM_RESOLUTION); // Channel 3 for M4

  // Attach pins to channels
  ledcAttachPin(PWM_PIN_M1, 0); // Attach GPIO 5 to channel 0
  ledcAttachPin(PWM_PIN_M2, 1); // Attach GPIO 18 to channel 1
  ledcAttachPin(PWM_PIN_M3, 2); // Attach GPIO 19 to channel 2
  ledcAttachPin(PWM_PIN_M4, 3); // Attach GPIO 15 to channel 3
}

void pwmloop(int motor_input, int PWM_PIN) {
  // Map GPIO pin to PWM channel
  int channel;
  if (PWM_PIN == PWM_PIN_M1) channel = 0;
  else if (PWM_PIN == PWM_PIN_M2) channel = 1;
  else if (PWM_PIN == PWM_PIN_M3) channel = 2;
  else if (PWM_PIN == PWM_PIN_M4) channel = 3;
  else return; // Invalid pin
  
  ledcWrite(channel, motor_input);
}

float batteryvoltage() {
  int rawValue = analogRead(analogPin);
  float voltage = (rawValue / 4095.0) * referenceVoltage;
  float inputVoltage = voltage * ((r1Value + r2Value) / r2Value);
  return inputVoltage;
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;

  if (Iterm > 400)
    Iterm = 400;
  else if (Iterm < -400)
    Iterm = -400;

  float Dterm = D * (Error - PrevError) / 0.004;
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
  } else {
    Serial.println("I2C mutex not created");
  }
}

// ===================== FREERTOS TASKS =====================

void batteryMonitorTask(void *pvParameters) {
  while (1) {
    Serial.print("\nBATTERY MONITOR TASK\n");
    battery_voltage = batteryvoltage();
    if (battery_voltage < 9) {
      Serial.println("Battery level low please Recharge the Battery");
      digitalWrite(2, HIGH);
      delay(500);
      digitalWrite(2, LOW);
      delay(500);
    } else {
      digitalWrite(2, HIGH);
    }
  }
}

void flightControlTask(void *pvParameters) {
  while (1) {
    gyro_signals();
    
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    // Apply Kalman filter using your tuned implementation
    KalmanAngleRoll = kalmanFilter(rollKalman, RateRoll, AngleRoll, 0.004);
    KalmanAnglePitch = kalmanFilter(pitchKalman, RatePitch, AnglePitch, 0.004);

    ibusloop(); // Changed from ppmloop()
    
    DesiredAngleRoll = 0.10 * (ibusData[0] - 1500);    // Roll
    DesiredAnglePitch = 0.10 * (ibusData[1] - 1500);  // Pitch

    Serial.print(" Roll Angle [°] ");
    Serial.print(KalmanAngleRoll);
    Serial.print(" Pitch Angle [°] ");
    Serial.println(KalmanAnglePitch);

    InputThrottle = ibusData[2];                      // Throttle
    DesiredRateYaw = 0.15 * (ibusData[3] - 1500);    // Yaw
    
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

    if (InputThrottle > 1800) InputThrottle = 1800;

    MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
    MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
    MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
    MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);

    if (MotorInput1 > 2000) MotorInput1 = 1989;
    if (MotorInput2 > 2000) MotorInput2 = 1989;
    if (MotorInput3 > 2000) MotorInput3 = 1989;
    if (MotorInput4 > 2000) MotorInput4 = 1989;

    int ThrottleIdle = 1180;
    if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

    int ThrottleCutOff = 1000;
    if (ibusData[2] < 1050) {  // Changed from ppmData[2]
      MotorInput1 = ThrottleCutOff;
      MotorInput2 = ThrottleCutOff;
      MotorInput3 = ThrottleCutOff;
      MotorInput4 = ThrottleCutOff;
      reset_pid();
    }

    pwmloop(MotorInput1, PWM_PIN_M1);
    pwmloop(MotorInput2, PWM_PIN_M2);
    pwmloop(MotorInput3, PWM_PIN_M3);
    pwmloop(MotorInput4, PWM_PIN_M4);

    while ((micros() - LoopTimer) < 4000);
    LoopTimer = micros();
  }
}

void gyroscope_calibration() {
  Serial.println("Starting gyroscope calibration...");
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  Serial.println("Gyroscope calibration complete!");
}

// ===================== SETUP =====================

void setup() {
  i2cMutex = xSemaphoreCreateMutex();
  Serial.begin(115200);
  
  if (i2cMutex == NULL) {
    Serial.println("Failed to create I2C mutex!");
    while (1);
  }

  pinMode(2, OUTPUT);
  analogReadResolution(12);
  pinMode(analogPin, INPUT);

  // Initialize iBUS on Serial2
  Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);
  ibus.begin(Serial2);
  Serial.println(F("\n=== iBUS Receiver Initialized (FS-iA6B) ==="));

  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("=== Initializing MPU9250 ==="));
  
  int status = IMU.begin();
  if (status < 0) {
    Serial.print(F("IMU initialization failed with code: "));
    Serial.println(status);
    while (1) {
      delay(100);
    }
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  IMU.setSrd(19); // Sample rate divider for 50 Hz

  Serial.println(F("IMU Ready!"));
  
  delay(250);
  
  gyroscope_calibration();
  pwmsetup();
  LoopTimer = micros();

  xTaskCreatePinnedToCore(batteryMonitorTask, "Battery monitor Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(flightControlTask, "Flight Control Task", 9216, NULL, 1, NULL, 1);

  Serial.println(F("Flight controller ready with iBUS!"));
}

void loop() {
  // Empty - FreeRTOS tasks handle everything
}