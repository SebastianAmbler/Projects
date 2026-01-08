/*
====================================================================================
🛩️  ESP32 Quadcopter Flight Controller with MPU9250 (FreeRTOS + PPM)
====================================================================================
Integration of tuned MPU9250 sensor with flight controller using PPM receiver
Fixed: PPM Signal Logic, Level Calibration, S3 ADC Pin, SPI Conflicts
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
// NOTE: Ensure your MPU9250 CS pin is connected to GPIO 10 (or change below)
MPU9250 IMU(SPI, 10);

// Calibration constants from your tuned code
constexpr float ACCEL_BIAS[3] = {0.22f, 0.20f, -0.13f};
constexpr float ACCEL_SCALE[3] = {1.00f, 1.00f, 0.99f};
constexpr float GYRO_BIAS[3] = {0.01f, 0.04f, -0.01f};
constexpr float MAG_BIAS[3] = {33.17f, 20.35f, -25.80f};
constexpr float MAG_SCALE[3] = {1.03f, 0.97f, 1.00f};

constexpr float G_TO_MPS2 = 9.80665f;

// Kalman filter parameters
constexpr float Q_ANGLE = 0.002f;
constexpr float Q_BIAS = 0.003f;
constexpr float R_MEASURE = 0.02f;

struct KalmanState {
  float angle;
  float bias;
  float rate;
  float P[2][2];
};

KalmanState pitchKalman = {0, 0, 0, {{0, 0}, {0, 0}}};
KalmanState rollKalman = {0, 0, 0, {{0, 0}, {0, 0}}};

// ===================== MUTEXES =====================
SemaphoreHandle_t sensorMutex;

// ===================== PID TUNING =====================
float PRate = 0.08, IRate = 0.04, DRate = 0.001;
float PAngle = 4, IAngle = 0.04, DAngle = 0.0;

// ===================== BATTERY MONITORING =====================
// CHANGED: GPIO 25 is NOT an ADC on ESP32-S3. Changed to GPIO 1.
// MOVE YOUR VOLTAGE DIVIDER WIRE TO GPIO 1!
const int analogPin = 17; 
float referenceVoltage = 3.3;
float r1Value = 77600.0;
float r2Value = 29400.0;
float battery_voltage = 10;

// ===================== PWM MOTOR CONTROL =====================
const int PWM_PIN_M1 = 6;   // Motor 1 - Counter-clockwise
const int PWM_PIN_M2 = 7;  // Motor 2 - Clockwise
const int PWM_PIN_M3 = 15;  // Motor 3 - Counter-clockwise
const int PWM_PIN_M4 = 41;  // Motor 4 - Clockwise
const int PWM_FREQUENCY = 250;
const int PWM_RESOLUTION = 12;

// ===================== PPM RECEIVER (FS-iA6B) =====================
byte ppmInterruptPin = 5;      // PPM input pin (interrupt-capable)
byte channelAmount = 8;         // FS-iA6B has 6 channels in PPM mode
PPMReader* ppm = nullptr;
int ppmData[10] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000}; // Default safe values

// PPM timeout
const unsigned long ppmTimeout = 2000000; // 2 second timeout
unsigned long lastPPMUpdateTime = 0;
bool ppmSignalLost = true;
int lastPPMData[10] = {0};

// ===================== FLIGHT CONTROL VARIABLES =====================
float RateRoll, RatePitch, RateYaw;
// Calibration Offsets
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
float AngleCalibrationRoll = 0, AngleCalibrationPitch = 0;

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

// Forward declarations
void reset_pid(void);
void reset_motors();

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

// Debug counter for reduced serial output
unsigned long debugCounter = 0;

void ppmloop() {
  unsigned long currentTime = micros();
  int validChannelCount = 0;

  // Read all channels
  for (int channel = 0; channel < channelAmount; channel++) {
    // get value, return 0 if invalid/timed out
    int value = ppm->latestValidChannelValue(channel + 1, 0); 
    
    // Check if the value is within a valid PPM range (e.g., 800us to 2200us)
    if (value >= 800 && value <= 2200) {
      ppmData[channel] = value;
      lastPPMData[channel] = value;
      validChannelCount++;
    }
  }

  // === SIGNAL LOSS LOGIC FIXED ===
  // If we received valid data for at least 4 channels, the link is ALIVE.
  if (validChannelCount >= 4) {
    lastPPMUpdateTime = currentTime;
    ppmSignalLost = false;
  }
  
  // Calculate time since last valid packet
  unsigned long elapsedTime = currentTime - lastPPMUpdateTime;

  if (elapsedTime > ppmTimeout) {
    ppmSignalLost = true;
  }

  // --- Failsafe ---
  if (ppmSignalLost) {
    // Set failsafe values
    ppmData[0] = 1500;
    ppmData[1] = 1500;
    ppmData[2] = 1000; // Throttle down
    ppmData[3] = 1500;
    ppmData[4] = 1000;
    ppmData[5] = 1000;
    
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
  ledcAttachPin(PWM_PIN_M1, 0); 
  ledcAttachPin(PWM_PIN_M2, 1); 
  ledcAttachPin(PWM_PIN_M3, 2); 
  ledcAttachPin(PWM_PIN_M4, 3); 
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
  if (sensorMutex != NULL) {
    if (xSemaphoreTake(sensorMutex, (TickType_t)10) == pdTRUE) {
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

      xSemaphoreGive(sensorMutex);
    }
  }
}

// ===================== CALIBRATION FUNCTION =====================
void gyroscope_calibration() {
  Serial.println("=== CALIBRATING GYRO & LEVEL ===");
  digitalWrite(2, HIGH); 

  RateCalibrationRoll = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw = 0;
  AngleCalibrationRoll = 0;
  AngleCalibrationPitch = 0;

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    
    // Accumulate Gyro (Speed)
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    
    // Accumulate Angle (Position)
    AngleCalibrationRoll += AngleRoll;
    AngleCalibrationPitch += AnglePitch;
    
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }

  // Calculate Averages
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  
  AngleCalibrationRoll /= 2000;
  AngleCalibrationPitch /= 2000;

  digitalWrite(2, LOW);
  
  Serial.print("Gyro Offsets -> R: "); Serial.print(RateCalibrationRoll);
  Serial.print(" P: "); Serial.println(RateCalibrationPitch);
  Serial.print("Level Offsets -> R: "); Serial.print(AngleCalibrationRoll);
  Serial.print(" P: "); Serial.println(AngleCalibrationPitch);
}

// ===================== FREERTOS TASKS =====================

void batteryMonitorTask(void *pvParameters) {
  while (1) {
    battery_voltage = batteryvoltage();
    if (battery_voltage < 9) {
      Serial.println("⚠️  Battery level low - please Recharge!");
      // Flash SOS or similar
      digitalWrite(2, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(2, LOW); vTaskDelay(pdMS_TO_TICKS(100));
    } else {
      // Heartbeat blink
      digitalWrite(2, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
      digitalWrite(2, LOW); vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

void flightControlTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(4); // 4ms = 250Hz loop
  
  bool previousCh5State = false;

  while (1) {
    // 1. Read Sensors and Radio
    gyro_signals();
    ppmloop(); 

    // ================== CALIBRATION LOGIC (Channel 5) ==================
    bool currentCh5State = ppmData[4] > 1800; // Aux 1 is ON
    bool isThrottleLow = ppmData[2] < 1050;   // Throttle is LOW

    if (currentCh5State && !previousCh5State && isThrottleLow) {
      reset_motors();
      pwmloop(1000, PWM_PIN_M1); pwmloop(1000, PWM_PIN_M2);
      pwmloop(1000, PWM_PIN_M3); pwmloop(1000, PWM_PIN_M4);
      
      gyroscope_calibration(); // Run calibration
      
      reset_pid();
      xLastWakeTime = xTaskGetTickCount(); // Reset loop timer
    }
    previousCh5State = currentCh5State;
    // ===================================================================

    // Apply Gyro Calibration
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    // Apply Level Calibration (Zero the accelerometer)
    AngleRoll -= AngleCalibrationRoll;
    AnglePitch -= AngleCalibrationPitch;

    // Apply Kalman filter
    KalmanAngleRoll = kalmanFilter(rollKalman, RateRoll, AngleRoll, 0.004);
    KalmanAnglePitch = kalmanFilter(pitchKalman, RatePitch, AnglePitch, 0.004);

    // ================== DEBUG PRINTING ==================
    debugCounter++;
    if (debugCounter % 50 == 0) {
      Serial.print("Roll: ");
      Serial.print(KalmanAngleRoll, 1);
      Serial.print("° | Pitch: ");
      Serial.print(KalmanAnglePitch, 1);
      Serial.println("°");
    }

    DesiredAngleRoll = 0.10 * (ppmData[0] - 1500);    
    DesiredAnglePitch = 0.10 * (ppmData[1] - 1500);  

    InputThrottle = ppmData[2];                      
    DesiredRateYaw = 0.15 * (ppmData[3] - 1500);    
    
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
    // Safety check - if throttle is low, cut motors
    if (ppmData[2] < 1050) {
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

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ===================== SETUP =====================

void setup() {
  sensorMutex = xSemaphoreCreateMutex();
  Serial.begin(115200);
  
  if (sensorMutex == NULL) {
    Serial.println("Failed to create sensor mutex!");
    while (1);
  }

  // ADC Setup for ESP32-S3 (GPIO 1 is valid ADC1_CH0)
  pinMode(2, OUTPUT);
  analogReadResolution(12);
  pinMode(analogPin, INPUT);

  // Initialize PPM
  ppm = new PPMReader(ppmInterruptPin, channelAmount);
  Serial.println(F("\n=== PPM Receiver Initialized (FS-iA6B) ==="));

  // Initialize MPU9250
  Serial.println(F("=== Initializing MPU9250 (SPI) ==="));
  int status = IMU.begin();
  if (status < 0) {
    Serial.print(F("IMU initialization failed with code: "));
    Serial.println(status);
    while (1) delay(100);
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  
  // === FIX IS HERE ===
  // 19 = 50Hz (Too Slow). 0 = 1000Hz (Fastest). 
  // We want fresh data every 4ms (250Hz), so 0 ensures data is always ready.
  IMU.setSrd(0); 
  
  // Enable Low Pass Filter (DLPF) to smooth out vibration before Kalman
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ); 

  Serial.println(F("IMU Ready!"));
  delay(250);
  
  // Initial Calibration
  gyroscope_calibration();
  
  pwmsetup();
  LoopTimer = micros();

  // Create Tasks
  xTaskCreatePinnedToCore(batteryMonitorTask, "Battery", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(flightControlTask, "Flight", 9216, NULL, 2, NULL, 1); // Priority 2 for Flight Task

  Serial.println(F("Flight controller ready!"));
}
void loop() {
  // Empty - FreeRTOS tasks handle everything
}