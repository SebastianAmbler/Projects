/*
====================================================================================
🛩️  ESP32 Quadcopter Flight Controller with MPU9250 (FreeRTOS + PPM)
====================================================================================
Integration of tuned MPU9250 sensor with flight controller using PPM receiver
Fixed: Motor Mixing, PPM Signal Logic, Level Calibration, GPIO Pins, ESC Arming
**NEW: CH5 Arming Switch - Throttle must be low to arm/disarm**
====================================================================================
Motor Layout:
        FRONT
          ↑
    M1(CCW)   M2(CW)
        \   /
         \ /
          X
         / \
        /   \
    M3(CW)    M4(CCW)
====================================================================================
*/

#include <Wire.h>
#include <Arduino.h>
#include <PPMReader.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include "MPU9250.h"
#define RGB_LED_PIN 38

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

// Add these global variables at the top with other flight control variables
float FilteredAccX = 0, FilteredAccY = 0, FilteredAccZ = 0;
float FilteredRateRoll = 0, FilteredRatePitch = 0, FilteredRateYaw = 0;

// Low-pass filter coefficient (0.0 to 1.0)
// Lower = more filtering (slower response)
// Higher = less filtering (faster response)
const float ACCEL_FILTER_ALPHA = 0.7;  // 70% new data, 30% old (moderate filtering)
const float GYRO_FILTER_ALPHA = 0.8;   // 80% new data, 20% old (light filtering)

// ===================== MUTEXES =====================
SemaphoreHandle_t sensorMutex;

float PRate = 0.3;    // Reduced from 0.6 (was too aggressive)
float IRate = 0.0;    // Set to 0 for initial testing
float DRate = 0.01;   // Reduced from 0.03

float PAngle = 1.2;   // Reduced from 3.0 (CRITICAL - was way too high)
float IAngle = 0.0;   // Keep at 0
float DAngle = 0.0;   // Keep at 0


// ===================== ARMING SYSTEM =====================
bool isArmed = false;
bool prevArmSwitch = false;
const int ARM_THRESHOLD_LOW = 1200;   // CH5 below this = disarmed
const int ARM_THRESHOLD_HIGH = 1700;  // CH5 above this = armed
const int THROTTLE_ARM_MAX = 1150;    // Throttle must be below this to arm/disarm

// ===================== BATTERY MONITORING =====================
const int analogPin = 2; 
float referenceVoltage = 3.3;
float r1Value = 100000.0;
float r2Value = 27000.0;
float battery_voltage = 10;

// ===================== PWM MOTOR CONTROL =====================
const int PWM_PIN_M1 = 15;   // Front Left - CCW
const int PWM_PIN_M2 = 1;    // Front Right - CW
const int PWM_PIN_M3 = 7;    // Rear Left - CW
const int PWM_PIN_M4 = 6;    // Rear Right - CCW
const int PWM_FREQUENCY = 250;  
const int PWM_RESOLUTION = 12;

// ===================== PPM RECEIVER (FS-iA6B) =====================
byte ppmInterruptPin = 17;
byte channelAmount = 6; 
PPMReader* ppm = nullptr;
int ppmData[10] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000};

// PPM timeout
const unsigned long ppmTimeout = 500000;
unsigned long lastPPMUpdateTime = 0;
bool ppmSignalLost = true;
int lastPPMData[10] = {0};

// ===================== FLIGHT CONTROL VARIABLES =====================
float RateRoll, RatePitch, RateYaw;
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
  state.rate = gyroRate - state.bias;
  state.angle += dt * state.rate;

  state.P[0][0] += dt * (dt * state.P[1][1] - state.P[0][1] - state.P[1][0] + Q_ANGLE);
  state.P[0][1] -= dt * state.P[1][1];
  state.P[1][0] -= dt * state.P[1][1];
  state.P[1][1] += Q_BIAS * dt;

  float S = state.P[0][0] + R_MEASURE;
  float K[2];
  K[0] = state.P[0][0] / S;
  K[1] = state.P[1][0] / S;

  float y = accelAngle - state.angle;
  state.angle += K[0] * y;
  state.bias += K[1] * y;

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

unsigned long debugCounter = 0;

void checkArmingSwitch() {
  int ch5Value = ppmData[4];  // Channel 5 (index 4)
  int throttle = ppmData[2];   // Throttle channel
  bool currentArmSwitch = (ch5Value > ARM_THRESHOLD_HIGH);
  
  // Only allow arming/disarming when throttle is low
  if (throttle < THROTTLE_ARM_MAX) {
    // Detect rising edge (switch flipped to armed position)
    if (currentArmSwitch && !prevArmSwitch && !isArmed) {
      isArmed = true;
      Serial.println("✅ ARMED - Motors active!");
      neopixelWrite(RGB_LED_PIN, 0, 0, 80);  // Blue = Armed
      delay(100);
      neopixelWrite(RGB_LED_PIN, 0, 0, 0);
    }
    // Detect falling edge (switch flipped to disarmed position)
    else if (!currentArmSwitch && prevArmSwitch && isArmed) {
      isArmed = false;
      reset_motors();
      reset_pid();
      Serial.println("🔴 DISARMED - Motors stopped");
      neopixelWrite(RGB_LED_PIN, 80, 0, 0);  // Red = Disarmed
      delay(100);
      neopixelWrite(RGB_LED_PIN, 0, 0, 0);
    }
  } else {
    // If someone tries to arm with throttle up, prevent it
    if (currentArmSwitch && !prevArmSwitch && !isArmed) {
      Serial.println("⚠️  Cannot arm: Lower throttle first!");
      for (int i = 0; i < 3; i ++) {
        neopixelWrite(RGB_LED_PIN, 80, 40, 0);  // Orange warning
        delay(100);
        neopixelWrite(RGB_LED_PIN, 0, 0, 0);
        delay(100);
      }
    }
  }
  
  prevArmSwitch = currentArmSwitch;
}

void ppmloop() {
  unsigned long currentTime = micros();
  int validChannelCount = 0;

  for (int channel = 0; channel < channelAmount; channel++) {
    int value = ppm->latestValidChannelValue(channel + 1, 0); 
    
    if (value >= 800 && value <= 2200) {
      ppmData[channel] = value;
      lastPPMData[channel] = value;
      validChannelCount++;
    }
  }

  if (validChannelCount >= 4) {
    lastPPMUpdateTime = currentTime;
    ppmSignalLost = false;
  }
  
  unsigned long elapsedTime = currentTime - lastPPMUpdateTime;

  if (elapsedTime > ppmTimeout) {
    ppmSignalLost = true;
  }

  if (ppmSignalLost) {
    ppmData[0] = 1500;
    ppmData[1] = 1500;
    ppmData[2] = 1000;
    ppmData[3] = 1500;
    ppmData[4] = 1000;
    ppmData[5] = 1000;
    
    // Auto-disarm on signal loss
    if (isArmed) {
      isArmed = false;
      Serial.println("🔴 SIGNAL LOST - Auto-disarmed!");
    }
    
    reset_pid();
    reset_motors();
  }
}

void pwmsetup() {
  pinMode(PWM_PIN_M1, OUTPUT);
  pinMode(PWM_PIN_M2, OUTPUT);
  pinMode(PWM_PIN_M3, OUTPUT);
  pinMode(PWM_PIN_M4, OUTPUT);

  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(PWM_PIN_M1, 0); 
  ledcAttachPin(PWM_PIN_M2, 1); 
  ledcAttachPin(PWM_PIN_M3, 2); 
  ledcAttachPin(PWM_PIN_M4, 3); 
}

void pwmloop(int motor_input, int PWM_PIN) {
  int channel;
  if (PWM_PIN == PWM_PIN_M1) channel = 0;
  else if (PWM_PIN == PWM_PIN_M2) channel = 1;
  else if (PWM_PIN == PWM_PIN_M3) channel = 2;
  else if (PWM_PIN == PWM_PIN_M4) channel = 3;
  else return;
  
  // Convert microseconds (1000-2000) to 12-bit duty cycle (0-4095)
  // PWM period at 250Hz = 4000μs
  int dutyCycle = map(motor_input, 0, 4000, 0, 4095);
  dutyCycle = constrain(dutyCycle, 0, 4095);
  
  ledcWrite(channel, dutyCycle);
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
      IMU.readSensor();

      float rawAccel[3] = {IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss()};
      float rawGyro[3] = {IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads()};

      float calAccel[3], calGyro[3];
      calibrateIMU(rawAccel, ACCEL_BIAS, ACCEL_SCALE, calAccel, true);
      calibrateIMU(rawGyro, GYRO_BIAS, nullptr, calGyro);

      float tempAccel[3], tempGyro[3];
      remapAxes(calAccel, tempAccel, true);
      remapAxes(calGyro, tempGyro, true);

      // Raw values before filtering
      float rawRateRoll  = -tempGyro[0] * RAD_TO_DEG;
      float rawRatePitch = -tempGyro[1] * RAD_TO_DEG;
      float rawRateYaw   =  tempGyro[2] * RAD_TO_DEG;
      float rawAccX = tempAccel[0];
      float rawAccY = tempAccel[1];
      float rawAccZ = tempAccel[2];

      // Apply low-pass filter (Exponential Moving Average)
      FilteredRateRoll  = (GYRO_FILTER_ALPHA * rawRateRoll)  + ((1.0 - GYRO_FILTER_ALPHA) * FilteredRateRoll);
      FilteredRatePitch = (GYRO_FILTER_ALPHA * rawRatePitch) + ((1.0 - GYRO_FILTER_ALPHA) * FilteredRatePitch);
      FilteredRateYaw   = (GYRO_FILTER_ALPHA * rawRateYaw)   + ((1.0 - GYRO_FILTER_ALPHA) * FilteredRateYaw);
      
      FilteredAccX = (ACCEL_FILTER_ALPHA * rawAccX) + ((1.0 - ACCEL_FILTER_ALPHA) * FilteredAccX);
      FilteredAccY = (ACCEL_FILTER_ALPHA * rawAccY) + ((1.0 - ACCEL_FILTER_ALPHA) * FilteredAccY);
      FilteredAccZ = (ACCEL_FILTER_ALPHA * rawAccZ) + ((1.0 - ACCEL_FILTER_ALPHA) * FilteredAccZ);

      // Use filtered values
      RateRoll  = FilteredRateRoll;
      RatePitch = FilteredRatePitch;
      RateYaw   = FilteredRateYaw;
      AccX = FilteredAccX;
      AccY = FilteredAccY;
      AccZ = FilteredAccZ;

      // Calculate angles from filtered accelerometer
      AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEG;
      AnglePitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG;

      xSemaphoreGive(sensorMutex);
    }
  }
}

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
    
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    
    AngleCalibrationRoll += AngleRoll;
    AngleCalibrationPitch += AnglePitch;
    
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }

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
    if (battery_voltage < 11) {
      Serial.println("⚠️  Battery level low - please Recharge!");
      neopixelWrite(RGB_LED_PIN, 80, 0, 0);
      vTaskDelay(pdMS_TO_TICKS(1000));
      neopixelWrite(RGB_LED_PIN, 0, 0, 0); 
      vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
      // Show armed/disarmed status on battery check
      if (isArmed) {
        neopixelWrite(RGB_LED_PIN, 0, 0, 80);  // Blue when armed
      } else {
        neopixelWrite(RGB_LED_PIN, 0, 80, 0);  // Green when disarmed
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      neopixelWrite(RGB_LED_PIN, 0, 0, 0); 
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void flightControlTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(4);

  while (1) {
    gyro_signals();
    ppmloop(); 
    checkArmingSwitch();  // Check CH5 arming switch

    // Apply gyro calibration
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    AngleRoll -= AngleCalibrationRoll;
    AnglePitch -= AngleCalibrationPitch;

    KalmanAngleRoll = kalmanFilter(rollKalman, RateRoll, AngleRoll, 0.004);
    KalmanAnglePitch = kalmanFilter(pitchKalman, RatePitch, AnglePitch, 0.004);

    debugCounter++;
 // Add this to flightControlTask() - Replace your existing debug output

debugCounter++;
if (debugCounter % 25 == 0) {
    // Calculate vibration metric
    float vibration = abs(AccX) + abs(AccY);
    
    // Calculate acceleration variance (detects oscillations)
    static float prevAccX = 0, prevAccY = 0, prevAccZ = 0;
    float accelChange = abs(AccX - prevAccX) + abs(AccY - prevAccY) + abs(AccZ - prevAccZ);
    prevAccX = AccX;
    prevAccY = AccY;
    prevAccZ = AccZ;
    
    Serial.print("Thr: "); Serial.print(InputThrottle);
    Serial.print(" | AccZ: "); Serial.print(AccZ, 2);
    Serial.print(" | Vib: "); Serial.print(vibration, 2);
    Serial.print(" | Jitter: "); Serial.print(accelChange, 2);
    Serial.print(" | Roll: "); Serial.print(KalmanAngleRoll, 1);
    Serial.print("° | Pitch: "); Serial.print(KalmanAnglePitch, 1);
    Serial.print("° | Armed: "); Serial.println(isArmed ? "YES" : "NO");
}

// CRITICAL: Add motor output monitoring
if (debugCounter % 100 == 0 && isArmed) {
    Serial.print("MOTORS -> M1:");
    Serial.print(MotorInput1);
    Serial.print(" M2:");
    Serial.print(MotorInput2);
    Serial.print(" M3:");
    Serial.print(MotorInput3);
    Serial.print(" M4:");
    Serial.println(MotorInput4);
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

    // Motor mixing - ONLY if armed
    // M1 (Front Left - CCW): Throttle + Roll - Pitch + Yaw
    // M2 (Front Right - CW): Throttle - Roll - Pitch - Yaw
    // M3 (Rear Left - CW):   Throttle + Roll + Pitch - Yaw
    // M4 (Rear Right - CCW): Throttle - Roll + Pitch + Yaw
    if (isArmed && ppmData[2] >= 1150) {
      MotorInput1 = (InputThrottle + InputRoll - InputPitch + InputYaw); 
      MotorInput2 = (InputThrottle - InputRoll - InputPitch - InputYaw); 
      MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw); 
      MotorInput4 = (InputThrottle - InputRoll + InputPitch + InputYaw);

      if (MotorInput1 > 2000) MotorInput1 = 1989;
      if (MotorInput2 > 2000) MotorInput2 = 1989;
      if (MotorInput3 > 2000) MotorInput3 = 1989;
      if (MotorInput4 > 2000) MotorInput4 = 1989;

      int ThrottleIdle = 1350;
      if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
      if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
      if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
      if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;
    } else {
      // Disarmed or throttle too low - motors off
      MotorInput1 = 1000;
      MotorInput2 = 1000;
      MotorInput3 = 1000;
      MotorInput4 = 1000;
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
  // === STEP 1: SETUP PWM IMMEDIATELY (BEFORE SERIAL!) ===
  pinMode(PWM_PIN_M1, OUTPUT);
  pinMode(PWM_PIN_M2, OUTPUT);
  pinMode(PWM_PIN_M3, OUTPUT);
  pinMode(PWM_PIN_M4, OUTPUT);

  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(PWM_PIN_M1, 0); 
  ledcAttachPin(PWM_PIN_M2, 1); 
  ledcAttachPin(PWM_PIN_M3, 2); 
  ledcAttachPin(PWM_PIN_M4, 3);
  
  // Send 1000μs immediately for ESC arming
  ledcWrite(0, 1024);
  ledcWrite(1, 1024);
  ledcWrite(2, 1024);
  ledcWrite(3, 1024);
  
  delay(3000); // Wait for ESC initialization
  
  // === STEP 2: NOW initialize Serial and other components ===
  sensorMutex = xSemaphoreCreateMutex();
  Serial.begin(115200);
  
  if (sensorMutex == NULL) {
    Serial.println("Failed to create sensor mutex!");
    while (1);
  }

  Serial.println("=== ESC ARMING SIGNAL SENT ===");
  
  pinMode(2, OUTPUT);
  analogReadResolution(12);
  pinMode(analogPin, INPUT);
  
  ppm = new PPMReader(ppmInterruptPin, channelAmount);
  Serial.println(F("=== PPM Receiver Initialized ==="));

  Serial.println(F("=== Initializing MPU9250 (SPI) ==="));
  int status = IMU.begin();
  if (status < 0) {
    Serial.print(F("IMU initialization failed with code: "));
    Serial.println(status);
    while (1) delay(100);
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  IMU.setSrd(0); 
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ); 

  Serial.println(F("IMU Ready!"));
  delay(250); 
  
  gyroscope_calibration();
  
  LoopTimer = micros();

  Serial.println("=== Flight controller ready! ===");
  Serial.println("🔴 DISARMED - Flip CH5 switch to arm");
  Serial.println("⚠️  Throttle must be LOW to arm/disarm");

  xTaskCreatePinnedToCore(batteryMonitorTask, "Battery", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(flightControlTask, "Flight", 9216, NULL, 2, NULL, 1);
}

void loop() {
  // Empty - FreeRTOS tasks handle everything
}