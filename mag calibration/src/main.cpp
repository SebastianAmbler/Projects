#include <Wire.h>
#include "MPU9250.h"

MPU9250 IMU(SPI, 10);

// ===================== CONFIGURATION =====================
constexpr int CALIBRATION_SAMPLES = 2000;  // Number of samples to collect
constexpr int SAMPLE_DELAY_MS = 10;        // Delay between samples
constexpr unsigned long CALIBRATION_TIME_MS = CALIBRATION_SAMPLES * SAMPLE_DELAY_MS;

// Use existing calibration for accel (for axis remapping verification)
constexpr float ACCEL_BIAS[3] = {0.22f, 0.20f, -0.13f};
constexpr float ACCEL_SCALE[3] = {1.00f, 1.00f, 0.99f};
constexpr float G_TO_MPS2 = 9.80665f;

// ===================== CALIBRATION DATA =====================
struct MagData {
  float x, y, z;
};

MagData magMin = {1e6, 1e6, 1e6};
MagData magMax = {-1e6, -1e6, -1e6};
MagData magBias = {0, 0, 0};
MagData magScale = {1.0f, 1.0f, 1.0f};

int sampleCount = 0;
unsigned long startTime = 0;

// ===================== FUNCTION PROTOTYPES =====================
void remapAxes(const float in[3], float out[3], bool invertZ = false);
void printProgress(int current, int total);
void calculateCalibration();
void printCalibrationResults();
void testCalibration();

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("\n========================================"));
  Serial.println(F("   MPU9250 Magnetometer Calibration"));
  Serial.println(F("========================================\n"));

  // Initialize MPU9250
  int status = IMU.begin();
  if (status < 0) {
    Serial.print(F("ERROR: IMU initialization failed with code: "));
    Serial.println(status);
    Serial.println(F("Check wiring and restart."));
    while (1) { delay(100); }
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  IMU.setSrd(19); // 50 Hz sample rate

  Serial.println(F("IMU Initialized Successfully!\n"));
  
  Serial.println(F("╔════════════════════════════════════════╗"));
  Serial.println(F("║        CALIBRATION INSTRUCTIONS        ║"));
  Serial.println(F("╠════════════════════════════════════════╣"));
  Serial.println(F("║ 1. Slowly rotate the sensor in ALL    ║"));
  Serial.println(F("║    directions (figure-8 pattern works) ║"));
  Serial.println(F("║                                        ║"));
  Serial.println(F("║ 2. Try to cover all orientations:     ║"));
  Serial.println(F("║    - Roll left/right                   ║"));
  Serial.println(F("║    - Pitch forward/backward            ║"));
  Serial.println(F("║    - Yaw (spin) clockwise/counter      ║"));
  Serial.println(F("║                                        ║"));
  Serial.println(F("║ 3. Keep moving for entire duration    ║"));
  Serial.println(F("║                                        ║"));
  Serial.println(F("║ 4. Avoid magnetic interference:        ║"));
  Serial.println(F("║    - Stay away from metal objects      ║"));
  Serial.println(F("║    - Keep away from motors/speakers    ║"));
  Serial.println(F("║    - Remove magnetic jewelry           ║"));
  Serial.println(F("╚════════════════════════════════════════╝\n"));

  Serial.println(F("Calibration will start in 3 seconds...\n"));
  delay(1000);
  Serial.println(F("3..."));
  delay(1000);
  Serial.println(F("2..."));
  delay(1000);
  Serial.println(F("1..."));
  delay(1000);
  Serial.println(F("\n>>> START MOVING THE SENSOR NOW! <<<\n"));
  
  startTime = millis();
}

// ===================== MAIN LOOP =====================
void loop() {
  if (sampleCount < CALIBRATION_SAMPLES) {
    // Read magnetometer data
    IMU.readSensor();
    
    float rawMag[3] = {IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT()};
    float remappedMag[3];
    remapAxes(rawMag, remappedMag, true);
    
    // Update min/max values
    if (remappedMag[0] < magMin.x) magMin.x = remappedMag[0];
    if (remappedMag[0] > magMax.x) magMax.x = remappedMag[0];
    
    if (remappedMag[1] < magMin.y) magMin.y = remappedMag[1];
    if (remappedMag[1] > magMax.y) magMax.y = remappedMag[1];
    
    if (remappedMag[2] < magMin.z) magMin.z = remappedMag[2];
    if (remappedMag[2] > magMax.z) magMax.z = remappedMag[2];
    
    sampleCount++;
    
    // Print progress every 50 samples
    if (sampleCount % 50 == 0) {
      printProgress(sampleCount, CALIBRATION_SAMPLES);
    }
    
    delay(SAMPLE_DELAY_MS);
    
  } else if (sampleCount == CALIBRATION_SAMPLES) {
    // Calibration complete
    Serial.println(F("\n\n>>> CALIBRATION COMPLETE! <<<\n"));
    
    calculateCalibration();
    printCalibrationResults();
    
    Serial.println(F("\n========================================"));
    Serial.println(F("      Testing Calibrated Values"));
    Serial.println(F("========================================\n"));
    
    sampleCount++; // Move to next state
    delay(2000);
    
  } else {
    // Continuously test calibration
    testCalibration();
    delay(100);
  }
}

// ===================== HELPER FUNCTIONS =====================

/**
 * Remap sensor axes to robot frame (same as main code)
 */
void remapAxes(const float in[3], float out[3], bool invertZ) {
  out[0] = in[1];
  out[1] = -in[0];
  out[2] = invertZ ? -in[2] : in[2];
}

/**
 * Print calibration progress bar
 */
void printProgress(int current, int total) {
  int percent = (current * 100) / total;
  int barWidth = 30;
  int pos = (barWidth * current) / total;
  
  unsigned long elapsed = millis() - startTime;
  unsigned long remaining = (CALIBRATION_TIME_MS - elapsed) / 1000;
  
  Serial.print(F("Progress: ["));
  for (int i = 0; i < barWidth; i++) {
    if (i < pos) Serial.print(F("█"));
    else Serial.print(F("░"));
  }
  Serial.print(F("] "));
  Serial.print(percent);
  Serial.print(F("% ("));
  Serial.print(remaining);
  Serial.println(F("s remaining)"));
}

/**
 * Calculate bias and scale factors
 */
void calculateCalibration() {
  // Hard iron correction (bias)
  magBias.x = (magMax.x + magMin.x) / 2.0f;
  magBias.y = (magMax.y + magMin.y) / 2.0f;
  magBias.z = (magMax.z + magMin.z) / 2.0f;
  
  // Soft iron correction (scale)
  float avgDelta = ((magMax.x - magMin.x) + (magMax.y - magMin.y) + (magMax.z - magMin.z)) / 3.0f;
  
  magScale.x = avgDelta / (magMax.x - magMin.x);
  magScale.y = avgDelta / (magMax.y - magMin.y);
  magScale.z = avgDelta / (magMax.z - magMin.z);
}

/**
 * Print calibration results in copy-paste format
 */
void printCalibrationResults() {
  Serial.println(F("╔════════════════════════════════════════╗"));
  Serial.println(F("║       CALIBRATION RESULTS              ║"));
  Serial.println(F("╚════════════════════════════════════════╝\n"));
  
  Serial.println(F("Raw min/max values (after axis remapping):"));
  Serial.print(F("  X: ")); Serial.print(magMin.x, 2); Serial.print(F(" to ")); Serial.println(magMax.x, 2);
  Serial.print(F("  Y: ")); Serial.print(magMin.y, 2); Serial.print(F(" to ")); Serial.println(magMax.y, 2);
  Serial.print(F("  Z: ")); Serial.print(magMin.z, 2); Serial.print(F(" to ")); Serial.println(magMax.z, 2);
  
  Serial.println(F("\n┌────────────────────────────────────────┐"));
  Serial.println(F("│   COPY THESE VALUES TO YOUR CODE      │"));
  Serial.println(F("└────────────────────────────────────────┘\n"));
  
  Serial.println(F("// Magnetometer biases (uT) and scale factors"));
  Serial.print(F("constexpr float MAG_BIAS[3] = {"));
  Serial.print(magBias.x, 2); Serial.print(F("f, "));
  Serial.print(magBias.y, 2); Serial.print(F("f, "));
  Serial.print(magBias.z, 2); Serial.println(F("f};"));
  
  Serial.print(F("constexpr float MAG_SCALE[3] = {"));
  Serial.print(magScale.x, 2); Serial.print(F("f, "));
  Serial.print(magScale.y, 2); Serial.print(F("f, "));
  Serial.print(magScale.z, 2); Serial.println(F("f};"));
  
  Serial.println();
}

/**
 * Test calibration in real-time
 */
void testCalibration() {
  IMU.readSensor();
  
  float rawMag[3] = {IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT()};
  float remappedMag[3];
  remapAxes(rawMag, remappedMag, true);
  
  // Apply calibration
  float calibratedMag[3];
  calibratedMag[0] = (remappedMag[0] - magBias.x) * magScale.x;
  calibratedMag[1] = (remappedMag[1] - magBias.y) * magScale.y;
  calibratedMag[2] = (remappedMag[2] - magBias.z) * magScale.z;
  
  // Calculate magnitude
  float magnitude = sqrt(calibratedMag[0] * calibratedMag[0] + 
                        calibratedMag[1] * calibratedMag[1] + 
                        calibratedMag[2] * calibratedMag[2]);
  
  Serial.println(F("─────────────────────────────────────────"));
  Serial.print(F("Raw Mag [uT]:        "));
  Serial.print(remappedMag[0], 2); Serial.print(F("\t"));
  Serial.print(remappedMag[1], 2); Serial.print(F("\t"));
  Serial.println(remappedMag[2], 2);
  
  Serial.print(F("Calibrated Mag [uT]: "));
  Serial.print(calibratedMag[0], 2); Serial.print(F("\t"));
  Serial.print(calibratedMag[1], 2); Serial.print(F("\t"));
  Serial.println(calibratedMag[2], 2);
  
  Serial.print(F("Magnitude: "));
  Serial.print(magnitude, 2);
  Serial.println(F(" uT"));
  
  // Magnitude should be relatively constant (~25-65 uT depending on location)
  Serial.print(F("Quality: "));
  if (magnitude > 20 && magnitude < 70) {
    Serial.println(F("✓ GOOD"));
  } else {
    Serial.println(F("⚠ CHECK CALIBRATION"));
  }
  
  Serial.println();
}