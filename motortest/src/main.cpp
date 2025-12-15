/*
 * BLDC Motor Testing for 4x Motors with SimonK ESC Protocol
 * 
 * Connections:
 * Motor 1 ESC Signal -> Pin 3
 * Motor 2 ESC Signal -> Pin 5
 * Motor 3 ESC Signal -> Pin 6
 * Motor 4 ESC Signal -> Pin 9
 * 
 * ESC Ground -> Arduino Ground
 * ESC Power -> External Battery (DO NOT power from Arduino)
 * 
 * Serial Commands:
 * '0' - Stop all motors
 * '1'-'4' - Test individual motor
 * 'a' - Test all motors together
 * 'r' - Ramp test (gradual increase/decrease)
 * 's' - Sequential test (one motor at a time)
 * 'c' - Calibrate ESCs
 * '+' - Increase throttle by 10%
 * '-' - Decrease throttle by 10%
 */

#include <ESP32Servo.h>

// Motor pins
const int MOTOR_PINS[] = {7, 6, 15, 16};
const int NUM_MOTORS = 4;

// SimonK ESC PWM ranges (microseconds)
const int ESC_MIN = 1000;  // Minimum thzrottle
const int ESC_MAX = 2000;  // Maximum throttle
const int ESC_ARM = 1000;  // Arming value

// Servo objects for each motor
Servo motors[NUM_MOTORS];

// Current throttle value (1000-2000)
int currentThrottle = ESC_MIN;

// Test parameters
const int TEST_THROTTLE = 1100; // Safe test speed (30% throttle)
const int RAMP_DELAY = 50;      // Delay for ramp tests
const int TEST_DURATION = 2000; // Test duration in ms

// Function prototypes
void armESCs();
void processCommand(char cmd);
void stopAllMotors();
void testSingleMotor(int motorIndex);
void testAllMotors();
void rampTest();
void sequentialTest();
void calibrateESCs();
void adjustThrottle(int delta);
void printMenu();

void setup() {
  Serial.begin(115200);
  
  // Attach all motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].attach(MOTOR_PINS[i], ESC_MIN, ESC_MAX);
    motors[i].writeMicroseconds(ESC_MIN);
  }
  
  Serial.println("====================================");
  Serial.println("BLDC Motor Testing - SimonK Protocol");
  Serial.println("====================================");
  Serial.println("Initializing ESCs...");
  delay(2000);
  
  // Arm ESCs
  armESCs();
  
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    processCommand(cmd);
  }
}

void armESCs() {
  Serial.println("Arming ESCs (sending minimum throttle)...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(ESC_ARM);
  }
  delay(3000);
  Serial.println("ESCs Armed!");
  Serial.println();
}

void processCommand(char cmd) {
  switch (cmd) {
    case '0':
      stopAllMotors();
      break;
      
    case '1':
    case '2':
    case '3':
    case '4':
      testSingleMotor(cmd - '1');
      break;
      
    case 'a':
      testAllMotors();
      break;
      
    case 'r':
      rampTest();
      break;
      
    case 's':
      sequentialTest();
      break;
      
    case 'c':
      calibrateESCs();
      break;
      
    case '+':
      adjustThrottle(100);
      break;
      
    case '-':
      adjustThrottle(-100);
      break;
      
    case 'm':
      printMenu();
      break;
      
    default:
      Serial.println("Unknown command. Press 'm' for menu.");
  }
}

void stopAllMotors() {
  Serial.println("Stopping all motors...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(ESC_MIN);
  }
  currentThrottle = ESC_MIN;
  Serial.println("All motors stopped.");
  Serial.println();
}

void testSingleMotor(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) return;
  
  Serial.print("Testing Motor ");
  Serial.print(motorIndex + 1);
  Serial.print(" at ");
  Serial.print(TEST_THROTTLE);
  Serial.println("us");
  
  motors[motorIndex].writeMicroseconds(TEST_THROTTLE);
  delay(TEST_DURATION);
  motors[motorIndex].writeMicroseconds(ESC_MIN);
  
  Serial.println("Test complete.");
  Serial.println();
}

void testAllMotors() {
  Serial.println("Testing all motors simultaneously...");
  Serial.print("Throttle: ");
  Serial.print(TEST_THROTTLE);
  Serial.println("us");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(TEST_THROTTLE);
  }
  
  delay(TEST_DURATION);
  stopAllMotors();
}

void rampTest() {
  Serial.println("Starting ramp test (all motors)...");
  Serial.println("Ramping up...");
  
  // Ramp up
  for (int throttle = ESC_MIN; throttle <= TEST_THROTTLE; throttle += 10) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].writeMicroseconds(throttle);
    }
    Serial.print("Throttle: ");
    Serial.println(throttle);
    delay(RAMP_DELAY);
  }
  
  delay(1000);
  
  Serial.println("Ramping down...");
  // Ramp down
  for (int throttle = TEST_THROTTLE; throttle >= ESC_MIN; throttle -= 10) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].writeMicroseconds(throttle);
    }
    Serial.print("Throttle: ");
    Serial.println(throttle);
    delay(RAMP_DELAY);
  }
  
  Serial.println("Ramp test complete.");
  Serial.println();
}

void sequentialTest() {
  Serial.println("Starting sequential test...");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.println(" spinning...");
    
    motors[i].writeMicroseconds(TEST_THROTTLE);
    delay(TEST_DURATION);
    motors[i].writeMicroseconds(ESC_MIN);
    delay(500);
  }
  
  Serial.println("Sequential test complete.");
  Serial.println();
}

void calibrateESCs() {
  Serial.println("====================================");
  Serial.println("ESC CALIBRATION MODE");
  Serial.println("====================================");
  Serial.println("1. Disconnect battery from ESCs");
  Serial.println("2. Press any key when ready...");
  
  while (!Serial.available());
  Serial.read();
  
  Serial.println("Sending maximum throttle...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(ESC_MAX);
  }
  
  Serial.println("3. Connect battery to ESCs now");
  Serial.println("4. Wait for ESC beep sequence");
  Serial.println("5. Press any key when done...");
  
  while (!Serial.available());
  Serial.read();
  
  Serial.println("Sending minimum throttle...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(ESC_MIN);
  }
  
  delay(2000);
  Serial.println("Calibration complete!");
  Serial.println("Disconnect and reconnect battery.");
  Serial.println();
}

void adjustThrottle(int delta) {
  currentThrottle = constrain(currentThrottle + delta, ESC_MIN, ESC_MAX);
  
  Serial.print("Setting throttle to: ");
  Serial.print(currentThrottle);
  Serial.print("us (");
  Serial.print(map(currentThrottle, ESC_MIN, ESC_MAX, 0, 100));
  Serial.println("%)");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(currentThrottle);
  }
}

void printMenu() {
  Serial.println("====================================");
  Serial.println("COMMAND MENU");
  Serial.println("====================================");
  Serial.println("0 - Stop all motors");
  Serial.println("1-4 - Test individual motor");
  Serial.println("a - Test all motors together");
  Serial.println("r - Ramp test (gradual speed change)");
  Serial.println("s - Sequential test (one at a time)");
  Serial.println("c - Calibrate ESCs");
  Serial.println("+ - Increase throttle by 10%");
  Serial.println("- - Decrease throttle by 10%");
  Serial.println("m - Show this menu");
  Serial.println("====================================");
  Serial.println();
}